// Streaming Motion (XOR) accelerator — bus master, CI id 0x0F.
//
// CI protocol (valueA):
//   0  read  status {error, busy, done}
//   1  write source-A frame address (valueB)  -- sobelCurr
//   2  write source-B frame address (valueB)  -- sobelPrev
//   3  write destination address    (valueB)  -- motion
//   4  control: valueB[0]=1 starts the accelerator (ignored while busy)
//   5  read back srcA address
//   6  read back srcB address
//   7  read back dst  address
//
// Operation sequence after start:
//   For each sampled row in [0, 2, 4, .. IMG_HEIGHT-2]:
//     1. Load one row from srcA into lineBufA (via burst reads from SDRAM).
//     2. Load one row from srcB into lineBufB (via burst reads from SDRAM).
//     3. Build a half-resolution RGB565 overlay: current edges white, new edges red.
//     4. Write outBuf back to SDRAM (via burst writes).
//   Then signal done.
//
// Assumptions: IMG_WIDTH * IMG_HEIGHT is a multiple of 4 (true for 640x480).
//
// Note: The XOR operation has no neighborhood dependency, so we *could*
// process it as one big linear stream rather than row-by-row. The row-based
// approach is used here for consistency with accSobel and to keep burst
// addressing simple. It costs at most one extra request-grant per row, which
// is negligible compared to the SDRAM read/write throughput.
module accMotion #(
    parameter [7:0]   customId         = 8'h0F,
    parameter integer IMG_WIDTH        = 640,
    parameter integer IMG_HEIGHT       = 480,
    parameter integer MAX_BURST_WORDS  = 8
) (
    input  wire        start,
                       clock,
                       reset,
    input  wire [31:0] valueA,
                       valueB,
    input  wire [7:0]  ciN,
    output wire        done,
    output wire [31:0] result,
    // Bus-master request/grant
    output wire        requestTransaction,
    input  wire        transactionGranted,
    // Bus inputs
    input  wire        endTransactionIn,
                       dataValidIn,
                       busErrorIn,
                       busyIn,
    input  wire [31:0] addressDataIn,
    // Bus outputs
    output reg         beginTransactionOut,
                       readNotWriteOut,
                       endTransactionOut,
    output wire        dataValidOut,
    output reg  [3:0]  byteEnablesOut,
    output reg  [7:0]  burstSizeOut,
    output wire [31:0] addressDataOut
);
// ─── Derived constants ────────────────────────────────────────────────────────
localparam integer WORDS_PER_ROW = IMG_WIDTH / 4;   // 160 for 640-wide image
localparam integer OUT_WIDTH = IMG_WIDTH / 2;
localparam integer OUT_HEIGHT = IMG_HEIGHT / 2;
localparam integer OUT_WORDS_PER_ROW = OUT_WIDTH / 2; // 160 RGB565 words for 320-wide output
// Bit widths — correct for the 640x480 defaults.
localparam integer ROW_BITS  = 9;   // ceil(log2(480))
localparam integer WORD_BITS = 8;   // ceil(log2(160))
localparam integer OUT_WORD_BITS = 9; // ceil(log2(320))

// ─── FSM state encoding ───────────────────────────────────────────────────────
localparam [3:0]
    S_IDLE        = 4'd0,
    S_INIT        = 4'd1,
    S_LOADA_REQ   = 4'd2,   // assert requestTransaction for srcA read burst
    S_LOADA_SETUP = 4'd3,   // assert beginTransaction (one cycle)
    S_LOADA_BURST = 4'd4,   // receive srcA words
    S_LOADB_REQ   = 4'd5,
    S_LOADB_SETUP = 4'd6,
    S_LOADB_BURST = 4'd7,
    S_COMPUTE     = 4'd8,   // expand one source word into two RGB565 words
    S_WRITE_REQ   = 4'd9,
    S_WRITE_SETUP = 4'd10,
    S_WRITE_BURST = 4'd11,
    S_WRITE_END   = 4'd12,
    S_ADVANCE     = 4'd13,  // advance to next row or finish
    S_DONE        = 4'd14;
reg [3:0] state;

// ─── CI configuration registers ──────────────────────────────────────────────
reg [31:0] srcAAddressReg, srcBAddressReg, dstAddressReg;
reg        doneReg, errorReg;
wire isMyCi   = start & (ciN == customId);
wire busyWire = (state != S_IDLE);
assign done   = isMyCi;
assign result =
    (isMyCi & (valueA == 32'd0)) ? {29'd0, errorReg, busyWire, doneReg} :
    (isMyCi & (valueA == 32'd5)) ? srcAAddressReg :
    (isMyCi & (valueA == 32'd6)) ? srcBAddressReg :
    (isMyCi & (valueA == 32'd7)) ? dstAddressReg  :
    32'd0;

// ─── Line buffers ─────────────────────────────────────────────────────────────
// Store as 32-bit words. Source rows contain four 8-bit pixels per word; the
// output row contains two RGB565 pixels per word. The output is downsampled
// by 2 horizontally and vertically to reduce SDRAM traffic.
reg [31:0] lineBufA [0:WORDS_PER_ROW-1];
reg [31:0] lineBufB [0:WORDS_PER_ROW-1];
reg [31:0] outBuf   [0:OUT_WORDS_PER_ROW-1];

// ─── Counters and shadow address registers ────────────────────────────────────
reg [ROW_BITS-1:0]  rowProc;       // row currently being processed (0..IMG_HEIGHT-1)
reg [WORD_BITS-1:0] wordIdx;       // counts received words during a load burst (0..159)
reg [OUT_WORD_BITS-1:0] writeWordIdx;  // counts words sent during a write burst
reg [WORD_BITS-1:0] computeIdx;    // index into lineBufA/B during S_COMPUTE
reg [5:0]           loadBurstWords;
reg [5:0]           writeBurstWords;
reg [8:0]           writeCount;    // burst countdown; [8]=1 when all words sent
reg [31:0]          loadAAddrReg;  // next SDRAM address for srcA reads
reg [31:0]          loadBAddrReg;  // next SDRAM address for srcB reads
reg [31:0]          writeAddrReg;  // next SDRAM address for dst writes

// Words still needed for the *current row* during a load burst phase.
wire [8:0] loadWordsRemaining  = WORDS_PER_ROW - wordIdx;
wire [8:0] writeWordsRemaining = OUT_WORDS_PER_ROW - writeWordIdx;
// Burst chunk size: capped at MAX_BURST_WORDS (== 16 by default → param matches Sobel).
// Use a 6-bit sized constant via concatenation-with-truncation to avoid any
// tool-specific quirks with part-selects on integer parameters.
localparam [5:0] MAX_BURST_W6 = MAX_BURST_WORDS[5:0];
wire [5:0] loadChunkWords  = (loadWordsRemaining  > MAX_BURST_WORDS) ? MAX_BURST_W6 : loadWordsRemaining[5:0];
wire [5:0] writeChunkWords = (writeWordsRemaining > MAX_BURST_WORDS) ? MAX_BURST_W6 : writeWordsRemaining[5:0];

// ─── Registered bus inputs (pipeline one stage for timing) ───────────────────
reg        endTxReg, dataValidReg;
reg [31:0] addrDataReg;
always @(posedge clock) begin
    endTxReg     <= endTransactionIn;
    dataValidReg <= dataValidIn;
    addrDataReg  <= addressDataIn;
end

// ─── Bus output signals ───────────────────────────────────────────────────────
assign requestTransaction = ((state == S_LOADA_REQ) | (state == S_LOADB_REQ) | (state == S_WRITE_REQ));
reg [31:0] addrDataOutReg;
reg        dataValidOutReg;
assign addressDataOut = addrDataOutReg;
assign dataValidOut   = dataValidOutReg;

// ─── Motion overlay datapath (purely combinational) ──────────────────────────
// Source pixels are 8-bit Sobel values. Output pixels are RGB565:
// current edge = white, newly appeared edge = red.
function [15:0] motionPixel;
    input [7:0] curr;
    input [7:0] prev;
    begin
        motionPixel = (curr != 8'd0 && prev == 8'd0) ? 16'hF800 :
                      (curr != 8'd0)                 ? 16'hFFFF :
                                                        16'h0000;
    end
endfunction

wire [31:0] motionWord = {motionPixel(lineBufA[computeIdx][23:16], lineBufB[computeIdx][23:16]),
                          motionPixel(lineBufA[computeIdx][7:0],   lineBufB[computeIdx][7:0])};

// doWrite: fire one word onto the bus when the burst is active and bus is ready
wire doWrite = (state == S_WRITE_BURST) & ~writeCount[8] & ~busyIn;

// ─── Main clocked FSM ─────────────────────────────────────────────────────────
always @(posedge clock) begin
    if (reset) begin
        state                <= S_IDLE;
        srcAAddressReg       <= 32'd0;
        srcBAddressReg       <= 32'd0;
        dstAddressReg        <= 32'd0;
        doneReg              <= 1'b0;
        errorReg             <= 1'b0;
        rowProc              <= {ROW_BITS{1'b0}};
        wordIdx              <= {WORD_BITS{1'b0}};
        writeWordIdx         <= {OUT_WORD_BITS{1'b0}};
        computeIdx           <= {WORD_BITS{1'b0}};
        loadBurstWords       <= 6'd0;
        writeBurstWords      <= 6'd0;
        writeCount           <= 9'd0;
        loadAAddrReg         <= 32'd0;
        loadBAddrReg         <= 32'd0;
        writeAddrReg         <= 32'd0;
        beginTransactionOut  <= 1'b0;
        readNotWriteOut      <= 1'b0;
        endTransactionOut    <= 1'b0;
        byteEnablesOut       <= 4'b0000;
        burstSizeOut         <= 8'd0;
        addrDataOutReg       <= 32'd0;
        dataValidOutReg      <= 1'b0;
    end else begin
        // ── One-cycle pulse defaults ──────────────────────────────────────────
        beginTransactionOut <= 1'b0;
        readNotWriteOut     <= 1'b0;
        endTransactionOut   <= 1'b0;
        byteEnablesOut      <= 4'b0000;
        burstSizeOut        <= 8'd0;
        addrDataOutReg      <= 32'd0;
        dataValidOutReg     <= 1'b0;

        // ── CI register writes (live-through while idle or busy) ──────────────
        if (isMyCi && (valueA == 32'd1)) srcAAddressReg <= valueB;
        if (isMyCi && (valueA == 32'd2)) srcBAddressReg <= valueB;
        if (isMyCi && (valueA == 32'd3)) dstAddressReg  <= valueB;
        if (isMyCi && (valueA == 32'd4) && valueB[0] && !busyWire) begin
            doneReg  <= 1'b0;
            errorReg <= 1'b0;
            state    <= S_INIT;
        end

        // ── FSM ───────────────────────────────────────────────────────────────
        case (state)
            //------------------------------------------------------------------
            S_IDLE: ; // wait for CI start command
            //------------------------------------------------------------------
            S_INIT: begin
                loadAAddrReg <= srcAAddressReg;
                loadBAddrReg <= srcBAddressReg;
                writeAddrReg <= dstAddressReg;
                rowProc      <= {ROW_BITS{1'b0}};
                wordIdx      <= {WORD_BITS{1'b0}};
                state        <= S_LOADA_REQ;
            end
            //------------------------------------------------------------------
            // Load source A row
            //------------------------------------------------------------------
            S_LOADA_REQ: begin
                if (transactionGranted) state <= S_LOADA_SETUP;
            end
            S_LOADA_SETUP: begin
                beginTransactionOut <= 1'b1;
                readNotWriteOut     <= 1'b1;
                byteEnablesOut      <= 4'hF;
                burstSizeOut        <= loadChunkWords - 8'd1;
                loadBurstWords      <= loadChunkWords;
                addrDataOutReg      <= loadAAddrReg;
                state               <= S_LOADA_BURST;
            end
            S_LOADA_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= S_IDLE;
                end else begin
                    if (dataValidReg) begin
                        lineBufA[wordIdx] <= addrDataReg;
                        wordIdx <= wordIdx + 1;
                    end
                    if (endTxReg) begin
                        loadAAddrReg <= loadAAddrReg + {loadBurstWords, 2'b00};
                        // Effective wordIdx for the next-burst check: account
                        // for the simultaneous dataValidReg pulse that arrives
                        // with endTxReg on the last word of a burst.
                        if ((wordIdx + {7'd0, dataValidReg}) < WORDS_PER_ROW) begin
                            // Need another burst for the rest of the row.
                            state <= S_LOADA_REQ;
                        end else begin
                            // Row fully loaded. Start loading row of srcB.
                            wordIdx <= {WORD_BITS{1'b0}};
                            state   <= S_LOADB_REQ;
                        end
                    end
                end
            end
            //------------------------------------------------------------------
            // Load source B row
            //------------------------------------------------------------------
            S_LOADB_REQ: begin
                if (transactionGranted) state <= S_LOADB_SETUP;
            end
            S_LOADB_SETUP: begin
                beginTransactionOut <= 1'b1;
                readNotWriteOut     <= 1'b1;
                byteEnablesOut      <= 4'hF;
                burstSizeOut        <= loadChunkWords - 8'd1;
                loadBurstWords      <= loadChunkWords;
                addrDataOutReg      <= loadBAddrReg;
                state               <= S_LOADB_BURST;
            end
            S_LOADB_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= S_IDLE;
                end else begin
                    if (dataValidReg) begin
                        lineBufB[wordIdx] <= addrDataReg;
                        wordIdx <= wordIdx + 1;
                    end
                    if (endTxReg) begin
                        loadBAddrReg <= loadBAddrReg + {loadBurstWords, 2'b00};
                        if ((wordIdx + {7'd0, dataValidReg}) < WORDS_PER_ROW) begin
                            state <= S_LOADB_REQ;
                        end else begin
                            // Both rows loaded: build the half-resolution RGB565 overlay.
                            wordIdx    <= {WORD_BITS{1'b0}};
                            computeIdx <= {WORD_BITS{1'b0}};
                            state      <= S_COMPUTE;
                        end
                    end
                end
            end
            //------------------------------------------------------------------
            // Compute: sample pixels 0 and 2 from each source word, producing
            // one output word with two RGB565 pixels.
            // After WORDS_PER_ROW cycles, start the
            // write burst back to SDRAM.
            //------------------------------------------------------------------
            S_COMPUTE: begin
                outBuf[computeIdx] <= motionWord;
                if (computeIdx == (WORDS_PER_ROW - 1)) begin
                    computeIdx   <= {WORD_BITS{1'b0}};
                    writeWordIdx <= {OUT_WORD_BITS{1'b0}};
                    state        <= S_WRITE_REQ;
                end else begin
                    computeIdx <= computeIdx + 1;
                end
            end
            //------------------------------------------------------------------
            // Write output row back to SDRAM
            //------------------------------------------------------------------
            S_WRITE_REQ: begin
                if (transactionGranted) state <= S_WRITE_SETUP;
            end
            S_WRITE_SETUP: begin
                beginTransactionOut <= 1'b1;
                readNotWriteOut     <= 1'b0;
                byteEnablesOut      <= 4'hF;
                burstSizeOut        <= writeChunkWords - 8'd1;
                writeBurstWords     <= writeChunkWords;
                addrDataOutReg      <= writeAddrReg;
                writeCount          <= {3'd0, writeChunkWords} - 9'd1;
                state               <= S_WRITE_BURST;
            end
            S_WRITE_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= S_IDLE;
                end else if (writeCount[8] && !busyIn) begin
                    // Last word was accepted → release the bus.
                    state <= S_WRITE_END;
                end else if (!writeCount[8] && !busyIn) begin
                    // Send next word.
                    addrDataOutReg  <= outBuf[writeWordIdx];
                    dataValidOutReg <= 1'b1;
                    writeWordIdx    <= writeWordIdx + 1;
                    writeCount      <= writeCount - 9'd1;
                end else begin
                    // busyIn==1: hold current data/valid.
                    addrDataOutReg  <= addrDataOutReg;
                    dataValidOutReg <= dataValidOutReg;
                end
            end
            S_WRITE_END: begin
                endTransactionOut <= 1'b1;
                writeAddrReg      <= writeAddrReg + {writeBurstWords, 2'b00};
                if (writeWordIdx < OUT_WORDS_PER_ROW) begin
                    // Row not fully written yet → request another burst.
                    state <= S_WRITE_REQ;
                end else begin
                    state <= S_ADVANCE;
                end
            end
            //------------------------------------------------------------------
            S_ADVANCE: begin
                if (rowProc < (OUT_HEIGHT - 1)) begin
                    rowProc <= rowProc + 1;
                    wordIdx <= {WORD_BITS{1'b0}};
                    loadAAddrReg <= loadAAddrReg + IMG_WIDTH;
                    loadBAddrReg <= loadBAddrReg + IMG_WIDTH;
                    state   <= S_LOADA_REQ;
                end else begin
                    state <= S_DONE;
                end
            end
            //------------------------------------------------------------------
            S_DONE: begin
                doneReg <= 1'b1;
                state   <= S_IDLE;
            end
            //------------------------------------------------------------------
            default: begin
                errorReg <= 1'b1;
                state    <= S_IDLE;
            end
        endcase
    end
end
endmodule

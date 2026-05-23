// Streaming Sobel edge-detection accelerator — bus master, CI id 0x0E.
//
// CI protocol (valueA):
//   0  read  status {error, busy, done}
//   1  write source  frame address    (valueB)
//   2  write dest    edge-map address (valueB)
//   3  control: valueB[0]=1 starts the accelerator (ignored while busy)
//   4  read back source address
//   5  read back dest   address
//
// Operation sequence after start:
//   1. Pre-fill: load rows 0, 1, 2 from SDRAM into three circular line buffers.
//   2. Loop rows 1 .. IMG_HEIGHT-2:
//        a. Compute Sobel for every pixel in the row (one pixel/clock).
//        b. Write the output row to SDRAM.
//        c. Rotate the circular buffers; load the next source row.
//   3. Signal done.
//
// Border pixels (row 0, row HEIGHT-1, col 0, col WIDTH-1) are not written;
// the caller should zero-initialise the destination buffer.
//
// Assumptions: IMG_WIDTH is a multiple of 4.

module accSobel #(
    parameter [7:0]   customId   = 8'h0E,
    parameter integer IMG_WIDTH  = 640,
    parameter integer IMG_HEIGHT = 480
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

// Bit widths — correct for the 640×480 defaults.
// Increase COL_BITS / ROW_BITS / WORD_BITS when IMG_WIDTH / IMG_HEIGHT grow.
localparam integer COL_BITS  = 10;  // ceil(log2(640))  — addresses 0..1023
localparam integer ROW_BITS  = 9;   // ceil(log2(480))  — addresses 0..511
localparam integer WORD_BITS = 8;   // ceil(log2(160))  — addresses 0..255

// ─── FSM state encoding ───────────────────────────────────────────────────────
localparam [3:0]
    S_IDLE        = 4'd0,
    S_INIT        = 4'd1,
    S_LOAD_REQ    = 4'd2,   // assert requestTransaction for a read burst
    S_LOAD_SETUP  = 4'd3,   // assert beginTransaction (one cycle)
    S_LOAD_BURST  = 4'd4,   // accumulate dataValidIn words into line buffer
    S_COMPUTE     = 4'd5,   // compute one Sobel pixel per clock
    S_WRITE_REQ   = 4'd6,   // assert requestTransaction for a write burst
    S_WRITE_SETUP = 4'd7,   // assert beginTransaction (one cycle)
    S_WRITE_BURST = 4'd8,   // drive output pixels onto bus
    S_WRITE_END   = 4'd9,   // assert endTransactionOut (one cycle)
    S_ADVANCE     = 4'd10,  // rotate buffers, decide whether to loop
    S_DONE        = 4'd11;

reg [3:0] state;

// ─── CI configuration registers ──────────────────────────────────────────────
reg [31:0] sourceAddressReg, destinationAddressReg;
reg        doneReg, errorReg;

wire isMyCi   = start & (ciN == customId);
wire busyWire = (state != S_IDLE);

assign done   = isMyCi;
assign result =
    (isMyCi & (valueA == 32'd0)) ? {29'd0, errorReg, busyWire, doneReg} :
    (isMyCi & (valueA == 32'd4)) ? sourceAddressReg :
    (isMyCi & (valueA == 32'd5)) ? destinationAddressReg :
    32'd0;

// ─── Three physical line buffers + one output buffer (byte-granular) ──────────
// Each buffer holds one full image row: IMG_WIDTH bytes.
// Valid index range: 0 .. IMG_WIDTH-1.
reg [7:0] lineBuf0 [0:IMG_WIDTH-1];
reg [7:0] lineBuf1 [0:IMG_WIDTH-1];
reg [7:0] lineBuf2 [0:IMG_WIDTH-1];
reg [7:0] outBuf   [0:IMG_WIDTH-1];

// ─── Circular buffer rotation indices ────────────────────────────────────────
// topIdx/midIdx/botIdx tell which physical buffer holds the top/mid/bottom row
// of the current 3×3 neighbourhood.
reg [1:0] topIdx, midIdx, botIdx;

// ─── Counters and shadow address registers ────────────────────────────────────
reg [ROW_BITS-1:0]  rowProc;       // row currently being processed (Sobel output)
reg [COL_BITS-1:0]  compCol;       // Which column (pixel x-coordinate) are we calculating right now
reg [1:0]           loadBufIdx;    // which physical buffer to write during load
reg [WORD_BITS-1:0] wordIdx;       // Counts up from 0 to 159 to keep track of how many 32-bit words we have read from the bus.
reg [WORD_BITS-1:0] writeWordIdx;  // Counts up from 0 to 159 to keep track of how many 32-bit words we have written to the bus during the write burst
reg [8:0]           writeCount;    // burst countdown; [8]=1 when all words sent since it weaps around to a negative number
reg [1:0]           prefillCount;  // Counts 0, 1, 2 as the accelerator fetches the very first three rows. Once it hits 3, the FSM knows the pipeline is primed and math can begin
reg [31:0]          loadAddrReg;   // The 32-bit SDRAM address where the next raw image row should be read from.
reg [31:0]          writeAddrReg;  // The 32-bit SDRAM address where the next finished edge map row should be written to.

// ─── Registered bus inputs (pipeline one stage for timing) ───────────────────
reg        endTxReg, dataValidReg;
reg [31:0] addrDataReg;

// reg for input from bus to avoid timing violations

always @(posedge clock) begin
    endTxReg    <= endTransactionIn;
    dataValidReg <= dataValidIn;
    addrDataReg <= addressDataIn;
end

// ─── Bus output signals ───────────────────────────────────────────────────────
assign requestTransaction = ((state == S_LOAD_REQ) | (state == S_WRITE_REQ));

reg [31:0] addrDataOutReg;
reg        dataValidOutReg;
assign addressDataOut = addrDataOutReg;
assign dataValidOut   = dataValidOutReg;

// ─── Pixel access: combinational mux into the three physical line buffers ─────
// compCol wraps on underflow/overflow; border pixels use the 0-output path.
reg [7:0] p1, p2, p3, p4, p6, p7, p8, p9;

always @(*) begin
    p1 = 8'd0; 
    p2 = 8'd0;
    p3 = 8'd0;
    p4 = 8'd0;             
    p6 = 8'd0;
    p7 = 8'd0; 
    p8 = 8'd0; 
    p9 = 8'd0;
    // initiazilation to 0 

    case (topIdx)
        2'd0: begin
            p1 = lineBuf0[compCol - 1]; p2 = lineBuf0[compCol]; p3 = lineBuf0[compCol + 1];
        end
        2'd1: begin
            p1 = lineBuf1[compCol - 1]; p2 = lineBuf1[compCol]; p3 = lineBuf1[compCol + 1];
        end
        default: begin
            p1 = lineBuf2[compCol - 1]; p2 = lineBuf2[compCol]; p3 = lineBuf2[compCol + 1];
        end
    endcase

    case (midIdx)
        2'd0: begin p4 = lineBuf0[compCol - 1]; p6 = lineBuf0[compCol + 1]; end
        2'd1: begin p4 = lineBuf1[compCol - 1]; p6 = lineBuf1[compCol + 1]; end
        default: begin p4 = lineBuf2[compCol - 1]; p6 = lineBuf2[compCol + 1]; end
    endcase

    case (botIdx)
        2'd0: begin
            p7 = lineBuf0[compCol - 1]; p8 = lineBuf0[compCol]; p9 = lineBuf0[compCol + 1];
        end
        2'd1: begin
            p7 = lineBuf1[compCol - 1]; p8 = lineBuf1[compCol]; p9 = lineBuf1[compCol + 1];
        end
        default: begin
            p7 = lineBuf2[compCol - 1]; p8 = lineBuf2[compCol]; p9 = lineBuf2[compCol + 1];
        end
    endcase
end

// ─── Sobel datapath — reuse sobelCi directly ─────────────────────────────────
// Pack individual pixels into sobelCi's two-register format:
//   valueA = { p4, p3, p2, p1 }
//   valueB = { p9, p8, p7, p6 }
// Tie start=1 and ciN to sobelCi's customId so the output is always valid.
wire [31:0] s_sobelValueA = {p4, p3, p2, p1};
wire [31:0] s_sobelValueB = {p9, p8, p7, p6};
wire [31:0] s_sobelResult;

sobelCi #(.customId(8'h00)) sobelInner (
    .start(1'b1),
    .clock(clock),
    .reset(reset),
    .stall(1'b0),
    .busIdle(1'b0),
    .valueA(s_sobelValueA),
    .valueB(s_sobelValueB),
    .ciN(8'h00),
    .done(),
    .result(s_sobelResult)
);

wire [7:0] sobelPixel = s_sobelResult[7:0];

// ─── Output word packing (4 bytes per 32-bit bus word) ────────────────────────
// writeWordIdx is the current word being sent; pixels are stored little-endian.
wire [31:0] outWord = {
    outBuf[{writeWordIdx, 2'b11}],
    outBuf[{writeWordIdx, 2'b10}],
    outBuf[{writeWordIdx, 2'b01}],
    outBuf[{writeWordIdx, 2'b00}]
};

/* 
In C, if you wanted to read 4 bytes at a time based on a word counter, 
you would write something like this:
outBuf[writeWordIdx * 4 + 0]
outBuf[writeWordIdx * 4 + 1]

Hardware multipliers take up a lot of physical space on an FPGA.
 But in binary, adding zeros to the end of a number is the same as multiplying by 
    powers of 2 (just like adding a zero in decimal multiplies by 10).

    list backwards because This is because the OpenRISC CPU and the SDRAM use Little-Endian byte ordering. 
*/

// doWrite: fire one word onto the bus when the burst is active and bus is ready
wire doWrite = (state == S_WRITE_BURST) & ~writeCount[8] & ~busyIn;

// ─── Main clocked FSM ─────────────────────────────────────────────────────────
always @(posedge clock) begin
    if (reset) begin
        state                 <= S_IDLE;
        sourceAddressReg      <= 32'd0;
        destinationAddressReg <= 32'd0;
        doneReg               <= 1'b0;
        errorReg              <= 1'b0;
        topIdx <= 2'd0; midIdx <= 2'd1; botIdx <= 2'd2;
        rowProc       <= {ROW_BITS{1'b0}};
        loadBufIdx    <= 2'd0;
        wordIdx       <= {WORD_BITS{1'b0}};
        writeWordIdx  <= {WORD_BITS{1'b0}};
        writeCount    <= 9'd0;
        compCol       <= {COL_BITS{1'b0}};
        prefillCount  <= 2'd0;
        loadAddrReg   <= 32'd0;
        writeAddrReg  <= 32'd0;
        // Bus output regs
        beginTransactionOut <= 1'b0;
        readNotWriteOut     <= 1'b0;
        endTransactionOut   <= 1'b0;
        byteEnablesOut      <= 4'b0000;
        burstSizeOut        <= 8'd0;
        addrDataOutReg      <= 32'd0;
        dataValidOutReg     <= 1'b0;
    end else begin

        // ── One-cycle pulse defaults ──────────────────────────────────────────
        beginTransactionOut <= 1'b0;
        readNotWriteOut     <= 1'b0;
        endTransactionOut   <= 1'b0;
        byteEnablesOut      <= 4'b0000;
        burstSizeOut        <= 8'd0;

        // ── Bus output signals: hold during busyIn, clear otherwise ──────────────
        addrDataOutReg      <= (doWrite) ? outWord :
                              (state == S_WRITE_BURST && busyIn) ? addrDataOutReg : 32'd0;
        dataValidOutReg     <= (doWrite) ? 1'b1 :
                              (state == S_WRITE_BURST && busyIn) ? dataValidOutReg : 1'b0;


        // ── CI register writes (live-through while idle or busy) ──────────────
        if (isMyCi && (valueA == 32'd1)) sourceAddressReg      <= valueB;
        if (isMyCi && (valueA == 32'd2)) destinationAddressReg <= valueB;
        if (isMyCi && (valueA == 32'd3) && valueB[0] && !busyWire) begin
            doneReg  <= 1'b0;
            errorReg <= 1'b0;
            state    <= S_INIT;
        end

        // ── FSM ───────────────────────────────────────────────────────────────
        case (state)

            //------------------------------------------------------------------
            S_IDLE: ; // wait for CI start command

            //------------------------------------------------------------------
            // Initialise shadow registers, then kick off the pre-fill sequence.
            S_INIT: begin
                loadAddrReg  <= sourceAddressReg; // start loading from the first source byte
                // Row 1 is the first output row (row 0 is a border).
                writeAddrReg <= destinationAddressReg + IMG_WIDTH; // start writing to the second row of the dest buffer, since the first row is a border and should be 0 to do so we skip the first row that is the first 640 bytes
                loadBufIdx   <= 2'd0; // start loading into lineBuf0, which will be the top row of the first 3×3, the loadBufIdx will rotate as we rotate the line buffers
                prefillCount <= 2'd0; // the counter for the pre-filling of the first three rows, here we set initialize it to 0
                rowProc      <= 1; // This tracks which row we are actively applying the Sobel math to. Again, because Row 0 is a border, the first row we will process is Row 1.
                topIdx <= 2'd0; midIdx <= 2'd1; botIdx <= 2'd2; // initialize the circular buffer indices so that lineBuf0 is top, lineBuf1 is mid, lineBuf2 is bot. We will rotate these as we go, but this is the starting configuration.
                state  <= S_LOAD_REQ; 
            end

            //------------------------------------------------------------------
            // Assert requestTransaction and wait for the bus grant.
            S_LOAD_REQ: begin
                if (transactionGranted) state <= S_LOAD_SETUP;
            end

            //------------------------------------------------------------------
            // One-cycle burst setup: address + readNotWrite + burst-size.
            S_LOAD_SETUP: begin
                beginTransactionOut <= 1'b1;
                readNotWriteOut     <= 1'b1;
                byteEnablesOut      <= 4'hF;
                burstSizeOut        <= WORDS_PER_ROW[7:0] - 8'd1;  // 159 in our default case
                addrDataOutReg      <= loadAddrReg;
                wordIdx             <= 8'd0;
                state               <= S_LOAD_BURST;
            end

            //------------------------------------------------------------------
            // Receive burst data word by word; wait for endTransactionIn.
            S_LOAD_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= S_IDLE;
                end else begin
                    // Store each arriving 32-bit word as four bytes.
                    if (dataValidReg) begin
                        case (loadBufIdx)
                            2'd0: begin
                                lineBuf0[{wordIdx, 2'b00}] <= addrDataReg[7:0];
                                lineBuf0[{wordIdx, 2'b01}] <= addrDataReg[15:8];
                                lineBuf0[{wordIdx, 2'b10}] <= addrDataReg[23:16];
                                lineBuf0[{wordIdx, 2'b11}] <= addrDataReg[31:24];
                            end
                            2'd1: begin
                                lineBuf1[{wordIdx, 2'b00}] <= addrDataReg[7:0];
                                lineBuf1[{wordIdx, 2'b01}] <= addrDataReg[15:8];
                                lineBuf1[{wordIdx, 2'b10}] <= addrDataReg[23:16];
                                lineBuf1[{wordIdx, 2'b11}] <= addrDataReg[31:24];
                            end
                            default: begin
                                lineBuf2[{wordIdx, 2'b00}] <= addrDataReg[7:0];
                                lineBuf2[{wordIdx, 2'b01}] <= addrDataReg[15:8];
                                lineBuf2[{wordIdx, 2'b10}] <= addrDataReg[23:16];
                                lineBuf2[{wordIdx, 2'b11}] <= addrDataReg[31:24];
                            end
                        endcase
                        wordIdx <= wordIdx + 1;
                    end

                    // End of burst: decide next step.
                    if (endTxReg) begin
                        loadAddrReg <= loadAddrReg + IMG_WIDTH; // advance to next row

                        if (prefillCount < 2'd2) begin
                            // Still pre-filling rows 0->1 or 1->2
                            prefillCount <= prefillCount + 2'd1;
                            loadBufIdx   <= loadBufIdx  + 2'd1;
                            state        <= S_LOAD_REQ;
                        end else begin
                            // Pre-fill row 2 just finished (or main-loop load done)
                            if (prefillCount == 2'd2) prefillCount <= 2'd3;
                            compCol <= {COL_BITS{1'b0}};
                            state   <= S_COMPUTE;
                        end
                    end
                end
            end

            //------------------------------------------------------------------
            // Compute one output pixel per clock cycle.
            // Border columns get 0; inner columns use the Sobel datapath.
            S_COMPUTE: begin
                outBuf[compCol] <= ((compCol == {COL_BITS{1'b0}}) ||
                                    (compCol == (IMG_WIDTH - 1)))
                                   ? 8'h00 : sobelPixel;

                // The Problem: The Sobel algorithm uses a 3x3 window. To compute the pixel at column 0, it needs the pixel at column -1 (which doesn't exist). To compute column 639, it needs column 640 (which doesn't exist).
                if (compCol == (IMG_WIDTH - 1)) begin
                    compCol <= {COL_BITS{1'b0}};
                    state   <= S_WRITE_REQ;
                end else begin
                    compCol <= compCol + 1;
                end


                // If we are NOT at the end of the row (else block):
                // It just does compCol <= compCol + 1;. On the next clock cycle, the whole state repeats for the next pixel to the right.

                // If we ARE at the end of the row (column 639):
                // The row is finished! It resets the column counter back to 0 so it's ready for the next row later, and changes the state to S_WRITE_REQ. This breaks the FSM out of the computation "loop" so it can ask the bus for permission to write the finished outBuf back to SDRAM.
            end

            //------------------------------------------------------------------
            S_WRITE_REQ: begin
                if (transactionGranted) state <= S_WRITE_SETUP;
            end

            //------------------------------------------------------------------
            S_WRITE_SETUP: begin
                beginTransactionOut <= 1'b1;
                readNotWriteOut     <= 1'b0;
                byteEnablesOut      <= 4'hF;
                burstSizeOut        <= WORDS_PER_ROW[7:0] - 8'd1;
                addrDataOutReg      <= writeAddrReg;
                writeWordIdx        <= {WORD_BITS{1'b0}};
                writeCount          <= {1'b0, WORDS_PER_ROW[7:0] - 8'd1}; // 9'd159
                state               <= S_WRITE_BURST;
            end

            //------------------------------------------------------------------
            // Drive output words; respect busyIn back-pressure.
            // writeCount[8] becomes 1 after the last word is accepted.
            S_WRITE_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= S_IDLE;
                end else if (writeCount[8] && !busyIn) begin
                    // Last word was accepted; release the bus.
                    state <= S_WRITE_END;
                end else if (!writeCount[8] && !busyIn) begin
                    // Send next word.
                    writeWordIdx    <= writeWordIdx + 1;
                    writeCount      <= writeCount - 9'd1;
                end
            end

            //------------------------------------------------------------------
            S_WRITE_END: begin
                endTransactionOut <= 1'b1;
                state             <= S_ADVANCE;
            end

            //------------------------------------------------------------------
            // Rotate the circular buffers.
            // The former-top buffer will hold the next source row (new botBuf).
            S_ADVANCE: begin
                writeAddrReg <= writeAddrReg + IMG_WIDTH; // we update the next write address that will be for the beginning of the next row

                if (rowProc < (IMG_HEIGHT - 2)) begin
                    // here we do the rotation of the sliding window. The mid row becomes the new top, the bot row becomes the new mid, and the old top becomes the one that will get the new data loaded into 
                    topIdx     <= midIdx; 
                    midIdx     <= botIdx;
                    botIdx     <= topIdx;    // former top becomes new bot
                    loadBufIdx <= topIdx;    // we will load the next row into it
                    rowProc    <= rowProc + 1;
                    state      <= S_LOAD_REQ;
                    // loadAddrReg already points to the next unloaded row
                    // (incremented at the end of each S_LOAD_BURST).
                end else begin 
                    // if rowProc is already at the last row, it means we have finished processing the whole image and we can move to the done state
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

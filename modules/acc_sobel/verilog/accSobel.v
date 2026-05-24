// Streaming Sobel edge-detection accelerator — bus master, CI id 0x0E.
//
// CI protocol (valueA):
//   0  read  status {error, busy, done}
//   1  write source  frame address    (valueB)
//   2  write dest    edge-map address (valueB)
//   3  control: valueB[0]=1 starts the accelerator (ignored while busy)
//   4  read back source address
//   5  read back dest   address

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
localparam integer MAX_BURST_WORDS = 16;

// Bit widths
localparam integer COL_BITS  = 10;  
localparam integer ROW_BITS  = 9;   
localparam integer WORD_BITS = 8;   

// ─── FSM state encoding ───────────────────────────────────────────────────────
localparam [3:0]
    S_IDLE        = 4'd0,
    S_INIT        = 4'd1,
    S_LOAD_REQ    = 4'd2,   
    S_LOAD_SETUP  = 4'd3,   
    S_LOAD_BURST  = 4'd4,   
    S_COMPUTE     = 4'd5,   
    S_WRITE_REQ   = 4'd6,   
    S_WRITE_SETUP = 4'd7,   
    S_WRITE_BURST = 4'd8,   
    S_WRITE_END   = 4'd9,   
    S_ADVANCE     = 4'd10,  
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

// ─── Three physical line buffers + one output buffer ──────────────────────────
reg [7:0] lineBuf0 [0:IMG_WIDTH-1];
reg [7:0] lineBuf1 [0:IMG_WIDTH-1];
reg [7:0] lineBuf2 [0:IMG_WIDTH-1];
reg [7:0] outBuf   [0:IMG_WIDTH-1];

// ─── Circular buffer rotation indices ────────────────────────────────────────
reg [1:0] topIdx, midIdx, botIdx;

// ─── Counters and shadow address registers ────────────────────────────────────
reg [ROW_BITS-1:0]  rowProc;       
reg [COL_BITS-1:0]  compCol;       
reg [1:0]           loadBufIdx;    
reg [WORD_BITS-1:0] wordIdx;       
reg [WORD_BITS-1:0] writeWordIdx;  
reg [5:0]           loadBurstWords;
reg [5:0]           writeBurstWords;
reg [8:0]           writeCount;    
reg [1:0]           prefillCount;  
reg [31:0]          loadAddrReg;   
reg [31:0]          writeAddrReg;  

// Filter delay registers
reg [7:0] p_left;
reg [7:0] p_center;

// Dynamic burst sizing chunk logic (Fixed the hardcoded 32 bug!)
wire [8:0] loadWordsRemaining  = WORDS_PER_ROW - wordIdx;
wire [8:0] writeWordsRemaining = WORDS_PER_ROW - writeWordIdx;
wire [5:0] loadChunkWords  = (loadWordsRemaining > MAX_BURST_WORDS) ? MAX_BURST_WORDS[5:0] : loadWordsRemaining[5:0];
wire [5:0] writeChunkWords = (writeWordsRemaining > MAX_BURST_WORDS) ? MAX_BURST_WORDS[5:0] : writeWordsRemaining[5:0];

// ─── Registered bus inputs ───────────────────────────────────────────────────
reg        endTxReg, dataValidReg;
reg [31:0] addrDataReg;

always @(posedge clock) begin
    endTxReg     <= endTransactionIn;
    dataValidReg <= dataValidIn;
    addrDataReg  <= addressDataIn;
end

// ─── Bus output signals ───────────────────────────────────────────────────────
assign requestTransaction = ((state == S_LOAD_REQ) | (state == S_WRITE_REQ));

reg [31:0] addrDataOutReg;
reg        dataValidOutReg;
assign addressDataOut = addrDataOutReg;
assign dataValidOut   = dataValidOutReg;

// ─── Pixel access: combinational mux ──────────────────────────────────────────
reg [7:0] p1, p2, p3, p4, p6, p7, p8, p9;

always @(*) begin
    p1 = 8'd0; p2 = 8'd0; p3 = 8'd0; p4 = 8'd0;            
    p6 = 8'd0; p7 = 8'd0; p8 = 8'd0; p9 = 8'd0;

    case (topIdx)
        2'd0: begin p1 = lineBuf0[compCol - 1]; p2 = lineBuf0[compCol]; p3 = lineBuf0[compCol + 1]; end
        2'd1: begin p1 = lineBuf1[compCol - 1]; p2 = lineBuf1[compCol]; p3 = lineBuf1[compCol + 1]; end
        default: begin p1 = lineBuf2[compCol - 1]; p2 = lineBuf2[compCol]; p3 = lineBuf2[compCol + 1]; end
    endcase
    case (midIdx)
        2'd0: begin p4 = lineBuf0[compCol - 1]; p6 = lineBuf0[compCol + 1]; end
        2'd1: begin p4 = lineBuf1[compCol - 1]; p6 = lineBuf1[compCol + 1]; end
        default: begin p4 = lineBuf2[compCol - 1]; p6 = lineBuf2[compCol + 1]; end
    endcase
    case (botIdx)
        2'd0: begin p7 = lineBuf0[compCol - 1]; p8 = lineBuf0[compCol]; p9 = lineBuf0[compCol + 1]; end
        2'd1: begin p7 = lineBuf1[compCol - 1]; p8 = lineBuf1[compCol]; p9 = lineBuf1[compCol + 1]; end
        default: begin p7 = lineBuf2[compCol - 1]; p8 = lineBuf2[compCol]; p9 = lineBuf2[compCol + 1]; end
    endcase
end

// ─── Sobel datapath ──────────────────────────────────────────────────────────
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

// ─── Output word packing ──────────────────────────────────────────────────────
wire [31:0] outWord = {
    outBuf[{writeWordIdx, 2'b11}],
    outBuf[{writeWordIdx, 2'b10}],
    outBuf[{writeWordIdx, 2'b01}],
    outBuf[{writeWordIdx, 2'b00}]
};

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
        loadBurstWords <= 6'd0;
        writeBurstWords <= 6'd0;
        writeCount    <= 9'd0;
        compCol       <= {COL_BITS{1'b0}};
        prefillCount  <= 2'd0;
        loadAddrReg   <= 32'd0;
        writeAddrReg  <= 32'd0;
        p_left        <= 8'd0;
        p_center      <= 8'd0;
        
        beginTransactionOut <= 1'b0;
        readNotWriteOut     <= 1'b0;
        endTransactionOut   <= 1'b0;
        byteEnablesOut      <= 4'b0000;
        burstSizeOut        <= 8'd0;
        addrDataOutReg      <= 32'd0;
        dataValidOutReg     <= 1'b0;
    end else begin
        beginTransactionOut <= 1'b0;
        readNotWriteOut     <= 1'b0;
        endTransactionOut   <= 1'b0;
        byteEnablesOut      <= 4'b0000;
        burstSizeOut        <= 8'd0;
        addrDataOutReg      <= 32'd0;
        dataValidOutReg     <= 1'b0;

        if (isMyCi && (valueA == 32'd1)) sourceAddressReg      <= valueB;
        if (isMyCi && (valueA == 32'd2)) destinationAddressReg <= valueB;
        if (isMyCi && (valueA == 32'd3) && valueB[0] && !busyWire) begin
            doneReg  <= 1'b0;
            errorReg <= 1'b0;
            state    <= S_INIT;
        end

        case (state)
            S_IDLE: ; 

            S_INIT: begin
                loadAddrReg  <= sourceAddressReg; 
                writeAddrReg <= destinationAddressReg + IMG_WIDTH; 
                loadBufIdx   <= 2'd0; 
                prefillCount <= 2'd0; 
                rowProc      <= 1; 
                topIdx <= 2'd0; midIdx <= 2'd1; botIdx <= 2'd2; 
                state  <= S_LOAD_REQ; 
            end

            S_LOAD_REQ: begin
                if (transactionGranted) state <= S_LOAD_SETUP;
            end

            S_LOAD_SETUP: begin
                beginTransactionOut <= 1'b1;
                readNotWriteOut     <= 1'b1;
                byteEnablesOut      <= 4'hF;
                burstSizeOut        <= loadChunkWords - 8'd1;
                loadBurstWords      <= loadChunkWords;
                addrDataOutReg      <= loadAddrReg;
                state               <= S_LOAD_BURST;
            end

            S_LOAD_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= S_IDLE;
                end else begin
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
                        // FIX: Safely increment by exactly 4 bytes!
                        loadAddrReg <= loadAddrReg + 32'd4;
                    end

                    if (endTxReg) begin
                        if ((wordIdx + {7'd0, dataValidReg}) < WORDS_PER_ROW) begin
                            state <= S_LOAD_REQ;
                        end else begin
                            wordIdx <= {WORD_BITS{1'b0}};

                            if (prefillCount < 2'd2) begin
                                prefillCount <= prefillCount + 2'd1;
                                loadBufIdx   <= loadBufIdx  + 2'd1;
                                state        <= S_LOAD_REQ;
                            end else begin
                                if (prefillCount == 2'd2) prefillCount <= 2'd3;
                                compCol <= {COL_BITS{1'b0}};
                                state   <= S_COMPUTE;
                            end
                        end
                    end
                end
            end

            S_COMPUTE: begin
                // Morphological Filter Shift Registers
                if (compCol == 0) begin
                    p_left <= 8'h00;
                end else begin
                    p_left <= p_center;
                end
                p_center <= sobelPixel;

                if (compCol > 0) begin
                    // Hardware Morphological Filter: 
                    // If the center pixel is White (0xFF) but both its left and right 
                    // neighbors are Black (0x00), it is Salt noise! Crush it to Black.
                    if (p_center == 8'hFF && p_left == 8'h00 && sobelPixel == 8'h00) begin
                        outBuf[compCol - 1] <= 8'h00; // Noise removed!
                    end else begin
                        outBuf[compCol - 1] <= (compCol - 1 == 0) ? 8'h00 : p_center;
                    end
                end

                if (compCol == (IMG_WIDTH - 1)) begin
                    outBuf[IMG_WIDTH - 1] <= 8'h00; // border
                    compCol <= {COL_BITS{1'b0}};
                    writeWordIdx <= {WORD_BITS{1'b0}};
                    state   <= S_WRITE_REQ;
                end else begin
                    compCol <= compCol + 1;
                end
            end

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
                end else begin
                    if (!busyIn && !writeCount[8]) begin
                        addrDataOutReg  <= outWord; 
                        dataValidOutReg <= 1'b1;
                        writeWordIdx    <= writeWordIdx + 1;
                        writeCount      <= writeCount - 9'd1;
                        // FIX: Safely increment by exactly 4 bytes!
                        writeAddrReg    <= writeAddrReg + 32'd4; 
                    end else if (busyIn) begin
                        addrDataOutReg  <= addrDataOutReg;
                        dataValidOutReg <= dataValidOutReg;
                    end else begin
                        dataValidOutReg <= 1'b0;
                    end

                    // FIX: Catch early bus aborts in the write state!
                    if (!busyIn && writeCount[8]) begin
                        state <= S_WRITE_END;
                    end else if (endTxReg) begin
                        if (!writeCount[8]) begin
                            state <= S_WRITE_REQ; 
                            dataValidOutReg <= 1'b0;
                        end else begin
                            state <= S_ADVANCE;
                        end
                    end
                end
            end

            S_WRITE_END: begin
                endTransactionOut <= 1'b1;
                
                if (writeWordIdx < WORDS_PER_ROW) begin
                    state <= S_WRITE_REQ;
                end else begin
                    state <= S_ADVANCE;
                end
            end

            S_ADVANCE: begin
                if (rowProc < (IMG_HEIGHT - 2)) begin
                    topIdx     <= midIdx; 
                    midIdx     <= botIdx;
                    botIdx     <= topIdx;  
                    loadBufIdx <= topIdx;  
                    rowProc    <= rowProc + 1;
                    state      <= S_LOAD_REQ;
                end else begin 
                    state <= S_DONE;
                end
            end

            S_DONE: begin
                doneReg <= 1'b1;
                state   <= S_IDLE;
            end

            default: begin
                errorReg <= 1'b1;
                state    <= S_IDLE;
            end

        endcase
    end
end

endmodule
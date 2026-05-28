// CI protocol (valueA):
//   0  read  status {error, busy, done}
//   1  write source grayscale frame address (valueB)
//   2  write previous edge-map address    (valueB)
//   3  write current edge-map address     (valueB)
//   4  write motion RGB565 address        (valueB)
//   5  control: valueB[0]=1 starts (ignored while busy)
//   6  read back grayscale source address
//   7  read back previous edge address
//   8  read back current edge address
//   9  read back motion destination address
//
// Per row: load grayscale row if needed -> compute Sobel row -> load previous
// edge row and compute motion -> write current edge row -> write motion row.

module accSobelMotion #(
    parameter [7:0]   customId   = 8'h09,
    parameter integer IMG_WIDTH  = 640,
    parameter integer IMG_HEIGHT = 480
) (
    input  wire        start,
    input  wire        clock,
    input  wire        reset,
    input  wire [31:0] valueA,
    input  wire [31:0] valueB,
    input  wire [7:0]  ciN,
    output wire        done,
    output wire [31:0] result,

    output wire        requestTransaction,
    input  wire        transactionGranted,

    input  wire        endTransactionIn,
    input  wire        dataValidIn,
    input  wire        busErrorIn,
    input  wire        busyIn,
    input  wire [31:0] addressDataIn,

    output reg         beginTransactionOut,
    output reg         readNotWriteOut,
    output reg         endTransactionOut,
    output wire        dataValidOut,
    output reg  [3:0]  byteEnablesOut,
    output reg  [7:0]  burstSizeOut,
    output wire [31:0] addressDataOut
);

localparam integer WORDS_PER_ROW      = IMG_WIDTH / 4;
localparam integer MOTION_WORDS_ROW   = IMG_WIDTH / 2;
localparam integer MAX_BURST_WORDS    = 32;
localparam [8:0] WORDS_PER_ROW_W    = WORDS_PER_ROW;
localparam [8:0] MOTION_WORDS_ROW_W = MOTION_WORDS_ROW;

localparam integer COL_BITS  = 10;
localparam integer ROW_BITS  = 9;
localparam integer WORD_BITS = 9;

localparam [3:0]
    IDLE          = 4'd0,
    INIT          = 4'd1,
    REQ           = 4'd2,
    SETUP         = 4'd3,
    LOAD_BURST    = 4'd4,
    COMPUTE       = 4'd5,
    WRITE_BURST   = 4'd6,
    WRITE_END     = 4'd7,
    DONE          = 4'd8,
    BUS_ERROR_END = 4'd9;
reg [3:0] state;

localparam [1:0]
    PH_LOAD_GRAY    = 2'd0,
    PH_LOAD_PREV    = 2'd1,
    PH_WRITE_EDGE   = 2'd2,
    PH_WRITE_MOTION = 2'd3;
reg [1:0] phase;

reg [31:0] sourceAddressReg, prevEdgeAddressReg, currEdgeAddressReg, motionAddressReg;
reg        doneReg, errorReg;

wire isMyCi   = start & (ciN == customId);
wire busyWire = (state != IDLE);

assign done = isMyCi;
assign result =
    (isMyCi & (valueA == 32'd0)) ? {29'd0, errorReg, busyWire, doneReg} :
    (isMyCi & (valueA == 32'd6)) ? sourceAddressReg :
    (isMyCi & (valueA == 32'd7)) ? prevEdgeAddressReg :
    (isMyCi & (valueA == 32'd8)) ? currEdgeAddressReg :
    (isMyCi & (valueA == 32'd9)) ? motionAddressReg :
    32'd0;

reg [7:0] lineBuf0 [0:IMG_WIDTH-1];
reg [7:0] lineBuf1 [0:IMG_WIDTH-1];
reg [7:0] lineBuf2 [0:IMG_WIDTH-1];
reg [7:0] edgeBuf  [0:IMG_WIDTH-1];
reg [31:0] motionBuf [0:MOTION_WORDS_ROW-1];

reg [1:0] topIdx, midIdx, botIdx;

reg [ROW_BITS-1:0]  rowProc;
reg [COL_BITS-1:0]  compCol;
reg [1:0]           loadBufIdx;
reg [WORD_BITS-1:0] wordIdx;
reg [WORD_BITS-1:0] writeWordIdx;
reg [8:0]           writeCount;
reg [31:0]          loadAddrReg, prevAddrReg, edgeWriteAddrReg, motionWriteAddrReg;
reg                 prefillDone;

wire [8:0] wordsThisWrite = (phase == PH_WRITE_MOTION) ? MOTION_WORDS_ROW_W : WORDS_PER_ROW_W;
wire [8:0] loadWordsRemaining  = WORDS_PER_ROW_W - wordIdx[8:0];
wire [8:0] writeWordsRemaining = wordsThisWrite - writeWordIdx[8:0];
wire [7:0] loadChunkWords  = (loadWordsRemaining  > MAX_BURST_WORDS) ? MAX_BURST_WORDS[7:0] : loadWordsRemaining[7:0];
wire [7:0] writeChunkWords = (writeWordsRemaining > MAX_BURST_WORDS) ? MAX_BURST_WORDS[7:0] : writeWordsRemaining[7:0];

reg        endTransactionInReg, dataValidReg;
reg [31:0] addrDataReg;
always @(posedge clock) begin
    endTransactionInReg <= endTransactionIn;
    dataValidReg        <= dataValidIn;
    addrDataReg         <= addressDataIn;
end

assign requestTransaction = (state == REQ);

reg [31:0] addrDataOutReg;
reg        dataValidOutReg;
assign addressDataOut = addrDataOutReg;
assign dataValidOut   = dataValidOutReg;

reg [7:0] p1, p2, p3, p4, p6, p7, p8, p9;
reg [7:0] p_left, p_center;

always @(*) begin
    case (topIdx)
        2'd0: begin p1 = (compCol == 0) ? 8'h00 : lineBuf0[compCol - 1]; p2 = lineBuf0[compCol]; p3 = (compCol == IMG_WIDTH - 1) ? 8'h00 : lineBuf0[compCol + 1]; end
        2'd1: begin p1 = (compCol == 0) ? 8'h00 : lineBuf1[compCol - 1]; p2 = lineBuf1[compCol]; p3 = (compCol == IMG_WIDTH - 1) ? 8'h00 : lineBuf1[compCol + 1]; end
        default: begin p1 = (compCol == 0) ? 8'h00 : lineBuf2[compCol - 1]; p2 = lineBuf2[compCol]; p3 = (compCol == IMG_WIDTH - 1) ? 8'h00 : lineBuf2[compCol + 1]; end
    endcase
    case (midIdx)
        2'd0: begin p4 = (compCol == 0) ? 8'h00 : lineBuf0[compCol - 1]; p6 = (compCol == IMG_WIDTH - 1) ? 8'h00 : lineBuf0[compCol + 1]; end
        2'd1: begin p4 = (compCol == 0) ? 8'h00 : lineBuf1[compCol - 1]; p6 = (compCol == IMG_WIDTH - 1) ? 8'h00 : lineBuf1[compCol + 1]; end
        default: begin p4 = (compCol == 0) ? 8'h00 : lineBuf2[compCol - 1]; p6 = (compCol == IMG_WIDTH - 1) ? 8'h00 : lineBuf2[compCol + 1]; end
    endcase
    case (botIdx)
        2'd0: begin p7 = (compCol == 0) ? 8'h00 : lineBuf0[compCol - 1]; p8 = lineBuf0[compCol]; p9 = (compCol == IMG_WIDTH - 1) ? 8'h00 : lineBuf0[compCol + 1]; end
        2'd1: begin p7 = (compCol == 0) ? 8'h00 : lineBuf1[compCol - 1]; p8 = lineBuf1[compCol]; p9 = (compCol == IMG_WIDTH - 1) ? 8'h00 : lineBuf1[compCol + 1]; end
        default: begin p7 = (compCol == 0) ? 8'h00 : lineBuf2[compCol - 1]; p8 = lineBuf2[compCol]; p9 = (compCol == IMG_WIDTH - 1) ? 8'h00 : lineBuf2[compCol + 1]; end
    endcase
end

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

wire [31:0] edgeWord = {
    edgeBuf[{writeWordIdx[7:0], 2'b11}],
    edgeBuf[{writeWordIdx[7:0], 2'b10}],
    edgeBuf[{writeWordIdx[7:0], 2'b01}],
    edgeBuf[{writeWordIdx[7:0], 2'b00}]
};

wire [31:0] currentEdgeWord = {
    edgeBuf[{wordIdx[7:0], 2'b11}],
    edgeBuf[{wordIdx[7:0], 2'b10}],
    edgeBuf[{wordIdx[7:0], 2'b01}],
    edgeBuf[{wordIdx[7:0], 2'b00}]
};

wire [63:0] s_motionResult;
motionCi #(.customId(8'h00)) motionInner (
    .start(1'b1),
    .clock(clock),
    .reset(reset),
    .stall(1'b0),
    .busIdle(1'b0),
    .valueA(currentEdgeWord),
    .valueB(addrDataReg),
    .ciN(8'h00),
    .done(),
    .result(s_motionResult)
);

always @(posedge clock) begin
    if (reset) begin
        state                 <= IDLE;
        phase                 <= PH_LOAD_GRAY;
        sourceAddressReg      <= 32'd0;
        prevEdgeAddressReg    <= 32'd0;
        currEdgeAddressReg    <= 32'd0;
        motionAddressReg      <= 32'd0;
        doneReg               <= 1'b0;
        errorReg              <= 1'b0;
        topIdx <= 2'd0; midIdx <= 2'd1; botIdx <= 2'd2;
        rowProc       <= {ROW_BITS{1'b0}};
        compCol       <= {COL_BITS{1'b0}};
        loadBufIdx    <= 2'd0;
        wordIdx       <= {WORD_BITS{1'b0}};
        writeWordIdx  <= {WORD_BITS{1'b0}};
        writeCount    <= 9'd0;
        loadAddrReg   <= 32'd0;
        prevAddrReg   <= 32'd0;
        edgeWriteAddrReg   <= 32'd0;
        motionWriteAddrReg <= 32'd0;
        prefillDone   <= 1'b0;
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

        if (isMyCi && (valueA == 32'd1)) sourceAddressReg   <= valueB;
        if (isMyCi && (valueA == 32'd2)) prevEdgeAddressReg <= valueB;
        if (isMyCi && (valueA == 32'd3)) currEdgeAddressReg <= valueB;
        if (isMyCi && (valueA == 32'd4)) motionAddressReg   <= valueB;
        if (isMyCi && (valueA == 32'd5) && valueB[0] && !busyWire) begin
            doneReg  <= 1'b0;
            errorReg <= 1'b0;
            state    <= INIT;
        end

        case (state)
            IDLE: ;

            INIT: begin
                loadAddrReg        <= sourceAddressReg;
                prevAddrReg        <= prevEdgeAddressReg + IMG_WIDTH;
                edgeWriteAddrReg   <= currEdgeAddressReg + IMG_WIDTH;
                motionWriteAddrReg <= motionAddressReg + (IMG_WIDTH * 2);
                loadBufIdx         <= 2'd0;
                prefillDone        <= 1'b0;
                rowProc            <= 1;
                topIdx <= 2'd0; midIdx <= 2'd1; botIdx <= 2'd2;
                phase              <= PH_LOAD_GRAY;
                state              <= REQ;
            end

            REQ: begin
                if (transactionGranted) state <= SETUP;
            end

            SETUP: begin
                beginTransactionOut <= 1'b1;
                readNotWriteOut     <= (phase == PH_LOAD_GRAY || phase == PH_LOAD_PREV);
                byteEnablesOut      <= 4'hF;
                if (phase == PH_LOAD_GRAY) begin
                    burstSizeOut   <= loadChunkWords - 8'd1;
                    addrDataOutReg <= loadAddrReg;
                    state          <= LOAD_BURST;
                end else if (phase == PH_LOAD_PREV) begin
                    burstSizeOut   <= loadChunkWords - 8'd1;
                    addrDataOutReg <= prevAddrReg;
                    state          <= LOAD_BURST;
                end else begin
                    burstSizeOut   <= writeChunkWords - 8'd1;
                    addrDataOutReg <= (phase == PH_WRITE_EDGE) ? edgeWriteAddrReg : motionWriteAddrReg;
                    writeCount     <= {1'b0, writeChunkWords} - 9'd1;
                    state          <= WRITE_BURST;
                end
            end

            LOAD_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= BUS_ERROR_END;
                end else begin
                    if (dataValidReg) begin
                        if (phase == PH_LOAD_PREV) begin
                            motionBuf[{wordIdx[7:0], 1'b0}] <= s_motionResult[31:0];
                            motionBuf[{wordIdx[7:0], 1'b1}] <= s_motionResult[63:32];
                            prevAddrReg <= prevAddrReg + 32'd4;
                        end else begin
                            case (loadBufIdx)
                                2'd0: begin
                                    lineBuf0[{wordIdx[7:0], 2'b00}] <= addrDataReg[7:0];
                                    lineBuf0[{wordIdx[7:0], 2'b01}] <= addrDataReg[15:8];
                                    lineBuf0[{wordIdx[7:0], 2'b10}] <= addrDataReg[23:16];
                                    lineBuf0[{wordIdx[7:0], 2'b11}] <= addrDataReg[31:24];
                                end
                                2'd1: begin
                                    lineBuf1[{wordIdx[7:0], 2'b00}] <= addrDataReg[7:0];
                                    lineBuf1[{wordIdx[7:0], 2'b01}] <= addrDataReg[15:8];
                                    lineBuf1[{wordIdx[7:0], 2'b10}] <= addrDataReg[23:16];
                                    lineBuf1[{wordIdx[7:0], 2'b11}] <= addrDataReg[31:24];
                                end
                                default: begin
                                    lineBuf2[{wordIdx[7:0], 2'b00}] <= addrDataReg[7:0];
                                    lineBuf2[{wordIdx[7:0], 2'b01}] <= addrDataReg[15:8];
                                    lineBuf2[{wordIdx[7:0], 2'b10}] <= addrDataReg[23:16];
                                    lineBuf2[{wordIdx[7:0], 2'b11}] <= addrDataReg[31:24];
                                end
                            endcase
                            loadAddrReg <= loadAddrReg + 32'd4;
                        end
                        wordIdx <= wordIdx + 1;
                    end

                    if (endTransactionInReg) begin
                        if ((wordIdx + {8'd0, dataValidReg}) < WORDS_PER_ROW) begin
                            state <= REQ;
                        end else begin
                            wordIdx <= {WORD_BITS{1'b0}};
                            if (phase == PH_LOAD_PREV) begin
                                phase        <= PH_WRITE_EDGE;
                                writeWordIdx <= {WORD_BITS{1'b0}};
                                state        <= REQ;
                            end else if (!prefillDone && loadBufIdx < 2'd2) begin
                                loadBufIdx <= loadBufIdx + 2'd1;
                                state      <= REQ;
                            end else begin
                                prefillDone <= 1'b1;
                                compCol     <= {COL_BITS{1'b0}};
                                state       <= COMPUTE;
                            end
                        end
                    end
                end
            end

            COMPUTE: begin
                if (compCol == 0) begin
                    p_left <= 8'h00;
                end else begin
                    p_left <= p_center;
                end
                p_center <= sobelPixel;

                if (compCol > 0) begin
                    if (p_center == 8'hFF && p_left == 8'h00 && sobelPixel == 8'h00) begin
                        edgeBuf[compCol - 1] <= 8'h00;
                    end else begin
                        edgeBuf[compCol - 1] <= (compCol - 1 == 0) ? 8'h00 : p_center;
                    end
                end

                if (compCol == (IMG_WIDTH - 1)) begin
                    edgeBuf[IMG_WIDTH - 1] <= 8'h00;
                    compCol <= {COL_BITS{1'b0}};
                    phase   <= PH_LOAD_PREV;
                    state   <= REQ;
                end else begin
                    compCol <= compCol + 1;
                end
            end

            WRITE_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= BUS_ERROR_END;
                end else if (!writeCount[8] && !busyIn) begin
                    addrDataOutReg  <= (phase == PH_WRITE_EDGE) ? edgeWord : motionBuf[writeWordIdx];
                    dataValidOutReg <= 1'b1;
                    writeWordIdx    <= writeWordIdx + 1;
                    writeCount      <= writeCount - 9'd1;
                    if (phase == PH_WRITE_EDGE) begin
                        edgeWriteAddrReg <= edgeWriteAddrReg + 32'd4;
                    end else begin
                        motionWriteAddrReg <= motionWriteAddrReg + 32'd4;
                    end
                end else if (busyIn) begin
                    addrDataOutReg  <= addrDataOutReg;
                    dataValidOutReg <= dataValidOutReg;
                end else begin
                    state <= WRITE_END;
                end
            end

            WRITE_END: begin
                endTransactionOut <= 1'b1;
                if (writeWordIdx < wordsThisWrite) begin
                    state <= REQ;
                end else if (phase == PH_WRITE_EDGE) begin
                    phase        <= PH_WRITE_MOTION;
                    writeWordIdx <= {WORD_BITS{1'b0}};
                    state        <= REQ;
                end else if (rowProc < (IMG_HEIGHT - 2)) begin
                    topIdx     <= midIdx;
                    midIdx     <= botIdx;
                    botIdx     <= topIdx;
                    loadBufIdx <= topIdx;
                    rowProc    <= rowProc + 1;
                    phase      <= PH_LOAD_GRAY;
                    state      <= REQ;
                end else begin
                    state <= DONE;
                end
            end

            DONE: begin
                doneReg <= 1'b1;
                state   <= IDLE;
            end

            BUS_ERROR_END: begin
                endTransactionOut <= 1'b1;
                state             <= IDLE;
            end

            default: begin
                errorReg <= 1'b1;
                state    <= IDLE;
            end
        endcase
    end
end
endmodule

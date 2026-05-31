// CI protocol (valueA):
//   0  read  status {error, busy, done}
//   1  write source frame address     (valueB)
//   2  write dest edge-map address    (valueB)
//   3  control: valueB[0]=1 starts (ignored while busy)
//   4  read back source address
//   5  read back dest address
//
// Per row: prefill 3 line buffers -> compute Sobel row -> write result.

module accSobel #(
    parameter [7:0]   customId   = 8'h0E,
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

    // Bus-master request/grant
    output wire        requestTransaction,
    input  wire        transactionGranted,

    // Bus inputs
    input  wire        endTransactionIn,
    input  wire        dataValidIn,
    input  wire        busErrorIn,
    input  wire        busyIn,
    input  wire [31:0] addressDataIn,

    // Bus outputs
    output reg         beginTransactionOut,
    output reg         readNotWriteOut,
    output reg         endTransactionOut,
    output wire        dataValidOut,
    output reg  [3:0]  byteEnablesOut,
    output reg  [7:0]  burstSizeOut,
    output wire [31:0] addressDataOut
);

localparam integer WORDS_PER_ROW   = IMG_WIDTH / 4; // each 32-bit word holds 4 pixels
localparam integer MAX_BURST_WORDS = 16;

// bit widths for index registers
localparam integer COL_BITS  = 10; // up to 1024 columns
localparam integer ROW_BITS  = 9;  // up to 512 rows
localparam integer WORD_BITS = 8;  // up to 256 words per row

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

reg [31:0] sourceAddressReg, destinationAddressReg;
reg        doneReg, errorReg;

wire isMyCi   = start & (ciN == customId);
wire busyWire = (state != IDLE);

assign done   = isMyCi;
// we give a different result depending on the CI command, otherwise we keep 0
assign result =
    (isMyCi & (valueA == 32'd0)) ? {29'd0, errorReg, busyWire, doneReg} :
    (isMyCi & (valueA == 32'd4)) ? sourceAddressReg :
    (isMyCi & (valueA == 32'd5)) ? destinationAddressReg :
    32'd0;

// three line buffers each holding one row of IMG_WIDTH bytes
reg [7:0] lineBuf0 [0:IMG_WIDTH-1];
reg [7:0] lineBuf1 [0:IMG_WIDTH-1];
reg [7:0] lineBuf2 [0:IMG_WIDTH-1];
reg [7:0] outBuf   [0:IMG_WIDTH-1];

// rotating index pointers, we swap indices instead of physically moving data between buffers
reg [1:0] topIdx, midIdx, botIdx;

reg [ROW_BITS-1:0]  rowProc;      // current row being processed
reg [COL_BITS-1:0]  compCol;      // current column in the COMPUTE state
reg [1:0]           loadBufIdx;   // which line buffer is currently being loaded
reg [WORD_BITS-1:0] wordIdx;      // 32-bit word position during load bursts
reg [WORD_BITS-1:0] writeWordIdx; // 32-bit word position during write bursts
reg [8:0]           writeCount;   // words remaining in the current write burst
reg [31:0]          loadAddrReg, writeAddrReg;
reg                 doingWrite;  // 0 = load phase, 1 = write phase (selects direction in REQ/SETUP)
reg                 prefillDone; // set once all 3 line buffers have been filled for the first time

wire [8:0] loadWordsRemaining  = WORDS_PER_ROW - wordIdx;
wire [8:0] writeWordsRemaining = WORDS_PER_ROW - writeWordIdx;
wire [7:0] loadChunkWords  = (loadWordsRemaining  > MAX_BURST_WORDS) ? MAX_BURST_WORDS[7:0] : loadWordsRemaining[7:0];
wire [7:0] writeChunkWords = (writeWordsRemaining > MAX_BURST_WORDS) ? MAX_BURST_WORDS[7:0] : writeWordsRemaining[7:0];

// registered bus inputs for timing
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

// pixel registers for the 3×3 Sobel neighbourhood (p5 is always 0 so not needed)
reg [7:0] p1, p2, p3, p4, p6, p7, p8, p9;

// noise filter reg
reg [7:0] p_left, p_center;

always @(*) begin
    // select the correct pixels from the rotating line buffers based on current index pointers
    // border neighbours are forced to black so the line-buffer addresses stay in range
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

wire [31:0] s_sobelValueA = {p4, p3, p2, p1}; // pack the 4 pixels into a 32-bit word for sobelCi
wire [31:0] s_sobelValueB = {p9, p8, p7, p6};
wire [31:0] s_sobelResult;

// inner sobelCi always running combinationally, driven by the current compCol pixels
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

wire [7:0] sobelPixel = s_sobelResult[7:0]; // only the last byte carries the pixel value

// pack 4 output bytes into one 32-bit word to send in a write burst
// {writeWordIdx, 2'bXX} is equivalent to writeWordIdx*4 + byte_offset
wire [31:0] outWord = {
    outBuf[{writeWordIdx, 2'b11}],
    outBuf[{writeWordIdx, 2'b10}],
    outBuf[{writeWordIdx, 2'b01}],
    outBuf[{writeWordIdx, 2'b00}]
};

always @(posedge clock) begin
    if (reset) begin
        state                 <= IDLE;
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
        loadAddrReg   <= 32'd0;
        writeAddrReg  <= 32'd0;
        doingWrite    <= 1'b0;
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
        // default these to 0 every cycle; states that need them set them to 1 explicitly
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
            state    <= INIT;
        end

        case (state)
            IDLE: ;

            INIT: begin
                loadAddrReg  <= sourceAddressReg;
                writeAddrReg <= destinationAddressReg + IMG_WIDTH; // skip first border row of output
                loadBufIdx   <= 2'd0;
                prefillDone  <= 1'b0;
                rowProc      <= 1; // row 0 is a border so we start processing from row 1
                topIdx <= 2'd0; midIdx <= 2'd1; botIdx <= 2'd2;
                doingWrite   <= 1'b0;
                state        <= REQ;
            end

            // bus request state, we just wait for transaction being granted 
            REQ: begin
                if (transactionGranted) state <= SETUP;
            end

            // read or write direction is selected by doingWrite
            SETUP: begin
                beginTransactionOut <= 1'b1;
                readNotWriteOut     <= !doingWrite;
                byteEnablesOut      <= 4'hF;
                if (doingWrite) begin
                    burstSizeOut   <= writeChunkWords - 8'd1;
                    addrDataOutReg <= {writeAddrReg[31:2], 2'b00};
                    writeCount     <= {1'b0, writeChunkWords} - 9'd1;
                    state          <= WRITE_BURST;
                end else begin
                    burstSizeOut   <= loadChunkWords - 8'd1;
                    addrDataOutReg <= {loadAddrReg[31:2], 2'b00};
                    state          <= LOAD_BURST;
                end
            end

            LOAD_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= BUS_ERROR_END;
                end else begin
                    if (dataValidReg) begin
                        case (loadBufIdx) // load the 4 bytes of the word into the correct line buffer
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
                        wordIdx     <= wordIdx + 1;
                        loadAddrReg <= loadAddrReg + 32'd4;
                    end

                    if (endTransactionInReg) begin
                        if ((wordIdx + {7'd0, dataValidReg}) < WORDS_PER_ROW) begin
                            state <= REQ; // more bursts needed for this row
                        end else begin
                            wordIdx <= {WORD_BITS{1'b0}};
                            if (!prefillDone && loadBufIdx < 2'd2) begin
                                loadBufIdx <= loadBufIdx + 2'd1; // move on to the next prefill buffer
                                state      <= REQ;
                            end else begin
                                prefillDone <= 1'b1; // all 3 line buffers are now ready
                                compCol     <= {COL_BITS{1'b0}};
                                state       <= COMPUTE;
                            end
                        end
                    end
                end
            end

            COMPUTE: begin
                // one column per cycle: shift the pipeline and write the noise-filtered result one cycle late
                if (compCol == 0) begin
                    p_left <= 8'h00; // no left neighbour at column 0
                end else begin
                    p_left <= p_center;
                end
                p_center <= sobelPixel;

                if (compCol > 0) begin
                    // if center is white but both neighbours are black it is isolated noise, remove it
                    if (p_center == 8'hFF && p_left == 8'h00 && sobelPixel == 8'h00) begin
                        outBuf[compCol - 1] <= 8'h00;
                    end else begin
                        outBuf[compCol - 1] <= (compCol - 1 == 0) ? 8'h00 : p_center; // left border forced black
                    end
                end

                if (compCol == (IMG_WIDTH - 1)) begin
                    outBuf[IMG_WIDTH - 1] <= 8'h00; // right border forced black
                    compCol      <= {COL_BITS{1'b0}};
                    writeWordIdx <= {WORD_BITS{1'b0}};
                    doingWrite   <= 1'b1; // switch to write phase
                    state        <= REQ;
                end else begin
                    compCol <= compCol + 1;
                end
            end

            WRITE_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= BUS_ERROR_END;
                end else if (!writeCount[8] && !busyIn) begin
                    addrDataOutReg  <= outWord;
                    dataValidOutReg <= 1'b1;
                    writeWordIdx    <= writeWordIdx + 1;
                    writeCount      <= writeCount - 9'd1; // writeCount[8] goes high on underflow to signal burst done
                    writeAddrReg    <= writeAddrReg + 32'd4;
                end else if (busyIn) begin
                    addrDataOutReg  <= addrDataOutReg; // hold while slave is busy
                    dataValidOutReg <= dataValidOutReg;
                end else begin
                    state <= WRITE_END;
                end
            end

            WRITE_END: begin
                endTransactionOut <= 1'b1;
                if (writeWordIdx < WORDS_PER_ROW) begin
                    state <= REQ; // more write bursts for this row
                end else if (rowProc < (IMG_HEIGHT - 2)) begin
                    // rotate index pointers so the oldest buffer is reused for the next row
                    topIdx     <= midIdx;
                    midIdx     <= botIdx;
                    botIdx     <= topIdx;
                    loadBufIdx <= topIdx; 
                    rowProc    <= rowProc + 1;
                    doingWrite <= 1'b0; // switch back to load phase
                    state      <= REQ;
                end else begin
                    state <= DONE; // row 479 is a border, we stop after processing row 478
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

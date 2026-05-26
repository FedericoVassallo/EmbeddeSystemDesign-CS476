// CI protocol (valueA):
//   0  read  status {error, busy, done}
//   1  write source-A frame address (valueB)
//   2  write source-B frame address (valueB)
//   3  write destination address    (valueB)
//   4  control: valueB[0]=1 starts (ignored while busy)
//   5  read back srcA address
//   6  read back srcB address
//   7  read back dst  address

module accMotion #(
    parameter [7:0]   customId   = 8'h0F,
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
    output wire        requestTransaction,
    input  wire        transactionGranted,
    input  wire        endTransactionIn,
                       dataValidIn,
                       busErrorIn,
                       busyIn,
    input  wire [31:0] addressDataIn,
    output reg         beginTransactionOut,
                       readNotWriteOut,
                       endTransactionOut,
    output wire        dataValidOut,
    output reg  [3:0]  byteEnablesOut,
    output reg  [7:0]  burstSizeOut,
    output wire [31:0] addressDataOut
);

localparam integer WORDS_PER_ROW  = IMG_WIDTH / 4;
localparam integer MAX_BURST_WORDS = 32;
localparam integer ROW_BITS  = 9;
localparam integer WORD_BITS = 8;

localparam [3:0]
    IDLE          = 4'd0,
    INIT          = 4'd1,
    REQ           = 4'd2,
    SETUP         = 4'd3,
    LOAD_BURST    = 4'd4,
    WRITE_BURST   = 4'd5,
    WRITE_END     = 4'd6,
    DONE          = 4'd7,
    BUS_ERROR_END = 4'd8;
reg [3:0] state;

reg [31:0] srcAAddressReg, srcBAddressReg, dstAddressReg;
reg        doneReg, errorReg;
wire isMyCi   = start & (ciN == customId);
wire busyWire = (state != IDLE);
assign done   = isMyCi;

// we give as an output a different result as usual depending on the Ci command, otherwise we keep 0
assign result =
    (isMyCi & (valueA == 32'd0)) ? {29'd0, errorReg, busyWire, doneReg} :
    (isMyCi & (valueA == 32'd5)) ? srcAAddressReg :
    (isMyCi & (valueA == 32'd6)) ? srcBAddressReg :
    (isMyCi & (valueA == 32'd7)) ? dstAddressReg  :
    32'd0;

// linebuffer to hold one row of the image divided in 32 bit words
reg [31:0] lineBuf [0:WORDS_PER_ROW-1];

reg doingWrite;  // in the load phase when we load the srcA or srcB we set it at 0, if we are writing to the destination we set it at 1
reg loadingSrcB; // when we are loading srcA is 0, when we are loading srcB is 1

reg [ROW_BITS-1:0]  rowProc; // we keep track of which row we are now processing
reg [WORD_BITS-1:0] wordIdx; // to keep track of how many words we have loaded/written
reg [8:0]           writeCount; // to keep track of how many words we have written in the current burst
reg [31:0]          loadAAddrReg, loadBAddrReg, writeAddrReg;

wire [8:0] wordsRemaining = WORDS_PER_ROW - wordIdx; // the words remaining to load/write in the current row
wire [7:0] usedBurstSize     = (wordsRemaining > MAX_BURST_WORDS) ? MAX_BURST_WORDS[7:0] : wordsRemaining[7:0];

reg        endTransactionInReg, dataValidReg;
reg [31:0] addrDataReg;
// here we store in reg the input from the bus
always @(posedge clock) begin
    endTransactionInReg <= endTransactionIn;
    dataValidReg <= dataValidIn;
    addrDataReg <= addressDataIn;
end

assign requestTransaction = (state == REQ); // we request the bus when we are in the REQ state
reg [31:0] addrDataOutReg;
reg        dataValidOutReg;
assign addressDataOut = addrDataOutReg;
assign dataValidOut   = dataValidOutReg;

always @(posedge clock) begin
    if (reset) begin
        state               <= IDLE;
        srcAAddressReg      <= 32'd0;
        srcBAddressReg      <= 32'd0;
        dstAddressReg       <= 32'd0;
        doneReg             <= 1'b0;
        errorReg            <= 1'b0;
        rowProc             <= {ROW_BITS{1'b0}};
        wordIdx             <= {WORD_BITS{1'b0}};
        writeCount          <= 9'd0;
        loadAAddrReg        <= 32'd0;
        loadBAddrReg        <= 32'd0;
        writeAddrReg        <= 32'd0;
        doingWrite          <= 1'b0;
        loadingSrcB         <= 1'b0;
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

        // we update the registers if we get the corrisponding correct command
        if (isMyCi && (valueA == 32'd1)) srcAAddressReg <= valueB;
        if (isMyCi && (valueA == 32'd2)) srcBAddressReg <= valueB;
        if (isMyCi && (valueA == 32'd3)) dstAddressReg  <= valueB;
        if (isMyCi && (valueA == 32'd4) && valueB[0] && !busyWire) begin
            doneReg  <= 1'b0;
            errorReg <= 1'b0;
            state    <= INIT;
        end

        case (state)
            IDLE: ;

            INIT: begin
                // in init we set up all the registers for the first row
                loadAAddrReg <= srcAAddressReg;
                loadBAddrReg <= srcBAddressReg;
                writeAddrReg <= dstAddressReg;
                rowProc      <= {ROW_BITS{1'b0}};
                wordIdx      <= {WORD_BITS{1'b0}};
                doingWrite   <= 1'b0;
                loadingSrcB  <= 1'b0; // we start with loading srcA
                state        <= REQ;
            end

            // in req we just wait to get the bus grant
            REQ: begin
                if (transactionGranted) state <= SETUP;
            end

            SETUP: begin
                beginTransactionOut <= 1'b1;
                readNotWriteOut     <= !doingWrite; // we set in the bus if we want to read or write
                byteEnablesOut      <= 4'hF; // all byte enabled
                burstSizeOut        <= usedBurstSize - 8'd1;
                if (doingWrite) begin
                    addrDataOutReg <= writeAddrReg; 
                    writeCount     <= {1'b0, usedBurstSize} - 9'd1;
                    state          <= WRITE_BURST;
                end else begin
                    addrDataOutReg <= loadingSrcB ? loadBAddrReg : loadAAddrReg; // we put the correct addr depending on if we are loading srcA or srcB
                    state          <= LOAD_BURST;
                end
            end

            LOAD_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= BUS_ERROR_END;
                end else begin
                    if (dataValidReg) begin
                        if (loadingSrcB) begin
                            // if we are loading srcB, we XOR it with what we have in the line buffer (which is srcA) and we save the result in the line buffer
                            lineBuf[wordIdx] <= lineBuf[wordIdx] ^ addrDataReg; 
                            loadBAddrReg     <= loadBAddrReg + 32'd4;
                        end else begin
                            // if we are loading srcA we just save it in the buffer
                            lineBuf[wordIdx] <= addrDataReg;
                            loadAAddrReg     <= loadAAddrReg + 32'd4;
                        end
                        wordIdx <= wordIdx + 1;
                    end
                    if (endTransactionInReg) begin
                        if ((wordIdx + {7'd0, dataValidReg}) < WORDS_PER_ROW) begin
                            state <= REQ; // more bursts needed for this row
                        end else begin
                            wordIdx <= {WORD_BITS{1'b0}};
                            if (!loadingSrcB) begin
                                loadingSrcB <= 1'b1; //if we were doing srcA, now we do srcB
                                state       <= REQ;
                            end else begin
                                loadingSrcB <= 1'b0;
                                doingWrite  <= 1'b1; // if we were doing srcB, now we do the write part
                                state       <= REQ;
                            end
                        end
                    end
                end
            end

            WRITE_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= BUS_ERROR_END;
                end else if (!writeCount[8] && !busyIn) begin
                    // usual trick for the counter when it gets to 0 and then does -1 the 9th bit becomes 1 (get negative)
                    addrDataOutReg  <= lineBuf[wordIdx]; // we just write the motion data from lineBuf
                    dataValidOutReg <= 1'b1;
                    wordIdx         <= wordIdx + 1;
                    writeCount      <= writeCount - 9'd1;
                    writeAddrReg    <= writeAddrReg + 32'd4;
                end else if (busyIn) begin
                    addrDataOutReg  <= addrDataOutReg;
                    dataValidOutReg <= dataValidOutReg;
                end else begin
                    state <= WRITE_END;
                end
            end

            WRITE_END: begin // if we have more burst to do we go to req, if we ha finished to done
                endTransactionOut <= 1'b1;
                if (wordIdx < WORDS_PER_ROW) begin
                    state <= REQ; // more write bursts for this row
                end else if (rowProc < (IMG_HEIGHT - 1)) begin
                    rowProc     <= rowProc + 1;
                    wordIdx     <= {WORD_BITS{1'b0}};
                    doingWrite  <= 1'b0;
                    loadingSrcB <= 1'b0;
                    state       <= REQ; // load next row's A
                end else begin
                    state <= DONE;
                end
            end

            DONE: begin // we just signal that we are done and go back to idle
                doneReg <= 1'b1;
                state   <= IDLE;
            end

            BUS_ERROR_END: begin // in case of error the master end the transaction
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
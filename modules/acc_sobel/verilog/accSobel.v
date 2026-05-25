// Streaming Sobel edge-detection accelerator — bus master, .
// We set the CI id for this to 0x0E
// CI protocol (valueA):
//   0  read status {error, busy, done}
//   1  write source frame address (valueB)
//   2  write dest edge-map address (valueB)
//   3  control: valueB[0]=1 starts the accelerator (ignored while busy)
//   4  read back source address
//   5  read back dest address

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


localparam integer WORDS_PER_ROW = IMG_WIDTH / 4;   // since each word is 4 pixels (32 bits)
localparam integer MAX_BURST_WORDS = 16; 

// Bit widths
localparam integer COL_BITS  = 10; // since the image width is 640, we need 10 bits to index columns (with 10 bits we can go up to 1024)
localparam integer ROW_BITS  = 9;   // since the image height is 480, we need 9 bits to index rows
localparam integer WORD_BITS = 8;   // since each word is 4 pixels, we need 8 bits to index within a word

// FSM states
localparam [3:0]
    IDLE        = 4'd0,
    INIT        = 4'd1,
    LOAD_REQ    = 4'd2,   
    LOAD_SETUP  = 4'd3,   
    LOAD_BURST  = 4'd4,   
    COMPUTE     = 4'd5,   
    WRITE_REQ   = 4'd6,   
    WRITE_SETUP = 4'd7,   
    WRITE_BURST = 4'd8,   
    WRITE_END   = 4'd9,   
    ADVANCE     = 4'd10,  
    DONE        = 4'd11;

reg [3:0] state;

reg [31:0] sourceAddressReg, destinationAddressReg; // reg that will hold the source and destination addresses set by the CI writes
reg        doneReg, errorReg;

wire isMyCi   = start & (ciN == customId); // usual wire to check if the instruction is meant for this CI
wire busyWire = (state != IDLE); // everytime we are not in the IDLE it means we are busy

assign done   = isMyCi; // we set the done output as soon as we receive the instruction 
// depending on the valueA we give a different result in output
assign result = (isMyCi & (valueA == 32'd0)) ? {29'd0, errorReg, busyWire, doneReg} :
                (isMyCi & (valueA == 32'd4)) ? sourceAddressReg :
                (isMyCi & (valueA == 32'd5)) ? destinationAddressReg :
                32'd0;



// each of these memory array holds one row of the image (it stores IMG_WIDTH bytes)
reg [7:0] lineBuf0 [0:IMG_WIDTH-1];
reg [7:0] lineBuf1 [0:IMG_WIDTH-1];
reg [7:0] lineBuf2 [0:IMG_WIDTH-1];
reg [7:0] outBuf   [0:IMG_WIDTH-1];

// index to indicate the order of the line buffer (added so that we avoid to physically change the line buffers and just change the mapping)
reg [1:0] topIdx, midIdx, botIdx;

reg [ROW_BITS-1:0]  rowProc; // is the current row being processed
reg [COL_BITS-1:0]  compCol; // is the current column being processed     
reg [1:0]           loadBufIdx; // which of the line buffer of above is now being loaded with the new row  
reg [WORD_BITS-1:0] wordIdx; // which 32-bit word is being loaded from memory into a line buffer
reg [WORD_BITS-1:0] writeWordIdx; // which 32-bit word is being written from the output buffer to memory 
reg [5:0]           loadBurstWords; // how many words are being loaded in the current burst (since the burst size can be less than 16 words for the last burst of a row)
reg [5:0]           writeBurstWords; // how many words are being written in the current burst (since the burst size can be less than 16 words for the last burst of a row) so same as above but for output data
reg [8:0]           writeCount; // words left to send in the current write burst
reg [1:0]           prefillCount; // how many input line buffers have already been filled since before starting the computation we need to fille the 3 line buffers 
reg [31:0]          loadAddrReg; // current memory address being read from
reg [31:0]          writeAddrReg; // current memory address being written to

// Filter delay registers
reg [7:0] p_left;
reg [7:0] p_center;

wire [8:0] loadWordsRemaining  = WORDS_PER_ROW - wordIdx; // how many words are left to read 
wire [8:0] writeWordsRemaining = WORDS_PER_ROW - writeWordIdx; // how many words are left to write
wire [5:0] loadChunkWords  = (loadWordsRemaining > MAX_BURST_WORDS) ? MAX_BURST_WORDS[5:0] : loadWordsRemaining[5:0]; // the burst size selected for the bus transfer (if the remaining words are more than the max burst size we select the max burst size, otherwise we select the remaining words)
wire [5:0] writeChunkWords = (writeWordsRemaining > MAX_BURST_WORDS) ? MAX_BURST_WORDS[5:0] : writeWordsRemaining[5:0]; // equal as above but for the write burst size

// the bus input that are registered for timing reasons
reg        endTransactionInReg, dataValidReg;
reg [31:0] addrDataReg;

// here we register the bus inputs
always @(posedge clock) begin
    endTransactionInReg     <= endTransactionIn;
    dataValidReg <= dataValidIn;
    addrDataReg  <= addressDataIn;
end

// we request the bus access when we are in the load or in the write state
assign requestTransaction = ((state == LOAD_REQ) | (state == WRITE_REQ));

reg [31:0] addrDataOutReg; // registered version of the bus output address/data
reg        dataValidOutReg; // registered version of the bus output data valid signal
assign addressDataOut = addrDataOutReg;
assign dataValidOut   = dataValidOutReg;

// reg that store the 3x3 pixel needed for the sobel (the p5 is not needed since is alway multiplied by 0)
reg [7:0] p1, p2, p3, p4, p6, p7, p8, p9;

always @(*) begin

    // depending on the index of the line buffer we are using as top, mid and bot we select the correct pixels for the sobel filter
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

wire [31:0] s_sobelValueA = {p4, p3, p2, p1}; // pack the 4 pixel in a 32 word to send to the sobelCi
wire [31:0] s_sobelValueB = {p9, p8, p7, p6};  // pack the other 4 pixel in a 32 word to send to the sobelCi
wire [31:0] s_sobelResult; // word that will hold the result from sobelCi

// here we instantiate the sobelCi that will do the Sobel filter
sobelCi #(.customId(8'h00)) sobelInner ( 
    .start(1'b1), // we always keep it with start 1
    .clock(clock),
    .reset(reset),
    .stall(1'b0),
    .busIdle(1'b0),
    .valueA(s_sobelValueA),
    .valueB(s_sobelValueB),
    .ciN(8'h00), // since this is an inner CI we can just set the ciN to 0
    .done(),
    .result(s_sobelResult)
);

wire [7:0] sobelPixel = s_sobelResult[7:0]; // since the sobelResult has just the last byte with the pixel value we take just that byte

// is a 32 bit word tha packs 4 pixels from the output buffer to be sent to memory
// for all of these we are shifting by 2 so its like writeWordIdx*4, and the adding the 2 final bits to select the correct byte within the word
wire [31:0] outWord = {
    outBuf[{writeWordIdx, 2'b11}], 
    outBuf[{writeWordIdx, 2'b10}],
    outBuf[{writeWordIdx, 2'b01}], // here is like  outBuf[writeWordIdx*4 + 1]
    outBuf[{writeWordIdx, 2'b00}] 
};

always @(posedge clock) begin
    if (reset) begin
        // here at reset we just initialazed everything to 0
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
        // since these signals just need to be on for one cycle when we start the transaction we can just set them to 0 at every cycle and then set them to 1 in the state where we need to start the transaction
        beginTransactionOut <= 1'b0;
        readNotWriteOut     <= 1'b0;
        endTransactionOut   <= 1'b0;
        byteEnablesOut      <= 4'b0000;
        burstSizeOut        <= 8'd0;
        addrDataOutReg      <= 32'd0;
        dataValidOutReg     <= 1'b0;

        if (isMyCi && (valueA == 32'd1)) sourceAddressReg      <= valueB; // if the instr is 1 we set the source address = valueB
        if (isMyCi && (valueA == 32'd2)) destinationAddressReg <= valueB; // id the instr is 2 we set the destination address = valueB
        if (isMyCi && (valueA == 32'd3) && valueB[0] && !busyWire) begin // if we receive the start instr (3) and the valueB[0] is 1 and we are not busy we start the accelerator by going to the init state
            doneReg  <= 1'b0;
            errorReg <= 1'b0;
            state    <= INIT;
        end

        case (state)
            IDLE: ; // in the idle state we just wait and do nothing

            INIT: begin
                loadAddrReg  <= sourceAddressReg; // we set the load address to the source and destination address received from the CI
                writeAddrReg <= destinationAddressReg + IMG_WIDTH;  // we set the write address to the second row of the output image since the first row is border and is always 0 so we can skip it
                loadBufIdx   <= 2'd0; // we start loading from the lineBuf0 
                prefillCount <= 2'd0; // at the beginning we haven't prefilled anything so the prefill count is 0
                rowProc      <= 1; // we start processing from the first row since the 0 row is border and is always 0 so we can skip it
                topIdx <= 2'd0; midIdx <= 2'd1; botIdx <= 2'd2; // initialize the idx of the buffer 
                state  <= LOAD_REQ; 
            end

            LOAD_REQ: begin
                if (transactionGranted) state <= LOAD_SETUP; // we just wait until we get a bus grant
            end

            LOAD_SETUP: begin
                beginTransactionOut <= 1'b1; // we set signal of transaction started
                readNotWriteOut     <= 1'b1; // we set it to one since we are reading from the bus 
                byteEnablesOut      <= 4'hF; // we set all the byte enables to 1 since we want to read a full word
                burstSizeOut        <= loadChunkWords - 8'd1; // we set the burst size to the number of words we want to read in this burst (minus 1 since the burst is +1 the burst size)
                loadBurstWords      <= loadChunkWords; // we save the burst size in a reg to use it later
                addrDataOutReg      <= loadAddrReg; // we set the address to read from to the current load address reg
                state               <= LOAD_BURST;
            end

            LOAD_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= IDLE;
                end else begin
                    if (dataValidReg) begin
                        case (loadBufIdx) // we switch what line buffer we load based on the loadBufIdx, and we load the 4 pixels of the current word based on the wordIdx
                            2'd0: begin
                                // as before for indexing the line buffer we do wordIdx*4 + the byte within the word
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
                        // we increment after loading of the word
                        wordIdx <= wordIdx + 1;
                        loadAddrReg <= loadAddrReg + 32'd4;
                    end

                    if (endTransactionInReg) begin // so it runs when we are at a burst end
                        if ((wordIdx + {7'd0, dataValidReg}) < WORDS_PER_ROW) begin // if we haven't loaded all the words of the row we start another burst to load the next words of the row (the {7'd0, dataValidReg} adds 1 if the very last word of this burst arrived in the same cycle of the end transaction)
                            state <= LOAD_REQ;
                        end else begin // if instead we have loaded the full row
                            wordIdx <= {WORD_BITS{1'b0}}; // reset word index to 0

                            if (prefillCount < 2'd2) begin // since we have filled one of the line if we were still in the prefill phase we increment the prefill count
                                prefillCount <= prefillCount + 2'd1;
                                loadBufIdx   <= loadBufIdx  + 2'd1;
                                state        <= LOAD_REQ;
                            end else begin
                                if (prefillCount == 2'd2) prefillCount <= 2'd3; // if we were in the last prefill we set the prefill count to 3 and go to computation
                                compCol <= {COL_BITS{1'b0}};
                                state   <= COMPUTE;
                            end
                        end
                    end
                end
            end

            COMPUTE: begin
                // when we are at cycle N p_left=sobel[N-2], p_center=sobel[N-1], sobelPixel=sobel[N]
                // we write outBuf[N- 1] in the cycle N so that there are the right and left neighbors
                p_left   <= (compCol == 0) ? 8'h00 : p_center;
                p_center <= sobelPixel;

                // write the previous pixel (compCol-1) into the output buffer
                if (compCol > 0) begin
                    if (compCol == 1) begin
                        outBuf[0] <= 8'h00; // the border pixel is 0                  
                    end else if (p_center == 8'hFF && p_left == 8'h00 && sobelPixel == 8'h00) begin
                        outBuf[compCol - 1] <= 8'h00; // if we see that its left and right neighbors are 0 it probably is noise so we put it to 0
                    end else begin
                        outBuf[compCol - 1] <= p_center; // otherewise we put the pixel value computed from sobel
                    end
                end

                if (compCol == (IMG_WIDTH - 1)) begin
                    outBuf[IMG_WIDTH - 1] <= 8'h00; // we set the border pixel to 0
                    compCol      <= {COL_BITS{1'b0}}; 
                    writeWordIdx <= {WORD_BITS{1'b0}};
                    state        <= WRITE_REQ; 
                end else begin
                    compCol <= compCol + 1;
                end
            end

            WRITE_REQ: begin
                if (transactionGranted) state <= WRITE_SETUP; // again we wait for the bus grant 
            end

            WRITE_SETUP: begin
                beginTransactionOut <= 1'b1; // we set the signal to start the transaction
                readNotWriteOut     <= 1'b0; // we want to write so is set to 0
                byteEnablesOut      <= 4'hF; // we write a full word
                burstSizeOut        <= writeChunkWords - 8'd1;
                writeBurstWords     <= writeChunkWords; //we save the burst size in a reg to use it later 
                addrDataOutReg      <= writeAddrReg; // we set the address to write to to the current write address reg
                writeCount          <= {3'd0, writeChunkWords} - 9'd1; 
                state               <= WRITE_BURST; 
            end

            WRITE_BURST: begin
                if (busErrorIn) begin
                    errorReg <= 1'b1;
                    state    <= IDLE;
                end else begin
                    if (!busyIn && !writeCount[8]) begin // write count is decremented by 1 at every word written, so when we finish the word written it does 0 - 1 so it sets the 9th bit to 1 since it becomes negative so we use the 9th bit to check if we have finished
                        addrDataOutReg  <= outWord; // outword is the word that pack the 4 sobel pixel
                        dataValidOutReg <= 1'b1;
                        writeWordIdx    <= writeWordIdx + 1; // we increment the word index to write the next word in the output buffer
                        writeCount      <= writeCount - 9'd1; // we decrement the write count to keep track of how many words we have left to write in this burst
                        writeAddrReg    <= writeAddrReg + 32'd4; // increase by 4 the write address since we write a word of 4 bytes
                    end else if (busyIn) begin
                        addrDataOutReg  <= addrDataOutReg; // if it is busy we keep the same address and data
                        dataValidOutReg <= dataValidOutReg;
                    end else begin
                        // if we are not busy and we have finished (writeCount[8] = 1)  we go to the end of write
                        dataValidOutReg <= 1'b0;
                        state           <= WRITE_END;
                    end
                end
            end

            WRITE_END: begin
                endTransactionOut <= 1'b1; // we send the signal to end the transaction
                
                if (writeWordIdx < WORDS_PER_ROW) begin // writeWordIdx tracks how many 32-bit words of the output row have been written so far, so if we haven't finished to write the full row we start another burst
                    state <= WRITE_REQ;
                end else begin
                    state <= ADVANCE;
                end
            end

            ADVANCE: begin // we have finished the previous row and we have to see if there are more row to process
                if (rowProc < (IMG_HEIGHT - 2)) begin // stop at 478 because the last row is border so always 0
                    // if we still have rows we rotate the indx pointer for teh row order 
                    topIdx     <= midIdx; 
                    midIdx     <= botIdx;
                    botIdx     <= topIdx;  
                    loadBufIdx <= topIdx;  
                    rowProc    <= rowProc + 1;
                    state      <= LOAD_REQ; // we go to request to load the next row
                end else begin 
                    state <= DONE;
                end
            end

            DONE: begin // we have finished we set the done reg to signal if the cpu was polling that we finished
                doneReg <= 1'b1;
                state   <= IDLE; 
            end

            default: begin
                errorReg <= 1'b1;
                state    <= IDLE;
            end

        endcase
    end
end

endmodule
module ramDmaCi # ( parameter[7:0] customId = 8'h00 )
            (  input  wire         start ,
                input  wire        clock ,
                input  wire        reset ,
                // CI interface
                input  wire [31:0] valueA ,
                input  wire [31:0] valueB ,
                input  wire [7:0]  ciN ,
                output wire        done ,
                output wire [31:0] result,
                // bus request signals
                input  wire        busGrants,
                output reg         busRequests,
                // Bus in data for DMA transfers
                input  wire [31:0] busIn_addressData,
                input  wire        busIn_endTransaction,
                input  wire        busIn_dataValid,
                input  wire        busIn_busy,
                input  wire        busIn_error,
                // Bus out signals for DMA transfers
                output reg  [31:0] busOut_addressData,
                output reg  [3:0]  busOut_byteEnables,
                output reg  [7:0]  busOut_burstSize,
                output reg         busOut_readNWrite,
                output reg         busOut_beginTransaction,
                output reg         busOut_endTransaction,
                output reg         busOut_dataValid,
                output reg         busOut_busy
            );

    localparam [2:0] IDLE = 3'b000;
    localparam [2:0] REQUEST = 3'b001;
    localparam [2:0] REQUEST_DONE= 3'b010; // we add the request done to manage the case in whîch the granted signal was already high in the same cycle of the request, so we need to wait one cycle to be sure that the granted signal is for our request
    localparam [2:0] INFO = 3'b011;
    localparam [2:0] BURST = 3'b100;
    localparam [2:0] END_TRANSACTION = 3'b101;
    localparam [2:0] ERROR = 3'b110;

    reg [2:0] FSM_state;

    wire [31:0] ram_dataout_A;
    wire isMine;
    wire isWriteOp;
    wire isReadOp;
    wire [2:0] opCode;

    reg delayedDoneFlag;

    // DMA configuration registers
    reg [31:0] bus_start_address;
    reg [8:0]  memory_start_address;
    reg [9:0]  block_size;
    reg [7:0]  burst_size;
    // status busy and status error registers, are the bit 0 and bit 1 of the STATUS REGISTER mentioned in the assignment
    reg status_busy;  // 1 when FSM is running
    reg status_error; // it 1 of this register indicates if a bus-error occurred during the transfer
    // foe control_register, writing a 1 to bit 0 of this register will start a DMA-transfer in case the DMA-controller is idle
    reg control_register; // start a DMA-transfer

    // Command decoding & safety checks
    assign opCode = valueA[12:10];

    // Are they talking to us?
    assign isMine = (ciN == customId) ? start : 1'b0;

    // Make sure upper bits are 0 so we don't trigger on garbage addresses
    wire isMemOpEnable = (valueA[31:13] == 19'b0);

    assign isWriteOp = isMine && isMemOpEnable && valueA[9];
    assign isReadOp  = isMine && isMemOpEnable && ~valueA[9];

    // only write to the physical RAM if it's a direct access (opcode 000).
    wire isRamWrite = isWriteOp && (opCode == 3'b000);

    // DMA-side signals for RAM access
    reg        dma_write_enable;
    reg [8:0]  dma_address;       // memory_start_address, auto-incremented
    reg [31:0] dma_data_in;       // data received from bus

    // Since in the assigment tells that to reduce the critical path on the bus,
    // all signals from the bus (hence all signals on the bus-in port)
    // need to be registered by using flipflops
    reg [31:0] busIn_addressData_reg;
    reg        busIn_endTransaction_reg;
    reg        busIn_dataValid_reg;
    reg        busIn_busy_reg;
    reg        busIn_error_reg;
    reg [9:0]  n_words_remaining;
  
    dualPortSSRAM #(
                    .bitwidth(32),
                    .nrOfEntries(512),
                    .readAfterWrite(1)
                    ) ram (
                    .clockA(clock),
                    .clockB(~clock),              // negedge as spec requires
                    .writeEnableA(isRamWrite),    // CPU writes (opCode 000 only)
                    .writeEnableB(dma_write_enable), // DMA writes during bus-to-memory transfer
                    .addressA(valueA[8:0]),       // CPU address
                    .addressB(dma_address),       // DMA address (auto-incremented)
                    .dataInA(valueB),             // CPU write data
                    .dataInB(dma_data_in),        // data from bus
                    .dataOutA(ram_dataout_A),     // CPU read data
                    .dataOutB()       // empty for now, (used for 2.4 maybe)
                    );

    // Register Writes & Timing
    always @(posedge clock)
    begin
        if (reset) begin
        delayedDoneFlag <= 1'b0;
        bus_start_address <= 32'b0;
        memory_start_address <= 9'b0;
        block_size <= 10'b0;
        burst_size <= 8'b0;
        status_busy <= 1'b0;
        status_error <= 1'b0;
        control_register <= 1'b0;
        n_words_remaining <= 10'b0;
        // Clear bus-in registers
        busIn_addressData_reg    <= 32'b0;
        busIn_endTransaction_reg <= 1'b0;
        busIn_dataValid_reg      <= 1'b0;
        busIn_busy_reg           <= 1'b0;
        busIn_error_reg          <= 1'b0;
        // we set the state to IDLE on reset
        FSM_state <= IDLE;
        dma_write_enable <= 1'b0;
        dma_address      <= 9'b0;
        dma_data_in      <= 32'b0;
        busRequests      <= 1'b0;
        busOut_addressData      <= 32'b0;
        busOut_byteEnables      <= 4'b0;
        busOut_burstSize        <= 8'b0;
        busOut_readNWrite       <= 1'b0;
        busOut_beginTransaction <= 1'b0;
        busOut_endTransaction   <= 1'b0;
        busOut_dataValid        <= 1'b0;
        busOut_busy             <= 1'b0;
        end
        else begin

        busIn_addressData_reg    <= busIn_addressData;
        busIn_endTransaction_reg <= busIn_endTransaction;
        busIn_dataValid_reg      <= busIn_dataValid;
        busIn_busy_reg           <= busIn_busy;
        busIn_error_reg          <= busIn_error;

        // 1-cycle delay for read operations (since RAM takes 2 cycles)
        delayedDoneFlag <= isReadOp;

        // Handle CPU writes to our config registers
        if (isWriteOp)
        begin
            case (opCode)
                3'b001:
                    bus_start_address <= valueB;
                3'b010:
                    memory_start_address <= valueB[8:0];
                3'b011:
                    block_size <= valueB[9:0];
                3'b100:
                    burst_size <= valueB[7:0];
                3'b101:
                begin
                    if (valueB[0] && !status_busy) begin
                        control_register <= valueB[0]; // Set control register to indicate transfer is starting
                        if (block_size == 10'd0) begin
                            // Nothing to transfer, don't start FSM
                            status_busy <= 1'b0;
                        end else begin
                            status_busy <= 1'b1;
                            status_error <= 1'b0;  // clear error from previous transfer
                            n_words_remaining <= block_size;
                            FSM_state <= REQUEST; //TODO: domanda al prof su domanda.txt!!!
                        end
                    end
                end
            endcase
        end

        case (FSM_state)
            IDLE:
            begin
                busOut_endTransaction <= 1'b0; // Ensure endTransaction is low in IDLE
                // TODO: check that maybe we can move this elsewhere and do effectively nothing in i
            end
            REQUEST:
            begin
                busRequests <= 1'b1; // Request the bus
                FSM_state <= REQUEST_DONE; // Move to REQUEST_DONE state to wait for grant
            end 
            REQUEST_DONE:
            begin
                if(busGrants)
                begin
                    busRequests <= 1'b0; // Stop requesting once granted
                    FSM_state <= INFO; // Move to next state
                end
            end
            INFO:
            begin
                busOut_addressData <= bus_start_address; // Set the starting address for the burst
                // In INFO state:
                busOut_burstSize <= (n_words_remaining < {2'b0, burst_size} + 10'd1) ? (n_words_remaining[7:0] - 8'd1) : burst_size; // if we are in the last transaction, we set the burst size to the number of bursts needed for the last transaction, otherwise we set it to the configured burst size
                busOut_byteEnables <= 4'b1111; // Enable all bytes (since is a 32-bit transfer)
                busOut_readNWrite <= 1'b1; // Set to read the bus data (1 for read, 0 for write)
                busOut_beginTransaction <= 1'b1; // Signal the start of the transaction
                // we send information about the transaction
                FSM_state <= BURST; // Move to BURST state to start the burst transfer
            end
            BURST:
            begin
                busOut_addressData <= 32'b0; // Clear address/data lines during burst 
                busOut_burstSize <= 8'b0; // Clear burst size during burst
                busOut_byteEnables <= 4'b0000; // Clear byte enables during burst
                busOut_readNWrite <= 1'b0; // Clear read/write signal during burst
                busOut_beginTransaction <= 1'b0; // Clear begin transaction after the first cycle

                // Perform burst read/writes to RAM and bus
                FSM_state <= (busIn_error_reg) ? ERROR :
                            (busIn_endTransaction_reg) ? END_TRANSACTION : BURST; // If an error occurs or the transaction ends, move to the respective state, otherwise stay in BURST
                           
                if (busIn_dataValid_reg && !busIn_error_reg)
                begin
                    dma_data_in <= busIn_addressData_reg; // Capture data from bus
                    dma_write_enable <= 1'b1; // Enable write to RAM
                    dma_address <= memory_start_address; // Set initial DMA address
                    memory_start_address <= memory_start_address + 9'd1; // Auto-increment DMA address
                    bus_start_address <= bus_start_address + 32'd4; // Auto-increment bus address for next beat
                    n_words_remaining    <= n_words_remaining - 10'd1;  // decrement the number of words remaining for the transaction
                end
                else
                begin
                    dma_write_enable <= 1'b0; // We implement the NOP_WAIT directly in the BURST state, by not enabling the write to the RAM if the data is not valid, effectively waiting in this state until the data is valid without needing an additional state
                end
            end
            END_TRANSACTION:
            begin
                dma_write_enable <= 1'b0;
                if (n_words_remaining == 10'd0) begin
                    status_busy <= 1'b0;
                    control_register <= 1'b0; // Clear control register to indicate transfer is done
                    FSM_state <= IDLE;
                end else begin
                    FSM_state <= REQUEST;
                end
            end
            ERROR:
            begin
                busOut_endTransaction <= 1'b1; // Signal end of transaction on error
                status_busy <= 1'b0; // Clear busy status on error
                status_error <= 1'b1; // Set error status
                FSM_state <= IDLE; // Move back to IDLE on error
                // TODO: we could set busy = 0 here to leave the IDLE state empty and just use it as a state in which we are not doing anything
            end
        endcase
    end
end
    
    // assign register to the output based on opcode
    wire [31:0] read_out;

    assign read_out = (opCode == 3'b000) ? ram_dataout_A :
            (opCode == 3'b001) ? bus_start_address :
            (opCode == 3'b010) ? {23'b0, memory_start_address} :
            (opCode == 3'b011) ? {22'b0, block_size} :
            (opCode == 3'b100) ? {24'b0, burst_size} :
            (opCode == 3'b101) ? {30'b0, status_error, status_busy} :
            32'b0;

    // done outputs
    assign done = (isWriteOp && ~reset) | delayedDoneFlag;

    // put data on the result
    assign result = (delayedDoneFlag) ? read_out : 32'b0;

endmodule
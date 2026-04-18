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
  localparam [2:0] READ_BUS = 3'b100;
  localparam [2:0] WRITE_BUS = 3'b101;
  localparam [2:0] END_TRANSACTION = 3'b110;
  localparam [2:0] ERROR = 3'b111;

  reg [2:0] FSM_state;

  wire [31:0] ram_dataout_A;
  wire [31:0] ram_dataout_B;
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
  reg [1:0] control_register; // start a DMA-transfer

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

  // two working copies to avoid modifying the actual registers
  reg [31:0] bus_addr_current;     // working copy, auto-incremented
  reg [8:0]  mem_addr_current;     // working copy, auto-incremented

  reg is_read; // reg to see if we are writing or reading from the bus, used because maybe the control register get changed externally why we are doing our operations (1 if we are reading, 0 if we are writing)
  reg [9:0] words_sent_in_burst; // reg to keep track of how many words we have written on the bus, used to understand when to end the transaction in the write case

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
                  .dataOutB(ram_dataout_B)       // used for the write on the bus case
                );

  // Register Writes & Timing
  always @(posedge clock)
  begin
    if (reset)
    begin
      delayedDoneFlag <= 1'b0;
      bus_start_address <= 32'b0;
      memory_start_address <= 9'b0;
      block_size <= 10'b0;
      burst_size <= 8'b0;
      status_busy <= 1'b0;
      status_error <= 1'b0;
      control_register <= 2'b0;
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
      bus_addr_current <= 32'b0;
      mem_addr_current <= 9'b0;
      words_sent_in_burst <= 10'b0;
      is_read<= 1'b0;
    end
    else
    begin

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
            if ((valueB[1] || valueB[0]) && !status_busy)
            begin
              control_register <= valueB[1:0]; // Set control register to indicate transfer is starting
              if (block_size == 10'd0 || (valueB[1:0] == 2'b11))
              begin  // if it is written 3 in the control register we don't start
                // Nothing to transfer, don't start FSM
                status_busy <= 1'b0;
              end
              else
              begin
                is_read<= (valueB[0]) ? 1'b1 : 1'b0; // we set the read if the control register bit 0 is set to 1, otherwise we are in the write case (since writing 3 to the control register is not possible)
                status_busy <= 1'b1;
                status_error <= 1'b0;  // clear error from previous transfer
                bus_addr_current  <= bus_start_address;   // initialize working copy
                mem_addr_current  <= memory_start_address; // initialize working copy
                n_words_remaining <= block_size;
                words_sent_in_burst <= 10'b0;
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
          busOut_endTransaction <= 1'b0;   // clear from previous burst
        end
        REQUEST_DONE:
        begin
          if(busGrants)
          begin
            busRequests <= 1'b0; // Stop requesting once granted
            busOut_dataValid        <= 1'b0;        // ADD THIS
            busOut_endTransaction   <= 1'b0;        // ADD THIS
            busOut_addressData <= bus_addr_current; // Set the starting address for the READ_BUS
            words_sent_in_burst <= 10'b0; // reset the number of written words on the bus to 0 at the beginning of the transaction
            // In INFO state:
            busOut_burstSize <= (n_words_remaining < {2'b0, burst_size} + 10'd1) ? (n_words_remaining[7:0] - 8'd1) : burst_size; // if we are in the last transaction, we set the READ_BUS size to the number of READ_BUSs needed for the last transaction, otherwise we set it to the configured READ_BUS size
            busOut_byteEnables <= 4'b1111; // Enable all bytes (since is a 32-bit transfer)
            busOut_readNWrite <= (is_read) ? 1'b1 : 1'b0; // Set to read the bus data (1 for read, 0 for write) so if the control register bit 0 is 1 we read from the bus, if it is 0 we write to the bus
            busOut_beginTransaction <= 1'b1; // Signal the start of the transaction
            // we send information about the transaction
            dma_address <= (is_read) ?  dma_address : mem_addr_current; // Set initial DMA address so that when we go in the write bus state we already have the address ready for the first write to the RAM, so we already have the read of the dat
            FSM_state <= (is_read) ? READ_BUS : WRITE_BUS; // Move to READ_BUS or WRITE_BUS state to start the transfer
          end
        end
        READ_BUS:
        begin
          busOut_addressData <= 32'b0; // Clear address/data lines during READ_BUS
          busOut_burstSize <= 8'b0; // Clear READ_BUS size during READ_BUS
          busOut_byteEnables <= 4'b0000; // Clear byte enables during READ_BUS
          busOut_readNWrite <= 1'b0; // Clear read/write signal during READ_BUS
          busOut_beginTransaction <= 1'b0; // Clear begin transaction after the first cycle

          // Perform READ_BUS read/writes to RAM and bus
          FSM_state <= (busIn_error_reg) ? ERROR :
            (busIn_endTransaction_reg) ? END_TRANSACTION : READ_BUS; // If an error occurs or the transaction ends, move to the respective state, otherwise stay in READ_BUS

          if (busIn_dataValid_reg && !busIn_error_reg)
          begin
            dma_data_in <= busIn_addressData_reg; // Capture data from bus
            dma_write_enable <= 1'b1; // Enable write to RAM
            dma_address       <= mem_addr_current;   // Set initial DMA address
            mem_addr_current   <= mem_addr_current + 9'd1; // Auto-increment DMA address
            bus_addr_current <= bus_addr_current + 32'd4; // Auto-increment bus address for next
            n_words_remaining    <= n_words_remaining - 10'd1;  // decrement the number of words remaining for the transaction
          end
          else
          begin
            dma_write_enable <= 1'b0; // We implement the NOP_WAIT directly in the READ_BUS state, by not enabling the write to the RAM if the data is not valid, effectively waiting in this state until the data is valid without needing an additional state
          end
        end
        WRITE_BUS:
        begin
          dma_write_enable <= 1'b0;  // RAM in read mode on port B during bus writes

          // Place the current RAM word on the bus, or hold during slave busy.
          busOut_addressData <= (busIn_busy_reg) ? busOut_addressData : ram_dataout_B;

          // Advance all counters together when not busy.
          dma_address        <= (busIn_busy_reg) ? dma_address        : dma_address + 9'd1;
          mem_addr_current   <= (busIn_busy_reg) ? mem_addr_current   : mem_addr_current + 9'd1;
          bus_addr_current   <= (busIn_busy_reg) ? bus_addr_current   : bus_addr_current + 32'd4;
          n_words_remaining  <= (busIn_busy_reg) ? n_words_remaining  : n_words_remaining - 10'd1;
          words_sent_in_burst <= (busIn_busy_reg) ? words_sent_in_burst : words_sent_in_burst + 10'd1;

          // Clear fields only meaningful in the address phase.
          busOut_burstSize        <= 8'b0;
          busOut_byteEnables      <= 4'b0000;
          busOut_readNWrite       <= 1'b0;
          busOut_beginTransaction <= 1'b0;

          // Transition on the same cycle we place the last word of the burst/block.
          FSM_state <= busIn_error_reg ? ERROR :
            (!busIn_busy_reg && ((words_sent_in_burst == {2'b0, burst_size}) || (n_words_remaining == 10'd1))) ? END_TRANSACTION :
              WRITE_BUS;

          // dataValid held high throughout WRITE_BUS; dropped only on error.
          busOut_dataValid <= (busIn_error_reg) ? 1'b0 : 1'b1;
        end
        END_TRANSACTION:
        begin
          dma_write_enable <= 1'b0;
          busOut_dataValid <= 1'b0;
          busOut_addressData <= 32'b0;
          busOut_endTransaction <= (is_read) ? 1'b0 : 1'b1; // we need to signal the end of the transaction only in the write case
          if (n_words_remaining == 10'd0 )
          begin
            status_busy <= 1'b0;
            control_register <= 2'b0; // Clear control register to indicate transfer is done
            FSM_state <= IDLE;
          end
          else
          begin
            FSM_state <= REQUEST;
          end
        end
        ERROR:
        begin
          busOut_dataValid <= 1'b0; // Clear data valid on error
          busOut_addressData <= 32'b0; // Clear address/data lines on error
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
module accSobel #(
    parameter [7:0] customId = 8'h0E,
    parameter integer IMG_WIDTH = 640,
    parameter integer IMG_HEIGHT = 480
) (
    input wire         start,
                       clock,
                       reset,
    input wire [31:0]  valueA,
                       valueB,
    input wire [7:0]   ciN,
    output wire        done,
    output wire [31:0] result,

    // Bus-master request/grant interface
    output wire        requestTransaction,
    input wire         transactionGranted,

    // bus input side
    input wire         endTransactionIn,
                       dataValidIn,
                       busErrorIn,
                       busyIn,
    input wire [31:0]  addressDataIn,

    // bus output side
    output reg         beginTransactionOut,
                       readNotWriteOut,
                       endTransactionOut,
    output wire        dataValidOut,
    output reg [3:0]   byteEnablesOut,
    output reg [7:0]   burstSizeOut,
    output wire [31:0] addressDataOut
);

/*

CI ID 0x0E

valueA = 0  -> read status
valueA = 1  -> write source grayscale frame address
valueA = 2  -> write destination edge-map address
valueA = 3  -> control register, bit 0 = start
valueA = 4  -> read back source address
valueA = 5  -> read back destination address

*/

// internal reg for the accelerator
reg [31:0] sourceAddressReg;
reg [31:0] destinationAddressReg;
reg        doneReg;
reg        errorReg;
reg [4:0]  stateReg;

localparam [4:0] STATE_IDLE   = 5'd0;
localparam [4:0] STATE_START  = 5'd1;
localparam [4:0] STATE_FINISH = 5'd2;

// For now the bus is not used.
// Later these signals will be driven by the bus-master FSM.
assign requestTransaction = 1'b0;
assign dataValidOut       = 1'b0;
assign addressDataOut     = 32'd0;

wire isMyCi;
wire busyWire;

assign isMyCi   = start && (ciN == customId);
assign busyWire = (stateReg != STATE_IDLE);

// we put the done the cycle we get the instr then we start the operation and signal the end with the done reg
assign done = isMyCi;

// if 0 we read the error busy and doneReg which is different from the done signal
// if 4 we read the source address
// if 5 we read the dest add
assign result =
    (isMyCi && valueA == 32'd0) ? {29'd0, errorReg, busyWire, doneReg} :
    (isMyCi && valueA == 32'd4) ? sourceAddressReg :
    (isMyCi && valueA == 32'd5) ? destinationAddressReg :
    32'd0;

always @(posedge clock) begin

    if (reset) begin
        sourceAddressReg      <= 32'd0;
        destinationAddressReg <= 32'd0;
        doneReg               <= 1'b0;
        errorReg              <= 1'b0;
        stateReg              <= STATE_IDLE;

        beginTransactionOut   <= 1'b0;
        readNotWriteOut       <= 1'b1;
        endTransactionOut     <= 1'b0;
        byteEnablesOut        <= 4'b1111;
        burstSizeOut          <= 8'd0;
    end else begin

        beginTransactionOut <= 1'b0;
        readNotWriteOut     <= 1'b0;
        endTransactionOut   <= 1'b0;
        byteEnablesOut      <= 4'b0000;
        burstSizeOut        <= 8'd0;

        // CI register writes
        if (isMyCi && valueA == 32'd1) begin
            sourceAddressReg <= valueB; // we read the source address from valueB
        end

        if (isMyCi && valueA == 32'd2) begin
            destinationAddressReg <= valueB; // we read the destination address from valueB
        end

        if (isMyCi && valueA == 32'd3) begin
            // control register, bit 0 = start
            if (valueB[0] == 1'b1 && !busyWire) begin
                // start the accelerator
                doneReg  <= 1'b0;
                errorReg <= 1'b0;
                stateReg <= STATE_START;
            end
        end

        // Temporary FSM
        // This only tests start/busy/done
        case (stateReg)

            STATE_IDLE: begin
                // Wait for start command
            end

            STATE_START: begin
                stateReg <= STATE_FINISH;
            end

            STATE_FINISH: begin
                doneReg  <= 1'b1;
                stateReg <= STATE_IDLE;
            end

            default: begin
                errorReg <= 1'b1;
                stateReg <= STATE_IDLE;
            end

        endcase

    end

end

endmodule
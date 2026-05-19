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
valueA = 4  -> read back source address, useful for debug
valueA = 5  -> read back destination address, useful for debug

*/

// internal reg for the accelerator
reg [31:0] sourceAddressReg;
reg [31:0] destinationAddressReg;
reg        doneReg;
reg        errorReg;
reg [4:0]  stateReg;

wire isMyCi = (ciN == customInstructionId) ? 1 : 0;





endmodule
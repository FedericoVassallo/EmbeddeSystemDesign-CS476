module sobelCi #( parameter [7:0] customId = 8'h00 )
                  ( input wire        start,
                                      clock,
                                      reset,
                                      stall,
                                      busIdle,
                    input wire [31:0] valueA,
                                      valueB,
                    input wire [7:0]  ciN,
                    output wire       done,
                    output wire [31:0] result );


assign wire [7:0] bit1_op_dx = ~valueA[7:0]; // opration on the bit 1 for the Dx is -1 X 
// on the bit 2 for the calculation of the Dx is 0
assign wire [7:0] bit3_op_dx = valueA[23:16]; // operation on bit 3 for dx is 1 X bit 

assign wire [8:0] bit4_op_dx = ~{valueA[31:24], 1'b0}; // op for bit 3  is -2 X
// bit 5 is not sent, since is always 0
assign wire [8:0] bit6_op_dx = {valueB[15:8], 1'b0}; // op for bit 6  is 2 X

assign wire [7:0] bit7_op_dx = ~valueB[23:16]; // op for bit 7  is -1 X

// bit 8 is just 0 x in Dx operation
assign wire [7:0] bit9_op_dx = valueB[31:24]; // op for bit 9  is 1 X

valueA[31:24]
 ~{valueA[23:16], 1'b0};

assign wire [8:0] bit1_op_dx = ~{valueA[7:0], 1'b0}; // opration on the bit 1 for the Dx




endmodule;
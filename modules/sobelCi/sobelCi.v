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

/*


wire [7:0] bit1_op_dx = ~valueA[7:0]; // opration on the bit 1 for the Dx is -1 X 
// on the bit 2 for the calculation of the Dx is 0
wire [7:0] bit3_op_dx = valueA[23:16]; // operation on bit 3 for dx is 1 X bit 

wire [8:0] bit4_op_dx = ~{valueA[31:24], 1'b0}; // op for bit 3  is -2 X
// bit 5 is not sent, since is always 0
wire [8:0] bit6_op_dx = {valueB[7:0], 1'b0}; // op for bit 6  is 2 X //////////////////////     (7:0)  ?       

wire [7:0] bit7_op_dx = ~valueB[15:8]; // op for bit 7  is -1 X  ////////////////////////       (15:8)  ? 

// bit 8 is just 0 x in Dx operation
wire [7:0] bit9_op_dx = valueB[31:24]; // op for bit 9  is 1 X ////////////////////////////  Questo ok forse 

/////////////////////////   Dy  //////////////////////////

wire [7:0] bit1_op_dy = valueA[7:0]; // operation on the bit 1 for the Dy is 1 X 

wire [8:0] bit2_op_dy = {valueA[15:8], 1'b0}; // operation on bit 2 for dy is 2 X 

wire [7:0] bit3_op_dy = valueA[23:16]; // operation on bit 3 for dy is -1 X

wire [7:0] bit7_op_dy = ~valueB[15:8]; // operation on bit 7 for dy is -1 X

wire [8:0] bit8_op_dy = ~{valueB[23:16], 1'b0};

wire [7:0] bit9_op_dy = ~valueB[31:24]; // operation on bit 9 for dy is 1 X


////// Final values //////

assign wire [8:0] valueX = bit1_op_dx + bit3_op_dx + bit4_op_dx + bit6_op_dx + bit7_op_dx + bit9_op_dx;
assign wire [8:0] valueY = bit1_op_dy + bit2_op_dy + bit3_op_dy + bit7_op_dy + bit8_op_dy + bit9_op_dy;

*/

// Pixel mapping 
wire [7:0] p1 = valueA[7:0];
wire [7:0] p2 = valueA[15:8];
wire [7:0] p3 = valueA[23:16];
wire [7:0] p4 = valueA[31:24];

wire [7:0] p6 = valueB[7:0];
wire [7:0] p7 = valueB[15:8];
wire [7:0] p8 = valueB[23:16];
wire [7:0] p9 = valueB[31:24];

// dx and dy wires (preventing overflow)
wire signed [10:0] dx;
wire signed [10:0] dy;

// assign results 
assign dx = - $signed({3'b0, p1}) 
                + $signed({3'b0, p3}) 
                - $signed({2'b0, p4, 1'b0}) 
                + $signed({2'b0, p6, 1'b0}) 
                - $signed({3'b0, p7}) 
                + $signed({3'b0, p9});

assign dy =   $signed({3'b0, p1}) 
                + $signed({2'b0, p2, 1'b0}) 
                + $signed({3'b0, p3}) 
                - $signed({3'b0, p7}) 
                - $signed({2'b0, p8, 1'b0}) 
                - $signed({3'b0, p9});

wire [10:0] abs_dx = (dx[10]) ? -dx : dx;
wire [10:0] abs_dy = (dy[10]) ? -dy : dy;

wire [10:0] magnitude = abs_dx + abs_dy;

// Pad with 21 zeros to reach the 32-bit output width
assign result = {21'b0, magnitude};

// Custom instruction finishes in a single clock cycle
assign done = 1'b1;


endmodule;
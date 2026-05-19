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

wire correctId = (ciN == customId);

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

// we oitput the result only if start is high and the ID matches, otherwise we put 0
assign result = ((start == 1'b1) && (correctId == 1'b1)) ? {21'b0, magnitude} : 32'd0;

// assign done only if we have start on and the corret ID
assign done = ((start == 1'b1) && (correctId == 1'b1)) ? 1'b1 : 1'b0;

endmodule;
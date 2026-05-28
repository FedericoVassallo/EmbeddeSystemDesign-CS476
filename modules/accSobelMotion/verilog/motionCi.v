module motionCi #( parameter [7:0] customId = 8'h00 )
                 ( input wire        start,
                                     clock,
                                     reset,
                                     stall,
                                     busIdle,
                   input wire [31:0] valueA,
                                     valueB,
                   input wire [7:0]  ciN,
                   output wire       done,
                   output wire [63:0] result );

wire correctId = (ciN == customId);

wire [7:0] curr0 = valueA[7:0];
wire [7:0] curr1 = valueA[15:8];
wire [7:0] curr2 = valueA[23:16];
wire [7:0] curr3 = valueA[31:24];

wire [7:0] prev0 = valueB[7:0];
wire [7:0] prev1 = valueB[15:8];
wire [7:0] prev2 = valueB[23:16];
wire [7:0] prev3 = valueB[31:24];

wire [15:0] pix0 = (curr0 != 8'd0 && prev0 == 8'd0) ? 16'hF800 :
                   (curr0 != 8'd0) ? 16'hFFFF : 16'h0000;
wire [15:0] pix1 = (curr1 != 8'd0 && prev1 == 8'd0) ? 16'hF800 :
                   (curr1 != 8'd0) ? 16'hFFFF : 16'h0000;
wire [15:0] pix2 = (curr2 != 8'd0 && prev2 == 8'd0) ? 16'hF800 :
                   (curr2 != 8'd0) ? 16'hFFFF : 16'h0000;
wire [15:0] pix3 = (curr3 != 8'd0 && prev3 == 8'd0) ? 16'hF800 :
                   (curr3 != 8'd0) ? 16'hFFFF : 16'h0000;

assign result = ((start == 1'b1) && (correctId == 1'b1)) ?
                {pix3, pix2, pix1, pix0} : 64'd0;

assign done = ((start == 1'b1) && (correctId == 1'b1)) ? 1'b1 : 1'b0;

endmodule

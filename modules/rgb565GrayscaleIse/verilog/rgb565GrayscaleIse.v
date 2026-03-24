module rgb565GrayscaleIse #(parameter [7:0] customInstructionId = 8'd0 )
                           ( input  wire         start ,
                             input  wire [31:0]  valueA ,
                             input  wire [31:0]  valueB ,
                             input  wire [7:0]   iseId ,
                             output wire         done,
                             output wire [31:0]  result );
wire isMine = (iseId == customInstructionId) ? start : 1'b0;
assign done = isMine;

// Swap bytes for each pixel (camera sends big-endian, CPU is little-endian)
wire [15:0] pixel_1 = {valueA[7:0],   valueA[15:8]};
wire [15:0] pixel_2 = {valueA[23:16], valueA[31:24]};
wire [15:0] pixel_3 = {valueB[7:0],   valueB[15:8]};
wire [15:0] pixel_4 = {valueB[23:16], valueB[31:24]};

// Extract components and pad to 8-bit values
wire [7:0] red_1   = {pixel_1[15:11], 3'b000};
wire [7:0] green_1 = {pixel_1[10:5],  2'b00};
wire [7:0] blue_1  = {pixel_1[4:0],   3'b000};
wire [7:0] red_2   = {pixel_2[15:11], 3'b000};
wire [7:0] green_2 = {pixel_2[10:5],  2'b00};
wire [7:0] blue_2  = {pixel_2[4:0],   3'b000};
wire [7:0] red_3   = {pixel_3[15:11], 3'b000};
wire [7:0] green_3 = {pixel_3[10:5],  2'b00};
wire [7:0] blue_3  = {pixel_3[4:0],   3'b000};
wire [7:0] red_4   = {pixel_4[15:11], 3'b000};
wire [7:0] green_4 = {pixel_4[10:5],  2'b00};
wire [7:0] blue_4  = {pixel_4[4:0],   3'b000};

// Calculate the weighted sum
wire [31:0] sum_1 = (red_1 * 8'd54) + (green_1 * 8'd183) + (blue_1 * 8'd19);
wire [31:0] sum_2 = (red_2 * 8'd54) + (green_2 * 8'd183) + (blue_2 * 8'd19);
wire [31:0] sum_3 = (red_3 * 8'd54) + (green_3 * 8'd183) + (blue_3 * 8'd19);
wire [31:0] sum_4 = (red_4 * 8'd54) + (green_4 * 8'd183) + (blue_4 * 8'd19);

wire [7:0] grayscaleValue_1 = sum_1[15:8];
wire [7:0] grayscaleValue_2 = sum_2[15:8];
wire [7:0] grayscaleValue_3 = sum_3[15:8];
wire [7:0] grayscaleValue_4 = sum_4[15:8];

assign result = (isMine) ? {grayscaleValue_4, grayscaleValue_3, grayscaleValue_2, grayscaleValue_1} : 32'b0;
endmodule
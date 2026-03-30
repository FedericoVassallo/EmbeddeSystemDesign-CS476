module rgb565GrayscaleIse #(parameter [7:0] customInstructionId = 8'd0 )
                           ( input  wire         start ,
                             input  wire [31:0]  valueA ,
                             input  wire [31:0]  valueB ,
                             input  wire [7:0]   iseId ,
                             output wire         done,
                             output wire [31:0]  result );

wire isMine = (iseId == customInstructionId) ? start : 1'b0;

assign done = isMine; // The instruction is done in one cycle, so done is just a copy of isMine

// Extract components and pad with 0s to make them full 8-bit values (0-255)
wire [7:0] red_1   = {valueA[15:11], 3'b000};
wire [7:0] green_1 = {valueA[10:5],  2'b00};
wire [7:0] blue_1  = {valueA[4:0],   3'b000};

wire [7:0] red_2   = {valueA[31:27], 3'b000};
wire [7:0] green_2 = {valueA[26:21],  2'b00};
wire [7:0] blue_2  = {valueA[20:16],   3'b000};

wire [7:0] red_3 = {valueB[15:11], 3'b000};
wire [7:0] green_3 = {valueB[10:5],  2'b00};
wire [7:0] blue_3 = {valueB[4:0],   3'b000};

wire [7:0] red_4 = {valueB[31:27], 3'b000};
wire [7:0] green_4 = {valueB[26:21],  2'b00};
wire [7:0] blue_4 = {valueB[20:16],   3'b000};

// Calculate the weighted sum. We use a 32-bit wire to prevent overflow.
wire [31:0] sum_1 = (red_1 * 8'd54) + (green_1 * 8'd183) + (blue_1 * 8'd19);
wire [31:0] sum_2 = (red_2 * 8'd54) + (green_2 * 8'd183) + (blue_2 * 8'd19);
wire [31:0] sum_3 = (red_3 * 8'd54) + (green_3 * 8'd183) + (blue_3 * 8'd19);
wire [31:0] sum_4 = (red_4 * 8'd54) + (green_4 * 8'd183) + (blue_4 * 8'd19);

// Divide by 256 by taking the upper 8 bits of the 32-bit sum (equivalent to >> 8)
wire [7:0] grayscaleValue_1 = sum_1[15:8];
wire [7:0] grayscaleValue_2 = sum_2[15:8];
wire [7:0] grayscaleValue_3 = sum_3[15:8];
wire [7:0] grayscaleValue_4 = sum_4[15:8];

// output the result as a 32-bit value where each byte is the grayscale value of one pixel
assign result = (isMine) ? {grayscaleValue_4, grayscaleValue_3, grayscaleValue_2, grayscaleValue_1} : 32'b0;

endmodule
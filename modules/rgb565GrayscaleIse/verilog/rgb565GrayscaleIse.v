module rgb565GrayscaleIse #(parameter [7:0] customInstructionId = 8'd0 ) 
                           ( input  wire         start ,
                             input  wire [31:0]  valueA ,
                             input  wire [7:0]   iseId ,
                             output wire         done,
                             output wire [31:0]  result );

wire isMine = (iseId == customInstructionId) ? start : 1'b0;

assign done = isMine; // The instruction is done in one cycle, so done is just a copy of isMine

// Extract components and pad with 0s to make them full 8-bit values (0-255)
wire [7:0] red   = {valueA[15:11], 3'b000}; 
wire [7:0] green = {valueA[10:5],  2'b00};  
wire [7:0] blue  = {valueA[4:0],   3'b000};

// Calculate the weighted sum. We use a 32-bit wire to prevent overflow.
wire [31:0] sum = (red * 8'd54) + (green * 8'd183) + (blue * 8'd19);

// Divide by 256 by taking the upper 8 bits of the 32-bit sum (equivalent to >> 8)
wire [7:0] grayscaleValue = sum[15:8];

// Output the grayscale value in the lower 8 bits, pad upper 24 bits with 0
assign result = (isMine) ? {24'b0, grayscaleValue} : 32'b0;

endmodule

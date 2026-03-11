module profileCi #( parameter [7:0] customId = 8'h00 )
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

wire [31:0] out_count0, out_count1, out_count2, out_count3; 

reg en0, en1, en2, en3;

wire rst0 = reset == 1'b1 | (correctId == 1'b1 & valueB[8] == 1'b1 & start == 1'b1); // valueB[8] resets counter 0
wire rst1 = reset == 1'b1 | (correctId == 1'b1 & valueB[9] == 1'b1 & start == 1'b1); // valueB[9] resets counter 1
wire rst2 = reset == 1'b1 | (correctId == 1'b1 & valueB[10] == 1'b1 & start == 1'b1); // valueB[10] resets counter 2 
wire rst3 = reset == 1'b1 | (correctId == 1'b1 & valueB[11] == 1'b1 & start == 1'b1); // valueB[11] resets counter 3 

counter #(.WIDTH(32)) counter0 ( .reset(rst0),
                                  .clock(clock),
                                  .enable(en0),
                                  .direction(1'b1),
                                  .counterValue(out_count0) );

counter #(.WIDTH(32)) counter1  ( .reset(rst1),
                                  .clock(clock),
                                  .enable(en1 & stall),
                                  .direction(1'b1),
                                  .counterValue(out_count1) );

counter #(.WIDTH(32)) counter2 ( .reset(rst2),
                                  .clock(clock),
                                  .enable(en2 & busIdle),
                                  .direction(1'b1),
                                  .counterValue(out_count2) );

counter #(.WIDTH(32)) counter3 ( .reset(rst3),
                                  .clock(clock),
                                  .enable(en3),
                                  .direction(1'b1),
                                  .counterValue(out_count3) );

always @(posedge clock) begin
    if (reset == 1'b1) begin
        en0 <= 1'b0; en1 <= 1'b0; en2 <= 1'b0; en3 <= 1'b0;
    end 
    else if (start == 1'b1 && correctId == 1'b1) begin
        // Se start è alto e l'ID è giusto, aggiorno gli stati di abilitazione
        if (valueB[4] == 1'b1)      en0 <= 1'b0;
        else if (valueB[0] == 1'b1) en0 <= 1'b1;

        if (valueB[5] == 1'b1)      en1 <= 1'b0;
        else if (valueB[1] == 1'b1) en1 <= 1'b1;

        if (valueB[6] == 1'b1)      en2 <= 1'b0;
        else if (valueB[2] == 1'b1) en2 <= 1'b1;

        if (valueB[7] == 1'b1)      en3 <= 1'b0;
        else if (valueB[3] == 1'b1) en3 <= 1'b1;
    end
    // Se start è basso, en0...en3 DEVONO mantenere il loro valore attuale (non fare nulla)
end

assign done = ((start == 1'b1) && (correctId == 1'b1)) ? 1'b1 : 1'b0;

// We creeate a 32-bit wire to hold the value of the selected counter. We select the counter based on the value of valueA[1:0]
wire [31:0] selected_counter;
assign selected_counter = (valueA[1:0] == 2'b00) ? out_count0 :
                          (valueA[1:0] == 2'b01) ? out_count1 :
                          (valueA[1:0] == 2'b10) ? out_count2 :
                          (valueA[1:0] == 2'b11) ? out_count3 :
                                                   32'b0;

// If the start signal is high and the custom ID matches, output the selected counter value, otherwise always output 0
assign result = ((start == 1'b1) && (correctId == 1'b1)) ? selected_counter : 32'b0;

endmodule


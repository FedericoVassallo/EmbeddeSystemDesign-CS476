module dualPortSSRAM #( parameter bitwidth = 8,
                        parameter nrOfEntries = 512,
                        parameter readAfterWrite = 0 )
                      ( input wire                             clockA, clockB,
                                                               writeEnableA, writeEnableB,
                        input wire [$clog2(nrOfEntries)-1 : 0] addressA, addressB,
                        input wire [bitwidth-1 : 0]            dataInA, dataInB,
                        output reg [bitwidth-1 : 0]            dataOutA, dataOutB);

reg [bitwidth-1 : 0] memoryContent [nrOfEntries-1 : 0];

always @(posedge clockA)
  begin
    if (readAfterWrite != 0) dataOutA = memoryContent[addressA];
    if (writeEnableA == 1'b1) memoryContent[addressA] = dataInA;
    if (readAfterWrite == 0) dataOutA = memoryContent[addressA];
  end

always @(posedge clockB)
  begin
    if (readAfterWrite != 0) dataOutB = memoryContent[addressB];
    if (writeEnableB == 1'b1) memoryContent[addressB] = dataInB;
    if (readAfterWrite == 0) dataOutB = memoryContent[addressB];
  end

endmodule
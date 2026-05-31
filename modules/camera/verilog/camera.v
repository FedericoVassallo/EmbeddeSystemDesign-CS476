module camera #(parameter [7:0] customInstructionId = 8'd0,
                parameter clockFrequencyInHz = 2000)
               (input wire         clock,
                                   pclk,
                                   reset,
                                   hsync,
                                   vsync,
                                   ciStart,
                                   ciCke,
                input wire [7:0]   ciN,
                                   camData,
                input wire [31:0]  ciValueA,
                                   ciValueB,
                output wire [31:0] ciResult,
                output wire        ciDone,
                // here the bus master interface is defined
                output wire        requestBus,
                input wire         busGrant,
                output reg         beginTransactionOut,
                output wire [31:0] addressDataOut,
                output reg         endTransactionOut,
                output reg  [3:0]  byteEnablesOut,
                output wire        dataValidOut,
                output reg  [7:0]  burstSizeOut,
                input wire         busyIn,
                                   busErrorIn);

  /*
   *
   * this module provides an interface to the OV7670 camera module
   *
   * different ci commands:
   * ciValueA:    Description:
   *     0        Read Nr. of Bytes per line
   *     1        Read Nr. of Lines per image
   *     2        Read PCLK frequency in kHz
   *     3        Read Frames per second (wait at least 2 s after initializing the camera for a valid value)
   *     4        Read frame buffer address
   *     5        Write frame buffer address (ciValueB)
   *     6        Start/stop image aquisition (ciValueb[1..0] = "01")
   *     6        Take single image (ciValueb[1..0] = "10")
   *     7        Read (self clearing): Single image grabbing done.
   *
   */
  function integer clog2;
    input integer value;
    begin
      for (clog2 = 0; value > 0 ; clog2= clog2 + 1)
      value = value >> 1;
    end
  endfunction
  
  localparam khzDivideValue = clockFrequencyInHz/1000; // here we basically have the number of clock cycles in 1 ms that will be used for the counter later
  localparam khzNrOfBits = clog2(khzDivideValue); // here we calculate how many bits we need for the khz counter (for example if clock is 2 MHz, then 2Mhz/1000 = 2000, which can be represented in 11 bits, so khzNrOfBits will be 11)
  localparam [2:0] IDLE         = 3'd0;
  localparam [2:0] REQUEST_BUS1 = 3'd1;
  localparam [2:0] INIT_BURST1  = 3'd2;
  localparam [2:0] DO_BURST1    = 3'd3;
  localparam [2:0] END_TRANS1   = 3'd4;
  localparam [2:0] END_TRANS2   = 3'd5;
  
  reg [2:0] s_stateMachineReg, s_stateMachineNext;
  reg s_singleShotDoneReg;
  
  wire s_isMyCi = (ciN == customInstructionId) ? ciStart & ciCke : 1'b0; 
  /*
   *
   * Here we define the counters for the 1KHz pulse and 1Hz pulse
   *
   */
  reg [khzNrOfBits-1:0] s_khzCountReg;
  reg [9:0] s_hzCountReg; // to count 1 second we need to count 1000 ms so we exploit the fact that we already have a khz counter that counts ms, and we just need to count 1000 ms
  wire s_khzCountZero = (s_khzCountReg == {khzNrOfBits{1'b0}}) ? 1'b1 : 1'b0; // basically when the khz counter reaches 0 we set the khzCountZero signal to 1
  wire s_hzCountZero  = (s_hzCountReg == 10'd0) ? s_khzCountZero : 1'b0; // when the hz counter reaches 0 (but we also check that the khz counter is at zero to precisely detect the 1 second mark)
  wire [khzNrOfBits-1:0] s_khzCountNext = (reset == 1'b1 || s_khzCountZero == 1'b1) ? khzDivideValue - 1 : s_khzCountReg - 1; // this store the value that will be loaded in the khz counter at the next cycle: if reset is high or if the counter has reached 0, we load the initial value (which is the number of clock cycles in 1 ms) otherwise we just decrement the counter
  wire [9:0] s_hzCountNext = (reset == 1'b1 || s_hzCountZero == 1'b1) ? 10'd999 : (s_khzCountZero == 1'b1) ? s_hzCountReg - 10'd1 : s_hzCountReg; // this store the value that will be loaded in the hz counter that counts 1 , same logic as before when it has counted 1 second it gets reset
  
  always @(posedge clock)
    begin
      // each cycle we update each of the counter with the value seen above
      s_khzCountReg <= s_khzCountNext;
      s_hzCountReg  <= s_hzCountNext;
    end
  
  /*
   *
   * Here we define the frame buffer parameters
   *
   */
  reg[31:0] s_frameBufferBaseReg; // holds the base address of the frame buffer in memory, can be set through the ci interface, it where the camera will write the captured image data
  reg s_grabberActiveReg,s_grabberSingleShotReg; // they are just flags to keep track if we are using continuous camera mode or take single shot
  
  always @(posedge clock)
    begin
      s_frameBufferBaseReg   <= (reset == 1'b1) ? 32'd0 : (s_isMyCi == 1'b1 && ciValueA[2:0] == 3'd5) ? {ciValueB[31:2],2'd0} : s_frameBufferBaseReg; // the frame buffer base address can be set through the ci interface with command 5 (the two final 0 forced are just to make sure the address is word aligned)
      s_grabberActiveReg     <= (reset == 1'b1) ? 1'b0 : (s_isMyCi == 1'b1 && ciValueA[2:0] == 3'd6) ? ciValueB[0]& ~ciValueB[1] : s_grabberActiveReg; // throw command 6 we can start and stop the continuos grabbing mode, it gets activeted if if the bottom two bits of ciValueB are exactly 01 in binary
      s_grabberSingleShotReg <= (reset == 1'b1 || s_singleShotActionReg[0] == 1'b1) ? 1'b0 : (s_isMyCi == 1'b1 && ciValueA[2:0] == 3'd6) ? ciValueB[1]& ~ciValueB[0] : s_grabberSingleShotReg; // togheter with the reset it also check for s_singleShotActionReg in the sense that if we are already in single shot mode we don't take another, to activate this mode the bottom two bits of ciValueB need to be 10 in binary
    end
  
  /*
   *
   * Here we do the measurements on the camera interface
   *
   */
  reg[1:0]  s_vsyncDetectReg;
  reg[1:0]  s_hsyncDetectReg;
  reg[10:0] s_pixelCountReg, s_pixelCountValueReg;
  reg[10:0] s_lineCountReg, s_lineCountValueReg;
  reg[16:0] s_pclkCountReg, s_pclkCountValueReg;
  reg[7:0]  s_fpsCountReg, s_fpsCountValueReg;
  wire      s_clockPclkValue, s_clockFPS;
  
  wire s_vsyncNegEdge = ~s_vsyncDetectReg[0] & s_vsyncDetectReg[1];
  wire s_hsyncNegEdge = ~s_hsyncDetectReg[0] & s_hsyncDetectReg[1];
  
  always @(posedge pclk)
    begin
      s_vsyncDetectReg     <= {s_vsyncDetectReg[0],vsync};
      s_hsyncDetectReg     <= {s_hsyncDetectReg[0],hsync};
      s_pixelCountValueReg <= (s_hsyncNegEdge == 1'b1) ? s_pixelCountReg : s_pixelCountValueReg;
      s_pixelCountReg      <= (s_hsyncNegEdge == 1'b1) ? 11'd0 : (hsync == 1'b1) ? s_pixelCountReg + 11'd1 : s_pixelCountReg;
      s_lineCountValueReg  <= (s_vsyncNegEdge == 1'b1) ? s_lineCountReg : s_lineCountValueReg;
      s_lineCountReg       <= (s_vsyncNegEdge == 1'b1) ? 11'd0 : (s_hsyncNegEdge == 1'b1) ? s_lineCountReg + 11'd1 : s_lineCountReg;
      s_pclkCountReg       <= (reset == 1'b1 || s_clockPclkValue == 1'b1) ? 17'd0 : s_pclkCountReg + 17'd1;
      s_pclkCountValueReg  <= (reset == 1'b1) ? 17'd0 : (s_clockPclkValue == 1'b1) ? s_pclkCountReg : s_pclkCountValueReg;
      s_fpsCountReg        <= (reset == 1'b1 || s_clockFPS == 1'b1) ? 8'd0 : (s_vsyncNegEdge == 1'b1) ? s_fpsCountReg + 8'd1 : s_fpsCountReg;
      s_fpsCountValueReg   <= (reset == 1'b1) ? 8'd0 : (s_clockFPS == 1'b1) ? s_fpsCountReg : s_fpsCountValueReg;
    end
  
    /*
    In this module, there are 2 completely independent clocks running at the same time:
    clock: The main system clock (running the 1 ms and 1 sec timers)
    pclk: The camera's pixel clock
    since these two clocks are not synchronized, they tick at slightly different times
    if we take the 1-millisecond alarm (s_khzCountZero) and the 1-second alarm (s_hzCountZero), these alarms are generated on the main system clock
    the registers that need to hear those alarms are running on the camera's pclk
    if the camera clock (pclk) tries to read the 1-second alarm at the exact microscopic fraction of a nanosecond that the alarm is flipping from 0 to 1, it might read a voltage that is halfway between 0 and 1 (metastable) 
    to solve this problem, we use The synchroFlop
    A synchroFlop acts as a safe "waiting room" between the two clock domains

    // so we give in input the alarm signal that is generated in the main clock domain, and we get out the same signal but safely synchronized to the camera clock domain
    */


   synchroFlop spclk ( .clockIn(clock),
                       .clockOut(pclk),
                       .reset(reset),
                       .D(s_khzCountZero),
                       .Q(s_clockPclkValue) );
   synchroFlop sfps ( .clockIn(clock),
                      .clockOut(pclk),
                      .reset(reset),
                      .D(s_hzCountZero),
                      .Q(s_clockFPS) );
  /*
   *
   * here the ci interface is defined
   *
   */
  reg [31:0] s_selectedResult;
  
  assign ciDone   = s_isMyCi;
  assign ciResult = (s_isMyCi == 1'b0) ? 32'd0 : s_selectedResult;

  always @*
    // depening on the command send we give a different result in output
    case (ciValueA[3:0])
      4'd0    : s_selectedResult <= {21'd0,s_pixelCountValueReg}; // the total number of pixels in a single horizontal line
      4'd1    : s_selectedResult <= {21'd0,s_lineCountValueReg}; // the total number of horizontal lines in a single video frame
      4'd2    : s_selectedResult <= {15'd0,s_pclkCountValueReg}; // freq of the camera pclk
      4'd3    : s_selectedResult <= {24'd0,s_fpsCountValueReg}; // video frame rate
      4'd4    : s_selectedResult <= s_frameBufferBaseReg; // base address of where the camera is writing
      4'd7    : s_selectedResult <= {31'd0,s_singleShotDoneReg}; // if a single shot has been taken and is done
      default : s_selectedResult <= 32'd0;
    endcase

  /*
   *
   * Here the grabber is defined
   *
   */
  reg [7:0] s_byte3Reg,s_byte2Reg,s_byte1Reg,s_byte4Reg,s_byte5Reg,s_byte6Reg,s_byte7Reg;
  reg [8:0] s_busSelectReg;
  wire [31:0] s_busPixelWord;
  wire [7:0] s_pixel1, s_pixel2,s_pixel3,s_pixel4;
  wire [31:0] s_rgb565Grayscale = {s_pixel4,s_pixel3,s_pixel2,s_pixel1}; // we pack 4 grayscale pixels in a single 32-bit word to be written in memory
  wire s_weLineBuffer = (s_pixelCountReg[2:0] == 3'b111) ? hsync : 1'b0; // once every 8 clock ticks (when the 32-bit word is fully packed), this wire goes high and it is used for the ram below

  rgb565Grayscale gray1 ( .rgb565({s_byte7Reg,s_byte6Reg}),
                          .grayscale(s_pixel1) );
  rgb565Grayscale gray2 ( .rgb565({s_byte5Reg,s_byte4Reg}),
                          .grayscale(s_pixel2) );
  rgb565Grayscale gray3 ( .rgb565({s_byte3Reg,s_byte2Reg}),
                          .grayscale(s_pixel3) );
  rgb565Grayscale gray4 ( .rgb565({s_byte1Reg,camData}),
                          .grayscale(s_pixel4) );

  always @(posedge pclk) // synchronous to the camera pixel clock
    // the camera sets hsync to 1 only when it is actively transmitting a valid line of the image
    // the full s_pixelCountReg is an 11-bit number that counts all the way up to the end of the line, by writing [2:0], the hardware is only looking at the bottom 3 bits of that counter
    // if you count upwards using only 3 binary bits, it naturally rolls over in an endless 8-step cycle
    // 2 byte makes 1 pixel rgb565 of the camera (that we convert with the grayscale converter above)
    begin
      // for each if we are in the right step of the 8-step cycle and hsync is high, we load the current pixel data from the camera in the corresponding byte register
      s_byte7Reg <= (s_pixelCountReg[2:0] == 3'b000 && hsync == 1'b1) ? camData : s_byte7Reg;
      s_byte6Reg <= (s_pixelCountReg[2:0] == 3'b001 && hsync == 1'b1) ? camData : s_byte6Reg;
      s_byte5Reg <= (s_pixelCountReg[2:0] == 3'b010 && hsync == 1'b1) ? camData : s_byte5Reg;
      s_byte4Reg <= (s_pixelCountReg[2:0] == 3'b011 && hsync == 1'b1) ? camData : s_byte4Reg;
      s_byte3Reg <= (s_pixelCountReg[2:0] == 3'b100 && hsync == 1'b1) ? camData : s_byte3Reg;
      s_byte2Reg <= (s_pixelCountReg[2:0] == 3'b101 && hsync == 1'b1) ? camData : s_byte2Reg;
      s_byte1Reg <= (s_pixelCountReg[2:0] == 3'b110 && hsync == 1'b1) ? camData : s_byte1Reg;
      // we don't have the byte0Reg because the last byte of the 32-bit word is directly feed into the grayscale converter
    end
  
  // this is the dual port ram 

  /*
  The camera is giving out data on its own clock (pclk), but if we want to read from the clock of system
  if they try to talk directly, there would be problem,  the dual port RAM has two separate "doors" that operate completely independently of each other
  */
  
  dualPortRam2k lineBuffer ( .address1({1'b0,s_pixelCountReg[10:3]}),
                             .address2(s_busSelectReg),
                             .clock1(pclk), // the door 1 operates at the camera pixel clock
                             .clock2(clock), // the door 2 operates at the system clock
                             .writeEnable(s_weLineBuffer), // once every 8 rgb565 pixels saved we write our grayscale 32 bit word (so 4 pixels)
                             .dataIn1(s_rgb565Grayscale), 
                             .dataOut2(s_busPixelWord)); // 

  /*
   *
   * Here the bus interface is defined
   *
   */
  reg [31:0] s_busAddressReg, s_addressDataOutReg;
  reg [8:0] s_nrOfPixelsPerLineReg;
  reg [1:0] s_singleShotActionReg;
  reg s_dataValidReg;
  reg [8:0] s_burstCountReg;
  reg  s_grabberRunningReg;
  wire s_newScreen, s_newLine;
  wire s_doWrite = ((s_stateMachineReg == DO_BURST1) && s_burstCountReg[8] == 1'b0) ? ~busyIn : 1'b0;
  wire [31:0] s_busAddressNext = (reset == 1'b1 || s_newScreen == 1'b1) ? s_frameBufferBaseReg : 
                                 (s_doWrite == 1'b1) ? s_busAddressReg + 32'd4 : s_busAddressReg; // calculates exactly where in RAM the next 32-bit pixel word should be saved.
  wire [7:0] s_burstSizeNext = ((s_stateMachineReg == INIT_BURST1) && s_nrOfPixelsPerLineReg > 9'd16) ? 8'd16 : s_nrOfPixelsPerLineReg[7:0]; // set the next burst size
  
  assign requestBus        = (s_stateMachineReg == REQUEST_BUS1) ? 1'b1 : 1'b0;
  assign addressDataOut    = s_addressDataOutReg;
  assign dataValidOut      = s_dataValidReg;
  
  always @*
    case (s_stateMachineReg)
      IDLE            : s_stateMachineNext <= ((s_grabberRunningReg == 1'b1 || s_singleShotActionReg[0] == 1'b1) && s_newLine == 1'b1) ? REQUEST_BUS1 : IDLE; // we move from idle if we are in grabber mode (either continuous or single shot) and we detect the start of a new line from the camera
      REQUEST_BUS1    : s_stateMachineNext <= (busGrant == 1'b1) ? INIT_BURST1 : REQUEST_BUS1; // we wait until the bus grant
      INIT_BURST1     : s_stateMachineNext <= DO_BURST1; 
      DO_BURST1       : s_stateMachineNext <= (busErrorIn == 1'b1) ? END_TRANS2 :
                                              (s_burstCountReg[8] == 1'b1 && busyIn == 1'b0) ? END_TRANS1 : DO_BURST1; // if the burst is done (burst count goes negative) and the bus is not busy we can end
      END_TRANS1      : s_stateMachineNext <= (s_nrOfPixelsPerLineReg != 9'd0) ? REQUEST_BUS1 : IDLE;
      default         : s_stateMachineNext <= IDLE;
    endcase
  
  always @(posedge clock)
    begin
      s_busAddressReg        <= s_busAddressNext;
      s_grabberRunningReg    <= (reset == 1'b1) ? 1'b0 : (s_newScreen == 1'b1) ? s_grabberActiveReg : s_grabberRunningReg; // copies the on/off of the grabbing but only if we are at the start of a new frame to not capture half broken frame
      s_singleShotActionReg  <= (reset == 1'b1 || s_singleShotActionReg[1] == 1'b1) ? 2'b0 : 
                                (s_newScreen == 1'b1) ? {s_singleShotActionReg[0],s_grabberSingleShotReg} : s_singleShotActionReg;
      s_singleShotDoneReg    <= (reset == 1'b1 || (s_isMyCi == 1'b1 && ciValueA[2:0] == 3'd6 && ciValueB[1] == 1'b1 && ciValueB[0] == 1'b0)) ? 1'b0 : 
                                (s_singleShotActionReg[1] == 1'b1) ? 1'b1 : s_singleShotDoneReg; // the flag the CPU reads to know if the snapshot is sitting safely in RAM 
      s_stateMachineReg      <= (reset == 1'b1) ? IDLE : s_stateMachineNext; // if reset we go idle
      beginTransactionOut    <= (s_stateMachineReg == INIT_BURST1) ? 1'd1 : 1'd0;
      byteEnablesOut         <= (s_stateMachineReg == INIT_BURST1) ? 4'hF : 4'd0;
      s_addressDataOutReg    <= (s_stateMachineReg == INIT_BURST1) ? s_busAddressReg : 
                                (s_doWrite == 1'b1) ? s_busPixelWord :
                                (busyIn == 1'b1) ? s_addressDataOutReg : 32'd0;
      s_dataValidReg         <= (s_doWrite == 1'b1) ? 1'b1 : (busyIn == 1'b1) ? s_dataValidReg : 1'b0;
      endTransactionOut      <= (s_stateMachineReg == END_TRANS1 || s_stateMachineReg == END_TRANS2) ? 1'b1 : 1'b0;
      burstSizeOut           <= (s_stateMachineReg == INIT_BURST1) ? s_burstSizeNext - 8'd1 : 8'd0;
      s_burstCountReg        <= (s_stateMachineReg == INIT_BURST1) ? s_burstSizeNext - 8'd1 :
                                (s_doWrite == 1'b1) ? s_burstCountReg - 9'd1 : s_burstCountReg;
      s_busSelectReg         <= (s_stateMachineReg == IDLE) ? 9'd0 : (s_doWrite == 1'b1) ? s_busSelectReg + 9'd1 : s_busSelectReg;
      s_nrOfPixelsPerLineReg <= (s_newLine == 1'b1) ? {1'b0, s_pixelCountValueReg[10:3]} : 
                                (s_stateMachineReg == INIT_BURST1) ? s_nrOfPixelsPerLineReg - {1'b0,s_burstSizeNext} : s_nrOfPixelsPerLineReg;
    end
  
  synchroFlop sns ( .clockIn(pclk),
                    .clockOut(clock),
                    .reset(reset),
                    .D(s_vsyncNegEdge),
                    .Q(s_newScreen) );
  
  synchroFlop snl ( .clockIn(pclk),
                    .clockOut(clock),
                    .reset(reset),
                    .D(s_hsyncNegEdge),
                    .Q(s_newLine) );
  
endmodule

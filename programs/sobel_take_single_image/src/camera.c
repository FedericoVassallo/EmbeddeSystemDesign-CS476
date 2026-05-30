#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
#ifdef __OR1300__
#include <cache.h>
#endif

// profileCi counter command
static const uint32_t PROF_START            = 7;          // start cycle/stall/idle counters
static const uint32_t PROF_STOP_READ_CYCLES = 1<<8|7<<4;  // freeze all counters and reset cycle counter
static const uint32_t PROF_SEL_STALL        = 1;          // valueA: select stall counter
static const uint32_t PROF_STALL_RESET      = 1<<9;       // reset stall counter
static const uint32_t PROF_SEL_IDLE         = 2;          // valueA: select bus-idle counter
static const uint32_t PROF_IDLE_RESET       = 1<<10;      // reset bus-idle counter

// accSobel hardware accelerator commands
static const uint32_t SOBEL_ACC_REG_STATUS  = 0;    // read: {error, busy, done}
static const uint32_t SOBEL_ACC_REG_SRC     = 1;    // write: source grayscale frame address
static const uint32_t SOBEL_ACC_REG_DST     = 2;    // write: destination edge-map address
static const uint32_t SOBEL_ACC_REG_CONTROL = 3;    // write: bit 0 = start
static const uint32_t SOBEL_ACC_STATUS_BUSY = 0x2;

// accMotion hardware accelerator commands
static const uint32_t MOTION_ACC_REG_STATUS  = 0;   // read: {error, busy, done}
static const uint32_t MOTION_ACC_REG_SRCA    = 1;   // write: source A address (current edge frame)
static const uint32_t MOTION_ACC_REG_SRCB    = 2;   // write: source B address (previous edge frame)
static const uint32_t MOTION_ACC_REG_DST     = 3;   // write: destination address (motion buffer)
static const uint32_t MOTION_ACC_REG_CONTROL = 4;   // write: bit 0 = start
static const uint32_t MOTION_ACC_STATUS_BUSY = 0x2;

// function that send the Ci command to the motion accelerator
static inline uint32_t motion_acc_ci(uint32_t a, uint32_t b) {
    uint32_t r;
    asm volatile("l.nios_rrr %[out],%[inA],%[inB],0xF"
                 : [out]"=r"(r): [inA]"r"(a), [inB]"r"(b));
    return r;
}

// function that send the Ci command to the Sobel accelerator
static inline uint32_t sobel_acc_ci(uint32_t a, uint32_t b)
{
    uint32_t r;
    asm volatile("l.nios_rrr %[out],%[inA],%[inB],0xE"
                 : [out]"=r"(r): [inA]"r"(a), [inB]"r"(b));
    return r;
}

volatile uint8_t  grayscaleA[640*480]; // capture buffer A
volatile uint8_t  grayscaleB[640*480]; // capture buffer B (pipeline: next frame)
volatile uint8_t  sobelA[640*480];    // edge map, buffer A
volatile uint8_t  sobelB[640*480];    // edge map, buffer B
// double buffers for motion output HDMI displays one buffer while the accelerator writes the other, then we ping-pong
volatile uint16_t motionA[640*480];
volatile uint16_t motionB[640*480];

int main () {
  volatile int result;
  volatile unsigned int *vga = (unsigned int *) 0X50000020;
  camParameters camParams;
  volatile uint32_t cycles, stall, idle;
  volatile uint8_t  *captureReady  = grayscaleA; // frame ready to be processed
  volatile uint8_t  *captureNext   = grayscaleB; // where the camera writes next
  volatile uint8_t  *sobelCurr     = sobelA;     // this frame's edges
  volatile uint8_t  *sobelPrev     = sobelB;     // last frame's edges
  volatile uint16_t *motionDisplay = motionA;    // what HDMI is currently showing
  volatile uint16_t *motionDraw    = motionB;    // what the accelerator writes into
#ifdef __OR1300__
  icache_write_cfg(CACHE_SIZE_8K|CACHE_FOUR_WAY|CACHE_REPLACE_LRU);
  dcache_write_cfg(CACHE_SIZE_8K|CACHE_FOUR_WAY|CACHE_WRITE_BACK|CACHE_REPLACE_PLRU);
  icache_enable(1);
  dcache_enable(1);
#endif
  vga_clear();

  printf("Initialising camera (this takes up to 3 seconds)!\n");
  camParams = initOv7670(VGA);
  printf("Done!\n");
  printf("NrOfPixels : %d\n", camParams.nrOfPixelsPerLine);
  result = (camParams.nrOfPixelsPerLine <= 320) ? camParams.nrOfPixelsPerLine | 0x80000000 : camParams.nrOfPixelsPerLine;
  vga[0] = swap_u32(result);
  printf("NrOfLines  : %d\n", camParams.nrOfLinesPerImage);
  result = (camParams.nrOfLinesPerImage <= 240) ? camParams.nrOfLinesPerImage | 0x80000000 : camParams.nrOfLinesPerImage;
  vga[1] = swap_u32(result);
  printf("PCLK (kHz) : %d\n", camParams.pixelClockInkHz );
  printf("FPS        : %d\n", camParams.framesPerSecond );

  // clear all buffers 
  for (int i = 0; i < 640*480; i++) {
      grayscaleA[i] = 0; grayscaleB[i] = 0;
      sobelA[i]     = 0; sobelB[i]     = 0;
      motionA[i]    = 0; motionB[i]    = 0;
  }

  vga[2] = swap_u32(1); // RGB565 display mode
  vga[3] = swap_u32((uint32_t) motionDisplay);

  // block until the very first frame is captured
  takeSingleImageBlocking((uint32_t) captureReady);

  while (1) {
    // Start capturing the next frame into captureNext immediately
    // This runs in parallel with the Sobel+Motion processing below
    takeSingleImageNonBlocking((uint32_t) captureNext);

    // point HDMI at the last completed motion result while we compute the new one
    vga[3] = swap_u32((uint32_t) motionDisplay);

    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(PROF_START)); // start profiling

    // we start the the hardware Sobel accelerator on captureReady and write the edges into sobelCurr
    sobel_acc_ci(SOBEL_ACC_REG_SRC, (uint32_t)captureReady);
    sobel_acc_ci(SOBEL_ACC_REG_DST, (uint32_t)sobelCurr);
    sobel_acc_ci(SOBEL_ACC_REG_CONTROL, 1u);
    uint32_t acc_status;
    do {
        acc_status = sobel_acc_ci(SOBEL_ACC_REG_STATUS, 0);
    } while ((acc_status & SOBEL_ACC_STATUS_BUSY) != 0); // we do polling until the accelerator is done

    // we start the hardware motion accelerator to compare this frame's edges with the previous frame's edges and write the result into motionDraw
    motion_acc_ci(MOTION_ACC_REG_SRCA, (uint32_t)sobelCurr);
    motion_acc_ci(MOTION_ACC_REG_SRCB, (uint32_t)sobelPrev);
    motion_acc_ci(MOTION_ACC_REG_DST,  (uint32_t)motionDraw);
    motion_acc_ci(MOTION_ACC_REG_CONTROL, 1u);
    uint32_t mot_status;
    do {
        mot_status = motion_acc_ci(MOTION_ACC_REG_STATUS, 0);
    } while ((mot_status & MOTION_ACC_STATUS_BUSY) != 0); // we do polling until the accelerator is done

    // we swap sobel pointers and this frame's edges become "previous" for next frame
    volatile uint8_t *temp = sobelPrev;
    sobelPrev = sobelCurr;
    sobelCurr = temp;

    // swap motion pointers
    volatile uint16_t *tempM = motionDisplay;
    motionDisplay = motionDraw;
    motionDraw    = tempM;

    // we wait for captureNext to be fully written before we process it
    waitForNextImage();

    // stop profiling and print results
    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(PROF_STOP_READ_CYCLES));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(PROF_SEL_STALL),[in2]"r"(PROF_STALL_RESET));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(PROF_SEL_IDLE),[in2]"r"(PROF_IDLE_RESET));
    printf("nrOfCycles for Sobel: %d %d %d\n", cycles, stall, idle);

    // we swap capture buffers so captureNext is now ready, captureReady is free
    volatile uint8_t *tempC = captureReady;
    captureReady = captureNext;
    captureNext  = tempC;
  }
}

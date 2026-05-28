#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
#ifdef __OR1300__
#include <cache.h>
#endif

// Fused Sobel + motion hardware accelerator (CI 0x09)
#define FUSION_ACC_REG_STATUS    0u   // read: {error, busy, done}
#define FUSION_ACC_REG_SRC       1u   // write: source grayscale frame address
#define FUSION_ACC_REG_PREV_EDGE 2u   // write: previous edge-map address
#define FUSION_ACC_REG_CURR_EDGE 3u   // write: current edge-map address
#define FUSION_ACC_REG_MOTION    4u   // write: destination motion buffer
#define FUSION_ACC_REG_CONTROL   5u   // write: bit 0 = start
#define FUSION_ACC_STATUS_BUSY   0x2u

static inline uint32_t fusion_acc_ci(uint32_t a, uint32_t b)
{
    uint32_t r;
    asm volatile("l.nios_rrr %[out],%[inA],%[inB],0x9"
                 : [out]"=r"(r)
                 : [inA]"r"(a), [inB]"r"(b));
    return r;
}

volatile uint8_t  grayscaleA[640*480]; // capture buffer A
volatile uint8_t  grayscaleB[640*480]; // capture buffer B (pipeline: next frame)
volatile uint8_t  sobelA[640*480];    // edge map, buffer A
volatile uint8_t  sobelB[640*480];    // edge map, buffer B
// Double-buffered motion output: HDMI displays one buffer while the
// accelerator writes the other, then we ping-pong. This prevents the
// visible tearing/flicker that happens when HDMI scans-out a buffer that
// the Motion accelerator is concurrently writing to.
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

  // Clear all buffers so the very first displayed frame is black rather
  // than uninitialised SDRAM.
  for (int i = 0; i < 640*480; i++) {
      grayscaleA[i] = 0; grayscaleB[i] = 0;
      sobelA[i]     = 0; sobelB[i]     = 0;
      motionA[i]    = 0; motionB[i]    = 0;
  }

  vga[2] = swap_u32(1); // RGB565 display mode
  vga[3] = swap_u32((uint32_t) motionDisplay);

  // block until the very first frame is captured.
  takeSingleImageBlocking((uint32_t) captureReady);

  while (1) {
    // Start capturing the next frame into captureNext immediately
    // This runs in parallel with the Sobel+Motion processing below.
    takeSingleImageNonBlocking((uint32_t) captureNext);

    // Point HDMI at the last completed motion result while we compute the new one.
    vga[3] = swap_u32((uint32_t) motionDisplay);

    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7)); // start profiling

    // The fused accelerator computes Sobel, stores this frame's edge map,
    // compares it with the previous edge map, and writes the RGB565 motion frame.
    fusion_acc_ci(FUSION_ACC_REG_SRC,       (uint32_t)captureReady);
    fusion_acc_ci(FUSION_ACC_REG_PREV_EDGE, (uint32_t)sobelPrev);
    fusion_acc_ci(FUSION_ACC_REG_CURR_EDGE, (uint32_t)sobelCurr);
    fusion_acc_ci(FUSION_ACC_REG_MOTION,    (uint32_t)motionDraw);
    fusion_acc_ci(FUSION_ACC_REG_CONTROL,   1u);
    volatile uint32_t acc_status;
    volatile uint32_t acc_timeout = 10000000u;
    do {
        acc_status = fusion_acc_ci(FUSION_ACC_REG_STATUS, 0);
        acc_timeout--;
    } while ((acc_status & FUSION_ACC_STATUS_BUSY) && acc_timeout != 0);

    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    printf("nrOfCycles for Fusion: %d %d %d status %d timeout %d\n",
           cycles, stall, idle, acc_status, acc_timeout);

    // we swap sobel pointers and this frame's edges become "previous" for next frame
    volatile uint8_t *temp = sobelPrev;
    sobelPrev = sobelCurr;
    sobelCurr = temp;

    // Swap motion pointers: the buffer we just wrote becomes the display
    // buffer for the next iteration; HDMI's old display buffer is now free
    // for the accelerator to overwrite.
    volatile uint16_t *tempM = motionDisplay;
    motionDisplay = motionDraw;
    motionDraw    = tempM;

    // Wait for captureNext to be fully written before we process it
    waitForNextImage();

    // we swap capture buffers so captureNext is now ready, captureReady is free
    volatile uint8_t *tempC = captureReady;
    captureReady = captureNext;
    captureNext  = tempC;
  }
}

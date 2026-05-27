#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
#include <sobel.h>
#ifdef __OR1300__
#include <cache.h>
#endif

volatile uint8_t  grayscaleA[640*480]; // capture buffer A
volatile uint8_t  grayscaleB[640*480]; // capture buffer B (pipeline: next frame)
volatile uint8_t  sobelA[640*480];     // edge map, buffer A
volatile uint8_t  sobelB[640*480];     // edge map, buffer B
// Double-buffered RGB565 motion output: same color coding as accMotion.v
// (red = new/moving edge, white = stable edge, black = no edge).
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
  volatile uint16_t *motionDraw    = motionB;    // what the CPU writes into

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
  printf("PCLK (kHz) : %d\n", camParams.pixelClockInkHz);
  printf("FPS        : %d\n", camParams.framesPerSecond);

  for (int i = 0; i < 640*480; i++) {
      grayscaleA[i] = 0; grayscaleB[i] = 0;
      sobelA[i]     = 0; sobelB[i]     = 0;
      motionA[i]    = 0; motionB[i]    = 0;
  }

  vga[2] = swap_u32(1); // RGB565 display mode
  vga[3] = swap_u32((uint32_t) motionDisplay);

  int totalPixels = camParams.nrOfPixelsPerLine * camParams.nrOfLinesPerImage;

  // Bootstrap: block until the very first frame is captured.
  takeSingleImageBlocking((uint32_t) captureReady);

  while (1) {
    // Start capturing the next frame into captureNext immediately.
    // This runs in parallel with the Sobel+motion processing below.
    takeSingleImageNonBlocking((uint32_t) captureNext);

    // Point HDMI at the last completed motion result while we compute the new one.
    vga[3] = swap_u32((uint32_t) motionDisplay);

    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7)); // start profiling

    // ── Software Sobel edge detection ────────────────────────────────────────
    edgeDetection(captureReady, sobelCurr, camParams.nrOfPixelsPerLine, camParams.nrOfLinesPerImage, 128);

    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    printf("nrOfCycles for Sobel: %d %d %d\n", cycles, stall, idle);

    // ── Software motion detection → RGB565 output ────────────────────────────
    // Same color coding as accMotion.v:
    //   current edge, previous no edge → red   (0xF800) new/moving edge
    //   current edge, previous edge    → white (0xFFFF) stable edge
    //   no current edge                → black (0x0000)

    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7)); // start profiling

    for (int i = 0; i < totalPixels; i++) {
        if (sobelCurr[i] != 0 && sobelPrev[i] == 0)
            motionDraw[i] = swap_u16(0xF800);
        else if (sobelCurr[i] != 0)
            motionDraw[i] = swap_u16(0xFFFF);
        else
            motionDraw[i] = 0x0000;
    }

    // Swap sobel pointers: this frame's edges become "previous" for next frame.
    volatile uint8_t *temp = sobelPrev;
    sobelPrev = sobelCurr;
    sobelCurr = temp;

    // Swap motion pointers: the buffer we just wrote becomes the display buffer.
    volatile uint16_t *tempM = motionDisplay;
    motionDisplay = motionDraw;
    motionDraw    = tempM;

    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    printf("nrOfCycles for Motion: %d %d %d\n", cycles, stall, idle);

    // Wait for captureNext to be fully written before we process it.
    waitForNextImage();

    // Swap capture buffers: captureNext is now ready, captureReady is free.
    volatile uint8_t *tempC = captureReady;
    captureReady = captureNext;
    captureNext  = tempC;
  }
}

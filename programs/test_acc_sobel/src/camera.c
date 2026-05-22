#include <stdio.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
// #include <floyd_steinberg.h>
#include <sobel.h>
#ifdef __OR1300__
#include <cache.h>
#endif

volatile uint16_t rgb565[640*480];
volatile uint8_t grayscale[640*480];
// volatile uint8_t floyd[640*480];
// volatile int16_t error_array[642<<1];
volatile uint8_t sobelA[640*480];   // edge map, buffer A
volatile uint8_t sobelB[640*480];   // edge map, buffer B
volatile uint8_t motion[640*480];   // motion result, that will be shown in HDMI

int main () {
  volatile int result;
  volatile unsigned int *vga = (unsigned int *) 0X50000020;
  camParameters camParams;
  volatile uint32_t cycles,stall,idle;

  volatile uint8_t *sobelCurr = sobelA;   // where this frame's edges go
  volatile uint8_t *sobelPrev = sobelB;   // last frame's edges

#ifdef __OR1300__
  icache_write_cfg(CACHE_SIZE_8K|CACHE_FOUR_WAY|CACHE_REPLACE_LRU);
  dcache_write_cfg(CACHE_SIZE_8K|CACHE_FOUR_WAY|CACHE_WRITE_BACK|CACHE_REPLACE_PLRU);
  icache_enable(1);
  dcache_enable(1);
#endif
  vga_clear();
  
  printf("Initialising camera (this takes up to 3 seconds)!\n" );
  camParams = initOv7670(VGA);
  printf("Done!\n" );
  printf("NrOfPixels : %d\n", camParams.nrOfPixelsPerLine );
  result = (camParams.nrOfPixelsPerLine <= 320) ? camParams.nrOfPixelsPerLine | 0x80000000 : camParams.nrOfPixelsPerLine;
  vga[0] = swap_u32(result);
  printf("NrOfLines  : %d\n", camParams.nrOfLinesPerImage );
  result =  (camParams.nrOfLinesPerImage <= 240) ? camParams.nrOfLinesPerImage | 0x80000000 : camParams.nrOfLinesPerImage;
  vga[1] = swap_u32(result);
  printf("PCLK (kHz) : %d\n", camParams.pixelClockInkHz );
  printf("FPS        : %d\n", camParams.framesPerSecond );
  for (int i = 0; i < 640*480; i++) { sobelA[i] = 0; sobelB[i] = 0; } // clear buffers

  while(1) {
    vga[2] = swap_u32(2); // 2 to set grayscale
    vga[3] = swap_u32((uint32_t) &motion[0]); // tell HDMI which buffer to display
    takeSingleImageBlocking((uint32_t) &grayscale[0]);
    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7)); // start the Profiling
    
    edgeDetection(grayscale, sobelCurr, camParams.nrOfPixelsPerLine, camParams.nrOfLinesPerImage, 128); // output buffer is sobelCurr
    // stop the profiling and then print the cycles
    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    printf("nrOfCycles: %d %d %d\n", cycles, stall, idle);
    // motion detection we see the difference from the previous frame
    int totalPixels = camParams.nrOfPixelsPerLine * camParams.nrOfLinesPerImage;
    for (int i = 0; i < totalPixels; i++) {
      int diff = (int)sobelCurr[i] - (int)sobelPrev[i];
      if (diff < 0) diff = -diff; // this is commented for new we have to decide if we want to show both edges or only the new one
      motion[i] = (diff > 0) ? 255 : 0;
    }

    // we swap the two pointers
    volatile uint8_t *temp = sobelPrev;
    sobelPrev = sobelCurr;
    sobelCurr = temp;
  }
}
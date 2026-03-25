#include <stdio.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>


int main () {
  volatile uint16_t rgb565[640*480];
  volatile uint8_t grayscale[640*480];
  volatile uint32_t result, cycles,stall,idle;
  volatile unsigned int *vga = (unsigned int *) 0X50000020;
  camParameters camParams;
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

  printf("Test11\n");
  uint32_t * rgb = (uint32_t *) &rgb565[0];
  uint32_t grayPixels;
  vga[2] = swap_u32(2);
  vga[3] = swap_u32((uint32_t) &grayscale[0]);
  while(1) {
    uint32_t * gray = (uint32_t *) &grayscale[0];
    takeSingleImageBlocking((uint32_t) &rgb565[0]);
    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7));
    for (int line = 0; line < camParams.nrOfLinesPerImage; line++) {
      for (int pixel = 0; pixel < camParams.nrOfPixelsPerLine; pixel += 4) {
        // Read and swap each pixel
        uint16_t sw0 = swap_u16(rgb565[line*camParams.nrOfPixelsPerLine+pixel]);
        uint16_t sw1 = swap_u16(rgb565[line*camParams.nrOfPixelsPerLine+pixel+1]);
        uint16_t sw2 = swap_u16(rgb565[line*camParams.nrOfPixelsPerLine+pixel+2]);
        //asm volatile ("l.nop"); // needed to avoid pipeline issue with 4 consecutive swap_u16 calls
        uint16_t sw3 = swap_u16(rgb565[line*camParams.nrOfPixelsPerLine+pixel+3]);
        // Pack 2 swapped pixels into each 32-bit word
        uint32_t inA = (uint32_t)sw0 | ((uint32_t)sw1 << 16);
        uint32_t inB = (uint32_t)sw2 | ((uint32_t)sw3 << 16);
        // Custom instruction converts 4 RGB565 pixels to 4 grayscale bytes in one cycle
        uint32_t gray4;
        asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xC"
                      :[out1]"=r"(gray4)
                      :[in1]"r"(inA),[in2]"r"(inB));
        // Write all 4 grayscale bytes at once
        *((uint32_t*)&grayscale[line*camParams.nrOfPixelsPerLine+pixel]) = gray4;
      }
    }
    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    printf("nrOfCycles: %d %d %d\n", cycles, stall, idle);
  }
}


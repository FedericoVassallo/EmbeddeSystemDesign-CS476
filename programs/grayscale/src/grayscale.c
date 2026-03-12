#include <stdio.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>


int main () {
  volatile uint16_t rgb565[640*480];
  volatile uint8_t grayscale[640*480];
  volatile uint32_t result, cycles,stall,idle;
  // we delare everything before staring the counters
  uint32_t count_reset = 0x700;
  uint32_t count_enable = 0x7;
  // Control = 0x70 disables counter0,1,2 (bits 4,5,6) while reading in the same instruction
  uint32_t control = 0x70;
  uint32_t cid0 = 0, cid1 = 1, cid2 = 2;
  volatile unsigned int *vga = (unsigned int *) 0xB0000020;
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
  uint32_t * rgb = (uint32_t *) &rgb565[0];
  uint32_t grayPixels;
  vga[2] = swap_u32(2);
  vga[3] = swap_u32((uint32_t) &grayscale[0]);
  while(1) {
    uint32_t * gray = (uint32_t *) &grayscale[0];
    takeSingleImageBlocking((uint32_t) &rgb565[0]);
    
    // First thing we rest the counters
    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB" :: [in2]"r"(count_reset));

    // Then we enable the counters
    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB" :: [in2]"r"(count_enable));

    for (int line = 0; line < camParams.nrOfLinesPerImage; line++) {
      for (int pixel = 0; pixel < camParams.nrOfPixelsPerLine; pixel++) {
        uint16_t rgb = swap_u16(rgb565[line*camParams.nrOfPixelsPerLine+pixel]);
        uint32_t red1 = ((rgb >> 11) & 0x1F) << 3;
        uint32_t green1 = ((rgb >> 5) & 0x3F) << 2;
        uint32_t blue1 = (rgb & 0x1F) << 3;
        uint32_t gray = ((red1*54+green1*183+blue1*19) >> 8)&0xFF;
        grayscale[line*camParams.nrOfPixelsPerLine+pixel] = gray;
      }
    }

    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB" : [out1]"=r"(cycles) : [in1]"r"(cid0), [in2]"r"(control));
    // Read the stall cycles from counter1 and disable it
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB" : [out1]"=r"(stall)  : [in1]"r"(cid1), [in2]"r"(control));
    // Read the bus idle cycles from counter2 and disable it
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB" : [out1]"=r"(idle)   : [in1]"r"(cid2), [in2]"r"(control));
    // Print the results 

    printf("Cycles    : %u\n", cycles);
    printf("Stall     : %u\n", stall);
    printf("Bus-idle  : %u\n", idle);
  }
}

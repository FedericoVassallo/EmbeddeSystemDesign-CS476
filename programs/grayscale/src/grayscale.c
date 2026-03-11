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
  result = (camParams.nrOfPixelsPerLine <= 320) ? camParams.nrOfPixelsPerLine | 0X50000000 : camParams.nrOfPixelsPerLine;
  vga[0] = swap_u32(result);
  printf("NrOfLines  : %d\n", camParams.nrOfLinesPerImage );
  result =  (camParams.nrOfLinesPerImage <= 240) ? camParams.nrOfLinesPerImage | 0X50000000 : camParams.nrOfLinesPerImage;
  vga[1] = swap_u32(result);
  printf("PCLK (kHz) : %d\n", camParams.pixelClockInkHz );
  printf("FPS        : %d\n", camParams.framesPerSecond );
  uint32_t * rgb = (uint32_t *) &rgb565[0];
  uint32_t grayPixels;
  vga[2] = swap_u32(2);
  vga[3] = swap_u32((uint32_t) &grayscale[0]);
  while(1) {
    uint32_t * gray = (uint32_t *) &grayscale[0];
    takeSingleImageBlocking((uint32_t) &rgb565[0]); //block the image that is going to be processed until it is ready

    uint32_t control = 0x0707; //At the same time, reset counters 0,1,2 and enable them (3 is skipped because it is the same as 0)
    asm volatile ("l.nios_rrr r0, r0, %[in2], 0X5"::[in2]"r"(control));
    
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

    uint32_t execCycles, stallCycles, busIdleCycles;
    control = 7 << 4;
    uint32_t counterId = 0;

    asm volatile ("l.nios_rrr %[out1], %[in1], %[in2], 0X5":
                  [out1]"=r"(execCycles):
                  [in1]"r"(counterId),
                  [in2]"r"(control));
    
    printf("execCycles=%d\n", execCycles);
    
    counterId = 1;

    asm volatile ("l.nios_rrr %[out1], %[in1], %[in2], 0X5":
                  [out1]"=r"(stallCycles):
                  [in1]"r"(counterId),
                  [in2]"r"(control));

    printf("stallCycles=%d\n", stallCycles);
    counterId = 2;
    
    asm volatile ("l.nios_rrr %[out1], %[in1], %[in2], 0X5":
                  [out1]"=r"(busIdleCycles):
                  [in1]"r"(counterId),
                  [in2]"r"(control));
    
    printf("busIdleCycles=%d\n", busIdleCycles);
    
  }
}

#include <stdio.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>

int main () {
  // CONST FOR THE DMA CI
  const uint32_t writeBit = 1<<9;
  const uint32_t busStartAddress = 1 << 10;
  const uint32_t memoryStartAddress = 2 << 10;
  const uint32_t blockSize = 3 << 10;
  const uint32_t burstSize = 4 << 10;
  const uint32_t statusControl = 5 << 10;
  const uint32_t usedBlocksize = 256;
  const uint32_t usedBurstSize = 15;
  // 
  const int iterations = 599; // we do 599 iterations
  volatile uint16_t rgb565[640*480];
  volatile uint8_t  grayscale[640*480]; 
  volatile uint32_t result, cycles,stall,idle;
  volatile uint32_t bufferA = 0;
  volatile uint32_t bufferB = 256;
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
  uint32_t grayPixels;
  vga[2] = swap_u32(2);
  vga[3] = swap_u32((uint32_t) &grayscale[0]);
  while(1) {
    takeSingleImageBlocking((uint32_t) &rgb565[0]);
    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7));
    uint32_t * rgb = (uint32_t *) &rgb565[0];
    uint32_t * gray = (uint32_t *) &grayscale[0];

    uint32_t polling;
    uint32_t grayPixels;
    uint32_t pixel1, pixel2;

    // ----------------------------------------------
    // ---------------- PHASE 1:   -----------------
    // ----------------------------------------------
    // we write the first 1kbyte of the dmaCi memory with the rgb565 data
    asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(busStartAddress | writeBit),[in2] "r"(rgb)); // we set the bus start address to the address of the rgb565 buffer
    asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(memoryStartAddress | writeBit),[in2] "r"(bufferA)); // we set the memory start address to the address of the first 1kbyte of the dmaCi memory
    asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(blockSize | writeBit),[in2] "r"(usedBlocksize)); // we set the block size to 1024 (512 pixels)
    asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(burstSize | writeBit),[in2] "r"(usedBurstSize)); // we set the burst size to 32 (16 pixels)
    asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(statusControl | writeBit),[in2] "r"(1)); // we start the dma transfer from memory to ciRam by writing 1 to the control register
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(polling):[inA]"r"(statusControl)); 
    } while (polling & 0x1); // we do (polling & 0x1) to wait until the busy bit is set to 0

    // ----------------------------------------------
    // ---------------- PHASE 2:   -----------------
    // ----------------------------------------------
    for (int i = 1; i <= iterations; i++) {
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(busStartAddress | writeBit),[in2] "r"(rgb + (i * 256))); // we set the bus start address to the address of the rgb565 buffer
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(memoryStartAddress | writeBit),[in2] "r"(bufferB)); // we set the memory start address to the address of the first 1kbyte of the dmaCi memory
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(blockSize | writeBit),[in2] "r"(usedBlocksize)); // reset blockSize to 256 words (was changed to 128 by the grayscale DMA)
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(burstSize | writeBit),[in2] "r"(usedBurstSize)); // reset burstSize to 32
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(statusControl | writeBit),[in2] "r"(1)); // we start the dma transfer from memory to ciRam by writing 1 to the control register

     // we we calculate the grayscale values in the buffer that contains already the RGB565-pixels, and we "overwrite" them with the grayscale pixels
     for (int j = 0; j < 256; j += 2) { 
        asm volatile("l.nios_rrr %[out1],%[in1],r0,0x8" :[out1]"=r"(pixel1):[in1] "r"(bufferA + j)); // we read the first 2 pixels from the dmaCi memory in bufferA where we already transferred the first batch of rgb565 data
        asm volatile("l.nios_rrr %[out1],%[in1],r0,0x8" :[out1]"=r"(pixel2):[in1] "r"(bufferA + j + 1)); // we read the first 2 pixels from the dmaCi memory in bufferA where we already transferred the first batch of rgb565 data

        pixel1 = swap_u32(pixel1);
        pixel2 = swap_u32(pixel2);

        asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xC":[out1]"=r"(grayPixels):[in1]"r"(pixel1),[in2]"r"(pixel2)); // we send these 2 pixels to the grayscale custom instruction and we get back 4 grayscale pixels in grayPixels
        asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(bufferA + (j/2) | writeBit), [in2]"r"(swap_u32(grayPixels))); // we write the 4 grayscale pixels back to the dmaCi memory in bufferA
      }

      do { // we check that the DMA-transfer to the bufferB has finished
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(polling):[inA]"r"(statusControl)); 
      } while (polling & 0x1); // we do (polling & 0x1) to wait until the busy bit is set to 0

      // We now transfer the calculated grayscale pixels to the grayscale screen buffer with DMA
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(busStartAddress | writeBit),[in2] "r"(gray + (i - 1)* 128)); // we set the bus start address to the address of the grayscale screen buffer
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(memoryStartAddress | writeBit),[in2] "r"(bufferA)); // we set the memory start address to the address of the first 1kbyte of the dmaCi memory where we have the grayscale pixels
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(blockSize | writeBit),[in2] "r"(usedBlocksize/2)); // we set the block size usedBlocksize/2 because the grayscale pixel occupy 1 byte instead of 2 bytes like the rgb565 pixels
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(statusControl | writeBit),[in2] "r"(2)); // we start the dma transfer from ciRam to memory by writing 2 to the control register

      do { // we check that the DMA-transfer to the grayscale screen buffer has finished
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(polling):[inA]"r"(statusControl)); 
      } while (polling & 0x1); // we do (polling & 0x1) to wait until the busy bit is set to 0

      // we swap the buffers for the next iteration
      uint32_t temp = bufferA;
      bufferA = bufferB;
      bufferB = temp;
    }
    // -------------------------------------------------
    // ---------------- PHASE 3: -----------------
    // --------------------------------------------
    // for the last iterationwe do the same as for the previous iteration, with the exception that we do
    // not initialize a DMA-transfer of 512 RGB565-pixels, as we already have them in our buffer
    // we we calculate the grayscale values in the buffer that contains already the RGB565-pixels, and we "overwrite" them with the grayscale pixels
     for (int j = 0; j < 256; j += 2) { // we do 216 because we can process 4 pixels at a time and we have 512 pixels in total, so we do 512/4 = 128 iterations
        asm volatile("l.nios_rrr %[out1],%[in1],r0,0x8" :[out1]"=r"(pixel1):[in1] "r"(bufferA + j)); // we read the first 2 pixels from the dmaCi memory in bufferA where we already transferred the first batch of rgb565 data
        asm volatile("l.nios_rrr %[out1],%[in1],r0,0x8" :[out1]"=r"(pixel2):[in1] "r"(bufferA + j + 1)); // we read the first 2 pixels from the dmaCi memory in bufferA where we already transferred the first batch of rgb565 data
        asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xC":[out1]"=r"(grayPixels):[in1]"r"(pixel1),[in2]"r"(pixel2)); // we send these 2 pixels to the grayscale custom instruction and we get back 4 grayscale pixels in grayPixels
        asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(bufferA + (j/2) | writeBit), [in2]"r"(grayPixels)); // we write the 4 grayscale pixels back to the dmaCi memory in bufferA
      }

      // We now transfer the calculated grayscale pixels to the grayscale screen buffer with DMA
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(busStartAddress | writeBit),[in2] "r"(gray + iterations* 128)); // we set the bus start address to the address of the grayscale screen buffer
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(memoryStartAddress | writeBit),[in2] "r"(bufferA)); // we set the memory start address to the address of the first 1kbyte of the dmaCi memory where we have the grayscale pixels
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(blockSize | writeBit),[in2] "r"(usedBlocksize/2)); // we set the block size usedBlocksize/2 because the grayscale pixel occupy 1 byte instead of 2 bytes like the rgb565 pixels
      asm volatile("l.nios_rrr r0,%[in1],%[in2],0x8" ::[in1] "r"(statusControl | writeBit),[in2] "r"(2)); // we start the dma transfer from ciRam to memory by writing 2 to the control register

      do { // we check that the DMA-transfer to the grayscale screen buffer has finished
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(polling):[inA]"r"(statusControl)); 
      } while (polling & 0x1); // we do (polling & 0x1) to wait until the busy bit is set to 0

    // change these numbers inside the asm instr even if they are taken from the solution, to make more clear what they mean
    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    printf("nrOfCycles: %d %d %d\n", cycles, stall, idle);
  }
}
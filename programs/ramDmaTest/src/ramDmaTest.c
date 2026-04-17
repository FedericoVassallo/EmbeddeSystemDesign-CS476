#include <stdio.h>
#include <swap.h>

// Direct memory access (opCode 000)
#define WRITE_TO_MEM(addr)    ((1 << 9) | (addr))  // bit 9 = write, bits[8:0] = address
#define READ_FROM_MEM(addr)   (addr)                // bit 9 = 0 (read), bits[8:0] = address

// DMA register access (opCode in bits[12:10], bit 9 = read/write)
#define SET_BUS_START_ADDR    ((1 << 10) | (1 << 9))   // opCode 001, write
#define GET_BUS_START_ADDR    (1 << 10)                 // opCode 001, read
#define SET_MEM_START_ADDR    ((2 << 10) | (1 << 9))   // opCode 010, write
#define GET_MEM_START_ADDR    (2 << 10)                 // opCode 010, read
#define SET_BLOCK_SIZE        ((3 << 10) | (1 << 9))   // opCode 011, write
#define GET_BLOCK_SIZE        (3 << 10)                 // opCode 011, read
#define SET_BURST_SIZE        ((4 << 10) | (1 << 9))   // opCode 100, write
#define GET_BURST_SIZE        (4 << 10)                 // opCode 100, read
#define READ_STATUS           (5 << 10)                 // opCode 101, read
#define WRITE_CONTROL         ((5 << 10) | (1 << 9))   // opCode 101, write

int main() {

    unsigned int result;

    // we write to the memory location 0 the value 0x12345678
    // no output register is needed since we are only writing to memory and not interested in the result of the instruction
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_TO_MEM(0)),[inB]"r"(0x12345678)); 

    // we read from the memory location 0 and write the result to the variable result
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                :[out]"=r"(result):[inA]"r"(READ_FROM_MEM(0))); 

    printf("result: %x expected: %x", result, 0x12345678);

    unsigned int *sdram_address = (unsigned int *) 0x00200000; // choosen an arbitrary address in SDRAM to test 

    // we write 16 values to the SDRAM using the custom instruction, we write to memory location 0,1,2,...15 and we write the value 0xF0000000, 0xF0000001,...0xF000000F
    for (int i = 0; i < 16; i++) {
            sdram_address[i] = i;
    }

    // set bus start address = SDRAM source
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                    ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));

    // set memory start address = 0 (start of CI memory)
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(0));

    // set block size = 16 words
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(16));

    // set burst size = 7 (means 8 words per burst)
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(7));

    // start DMA: write 1 to control register
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(1));

    // poll status register until not busy
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1); // check the busy bit and we stay here until the busy bit is cleared

    // check if there was error, the error bit is bit 1 of the status register, if it is set then we print an error message
    if (result & 0x2) { 
        printf("DMA ERROR\n");
    } else {
        printf("DMA correct\n");
    }

    // we verify that the 16 values were correctly transferred to the CI memory by reading them back using the custom instruction and comparing with the expected value, if there is a mismatch we print an error message
    int errors = 0;
    for (int i = 0; i < 16; i++) {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_FROM_MEM(i)));
        unsigned int expected = swap_u32(i); // we need to swap for the endianess 
        if (result != expected) {
            printf("MISMATCH at %d: got 0x%08X, expected 0x%08X\n", i, result, expected);
            errors++;
        }
    }

    if (errors == 0)
        printf("All 16 words verified OK!\n");
    else
        printf("%d errors found!\n", errors);

    return 0;
}
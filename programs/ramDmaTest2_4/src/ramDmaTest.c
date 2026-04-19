#include <stdio.h>
#include <swap.h>

// Direct memory access (opCode 000)
#define WRITE_TO_MEM(addr)    ((1 << 9) | (addr))
#define READ_FROM_MEM(addr)   (addr)

// DMA register access
#define SET_BUS_START_ADDR    ((1 << 10) | (1 << 9))
#define GET_BUS_START_ADDR    (1 << 10)
#define SET_MEM_START_ADDR    ((2 << 10) | (1 << 9))
#define GET_MEM_START_ADDR    (2 << 10)
#define SET_BLOCK_SIZE        ((3 << 10) | (1 << 9))
#define GET_BLOCK_SIZE        (3 << 10)
#define SET_BURST_SIZE        ((4 << 10) | (1 << 9))
#define GET_BURST_SIZE        (4 << 10)
#define READ_STATUS           (5 << 10)
#define WRITE_CONTROL         ((5 << 10) | (1 << 9))

// Control register values
#define DMA_FROM_BUS     1   // bus -> CI memory
#define DMA_TO_BUS       2   // CI memory -> bus
#define DMA_INVALID      3   // should NOT start DMA

int main() {

    printf("RAM DMA Test 2.4: non-multiple of burst and single-word transfers\n");
    unsigned int result;

    printf("=== DMA from CI-memory to SDRAM tests ===\n");

    //////////////////// TEST 1 ////////////////////
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_TO_MEM(0)),[inB]"r"(0x12345678)); 

    // we read from the memory location 0 and write the result to the variable result
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                :[out]"=r"(result):[inA]"r"(READ_FROM_MEM(0))); 

    printf("result: %x      expected: %x\n", result, 0x12345678);
    if (result == 0x12345678) {
        printf("Test 1 PASSED\n");
    } else {
        printf("Test 1 FAILED\n");
    }

    // TEST 2 
    printf("\nTest 2: Block multiple of burst (8 words, burst 8)\n");
    unsigned int *sdram_address = (unsigned int *) 0x00300000; // choosen an arbitrary address in SDRAM to test

    for (int i = 0; i < 8; i++) {
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_TO_MEM(i)), [inB]"r"(0x00000001));
    }

    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR), [inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nop 0");
    printf("Bus start address set\n");
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR), [inB]"r"(0));
    asm volatile ("l.nop 0");
    printf("Memory start address set\n");
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE), [inB]"r"(8));
    asm volatile ("l.nop 0");
    printf("Block size set\n");
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE), [inB]"r"(7));
    asm volatile ("l.nop 0");
    printf("Burst size set\n");

    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL), [inB]"r"(2));
    asm volatile ("l.nop 0");
    printf("Control register set\n");

    printf("Waiting for DMA transfer to complete...\n");

    int timeout = 5000000;
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
        timeout--;
    } while ((result & 0x1) && timeout > 0);

    if (timeout <= 0) { printf("TIMEOUT\n"); return 1; }

    int errors = 0;
    for (int i = 0; i < 8; i++) {
        if (sdram_address[i] != (0x00000001)) {
            printf("MISMATCH [%d]: got 0x%08X, expected 0x%08X\n",
                   i, sdram_address[i], 0x00000001);
            errors++;
        }
    }
    printf(errors == 0 ? "Test 2 PASSED\n" : "Test 2 FAILED\n");




    return 0;
}
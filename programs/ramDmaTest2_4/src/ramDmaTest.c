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

// Control register values:
//   1 = DMA from bus -> CI memory (read)
//   2 = DMA from CI memory -> bus (write)
//   3 = invalid (should NOT start)
#define DMA_START_READ   1
#define DMA_START_WRITE  2

int main() {
    unsigned int result;
    unsigned int *sdram_address = (unsigned int *) 0x00200000;

    //////////////////// TEST 1 ////////////////////
    printf("\nTest 1: DMA write (CI memory -> SDRAM), single burst of 8 words\n");
    // Fill CI memory with known pattern at addresses 0..7
    for (int i = 0; i < 8; i++) {
        unsigned int val = swap_u32(0xBEEF0000 | i);  // byte-swap so SDRAM sees 0xBEEF000i
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_TO_MEM(i)),[inB]"r"(val));
    }
    // Clear SDRAM destination first so we know the DMA actually wrote there
    for (int i = 0; i < 8; i++) {
        sdram_address[i] = 0;
    }
    // Configure DMA: CI mem addr 0..7 -> SDRAM
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(8));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(7));
    // Start DMA write (control = 2)
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_START_WRITE));
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1);
    if (result & 0x2) {
        printf("DMA ERROR (error flag set)\n");
    } else {
        printf("DMA correct (no error flag set)\n");
    }
    int errors = 0;
    for (int i = 0; i < 8; i++) {
        unsigned int expected = 0xBEEF0000 | i;
        if (sdram_address[i] != expected) {
            printf("MISMATCH at %d: got 0x%08X, expected 0x%08X\n", i, sdram_address[i], expected);
            errors++;
        }
    }
    if (errors == 0)
        printf("Test 1 PASSED: All 8 words transferred correctly\n");
    else
        printf("Test 1 FAILED: %d errors\n", errors);

    //////////////////// TEST 2 ////////////////////
    printf("\nTest 2: DMA write, block size not multiple of burst size (10 words, burst 8)\n");
    // Pattern in CI memory at addresses 20..29
    for (int i = 0; i < 10; i++) {
        unsigned int val = swap_u32(0xA0000000 | i);
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_TO_MEM(20 + i)),[inB]"r"(val));
    }
    // Clear SDRAM target
    for (int i = 0; i < 10; i++) {
        sdram_address[i] = 0;
    }
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(20));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(10));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(7));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_START_WRITE));
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1);
    if (result & 0x2) {
        printf("DMA ERROR (error flag set)\n");
    }
    errors = 0;
    for (int i = 0; i < 10; i++) {
        unsigned int expected = 0xA0000000 | i;
        if (sdram_address[i] != expected) {
            printf("MISMATCH at %d: got 0x%08X, expected 0x%08X\n", i, sdram_address[i], expected);
            errors++;
        }
    }
    if (errors == 0)
        printf("Test 2 PASSED (multi-burst write)\n");
    else
        printf("Test 2 FAILED: %d errors\n", errors);

    //////////////////// TEST 3 ////////////////////
    printf("\nTest 3: DMA write, single word transfer\n");
    unsigned int val = swap_u32(0xCAFE0001);
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_TO_MEM(50)),[inB]"r"(val));
    sdram_address[0] = 0;  // clear target
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(50));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(1));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_START_WRITE));
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1);
    if (sdram_address[0] == 0xCAFE0001)
        printf("Test 3 PASSED (single word write)\n");
    else
        printf("Test 3 FAILED: got 0x%08X, expected 0xCAFE0001\n", sdram_address[0]);

    //////////////////// TEST 4 ////////////////////
    printf("\nTest 4: Round-trip (CI->SDRAM->CI)\n");
    // Write pattern to CI mem at 100..115
    for (int i = 0; i < 16; i++) {
        unsigned int v = swap_u32(0x5A5A0000 | i);
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_TO_MEM(100 + i)),[inB]"r"(v));
    }
    // Clear SDRAM and destination CI memory
    for (int i = 0; i < 16; i++) sdram_address[i] = 0;
    for (int i = 0; i < 16; i++) {
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_TO_MEM(300 + i)),[inB]"r"(0));
    }
    // Step A: DMA write CI[100..115] -> SDRAM
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(100));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(16));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(7));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_START_WRITE));
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1);
    // Step B: DMA read SDRAM -> CI[300..315]
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(300));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_START_READ));
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1);
    // Compare CI[100..115] with CI[300..315]
    errors = 0;
    for (int i = 0; i < 16; i++) {
        unsigned int src, dst;
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(src):[inA]"r"(READ_FROM_MEM(100 + i)));
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(dst):[inA]"r"(READ_FROM_MEM(300 + i)));
        if (src != dst) {
            printf("MISMATCH at %d: src=0x%08X dst=0x%08X\n", i, src, dst);
            errors++;
        }
    }
    if (errors == 0)
        printf("Test 4 PASSED (round-trip)\n");
    else
        printf("Test 4 FAILED: %d errors\n", errors);

    //////////////////// TEST 5 ////////////////////
    printf("\nTest 5: Control register value 3 should be rejected\n");
    // Set up a normal config, then try to start with control=3
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(4));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(3));
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(READ_STATUS));
    if ((result & 0x1) == 0)
        printf("Test 5 PASSED: DMA stayed idle on control=3\n");
    else
        printf("Test 5 FAILED: DMA became busy on control=3\n");

    //////////////////// TEST 6 ////////////////////
    printf("\nTest 6: DMA write with block_size=0 (no transfer)\n");
    sdram_address[50] = 0xDEADBEEF;  // guard value
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)&sdram_address[50]));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_START_WRITE));
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(READ_STATUS));
    if (result & 0x1) {
        printf("Test 6 FAILED: DMA should not be busy with block_size=0\n");
    } else if (sdram_address[50] == 0xDEADBEEF) {
        printf("Test 6 PASSED: guard value preserved\n");
    } else {
        printf("Test 6 FAILED: SDRAM modified (got 0x%08X)\n", sdram_address[50]);
    }

    return 0;
}
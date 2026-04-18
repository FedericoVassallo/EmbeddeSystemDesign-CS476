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
    unsigned int result;
    unsigned int *sdram_address = (unsigned int *) 0x00200000;

    printf("=== DMA from CI-memory to SDRAM tests ===\n");

    //////////////////// TEST 1 ////////////////////
    int timeout, errors;

    //////////////////// TEST 2 ////////////////////
    printf("\nTest 2: Block not multiple of burst (10 words, burst 8)\n");

    for (int i = 0; i < 10; i++) {
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_TO_MEM(i)), [inB]"r"(swap_u32(0xB0000000 | i)));
    }
    for (int i = 0; i < 10; i++) sdram_address[i] = 0xDEADBEEF;

    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR), [inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR), [inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE), [inB]"r"(10));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE), [inB]"r"(7));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL), [inB]"r"(2));

    timeout = 5000000;
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
        timeout--;
    } while ((result & 0x1) && timeout > 0);

    if (timeout <= 0) { printf("TIMEOUT\n"); return 1; }

    errors = 0;
    for (int i = 0; i < 10; i++) {
        if (sdram_address[i] != (0xB0000000 | i)) {
            printf("MISMATCH [%d]: got 0x%08X, expected 0x%08X\n",
                   i, sdram_address[i], 0xB0000000 | i);
            errors++;
        }
    }
    printf(errors == 0 ? "Test 2 PASSED\n" : "Test 2 FAILED\n");

    //////////////////// TEST 3 ////////////////////
    printf("\nTest 3: Single word\n");

    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_TO_MEM(0)), [inB]"r"(swap_u32(0xABCD1234)));
    sdram_address[0] = 0xDEADBEEF;

    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR), [inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR), [inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE), [inB]"r"(1));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE), [inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL), [inB]"r"(2));

    timeout = 5000000;
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
        timeout--;
    } while ((result & 0x1) && timeout > 0);

    if (timeout <= 0) { printf("TIMEOUT\n"); return 1; }
    printf(sdram_address[0] == 0xABCD1234 ? "Test 3 PASSED\n"
           : "Test 3 FAILED: got 0x%08X\n", sdram_address[0]);

    //////////////////// TEST 4 ////////////////////
    printf("\nTest 4: control=3 must be ignored\n");

    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_TO_MEM(0)), [inB]"r"(swap_u32(0x99999999)));
    sdram_address[0] = 0x00000000;

    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR), [inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR), [inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE), [inB]"r"(4));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE), [inB]"r"(3));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL), [inB]"r"(3));  // invalid - must be ignored

    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(READ_STATUS));

    if ((result & 0x1) == 0 && sdram_address[0] == 0x00000000)
        printf("Test 4 PASSED: control=3 ignored\n");
    else
        printf("Test 4 FAILED: status=0x%X, sdram[0]=0x%08X\n", result, sdram_address[0]);

    //////////////////// TEST 5 ////////////////////
    printf("\nTest 5: Round-trip SDRAM->CI->SDRAM\n");

    unsigned int *sdram_src = sdram_address;
    unsigned int *sdram_dst = sdram_address + 200;

    for (int i = 0; i < 8; i++) sdram_src[i] = 0x55AA0000 | i;
    for (int i = 0; i < 8; i++) sdram_dst[i] = 0xDEADBEEF;

    // DMA: SDRAM -> CI (control=1)
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR), [inB]"r"((unsigned int)sdram_src));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR), [inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE), [inB]"r"(8));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE), [inB]"r"(7));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL), [inB]"r"(1));

    timeout = 5000000;
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
        timeout--;
    } while ((result & 0x1) && timeout > 0);
    if (timeout <= 0) { printf("TIMEOUT on first DMA\n"); return 1; }

    // DMA: CI -> SDRAM (control=2)
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR), [inB]"r"((unsigned int)sdram_dst));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR), [inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE), [inB]"r"(8));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE), [inB]"r"(7));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL), [inB]"r"(2));

    timeout = 5000000;
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
        timeout--;
    } while ((result & 0x1) && timeout > 0);
    if (timeout <= 0) { printf("TIMEOUT on second DMA\n"); return 1; }

    errors = 0;
    for (int i = 0; i < 8; i++) {
        if (sdram_dst[i] != (0x55AA0000 | i)) {
            printf("MISMATCH [%d]: got 0x%08X, expected 0x%08X\n",
                   i, sdram_dst[i], 0x55AA0000 | i);
            errors++;
        }
    }
    printf(errors == 0 ? "Test 5 PASSED: round-trip OK\n"
                       : "Test 5 FAILED: %d errors\n", errors);

    return 0;
}
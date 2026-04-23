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

// Control register values
#define DMA_FROM_BUS  1   // bus -> CI memory (read side)
#define DMA_TO_BUS    2   // CI memory -> bus (this file's focus)

int main() {
    unsigned int result;

    //////////////////// TEST 1 ////////////////////
    printf("\nTest 1: Writing and reading a value to/from CI-attached memory using custom instruction\n");
    // sanity check that the CI-memory direct path still works (this is the same as
    // Test 1 in the read-side file: it does not exercise the DMA at all, but if it
    // fails everything below is meaningless)
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_TO_MEM(0)),[inB]"r"(0x12345678));
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                :[out]"=r"(result):[inA]"r"(READ_FROM_MEM(0)));
    printf("result: %x      expected: %x\n", result, 0x12345678);
    if (result == 0x12345678) {
        printf("Test 1 PASSED\n");
    } else {
        printf("Test 1 FAILED\n");
    }

    volatile unsigned int *sdram_address = (volatile unsigned int *) 0x00200000; // arbitrary SDRAM destination

    //////////////////// TEST 2 ////////////////////
    printf("\nTest 2: Using custom instruction to perform a DMA transfer from CI-attached memory to SDRAM\n");
    // we write 16 known values directly to CI memory using the custom instruction;
    // the DMA will then push them out to SDRAM, and we'll check the SDRAM side.
    // note: the DMA puts on the bus exactly what is in CI memory, so we must
    // pre-swap each value here so that what lands in SDRAM (which the CPU reads
    // big-endian-style through swap_u32) comes out as the expected incrementing sequence.
    for (int i = 0; i < 16; i++) {
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_TO_MEM(i)),[inB]"r"(swap_u32(5 + i)));
    }
    // wipe SDRAM destination so we can tell the difference between "DMA wrote it"
    // and "value happened to already be there"
    for (int i = 0; i < 16; i++) {
        sdram_address[i] = 0xDEADBEEF;
    }
    // set bus start address = SDRAM destination
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    // set memory start address = 0 (start of CI memory)
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(0));
    // set block size = 16 words
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(16));
    // set burst size = 7 (means 8 words per burst, so 2 bursts in total)
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(7));
    // start DMA: write 2 to control register (CI memory -> bus)
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_TO_BUS));
    // poll status register until not busy
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1);
    if (result & 0x2) {
        printf("DMA ERROR (error flag set)\n");
    } else {
        printf("DMA correct (no error flag set)\n");
    }
    // verify the SDRAM destination word-by-word; expected value starts at 5
    int errors = 0;
    for (int i = 0; i < 16; i++) {
        unsigned int expected = 5 + i;
        if (sdram_address[i] != expected) {
            printf("MISMATCH at %d: got 0x%08X, expected 0x%08X\n", i, sdram_address[i], expected);
            errors++;
        }
    }
    if (errors == 0)
        printf("Test 2 PASSED: All 16 words verified\n");
    else
        printf("Test 2 FAILED: %d errors found!\n", errors);

    //////////////////// TEST 3 ////////////////////
    printf("\nTest 3: Verify DMA config registers read-back after DMA transfer\n");
    // same sanity check as on the read side: the registers should still hold the
    // values we last programmed, even after a transfer completes
    int test3_errors = 0;
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(GET_BURST_SIZE));
    if (result != 7) {
        printf("Burst size: got %d, expected 7\n", result);
        test3_errors++;
    }
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(GET_BLOCK_SIZE));
    if (result != 16) {
        printf("Block size: got %d, expected 16\n", result);
        test3_errors++;
    }
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(GET_BUS_START_ADDR));
    if (result != (unsigned int)sdram_address) {
        printf("Bus addr: got 0x%08X, expected 0x%08X\n", result, (unsigned int)sdram_address);
        test3_errors++;
    }
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(GET_MEM_START_ADDR));
    if (result != 0) {
        printf("Mem addr: got %d, expected 0\n", result);
        test3_errors++;
    }
    if (test3_errors == 0)
        printf("Test 3 PASSED\n");
    else
        printf("Test 3 FAILED: %d errors\n", test3_errors);

    //////////////////// TEST 4 ////////////////////
    printf("\nTest 4: Block size not multiple of burst size (10 words, burst 8)\n");
    // here we exercise the multi-burst path with a leftover partial burst
    // (one full burst of 8 words + one partial burst of 2 words)
    for (int i = 0; i < 10; i++) {
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_TO_MEM(i)),[inB]"r"(swap_u32(0xA0000000 | i)));
    }
    for (int i = 0; i < 10; i++) {
        sdram_address[i] = 0xDEADBEEF;
    }
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(10));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(7));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_TO_BUS));
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
        printf("Test 4 PASSED\n");
    else
        printf("Test 4 FAILED: %d errors\n", errors);

    //////////////////// TEST 5 ////////////////////
    printf("\nTest 5: Single word DMA transfer\n");
    // single-word transfers exercise the corner case where block_size = 1 and
    // burst_size = 0; this is the simplest possible DMA but shares the same
    // END_TRANSACTION code path as multi-word bursts
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_TO_MEM(100)),[inB]"r"(swap_u32(0x10101010)));
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8" :[out]"=r"(result):[inA]"r"(READ_FROM_MEM(100))); 
    sdram_address[0] = 0xDEADBEEF;
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(100)); // CI mem source addr 100
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(1));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_TO_BUS));
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1);
    if (result & 0x2) {
        printf("DMA ERROR (error flag set)\n");
    }

    if (sdram_address[0] == 0x10101010)
        printf("Test 5 PASSED\n");
    else
        printf("Test 5 FAILED: got 0x%08X, expected 0x%08X\n", sdram_address[0], 0x10101010);

    //////////////////// TEST 6 ////////////////////
    printf("\nTest 6: Block size = 0 (no transfer should happen)\n");
    // pre-fill SDRAM with a known value; after a block_size=0 DMA, it should
    // remain untouched and the controller should never go busy
    sdram_address[0] = 0x11111111;
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_TO_BUS));
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(READ_STATUS));
    if (result & 0x1) {
        printf("Test 6 FAILED: DMA should not be busy with block_size=0\n");
    }
    if (sdram_address[0] == 0x11111111)
        printf("Test 6 PASSED\n");
    else
        printf("Test 6 FAILED: SDRAM[0] got 0x%08X, expected 0x11111111\n", sdram_address[0]);

    //////////////////// TEST 7 ////////////////////
    printf("\nTest 7: Two back-to-back DMA transfers (verify status_busy clears between transfers)\n");
    // this catches a class of bugs where status_busy never clears after the first
    // transfer, or where state from the first transfer leaks into the second
    for (int round = 0; round < 2; round++) {
        unsigned int marker = 0xB0000000 | round;
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_TO_MEM(0)),[inB]"r"(swap_u32(marker)));
        sdram_address[round] = 0xDEADBEEF;
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)&sdram_address[round]));
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(0));
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(1));
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(0));
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_TO_BUS));
        do {
            asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                         :[out]"=r"(result):[inA]"r"(READ_STATUS));
        } while (result & 0x1);
    }
    int test7_errors = 0;
    for (int round = 0; round < 2; round++) {
        unsigned int expected = 0xB0000000 | round;
        if (sdram_address[round] != expected) {
            printf("MISMATCH round %d: got 0x%08X, expected 0x%08X\n", round, sdram_address[round], expected);
            test7_errors++;
        }
    }
    if (test7_errors == 0)
        printf("Test 7 PASSED\n");
    else
        printf("Test 7 FAILED: %d errors\n", test7_errors);

    //////////////////// TEST 8 ////////////////////
    printf("\nTest 8: Round-trip CI -> SDRAM -> CI (write 32 words out then DMA back)\n");
    // writes a pattern out via DMA, reads it back via DMA into a different CI
    // memory region, and checks the round-trip preserves every word. This is the
    // strongest end-to-end test: it exercises both 2.3 and 2.4 paths in one go.
    for (int i = 0; i < 32; i++) {
        asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                     ::[inA]"r"(WRITE_TO_MEM(i)),[inB]"r"(swap_u32(0xC0000000 | i)));
    }
    for (int i = 0; i < 32; i++) {
        sdram_address[i] = 0xDEADBEEF;
    }
    // CI mem -> SDRAM
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(32));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(7));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_TO_BUS));
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1);
    // SDRAM -> CI mem (into a different region of CI memory, addresses 100..131)
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(100));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(32));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(7));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(DMA_FROM_BUS));
    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1);
    int test8_errors = 0;
    for (int i = 0; i < 32; i++) {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_FROM_MEM(100 + i)));
        unsigned int expected = swap_u32(0xC0000000 | i);
        if (result != expected) {
            printf("MISMATCH at %d: got 0x%08X, expected 0x%08X\n", i, result, expected);
            test8_errors++;
        }
    }
    if (test8_errors == 0)
        printf("Test 8 PASSED\n");
    else
        printf("Test 8 FAILED: %d errors\n", test8_errors);

    return 0;
}
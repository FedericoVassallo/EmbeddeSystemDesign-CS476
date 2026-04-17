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

    //////////////////// TEST 1 ////////////////////
    printf("\nTest 1: Writing and reading a value to/from CI-attached memory using custom instruction\n");

    // we write to the memory location 0 the value 0x12345678
    // no output register is needed since we are only writing to memory and not interested in the result of the instruction
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

    unsigned int *sdram_address = (unsigned int *) 0x00200000; // choosen an arbitrary address in SDRAM to test 

    //////////////////// TEST 2 ////////////////////
    printf("\nTest 2: Using custom instruction to perform a DMA transfer from SDRAM to CI-attached memory\n");

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
        printf("DMA ERROR (error flag set)\n");
    } else {
        printf("DMA correct (no error flag set)\n");
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
        printf("Test 2 PASSED: All 16 words verified\n");
    else
        printf("Test 2 FAILED: %d errors found!\n", errors);


    //////////////////// TEST 3 ////////////////////
    printf("\nTest 3: Verify DMA config registers read-back after DMA transfer\n");
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

    for (int i = 0; i < 10; i++) {
        sdram_address[i] = 0xA0000000 | i;
    }

    // here is the same as in the test 2 but we set block size to 10 (not a multiple of burst size 8) to test that the DMA correctly handles the last partial burst
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(10));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(7));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(1));

    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1);

    if (result & 0x2) {
        printf("DMA ERROR (error flag set)\n");
    }

    errors = 0;
    for (int i = 0; i < 10; i++) {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_FROM_MEM(i)));
        unsigned int expected = swap_u32(0xA0000000 | i);
        if (result != expected) {
            printf("MISMATCH at %d: got 0x%08X, expected 0x%08X\n", i, result, expected);
            errors++;
        }
    }

    if (errors == 0)
        printf("Test 4 PASSED\n");
    else
        printf("Test 4 FAILED: %d errors\n", errors);

    //////////////////// TEST 5 ////////////////////
    printf("\nTest 5: Single word DMA transfer\n");

    sdram_address[0] = 0x10101010; // test value 

    // here we set block size to 1 to test that the DMA can also handle single word transfers
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(100)); //we set the memory start address to 100 
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(1)); // set the block size to 1 for single word transfer
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(0)); // set the burst size to 0 (1 word) since we are only transferring 1 word
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(1)); // start the DMA transfer

    do {
        asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                     :[out]"=r"(result):[inA]"r"(READ_STATUS));
    } while (result & 0x1); // wait until DMA is done

    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(READ_FROM_MEM(100))); // we read from the CI memory address 100 where we expected the value to be transferred
    unsigned int expected5 = swap_u32(0x10101010);

    if (result == expected5)
        printf("Test 5 PASSED\n");
    else
        printf("Test 5 FAILED: got 0x%08X, expected 0x%08X\n", result, expected5);

    //////////////////// TEST 6 ////////////////////
    printf("\nTest 6: Block size = 0 (no transfer should happen)\n");

    // write the known value 0x11111111 to CI mem addr 200
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_TO_MEM(200)),[inB]"r"(0x11111111));

    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BUS_START_ADDR),[inB]"r"((unsigned int)sdram_address));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_MEM_START_ADDR),[inB]"r"(200)); // we set the mem start addr to 200 where we wrote the known value to test that it is not overwritten
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BLOCK_SIZE),[inB]"r"(0)); // set block size to 0, this should result in no transfer and the DMA should immediately be idle
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(SET_BURST_SIZE),[inB]"r"(0));
    asm volatile ("l.nios_rrr r0,%[inA],%[inB],0x8"
                 ::[inA]"r"(WRITE_CONTROL),[inB]"r"(1)); // start the DMA transfer

    // status should already be idle since block_size = 0
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(READ_STATUS));
    if (result & 0x1) {
        printf("Test 6 FAILED: DMA should not be busy with block_size=0\n");
    }

    // CI mem addr 200 should still be 0x11111111 (non modified)
    asm volatile ("l.nios_rrr %[out],%[inA],r0,0x8"
                 :[out]"=r"(result):[inA]"r"(READ_FROM_MEM(200)));
    if (result == 0x11111111)
        printf("Test 6 PASSED\n");
    else
        printf("Test 6 FAILED: got 0x%08X, expected 0x11111111\n", result);

    return 0;
}
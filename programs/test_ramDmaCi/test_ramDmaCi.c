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
asm volatile ("l.nios_rrr r0,[inA],%[inB],0x8"
            :[inA]"r"(WRITE_TO_MEM(0)),[inB]"r"(0x12345678)); 

// we read from the memory location 0 and write the result to the variable result
asm volatile ("l.nios_rrr %[out],[inA],r0,0x8"
            :[out]"=r"(result):[inA]"r"(READ_FROM_MEM(0)); 

printf("result: %x\n expected: %x", result, 0x12345678);

unsigned int *sdram_address = (unsigned int *) 0x002000000; // choosen an arbitrary address in SDRAM to test 

// we write 16 values to the SDRAM using the custom instruction, we write to memory location 0,1,2,...15 and we write the value 0xF0000000, 0xF0000001,...0xF000000F
for (int i = 0; i < 16; i++) {
        sdram_address[i] = 0xF0000000 | i;
}

asm volatile ("l.nios_rrr r0,%[inA],[inB],0x8"
            :[inA]"r"(SET_BUS_START_ADDR),[inB]"r"(0)); 



}
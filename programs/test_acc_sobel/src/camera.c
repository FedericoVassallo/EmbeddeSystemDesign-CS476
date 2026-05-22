#include <stdint.h>
#include <stdio.h>

#define SOBEL_ACC_REG_STATUS   0u
#define SOBEL_ACC_REG_SRC      1u
#define SOBEL_ACC_REG_DST      2u
#define SOBEL_ACC_REG_CONTROL  3u
#define SOBEL_ACC_REG_READ_SRC 4u
#define SOBEL_ACC_REG_READ_DST 5u

#define SOBEL_ACC_STATUS_DONE  0x1u
#define SOBEL_ACC_STATUS_BUSY  0x2u
#define SOBEL_ACC_STATUS_ERROR 0x4u

// Add the forward declaration here
void test_sobel_accelerator_ci(void);

int main(void)
{
    test_sobel_accelerator_ci();

    while (1) {
    }

    return 0;
}

static inline uint32_t sobel_acc_ci(uint32_t a, uint32_t b)
{
    uint32_t r;

    asm volatile (
        "l.nios_rrr %[out],%[inA],%[inB],0xE"
        : [out] "=r" (r)
        : [inA] "r" (a), [inB] "r" (b)
    );

    return r;
}

void test_sobel_accelerator_ci(void)
{
    uint32_t src_addr = 0x00100000u;
    uint32_t dst_addr = 0x00200000u;

    uint32_t status;
    uint32_t read_src;
    uint32_t read_dst;
    uint32_t timeout;

    printf("Testing Sobel accelerator CI 0x0E...\n");

    sobel_acc_ci(SOBEL_ACC_REG_SRC, src_addr);
    sobel_acc_ci(SOBEL_ACC_REG_DST, dst_addr);

    read_src = sobel_acc_ci(SOBEL_ACC_REG_READ_SRC, 0);
    read_dst = sobel_acc_ci(SOBEL_ACC_REG_READ_DST, 0);

    printf("SRC written: 0x%08lx, SRC read: 0x%08lx\n",
           (unsigned long)src_addr,
           (unsigned long)read_src);

    printf("DST written: 0x%08lx, DST read: 0x%08lx\n",
           (unsigned long)dst_addr,
           (unsigned long)read_dst);

    if (read_src != src_addr) {
        printf("ERROR: source address readback failed\n");
        return;
    }

    if (read_dst != dst_addr) {
        printf("ERROR: destination address readback failed\n");
        return;
    }

    status = sobel_acc_ci(SOBEL_ACC_REG_STATUS, 0);
    printf("Initial status: 0x%08lx\n", (unsigned long)status);

    sobel_acc_ci(SOBEL_ACC_REG_CONTROL, 1);

    timeout = 1000000u;

    do {
        status = sobel_acc_ci(SOBEL_ACC_REG_STATUS, 0);
        timeout--;
    } while ((status & SOBEL_ACC_STATUS_BUSY) && timeout != 0);

    printf("Final status: 0x%08lx\n", (unsigned long)status);

    if (timeout == 0) {
        printf("ERROR: accelerator timeout\n");
        return;
    }

    if (status & SOBEL_ACC_STATUS_ERROR) {
        printf("ERROR: accelerator error bit set\n");
        return;
    }

    if (!(status & SOBEL_ACC_STATUS_DONE)) {
        printf("ERROR: done bit not set\n");
        return;
    }

    printf("Sobel accelerator CI test passed\n");
}
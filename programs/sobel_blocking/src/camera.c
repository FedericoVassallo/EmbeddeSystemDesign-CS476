#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
#ifdef __OR1300__
#include <cache.h>
#endif

// ── Sobel hardware accelerator (CI 0x0E) ─────────────────────────────────────
#define SOBEL_ACC_REG_STATUS   0u   // read: {error, busy, done}
#define SOBEL_ACC_REG_SRC      1u   // write: source grayscale frame address
#define SOBEL_ACC_REG_DST      2u   // write: destination edge-map address
#define SOBEL_ACC_REG_CONTROL  3u   // write: bit 0 = start
#define SOBEL_ACC_STATUS_BUSY  0x2u

// ── Motion hardware accelerator (CI 0x0F) ────────────────────────────────────
#define MOTION_ACC_REG_STATUS  0u   // read: {error, busy, done}
#define MOTION_ACC_REG_SRCA    1u   // write: source A address (current edge frame)
#define MOTION_ACC_REG_SRCB    2u   // write: source B address (previous edge frame)
#define MOTION_ACC_REG_DST     3u   // write: destination address (motion buffer)
#define MOTION_ACC_REG_CONTROL 4u   // write: bit 0 = start
#define MOTION_ACC_STATUS_BUSY  0x2u

static inline uint32_t motion_acc_ci(uint32_t a, uint32_t b) {
    uint32_t r;
    asm volatile("l.nios_rrr %[out],%[inA],%[inB],0xF"
                 : [out]"=r"(r)
                 : [inA]"r"(a), [inB]"r"(b));
    return r;
}

static inline uint32_t sobel_acc_ci(uint32_t a, uint32_t b)
{
    uint32_t r;
    asm volatile("l.nios_rrr %[out],%[inA],%[inB],0xE"
                 : [out]"=r"(r)
                 : [inA]"r"(a), [inB]"r"(b));
    return r;
}

volatile uint16_t rgb565[640*480];
volatile uint8_t  grayscale[640*480];
volatile uint8_t  sobelA[640*480];    // edge map, buffer A
volatile uint8_t  sobelB[640*480];    // edge map, buffer B
// Double-buffered motion output: HDMI displays one buffer while the
// accelerator writes the other, then we ping-pong. This prevents the
// visible tearing/flicker that happens when HDMI scans-out a buffer that
// the Motion accelerator is concurrently writing to.
volatile uint16_t motionA[640*480];
volatile uint16_t motionB[640*480];

int main () {
  volatile int result;
  volatile unsigned int *vga = (unsigned int *) 0X50000020;
  camParameters camParams;
  volatile uint32_t cycles, stall, idle;
  volatile uint8_t *sobelCurr     = sobelA;    // this frame's edges
  volatile uint8_t *sobelPrev     = sobelB;    // last frame's edges
  volatile uint16_t *motionDisplay = motionA;   // what HDMI is currently showing
  volatile uint16_t *motionDraw    = motionB;   // what the accelerator writes into
#ifdef __OR1300__
  icache_write_cfg(CACHE_SIZE_8K|CACHE_FOUR_WAY|CACHE_REPLACE_LRU);
  dcache_write_cfg(CACHE_SIZE_8K|CACHE_FOUR_WAY|CACHE_WRITE_BACK|CACHE_REPLACE_PLRU);
  icache_enable(1);
  dcache_enable(1);
#endif
  vga_clear();

  printf("Initialising camera (this takes up to 3 seconds)!\n");
  camParams = initOv7670(VGA);
  printf("Done!\n");
  printf("NrOfPixels : %d\n", camParams.nrOfPixelsPerLine);
  result = (camParams.nrOfPixelsPerLine <= 320) ? camParams.nrOfPixelsPerLine | 0x80000000 : camParams.nrOfPixelsPerLine;
  vga[0] = swap_u32(result);
  printf("NrOfLines  : %d\n", camParams.nrOfLinesPerImage);
  result = (camParams.nrOfLinesPerImage <= 240) ? camParams.nrOfLinesPerImage | 0x80000000 : camParams.nrOfLinesPerImage;
  vga[1] = swap_u32(result);
  printf("PCLK (kHz) : %d\n", camParams.pixelClockInkHz );
  printf("FPS        : %d\n", camParams.framesPerSecond );

  // Clear all buffers so the very first displayed frame is black rather
  // than uninitialised SDRAM.
  for (int i = 0; i < 640*480; i++) {
      sobelA[i]  = 0; sobelB[i]  = 0;
      motionA[i] = 0; motionB[i] = 0;
  }

  while (1) {
    vga[2] = swap_u32(1); // 1 → RGB565 display mode
    // Point HDMI at the COMPLETED buffer (motionDisplay). The accelerator
    // will write into motionDraw, which HDMI is not reading. After both
    // accelerators finish, we swap so HDMI shows the just-written buffer
    // on the next iteration.
    vga[3] = swap_u32((uint32_t) motionDisplay);

    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7)); // start profiling

    takeSingleImageBlocking((uint32_t) &grayscale[0]);

    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    printf("nrOfCycles: %d %d %d\n", cycles, stall, idle);

    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7)); // start profiling

    // ── Fire the hardware Sobel accelerator ──────────────────────────────────
    sobel_acc_ci(SOBEL_ACC_REG_SRC, (uint32_t)&grayscale[0]);
    sobel_acc_ci(SOBEL_ACC_REG_DST, (uint32_t)sobelCurr);
    sobel_acc_ci(SOBEL_ACC_REG_CONTROL, 1u); // start
    volatile uint32_t acc_status;
    volatile uint32_t acc_timeout = 10000000u;
    do {
        acc_status = sobel_acc_ci(SOBEL_ACC_REG_STATUS, 0);
        acc_timeout--;
    } while ((acc_status & SOBEL_ACC_STATUS_BUSY) && acc_timeout != 0);

    // ── Fire the hardware Motion (XOR) accelerator ───────────────────────────
    // Writes into motionDraw — the buffer NOT currently being scanned by HDMI.
    motion_acc_ci(MOTION_ACC_REG_SRCA, (uint32_t)sobelCurr);
    motion_acc_ci(MOTION_ACC_REG_SRCB, (uint32_t)sobelPrev);
    motion_acc_ci(MOTION_ACC_REG_DST,  (uint32_t)motionDraw);
    motion_acc_ci(MOTION_ACC_REG_CONTROL, 1u);
    volatile uint32_t mot_status;
    volatile uint32_t mot_timeout = 10000000u;
    do {
        mot_status = motion_acc_ci(MOTION_ACC_REG_STATUS, 0);
        mot_timeout--;
    } while ((mot_status & MOTION_ACC_STATUS_BUSY) && mot_timeout != 0);

    // Swap sobel pointers: this frame's edges become "previous" for next frame.
    volatile uint8_t *temp = sobelPrev;
    sobelPrev = sobelCurr;
    sobelCurr = temp;

    // Swap motion pointers: the buffer we just wrote becomes the display
    // buffer for the next iteration; HDMI's old display buffer is now free
    // for the accelerator to overwrite.
    volatile uint16_t *tempM = motionDisplay;
    motionDisplay = motionDraw;
    motionDraw    = tempM;

    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    printf("nrOfCycles: %d %d %d\n", cycles, stall, idle);
  }
}
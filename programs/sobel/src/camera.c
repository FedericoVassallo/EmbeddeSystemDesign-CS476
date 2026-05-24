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
#define SOBEL_ACC_STATUS_DONE  0x1u
#define SOBEL_ACC_STATUS_BUSY  0x2u
#define SOBEL_ACC_STATUS_ERROR 0x4u

// ── Motion hardware accelerator (CI 0x0F) ────────────────────────────────────
#define MOTION_ACC_REG_STATUS  0u   // read: {error, busy, done}
#define MOTION_ACC_REG_SRCA    1u   // write: source A address (current edge frame)
#define MOTION_ACC_REG_SRCB    2u   // write: source B address (previous edge frame)
#define MOTION_ACC_REG_DST     3u   // write: destination address (motion buffer)
#define MOTION_ACC_REG_CONTROL 4u   // write: bit 0 = start
#define MOTION_ACC_STATUS_DONE  0x1u
#define MOTION_ACC_STATUS_BUSY  0x2u
#define MOTION_ACC_STATUS_ERROR 0x4u

#define FRAME_PROFILE_LOG 1
#define FRAME_PROFILE_INTERVAL 15u
#define FRAME_PIXELS (640*480)
#define NR_CAPTURE_BUFFERS 3u

#define CAMERA_CI_REG_FRAMEBUFFER   5u
#define CAMERA_CI_REG_FRAME_COUNTER 8u

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

static inline uint32_t camera_ci(uint32_t a, uint32_t b)
{
    uint32_t r;
    asm volatile("l.nios_rrr %[out],%[inA],%[inB],0x7"
                 : [out]"=r"(r)
                 : [inA]"r"(a), [inB]"r"(b));
    return r;
}

static inline uint32_t camera_frame_counter(void)
{
    return camera_ci(CAMERA_CI_REG_FRAME_COUNTER, 0);
}

static inline void camera_set_framebuffer(uint32_t framebuffer)
{
    camera_ci(CAMERA_CI_REG_FRAMEBUFFER, framebuffer);
}

static uint32_t wait_for_camera_frame(uint32_t *lastCounter)
{
    uint32_t now;
    do {
        now = camera_frame_counter();
    } while (now == *lastCounter);

    uint32_t delta = now - *lastCounter;
    *lastCounter = now;
    return delta;
}

volatile uint8_t  grayscaleA[FRAME_PIXELS];
volatile uint8_t  grayscaleB[FRAME_PIXELS];
volatile uint8_t  grayscaleC[FRAME_PIXELS];
volatile uint8_t  sobelA[FRAME_PIXELS];    // edge map, buffer A
volatile uint8_t  sobelB[FRAME_PIXELS];    // edge map, buffer B
// Double-buffered motion output: HDMI displays one buffer while the
// accelerator writes the other, then we ping-pong. This prevents the
// visible tearing/flicker that happens when HDMI scans-out a buffer that
// the Motion accelerator is concurrently writing to.
volatile uint8_t  motionA[FRAME_PIXELS];
volatile uint8_t  motionB[FRAME_PIXELS];

int main () {
  volatile int result;
  volatile unsigned int *vga = (unsigned int *) 0X50000020;
  camParameters camParams;
#if FRAME_PROFILE_LOG
  volatile uint32_t cycles, stall, idle;
  volatile uint32_t frameCounter = 0;
#endif
  volatile uint8_t *sobelCurr     = sobelA;    // this frame's edges
  volatile uint8_t *sobelPrev     = sobelB;    // last frame's edges
  volatile uint8_t *motionDisplay = motionA;   // what HDMI is currently showing
  volatile uint8_t *motionDraw    = motionB;   // what the accelerator writes into
  volatile uint8_t *captureBuffers[NR_CAPTURE_BUFFERS] = {grayscaleA, grayscaleB, grayscaleC};
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
  for (int i = 0; i < FRAME_PIXELS; i++) {
      grayscaleA[i] = 0; grayscaleB[i] = 0; grayscaleC[i] = 0;
      sobelA[i]     = 0; sobelB[i]     = 0;
      motionA[i]    = 0; motionB[i]    = 0;
  }

  vga[2] = swap_u32(2); // 2 -> grayscale display mode
  vga[3] = swap_u32((uint32_t) motionDisplay);

  uint32_t cameraCounter = camera_frame_counter();
  uint32_t writingIndex = 0;

  enableContinues((uint32_t)captureBuffers[writingIndex]);
  wait_for_camera_frame(&cameraCounter);
  camera_set_framebuffer((uint32_t)captureBuffers[1]);
  wait_for_camera_frame(&cameraCounter);
  writingIndex = 1;

#if FRAME_PROFILE_LOG
  printf("Continuous capture enabled, camera frame counter %d\n", cameraCounter);
#endif

  while (1) {
    uint32_t completedIndex = (writingIndex + 2u) % NR_CAPTURE_BUFFERS;
    uint32_t nextIndex = (writingIndex + 1u) % NR_CAPTURE_BUFFERS;
    volatile uint8_t *completedFrame = captureBuffers[completedIndex];

    camera_set_framebuffer((uint32_t)captureBuffers[nextIndex]);

#if FRAME_PROFILE_LOG
    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7)); // start profiling
#endif

    // ── Fire the hardware Sobel accelerator ──────────────────────────────────
    sobel_acc_ci(SOBEL_ACC_REG_SRC, (uint32_t)completedFrame);
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
    volatile uint8_t *tempM = motionDisplay;
    motionDisplay = motionDraw;
    motionDraw    = tempM;
    vga[3] = swap_u32((uint32_t) motionDisplay);

#if FRAME_PROFILE_LOG
    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    if ((frameCounter % FRAME_PROFILE_INTERVAL) == 0) {
        printf("frame %d cam %d src %d write %d next %d sobel %d/%d motion %d/%d cycles %d stall %d idle %d\n",
               frameCounter, cameraCounter, completedIndex, writingIndex, nextIndex,
               acc_status, acc_timeout, mot_status, mot_timeout, cycles, stall, idle);
    }
    frameCounter++;
#endif

    uint32_t cameraDelta = wait_for_camera_frame(&cameraCounter);
#if FRAME_PROFILE_LOG
    if (cameraDelta != 1u) {
        printf("camera frame skip: delta %d counter %d\n", cameraDelta, cameraCounter);
    }
#endif
    writingIndex = nextIndex;
  }
}

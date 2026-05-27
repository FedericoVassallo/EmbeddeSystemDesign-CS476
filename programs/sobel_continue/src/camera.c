#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
#ifdef __OR1300__
#include <cache.h>
#endif

// accSobel.v hardware accelerator (CI 0x0E)
static const uint32_t SOBEL_ACC_REG_STATUS   = 0;    // read {error, busy, done}
static const uint32_t SOBEL_ACC_REG_SRC      = 1;    // write source grayscale frame address
static const uint32_t SOBEL_ACC_REG_DST      = 2;    // write destination edge-map address
static const uint32_t SOBEL_ACC_REG_CONTROL  = 3;    // write bit 0 = start
static const uint32_t SOBEL_ACC_STATUS_BUSY  = 0x2;

// accMotion.v hardware accelerator (CI 0x0F)
static const uint32_t MOTION_ACC_REG_STATUS  = 0;    // read {error, busy, done}
static const uint32_t MOTION_ACC_REG_SRCA    = 1;    // write source A address (current Sobel frame)
static const uint32_t MOTION_ACC_REG_SRCB    = 2;    // write source B address (previous Sobel frame)
static const uint32_t MOTION_ACC_REG_DST     = 3;    // write destination address (motion buffer)
static const uint32_t MOTION_ACC_REG_CONTROL = 4;    // write bit 0 = start
static const uint32_t MOTION_ACC_STATUS_BUSY  = 0x2;

// camera CI (CI 0x07)
static const uint32_t CAMERA_CI_FRAME_WRITE_ADDR   = 5; // command to set the camera's address for writing the next frame into
static const uint32_t CAMERA_CI_FRAME_COUNTER = 8; // command to read the number of frames captured from camera.v

#define FRAME_PROFILE_INTERVAL 15
#define FRAME_PIXELS           (640 * 480)
#define NR_CAPTURE_BUFFERS     3

// function to send command to the custom instruction for motion accelerator (address 0xF)
static inline uint32_t motion_acc_ci(uint32_t a, uint32_t b) {
    uint32_t r;
    asm volatile("l.nios_rrr %[out],%[inA],%[inB],0xF"
                 : [out]"=r"(r): [inA]"r"(a), [inB]"r"(b));
    return r;
}
// function to send command to the custom instruction for Sobel accelerator (address 0xE)
static inline uint32_t sobel_acc_ci(uint32_t a, uint32_t b)
{
    uint32_t r;
    asm volatile("l.nios_rrr %[out],%[inA],%[inB],0xE"
                 : [out]"=r"(r): [inA]"r"(a), [inB]"r"(b));
    return r;
}
// function to send command to the custom instruction for camera CI (address 0x7)
static inline uint32_t camera_ci(uint32_t a, uint32_t b)
{
    uint32_t r;
    asm volatile("l.nios_rrr %[out],%[inA],%[inB],0x7"
                 : [out]"=r"(r): [inA]"r"(a), [inB]"r"(b));
    return r;
}
// helper function to wait for the next camera frame, returns the number of frames that have passed since lastCounter (to check for frame drops)
static uint32_t wait_for_camera_frame(uint32_t *lastCounter)
{
    uint32_t now;
    do {
        now = camera_ci(CAMERA_CI_FRAME_COUNTER, 0);
    } while (now == *lastCounter);

    uint32_t delta = now - *lastCounter;
    *lastCounter = now;
    return delta;
}

volatile uint8_t  grayscaleA[FRAME_PIXELS];
volatile uint8_t  grayscaleB[FRAME_PIXELS];
volatile uint8_t  grayscaleC[FRAME_PIXELS];
volatile uint8_t  sobelA[FRAME_PIXELS];
volatile uint8_t  sobelB[FRAME_PIXELS];
volatile uint16_t motionA[FRAME_PIXELS];  // RGB565: 2 bytes per pixel
volatile uint16_t motionB[FRAME_PIXELS];

int main () {
  volatile int result;
  volatile unsigned int *vga = (unsigned int *) 0X50000020;
  camParameters camParams;
  volatile uint32_t cycles, stall, idle;
  volatile uint32_t frameCounter = 0;
  volatile uint8_t *sobelCurr     = sobelA;    // this frame's edges
  volatile uint8_t *sobelPrev     = sobelB;    // last frame's edges
  volatile uint16_t *motionDisplay = motionA;   // what HDMI is currently showing
  volatile uint16_t *motionDraw    = motionB;   // what the accelerator writes into
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

  // Clear all buffers
  for (int i = 0; i < FRAME_PIXELS; i++) {
      grayscaleA[i] = 0; grayscaleB[i] = 0; grayscaleC[i] = 0;
      sobelA[i]     = 0; sobelB[i]     = 0;
      motionA[i]    = 0; motionB[i]    = 0;
  }

  vga[2] = swap_u32(1); // 1 for RGB565 display mode
  vga[3] = swap_u32((uint32_t) motionDisplay);

  uint32_t cameraCounter = camera_ci(CAMERA_CI_FRAME_COUNTER, 0);
  uint32_t writingIndex = 0;

  ///////
  // TO CHECK MAYBE WE CAN SUBSTITUTE WITH TAKEIMAGE BLOCKING FOR THE FIRST 2 FRAMES, THEN START CONTINUOUS MODE
  ////////

  enableContinues((uint32_t)captureBuffers[writingIndex]); // tell camera to start writing into the first capture buffer
  wait_for_camera_frame(&cameraCounter); // we wait for the first frame to be captured
  camera_ci(CAMERA_CI_FRAME_WRITE_ADDR, (uint32_t)captureBuffers[1]); // we set the camera to set the seoond capture frame
  wait_for_camera_frame(&cameraCounter); // we wait for the second frame to be captured
  writingIndex = 1;

  printf("Continuous capture enabled, camera frame counter %d\n", cameraCounter);

  while (1) {
    uint32_t completedIndex = (writingIndex + 2) % NR_CAPTURE_BUFFERS;
    uint32_t nextIndex = (writingIndex + 1) % NR_CAPTURE_BUFFERS;
    volatile uint8_t *completedFrame = captureBuffers[completedIndex];

    camera_ci(CAMERA_CI_FRAME_WRITE_ADDR, (uint32_t)captureBuffers[nextIndex]);

    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7)); // start profiling

    // we start the Sobel accelerator on the completed camera frame
    sobel_acc_ci(SOBEL_ACC_REG_SRC, (uint32_t)completedFrame);
    sobel_acc_ci(SOBEL_ACC_REG_DST, (uint32_t)sobelCurr); // we set the write address of the result sobel frame to sobelCurr
    sobel_acc_ci(SOBEL_ACC_REG_CONTROL, 1); // we set the signal to start by putting 1 in valueB[0]
    volatile uint32_t acc_status;
    volatile uint32_t acc_timeout = 10000000;
    do {
        acc_status = sobel_acc_ci(SOBEL_ACC_REG_STATUS, 0);
        acc_timeout--;
    } while ((acc_status & SOBEL_ACC_STATUS_BUSY) && acc_timeout != 0);

    // we start the motion accelerator to compare this frame's edges with the previous frame's edges and write the result into motionDraw
    motion_acc_ci(MOTION_ACC_REG_SRCA, (uint32_t)sobelCurr); // set source A to this frame's Sobel edges
    motion_acc_ci(MOTION_ACC_REG_SRCB, (uint32_t)sobelPrev); // set source B to previous frame's Sobel edges
    motion_acc_ci(MOTION_ACC_REG_DST,  (uint32_t)motionDraw); // write the result into the buffer motionDraw
    motion_acc_ci(MOTION_ACC_REG_CONTROL, 1); // we set the signal to start by putting 1 in valueB[0]
    volatile uint32_t mot_status;
    volatile uint32_t mot_timeout = 10000000;
    do {
        mot_status = motion_acc_ci(MOTION_ACC_REG_STATUS, 0);
        mot_timeout--;
    } while ((mot_status & MOTION_ACC_STATUS_BUSY) && mot_timeout != 0);

    // we swap the pointer to the sobel buffers, so that the current sobel frame becomes the previous sobel, and the next accelerator result will be written into the other buffer 
    volatile uint8_t *temp = sobelPrev;
    sobelPrev = sobelCurr;
    sobelCurr = temp;

    // we swap the pointers for motion display and motion draw, same logic as for the sobel buffers
    volatile uint16_t *tempM = motionDisplay;
    motionDisplay = motionDraw;
    motionDraw    = tempM;
    vga[3] = swap_u32((uint32_t) motionDisplay);

    // profiling CI for stopping profiling counter and saving results
    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    if ((frameCounter % FRAME_PROFILE_INTERVAL) == 0) {
        printf("frame %d cam %d src %d write %d next %d sobel %d/%d motion %d/%d cycles %d stall %d idle %d\n",
               frameCounter, cameraCounter, completedIndex, writingIndex, nextIndex,
               acc_status, acc_timeout, mot_status, mot_timeout, cycles, stall, idle);
    }
    frameCounter++;

    uint32_t cameraDelta = wait_for_camera_frame(&cameraCounter);

    if (cameraDelta != 1u) {
        printf("camera frame skip: delta %d counter %d\n", cameraDelta, cameraCounter);
    }

    writingIndex = nextIndex;
  }
}
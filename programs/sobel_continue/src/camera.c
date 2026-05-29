#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
#ifdef __OR1300__
#include <cache.h>
#endif

// accSobelMotion.v fused Sobel + motion accelerator (CI 0x09)
static const uint32_t SOBEL_MOTION_ACC_REG_STATUS    = 0; // read {error, busy, done}
static const uint32_t SOBEL_MOTION_ACC_REG_SRC       = 1; // write source grayscale frame address
static const uint32_t SOBEL_MOTION_ACC_REG_PREV_EDGE = 2; // write previous edge-map address
static const uint32_t SOBEL_MOTION_ACC_REG_CURR_EDGE = 3; // write current edge-map address
static const uint32_t SOBEL_MOTION_ACC_REG_MOTION    = 4; // write RGB565 motion buffer address
static const uint32_t SOBEL_MOTION_ACC_REG_CONTROL   = 5; // write bit 0 = start
static const uint32_t SOBEL_MOTION_ACC_STATUS_BUSY   = 0x2;

// camera CI (CI 0x07)
static const uint32_t CAMERA_CI_FRAME_WRITE_ADDR   = 5; // command to set the camera's address for writing the next frame into
static const uint32_t CAMERA_CI_FRAME_COUNTER = 8; // command to read the number of frames captured from camera.v

#define FRAME_PROFILE_INTERVAL 15
#define FRAME_PIXELS           (640 * 480)

// function to send command to the fused Sobel + motion accelerator (address 0x9)
static inline uint32_t sobel_motion_acc_ci(uint32_t a, uint32_t b)
{
    uint32_t r;
    asm volatile("l.nios_rrr %[out],%[inA],%[inB],0x9"
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
  volatile uint8_t *captureReady   = grayscaleA; // completed frame ready to be processed
  volatile uint8_t *captureWriting = grayscaleB; // buffer being filled by the camera
  volatile uint8_t *captureFree    = grayscaleC; // armed for the next camera frame
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

  // Clear all buffers
  for (int i = 0; i < FRAME_PIXELS; i++) {
      grayscaleA[i] = 0; grayscaleB[i] = 0; grayscaleC[i] = 0;
      sobelA[i]     = 0; sobelB[i]     = 0;
      motionA[i]    = 0; motionB[i]    = 0;
  }

  vga[2] = swap_u32(1); // 1 for RGB565 display mode
  vga[3] = swap_u32((uint32_t)motionDisplay);

  uint32_t cameraCounter = camera_ci(CAMERA_CI_FRAME_COUNTER, 0);

  // The camera frame counter changes at the start of a new frame. Therefore
  // the buffer from the previous camera interval is complete only after the
  // next counter change. Keep one buffer ready, one writing, and one already
  // armed for the next camera boundary.
  enableContinues((uint32_t)captureReady);
  wait_for_camera_frame(&cameraCounter);
  camera_ci(CAMERA_CI_FRAME_WRITE_ADDR, (uint32_t)captureWriting);
  wait_for_camera_frame(&cameraCounter);
  camera_ci(CAMERA_CI_FRAME_WRITE_ADDR, (uint32_t)captureFree);

  printf("Continuous capture enabled, camera frame counter %d\n", cameraCounter);

  while (1) {
    asm volatile ("l.nios_rrr r0,r0,%[in2],0xB"::[in2]"r"(7)); // start profiling

    // The fused accelerator computes this frame's Sobel edges, compares them
    // with the previous edge map, writes the new edge map, and draws RGB565 motion.
    sobel_motion_acc_ci(SOBEL_MOTION_ACC_REG_SRC,       (uint32_t)captureReady);
    sobel_motion_acc_ci(SOBEL_MOTION_ACC_REG_PREV_EDGE, (uint32_t)sobelPrev);
    sobel_motion_acc_ci(SOBEL_MOTION_ACC_REG_CURR_EDGE, (uint32_t)sobelCurr);
    sobel_motion_acc_ci(SOBEL_MOTION_ACC_REG_MOTION,    (uint32_t)motionDraw);
    sobel_motion_acc_ci(SOBEL_MOTION_ACC_REG_CONTROL,   1);
    uint32_t acc_status;
    uint32_t acc_timeout = 10000000;
    do {
        acc_status = sobel_motion_acc_ci(SOBEL_MOTION_ACC_REG_STATUS, 0);
        acc_timeout--;
    } while ((acc_status & SOBEL_MOTION_ACC_STATUS_BUSY) && acc_timeout != 0);

    // we swap the pointer to the sobel buffers, so that the current sobel frame becomes the previous sobel, and the next accelerator result will be written into the other buffer 
    volatile uint8_t *temp = sobelPrev;
    sobelPrev = sobelCurr;
    sobelCurr = temp;

    // The buffer we just wrote becomes visible; HDMI's old display buffer is
    // then free for the next accelerator output.
    volatile uint16_t *tempM = motionDisplay;
    motionDisplay = motionDraw;
    motionDraw    = tempM;
    vga[3] = swap_u32((uint32_t)motionDisplay);

    // profiling CI for stopping profiling counter and saving results
    asm volatile ("l.nios_rrr %[out1],r0,%[in2],0xB":[out1]"=r"(cycles):[in2]"r"(1<<8|7<<4));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(stall):[in1]"r"(1),[in2]"r"(1<<9));
    asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xB":[out1]"=r"(idle):[in1]"r"(2),[in2]"r"(1<<10));
    if ((frameCounter % FRAME_PROFILE_INTERVAL) == 0) {
        printf("frame %d cam %d fused %d/%d cycles %d stall %d idle %d\n",
               frameCounter, cameraCounter,
               acc_status, acc_timeout, cycles, stall, idle);
    }
    frameCounter++;

    uint32_t cameraDelta = wait_for_camera_frame(&cameraCounter);

    if (cameraDelta != 1u) {
        printf("camera frame skip: delta %d counter %d\n", cameraDelta, cameraCounter);
    }

    // At this point the camera has started writing captureFree. The buffer
    // it was writing before this boundary is complete and safe to process.
    volatile uint8_t *completedFrame = captureWriting;
    volatile uint8_t *processedFrame = captureReady;
    captureWriting = captureFree;
    captureReady   = completedFrame;
    captureFree    = processedFrame;

    // Arm the buffer we just finished processing for the frame after the one
    // currently being captured. This must happen immediately after the camera
    // boundary so the next newScreen does not reuse the current write buffer.
    camera_ci(CAMERA_CI_FRAME_WRITE_ADDR, (uint32_t)captureFree);
  }
}

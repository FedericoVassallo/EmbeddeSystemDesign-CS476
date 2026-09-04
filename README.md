# Real-Time Motion Detection on an OpenRISC FPGA SoC

Sobel edge detection and frame-difference motion detection on a live 640x480 camera feed,
running on an OR1420 (OpenRISC) soft core at 74.25 MHz on a Lattice ECP5 FPGA (GECKO5).
Verilog, C.

Pure software ran at 0.16 fps. Three rounds of profiling and hardware design brought the
pipeline to 7.7 fps, at which point the camera, not the computation, sets the limit.

- Profiled the software baseline and found the Sobel stage to be 88% of the frame cost, of
  which 81% was CPU stall rather than arithmetic.
- Implemented Sobel as a custom CPU instruction with a multiplier-free datapath, packing a
  3x3 neighbourhood into two 32-bit operands by dropping the zero-weighted centre pixel:
  6.9x on the stage. Re-profiling showed the cost had shifted to feeding the unit.
- Designed two bus-master accelerators with line buffering and 16-word burst transfers,
  taking the CPU out of the inner loop: 216x on Sobel and 17.4x on motion detection, with
  CPU stalls down from 326M to ~100 cycles per frame.
- Overlapped camera acquisition with computation using a non-blocking capture loop and
  three pairs of double buffers, swapped by pointer rather than copied.

Per-frame compute went from 6.12 s to 66 ms (93x); system throughput from 0.16 to 7.7 fps
(48x).

Built with the OSS CAD Suite (Yosys, nextpnr-ecp5, openFPGALoader) and an OpenRISC GCC
toolchain, and deployed on the board. The GECKO5 is EPFL teaching hardware, so the design
is not reproducible without it.

---

EPFL CS-476, two-person project with Matteo Barberis. SoC skeleton provided by the course;
the accelerators and the final capture software are ours.

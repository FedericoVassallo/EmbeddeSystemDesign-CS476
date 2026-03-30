# PW2 Part2

Federico Vassallo SCIPER 416376
Matteo Barberis SCIPER 413877

## Changes

We implemented the Verilog and C code for both the single-pixel and the 4-pixel parallel conversions. Because our 4-pixel Verilog implementation also works correctly for single pixels, this is the version we included in the main virtual prototype directory. However, we also provided our initial single-pixel Verilog implementation (created before starting the additional task) in the `verilog1pixel/` folder (`verilog1pixel/rgb565GrayscaleIse.v`).

To measure the impact of each optimization, we created three different folders inside `virtualprototype/programs`:

- `grayscale`: No custom instruction used; conversion is done entirely in C using standard arithmetic.
- `grayscaleCi`: Custom instruction used for single-pixel conversion.
- `grayscaleCi4pixel`: Custom instruction used to convert 4 pixels in parallel.

We also made the following modifications:

- `virtualprototype/modules/rgb565GrayscaleIse/verilog/rgb565GrayscaleIse.v`: Added the grayscale Verilog module. It takes one or two 32-bit inputs (`valueA` and `valueB`), each containing two RGB565 pixels, and converts them to grayscale. It processes up to 4 pixels in a single cycle and returns 4 grayscale bytes packed into a single 32-bit result.
- `virtualprototype/systems/singleCore/verilog/or1420SingleCore.v`: Instantiated the grayscale custom instruction module. To support the 4-pixel parallel version, we connected `valueB` in addition to `valueA` (which was already connected for the single-pixel version).
- `virtualprototype/programs/grayscale/src/grayscale.c`: Single-pixel C code version that does not use the grayscale custom instruction. It only implements the profiling custom instruction.
- `virtualprototype/programs/grayscaleCi/src/grayscale.c`: Single-pixel C code version that uses the custom instruction to convert 1 pixel per cycle.
- `virtualprototype/programs/grayscaleCi4pixel/src/grayscale.c`: 4-pixel C code version. It uses four `swap_u16` calls to byte-swap the pixels, packs them into two 32-bit words, and calls the custom instruction with both inputs. The result is written as a single 32-bit word containing 4 grayscale bytes.
- `virtualprototype/systems/singleCore/scripts/yosysOr1420.script`: Added the `read -sv` directive for `rgb565GrayscaleIse.v`.

## Results

Software grayscale, no custom instruction (`grayscale`):

- Execution cycles: 29,120,802
- Stall cycles: 17,747,674
- Bus-idle cycles: 16,754,534
- Real work (exec - stall): 11,373,128

Single-pixel custom instruction (`grayscaleCi`):

- Execution cycles: 23,181,464
- Stall cycles: 17,030,736
- Bus-idle cycles: 11,652,005
- Real work (exec - stall): 6,150,728

4-pixel parallel custom instruction (`grayscaleCi4pixel`):

- Execution cycles: 16,557,608
- Stall cycles: 11,788,800
- Bus-idle cycles: 8,170,942
- Real work (exec - stall): 4,768,808

## Comment

The single-pixel custom instruction reduces real work cycles from ~11.4M to ~6.2M, a 46% reduction (1.85x speedup). All the multiplications, additions, and shifts that the software version needed are now directly implemented in hardware, which enables this speedup.

The 4-pixel version further reduces real work cycles to ~4.8M, a 58% reduction vs. no CI (2.38x speedup), and a 1.29x speedup over the single-pixel CI.

The improvement from single-pixel to 4-pixel is smaller than expected. We identified a few reasons for this:

- We still need four separate `swap_u16` calls per iteration.
- The packing process (using OR and shift operations to combine two 16-bit pixels into one 32-bit word) adds extra software instructions.
- One `swap_u32` call is needed to fix the output byte order before storing.

However, the stall cycles did drop significantly (from 17.0M to 11.8M). This is because the 4-pixel version performs fewer memory operations: it reads 4 pixels per cycle and writes 4 grayscale bytes as a single 32-bit store, instead of executing individual byte-level loads and stores.

The 4-pixel version could be made faster by moving the byte swap into the Verilog module (which is just simple rewiring in hardware) and loading the raw pixels directly from memory as 32-bit words. This would completely eliminate all four `swap_u16` calls and the packing shifts. However, to stay consistent with the single-pixel version and ensure both C codes could run on the exact same Verilog implementation, we kept it this way.

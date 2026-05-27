/*
 * sobel.c
 *
 *  Created on: Sep 12, 2015
 *      Author: theo
 */

#include <stdio.h>
#include <stdint.h>
#include <swap.h>

void edgeDetection( const uint8_t *grayscale,    // Removed volatile
                    uint8_t *sobelResult,        // Removed volatile
                    int32_t width,
                    int32_t height,
                    int32_t threshold ) {
                      
    uint32_t valueA, valueB, edge;
    
    for (int line = 1; line < height - 1; line++) {
        // Set up three linear pointers for the top, middle, and bottom rows
        const uint8_t *rTop = &grayscale[(line - 1) * width];
        const uint8_t *rMid = &grayscale[ line      * width];
        const uint8_t *rBot = &grayscale[(line + 1) * width];

        // Pre-load the first two columns of the sliding window
        uint32_t t0 = rTop[0], t1 = rTop[1];
        uint32_t m0 = rMid[0], m1 = rMid[1];
        uint32_t b0 = rBot[0], b1 = rBot[1];

        // Pointer for the output pixel to avoid 2D math
        uint8_t *outPixel = &sobelResult[line * width + 1];

        for (int pixel = 1; pixel < width - 1; pixel++) {
            // Read ONLY the 3 new pixels for the right side of the window
            uint32_t t2 = rTop[pixel + 1];
            uint32_t m2 = rMid[pixel + 1];
            uint32_t b2 = rBot[pixel + 1];

            // Pack the registers for the hardware instruction
            valueA = (t0 << 24) | (t1 << 16) | (t2 << 8) | m0;
            valueB = (m2 << 24) | (b0 << 16) | (b1 << 8) | b2;

            // Execute custom hardware instruction
            asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xD"
                          : [out1]"=r"(edge)
                          : [in1]"r"(valueA), [in2]"r"(valueB));

            // Apply the threshold to the raw hardware magnitude
            *outPixel = (edge > threshold) ? 255 : 0;
            outPixel++;

            // Shift the window left for the next iteration (pure register moves)
            t0 = t1; t1 = t2;
            m0 = m1; m1 = m2;
            b0 = b1; b1 = b2;
        }
    }
}
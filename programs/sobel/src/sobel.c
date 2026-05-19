/*
 * sobel.c
 *
 *  Created on: Sep 12, 2015
 *      Author: theo
 */

#include <stdio.h>
#include <stdint.h>
#include <swap.h>

void edgeDetection( volatile uint8_t *grayscale,
                    volatile uint8_t *sobelResult,
                    int32_t width,
                    int32_t height,
                    int32_t threshold ) {
                      // for now threshold is hard-wired

    for (int line = 1; line < height - 1; line++) {
        for (int pixel = 1; pixel < width - 1; pixel++) {
            /* gather the 8 neighbours of (line,pixel) */
            uint32_t p1 = grayscale[(line-1)*width + (pixel-1)];
            uint32_t p2 = grayscale[(line-1)*width + (pixel  )];
            uint32_t p3 = grayscale[(line-1)*width + (pixel+1)];
            uint32_t p4 = grayscale[(line  )*width + (pixel-1)];
            uint32_t p6 = grayscale[(line  )*width + (pixel+1)];
            uint32_t p7 = grayscale[(line+1)*width + (pixel-1)];
            uint32_t p8 = grayscale[(line+1)*width + (pixel  )];
            uint32_t p9 = grayscale[(line+1)*width + (pixel+1)];

            uint32_t valueA = (p1 << 24) | (p2 << 16) | (p3 << 8) | p4;
            uint32_t valueB = (p6 << 24) | (p7 << 16) | (p8 << 8) | p9;

            uint32_t edge;
            asm volatile ("l.nios_rrr %[out1],%[in1],%[in2],0xD": [out1]"=r"(edge): [in1]"r"(valueA), [in2]"r"(valueB));
            sobelResult[line*width + pixel] = (uint8_t)edge;
        }
    }
}



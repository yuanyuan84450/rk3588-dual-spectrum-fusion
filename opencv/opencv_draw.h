#ifndef __OPENCV_DRAW_H
#define __OPENCV_DRAW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "heimann_inter.h"
#include "heimann_drv.h"
#include "colormap.h"

// #include "sdl2_draw.h"


#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480
#define THERMAL_COLS 32
#define THERMAL_ROWS 30
#define BLOCK_SIZE 2
#define PROB_SCALE 8

#define IMAGE_WIDTH (THERMAL_COLS * PROB_SCALE * BLOCK_SIZE)
#define IMAGE_HEIGHT (THERMAL_ROWS * PROB_SCALE * BLOCK_SIZE)
#define OFFSET_X ((WINDOW_WIDTH - IMAGE_WIDTH) / 2)
#define OFFSET_Y ((WINDOW_HEIGHT - IMAGE_HEIGHT) / 2)

int opencv_main(uint16_t* data_pixel);

#ifdef __cplusplus
}
#endif


#endif

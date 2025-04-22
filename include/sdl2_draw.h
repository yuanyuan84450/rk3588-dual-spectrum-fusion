#ifndef __SDL2_DRAW_H
#define __SDL2_DRAW_H

#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

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



int sdl2_init(SDL_Window** win, SDL_Renderer** renderer);
int sdl2_main(SDL_Window* win, SDL_Renderer* renderer, uint16_t* data_pixel);
int sdl2_free(SDL_Window* win, SDL_Renderer* renderer);

#endif
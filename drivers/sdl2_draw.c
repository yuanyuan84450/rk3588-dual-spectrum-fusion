#include "sdl2_draw.h"
#include "heimann_inter.h"
#include "heimann_drv.h"


unsigned short draw_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW] = {{0}};

extern void load_colormap(uint8_t index);
extern uint16_t classic[180];
extern void get_rgb888_from_rgb565(uint16_t val, uint8_t* r8, uint8_t* g8, uint8_t* b8);
// 简单的温度到颜色映射


void temperature_to_color(uint8_t temp, Uint8* r, Uint8* g, Uint8* b) {
    if (temp < 50) {
        *r = 0; 
        *g = temp*10; 
        *b = 255 - temp * 5;
    } else if (temp < 100) {
        *r = 0;
        *g = (temp - 50) * 5;
        *b = 255 - (temp - 50) * 5;
    } else {
        *r = (temp - 100) * 5;
        *g = 255 - (temp - 100) * 5;
        *b = 0;
    }
}

//初始化sdl2界面
int sdl2_init(SDL_Window** win, SDL_Renderer** renderer)
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        printf("SDL_Init Error: %s\n", SDL_GetError());
        return -1;
    }

    *win = SDL_CreateWindow("Thermal Imaging - Centered",
                                       SDL_WINDOWPOS_CENTERED,
                                       SDL_WINDOWPOS_CENTERED,
                                       WINDOW_WIDTH, WINDOW_HEIGHT,
                                       SDL_WINDOW_SHOWN);
    if (!*win) {
        printf("SDL_CreateWindow Error: %s\n", SDL_GetError());
        SDL_Quit();
        return -1;
    }

    *renderer = SDL_CreateRenderer(*win, -1, SDL_RENDERER_ACCELERATED);
    if (!*renderer) {
        printf("SDL_CreateRenderer Error: %s\n", SDL_GetError());
        SDL_DestroyWindow(*win);
        SDL_Quit();
        return -1;
    }

    SDL_SetRenderDrawColor(*renderer, 0, 0, 0, 255);  // 清屏黑色
    SDL_RenderClear(*renderer);
    printf("sdl2_init:Renderer = %p\n", *renderer);

    return 0;
}


int sdl2_main(SDL_Window* win, SDL_Renderer* renderer, uint16_t* data_pixel)
{
    unsigned short temp_value=0;
    int temp_inter=0;
    printf("sdl2_main:Renderer = %p\n", renderer);

    //将data_pixel转换为draw_pixel
    for (int i = 0; i < THERMAL_ROWS; i++) {
        for (int j = 0; j < THERMAL_COLS; j++) {
           // 拷贝温度信息, 并提前映射到色彩空间中
           temp_value = (180.0 * (data_pixel[i * THERMAL_COLS + j] - T_min) / (T_max - T_min));
           if (temp_value < 180) {
           draw_pixel[i][j] = temp_value;
           }else {
                draw_pixel[i][j] = 179;
           }
        }
    }
    // printf("sdl2_main:draw_pixel ok");

    for(int y=0; y<THERMAL_COLS * PROB_SCALE; y++){ 
        for(int x=0; x<THERMAL_ROWS * PROB_SCALE; x++){
            uint8_t r, g, b;
            temp_inter = bio_linear_interpolation(x, y, &draw_pixel[0][0]);
            // printf("sdl2_main:temp_inter[%d][%d] = %d\n",x,y,temp_inter);
            get_rgb888_from_rgb565(classic[(uint16_t)temp_inter], &r, &g, &b);
            // printf("sdl2_main: r=%d, g=%d, b=%d\n",r,g,b);

            SDL_SetRenderDrawColor(renderer, r, g, b, 255);
            SDL_Rect rect = {
                OFFSET_X + x * BLOCK_SIZE,
                OFFSET_Y + y * BLOCK_SIZE,
                BLOCK_SIZE,
                BLOCK_SIZE
            };
            SDL_RenderFillRect(renderer, &rect);

        }
    }

    // for (int y = 0; y < THERMAL_ROWS; ++y) {
    //     for (int x = 0; x < THERMAL_COLS; ++x) {
    //         Uint8 r, g, b;
    //         uint16_t value = (uint16_t)(data_pixel[y * THERMAL_COLS + x]/10.0-273.15);
            
    //         temperature_to_color(value, &r, &g, &b);
    //         // printf("data_pixel[%d][%d]=%d,r=%d, g=%d, b=%d\n", y,x,value, r,g,b);
    //         SDL_SetRenderDrawColor(renderer, r, g, b, 255);

    //         SDL_Rect rect = {
    //             OFFSET_X + x * BLOCK_SIZE,
    //             OFFSET_Y + y * BLOCK_SIZE,
    //             BLOCK_SIZE,
    //             BLOCK_SIZE
    //         };
    //         SDL_RenderFillRect(renderer, &rect);
    //     }
    // }

    SDL_RenderPresent(renderer);

    return 0;
}

int sdl2_free(SDL_Window* win, SDL_Renderer* renderer)
{
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
#include "opencv2/opencv.hpp"

#include "opencv_draw.h"



extern void load_colormap(uint8_t index);
extern uint16_t classic[180];
extern void get_rgb888_from_rgb565(uint16_t val, uint8_t* r8, uint8_t* g8, uint8_t* b8);

// OpenCV 显示热图
int opencv_main(uint16_t* data_pixel) {
    unsigned short draw_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW] = {{0}};
    int temp_inter = 0;

    // 初始化图像矩阵
    int disp_rows = THERMAL_COLS * PROB_SCALE;
    int disp_cols = THERMAL_ROWS * PROB_SCALE;
    cv::Mat img(disp_rows, disp_cols, CV_8UC3, cv::Scalar(0, 0, 0)); // BGR 格式

    // 生成 draw_pixel（温度索引映射）
    for (int i = 0; i < THERMAL_ROWS; i++) {
        for (int j = 0; j < THERMAL_COLS; j++) {
            int val = (180.0 * (data_pixel[i * THERMAL_COLS + j] - T_min) / (T_max - T_min));
            draw_pixel[i][j] = val < 180 ? val : 179;
        }
    }

    // 插值并绘制图像
    for (int y = 0; y < disp_rows; y++) {
        for (int x = 0; x < disp_cols; x++) {
            temp_inter = bio_linear_interpolation(x, y, &draw_pixel[0][0]);
            uint8_t r, g, b;
            get_rgb888_from_rgb565(classic[temp_inter], &r, &g, &b);
            img.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r); // BGR 顺序
        }
    }

    cv::imshow("Thermal Image", img);
    cv::waitKey(1); // 不阻塞，可设更大等待时间查看慢动作效果

    return 0;
}
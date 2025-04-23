#include "opencv2/opencv.hpp"

#include "opencv_draw.h"

extern void load_colormap(uint8_t index);
extern uint16_t classic[180];
extern void get_rgb888_from_rgb565(uint16_t val, uint8_t* r8, uint8_t* g8, uint8_t* b8);

// OpenCV 显示热图
int cv_show_heimann_classic(const uint16_t* data_pixel) {
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

void draw_roi_frame(const uint8_t* yuv_data) {
    // 构造 NV12 原始帧（Y + UV，连续的）
    cv::Mat nv12(MIX_HEIGHT + MIX_HEIGHT / 2, MIX_WIDTH, CV_8UC1, (void*)yuv_data);

    // 转换为 BGR 彩色图像
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);

    // 取 ROI 区域
    cv::Rect roi_rect(ROI_X, ROI_Y, ROI_W, ROI_H);
    cv::Mat roi_bgr = bgr(roi_rect);

    // 显示 ROI 彩色图
    cv::imshow("Visible ROI - Color", roi_bgr);
    cv::waitKey(1);
}

void* opencv_thread(void *arg){
    thread_context_t* ctx = (thread_context_t*)arg;
    // int argc = ctx->thread_args.argc;
    // char **argv = ctx->thread_args.argv;
    // printf("opencv_thread!!!\n");

    while(1){

        pthread_mutex_lock(&ctx->thermal_buf.mutex);
        while (!ctx->thermal_buf.updated)
            pthread_cond_wait(&ctx->thermal_buf.cond, &ctx->thermal_buf.mutex);
        pthread_mutex_unlock(&ctx->thermal_buf.mutex);
        // printf("opencv_thread-thermal_buf\n");

        pthread_mutex_lock(&ctx->yuv_buf.mutex);
        while (!ctx->yuv_buf.updated)
            pthread_cond_wait(&ctx->yuv_buf.cond, &ctx->yuv_buf.mutex);
        pthread_mutex_unlock(&ctx->yuv_buf.mutex);
        // printf("opencv_thread-yuv_buf\n");

        pthread_mutex_lock(&ctx->fusion_buf.mutex);
        // printf("opencv!!!\n");
        draw_roi_frame(ctx->yuv_buf.yuv_data);
        cv_show_heimann_classic(&ctx->thermal_buf.thermal_data[0][0]);
        

        pthread_mutex_unlock(&ctx->fusion_buf.mutex);
    }

    return NULL;

}
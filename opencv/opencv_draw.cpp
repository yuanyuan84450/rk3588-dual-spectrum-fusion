#include "opencv2/opencv.hpp"

#include "opencv_draw.h"

extern void load_colormap(uint8_t index);
extern uint16_t viridis[180];
extern uint16_t classic[180];
extern uint16_t hot[180];
extern uint16_t turbo[180];
extern uint16_t inferno[180];
extern uint16_t greys_r[180];
extern uint16_t greys[180];
extern void get_rgb888_from_rgb565(uint16_t val, uint8_t* r8, uint8_t* g8, uint8_t* b8);

struct AdjustParams {
    float shift_x;  // X方向微调（像素）
    float shift_y;  // Y方向微调（像素）
    float scale;    // 缩放（比例，比如1.0不变，1.01放大1%）
    float angle;    // 旋转角度（单位度°）
    AdjustParams() : shift_x(0), shift_y(0), scale(1.0), angle(0.0) {}
};

// 进行图像配准并返回配准后的热成像图
cv::Mat register_thermal_to_visible(const cv::Mat& thermal_img, const AdjustParams& adjust = AdjustParams()) {
    // 热成像的4个点
    std::vector<cv::Point2f> pts_thermal = {
        cv::Point2f(42, 51),
        cv::Point2f(212, 56),
        cv::Point2f(15, 197),
        cv::Point2f(212, 212)
    };

    // 可见光ROI图像中的4个对应点
    std::vector<cv::Point2f> pts_visible = {
        cv::Point2f(90, 50),
        cv::Point2f(415, 55),
        cv::Point2f(40, 345),
        cv::Point2f(458, 351)
    };

    // 计算单应矩阵
    cv::Mat H = cv::findHomography(pts_thermal, pts_visible);

    // 初步配准
    cv::Mat registered_img;
    cv::warpPerspective(thermal_img, registered_img, H, cv::Size(ROI_W, ROI_H));

    // ========== 追加微调 ==========
    if (adjust.shift_x != 0 || adjust.shift_y != 0 || adjust.scale != 1.0 || adjust.angle != 0.0) {
        // 以图像中心为基准
        cv::Point2f center(registered_img.cols / 2.0f, registered_img.rows / 2.0f);

        // 生成仿射矩阵（旋转+缩放）
        cv::Mat M = cv::getRotationMatrix2D(center, adjust.angle, adjust.scale);

        // 加上平移
        M.at<double>(0, 2) += adjust.shift_x;
        M.at<double>(1, 2) += adjust.shift_y;

        // 应用微调
        cv::warpAffine(registered_img, registered_img, M, registered_img.size());
    }

    return registered_img;
}

// 使用sobel提取边缘
cv::Mat extract_edges_sobel(const cv::Mat& roi_img) {
    cv::Mat gray, blurred, grad_x, grad_y, abs_grad_x, abs_grad_y, edge_map, edge_colored, result;

    // 1. 转换为灰度图
    cv::cvtColor(roi_img, gray, cv::COLOR_BGR2GRAY);

    // 2. 可选：高斯滤波去噪
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.5);

    // 3. 计算 Sobel 梯度
    cv::Sobel(blurred, grad_x, CV_16S, 1, 0, 3); // x 方向梯度
    cv::Sobel(blurred, grad_y, CV_16S, 0, 1, 3); // y 方向梯度

    // 4. 计算梯度幅值（取绝对值并转换到 8-bit）
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    cv::addWeighted(abs_grad_x, 1.5, abs_grad_y, 1.5, 0, edge_map);  //组合梯度

    cv::bitwise_not(edge_map, edge_map);

    cv::cvtColor(edge_map, edge_colored, cv::COLOR_GRAY2BGR);
    cv::addWeighted(roi_img, 0.4, edge_colored, 0.6, 0, result);

    return result;
}



int cv_show_fusion_display(const uint16_t* thermal_pixel, const uint8_t* yuv_data, cv::Mat& out_bgr_img, cv::Mat& out_thermal_img) {
    // === 步骤 1: 处理摄像头图像 YUV → BGR ===
    cv::Mat yuv_img(MIX_HEIGHT * 3 / 2, MIX_WIDTH, CV_8UC1, (void*)yuv_data);  // 假设 NV12 格式
    cv::Mat bgr_img;
    cv::cvtColor(yuv_img, bgr_img, cv::COLOR_YUV2BGR_NV12);       // 转换成 BGR 格式

    // === 步骤 2: 处理热成像图像 ===
    unsigned short draw_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW] = {{0}};
    int temp_inter = 0;
    const int disp_rows = 32 * PROB_SCALE;  // 128
    const int disp_cols = 30 * PROB_SCALE;  // 128
    cv::Mat thermal_img(disp_rows, disp_cols, CV_8UC3, cv::Scalar(0, 0, 0));
    // cv::Mat thermal_img(THERMAL_ROWS, THERMAL_COLS, CV_8UC3, cv::Scalar(0, 0, 0));

    for (int i = 0; i < 32; i++) {
        for (int j = 0; j < 32; j++) {
            int val = (180.0 * (thermal_pixel[i * THERMAL_COLS + j] - T_min) / (T_max - T_min));
            val = (val < 0) ? 0 : (val > 179 ? 179 : val);
            draw_pixel[i][j] = val;
            printf("thermal_pixel[%d][%d]=%d; ",i, j, thermal_pixel[i * THERMAL_COLS + j]);
            printf("T_min=%d, T_max=%d; ", T_min, T_max);
            printf("draw_pixel=%d\n", draw_pixel[i][j]);
            
        }
    }

    for (int y = 0; y < disp_rows; y++) {
        for (int x = 0; x < disp_cols; x++) {
            temp_inter = bio_linear_interpolation(x, y, &draw_pixel[0][0]);
            // printf("temp_inter=[%d][%d]=%d\n", y, x, temp_inter);
            uint8_t r, g, b;
            get_rgb888_from_rgb565(turbo[temp_inter], &r, &g, &b);
            thermal_img.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
        }
    }

    // for (int y = 0; y < THERMAL_ROWS; y++) {
    //     for (int x = 0; x < THERMAL_COLS; x++) {
    //         uint8_t r, g, b;
    //         temp_inter = draw_pixel[y][x];
    //         printf("temp_inter=[%d][%d]=%d\n", y, x, temp_inter);
    //         get_rgb888_from_rgb565(classic[temp_inter], &r, &g, &b);
    //         thermal_img.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
    //     }
    // }

    // // === 步骤 3: 创建总显示图像 ===
    // cv::Mat final_img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0)); // 创建黑底画布

    // // 左边放摄像头图像（480x480）
    // // cv::resize(bgr_img, bgr_img, cv::Size(MIX_WIDTH, MIX_WIDTH));
    // cv::Rect roi_rect(ROI_X, ROI_Y, ROI_W, ROI_H);
    // cv::Mat roi_img = bgr_img(roi_rect);
    // // roi_img.copyTo(final_img(cv::Rect(0, 0, ROI_W, ROI_H)));

    // // // 右边中央放热成像图像（128x128）
    // // thermal_img.copyTo(final_img(cv::Rect(ROI_X+ROI_W+10, 0, disp_cols, disp_rows)));

    // out_bgr_img = roi_img.clone();    // 只保存ROI区域
    // out_thermal_img = thermal_img.clone();

    // cv::Mat edge_img = extract_edges_sobel(roi_img);

    // // 配准热成像图像到可见光图像ROI

    // AdjustParams adjust;
    // adjust.shift_x = 35;   // 往右移动10个像素
    // adjust.shift_y = 0;    
    // adjust.scale = 0.85;    // 不缩放
    // adjust.angle = 0.0;    // 不旋转
    // cv::Mat registered_thermal = register_thermal_to_visible(thermal_img, adjust);

    // // 融合显示（简单叠加）
    // cv::Mat fusion_img;
    // double alpha = 0.4; // 透明度，可调整
    // cv::addWeighted(edge_img, alpha, registered_thermal, 1.0 - alpha, 0, fusion_img);

    // // 把融合结果更新到显示图像左边区域
    // fusion_img.copyTo(final_img(cv::Rect(0, 0, ROI_W, ROI_H)));
    

    // === 步骤 4: 显示窗口 ===
    cv::imshow("Fusion Display", thermal_img);
    cv::waitKey(1);

    return 0;
}


void* opencv_thread(void *arg){
    thread_context_t* ctx = (thread_context_t*)arg;
    // int argc = ctx->thread_args.argc;
    // char **argv = ctx->thread_args.argv;
    // printf("opencv_thread!!!\n");
    cv::Mat save_bgr, save_thermal;

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
        // draw_roi_frame(ctx->yuv_buf.yuv_data);
        // cv_show_heimann_classic(&ctx->thermal_buf.thermal_data[0][0]);
        cv_show_fusion_display(&ctx->thermal_buf.thermal_data[0][0], ctx->yuv_buf.yuv_data, save_bgr, save_thermal);
        if (ctx->cmd_req.snapshot_request) {
            // 保存热成像和可见光图像
            static int snapshot_count = 0;
            char filename_rgb[128];
            char filename_thermal[128];
            snprintf(filename_rgb, sizeof(filename_rgb), "snapshot_rgb_%d.png", snapshot_count);
            snprintf(filename_thermal, sizeof(filename_thermal), "snapshot_thermal_%d.png", snapshot_count);
        
            // 保存
            cv::imwrite(filename_rgb, save_bgr);
            cv::imwrite(filename_thermal, save_thermal);
        
            printf("Saved snapshot %d\n", snapshot_count);
            snapshot_count++;
            if (snapshot_count >= 4) {
                printf("Already captured 4 snapshots, ignoring further requests.\n");
            }
        
            ctx->cmd_req.snapshot_request = 0; // 重置请求标志
        }

        
        

        pthread_mutex_unlock(&ctx->fusion_buf.mutex);
    }

    return NULL;

}
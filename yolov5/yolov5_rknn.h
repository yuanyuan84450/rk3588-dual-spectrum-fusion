#ifndef YOLOV5_RKNN_H
#define YOLOV5_RKNN_H

#include <cstdint>
#include <opencv2/opencv.hpp>

struct yolov5_timing_t {
    uint64_t preprocess_us;
    uint64_t input_set_us;
    uint64_t inference_us;
    uint64_t output_get_us;
    uint64_t postprocess_us;
    uint64_t total_us;
};

int yolov5_init(const char *model_path);
void yolov5_deinit();
bool yolov5_is_initialized();
int yolov5_detect(const cv::Mat &image, cv::Mat &result,
                  yolov5_timing_t *timing = nullptr);

// The caller must keep the V4L2 buffer dequeued until this synchronous call
// returns. Re-queuing it earlier allows the camera to overwrite RGA input.
int yolov5_detect_nv12_dmabuf(int dma_fd,
                              int width, int height,
                              int width_stride, int height_stride,
                              const cv::Mat &display_image,
                              cv::Mat &result,
                              yolov5_timing_t *timing = nullptr);

#endif

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <iostream>

#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "perf_stats.h"
#include "yolov5_rknn.h"

namespace {

struct DmaBuffer {
    int fd = -1;
    void *address = MAP_FAILED;
    size_t size = 0;

    ~DmaBuffer()
    {
        if (address != MAP_FAILED) munmap(address, size);
        if (fd >= 0) close(fd);
    }
};

bool allocate_nv12_dma_buffer(int width_stride, int height_stride,
                              DmaBuffer *buffer)
{
    if (!buffer || width_stride <= 0 || height_stride <= 0) return false;
    buffer->size = static_cast<size_t>(width_stride) * height_stride * 3 / 2;

    int heap_fd = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (heap_fd < 0) {
        std::cerr << "open dma-heap failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    dma_heap_allocation_data allocation{};
    allocation.len = buffer->size;
    allocation.fd_flags = O_RDWR | O_CLOEXEC;
    if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &allocation) < 0) {
        std::cerr << "DMA_HEAP_IOCTL_ALLOC failed: " << std::strerror(errno) << std::endl;
        close(heap_fd);
        return false;
    }
    close(heap_fd);
    buffer->fd = static_cast<int>(allocation.fd);
    buffer->address = mmap(nullptr, buffer->size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, buffer->fd, 0);
    if (buffer->address == MAP_FAILED) {
        std::cerr << "mmap dma-buf failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    dma_buf_sync sync{DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE};
    if (ioctl(buffer->fd, DMA_BUF_IOCTL_SYNC, &sync) < 0) {
        std::cerr << "DMA_BUF_SYNC_START failed: " << std::strerror(errno) << std::endl;
        return false;
    }
    std::memset(buffer->address, 114,
                static_cast<size_t>(width_stride) * height_stride);
    std::memset(static_cast<uint8_t *>(buffer->address) +
                    static_cast<size_t>(width_stride) * height_stride,
                128, buffer->size - static_cast<size_t>(width_stride) * height_stride);
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
    if (ioctl(buffer->fd, DMA_BUF_IOCTL_SYNC, &sync) < 0) {
        std::cerr << "DMA_BUF_SYNC_END failed: " << std::strerror(errno) << std::endl;
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char **argv)
{
    const char *model_path = argc >= 2
                                 ? argv[1]
                                 : "/home/cat/project_clean/yolov5/yolov5n.rknn";
    const int iterations = argc >= 3 ? std::max(1, std::atoi(argv[2])) : 100;
    const bool use_nv12_dmabuf = argc >= 4 &&
                                std::strcmp(argv[3], "nv12-dmabuf") == 0;
    constexpr int kWarmupIterations = 5;

    if (yolov5_init(model_path) != 0) {
        return 1;
    }

    perf_metric_t preprocess, input_set, inference, output_get, postprocess, total;
    perf_metric_init(&preprocess, "yolo_preprocess");
    perf_metric_init(&input_set, "yolo_input_set");
    perf_metric_init(&inference, "yolo_inference");
    perf_metric_init(&output_get, "yolo_output_get");
    perf_metric_init(&postprocess, "yolo_postprocess");
    perf_metric_init(&total, "yolo_total");

    // Deterministic synthetic 640x360 BGR frame. It isolates the inference
    // pipeline from camera and sensor availability.
    cv::Mat input(360, 640, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Mat result;
    DmaBuffer nv12_buffer;
    if (use_nv12_dmabuf &&
        !allocate_nv12_dma_buffer(input.cols, input.rows, &nv12_buffer)) {
        yolov5_deinit();
        return 2;
    }

    for (int i = 0; i < kWarmupIterations + iterations; ++i) {
        yolov5_timing_t timing{};
        const int ret = use_nv12_dmabuf
                            ? yolov5_detect_nv12_dmabuf(
                                  nv12_buffer.fd, input.cols, input.rows,
                                  input.cols, input.rows, input, result, &timing)
                            : yolov5_detect(input, result, &timing);
        if (ret != 0) {
            std::cerr << "Benchmark inference failed at iteration " << i << std::endl;
            yolov5_deinit();
            return 3;
        }
        if (i < kWarmupIterations) continue;

        perf_metric_record(&preprocess, timing.preprocess_us);
        perf_metric_record(&input_set, timing.input_set_us);
        perf_metric_record(&inference, timing.inference_us);
        perf_metric_record(&output_get, timing.output_get_us);
        perf_metric_record(&postprocess, timing.postprocess_us);
        perf_metric_record(&total, timing.total_us);
    }

    std::cout << "[BENCH] mode=" << (use_nv12_dmabuf ? "nv12-dmabuf" : "bgr")
              << " warmup=" << kWarmupIterations
              << " measured=" << iterations << std::endl;
    perf_metric_print(&preprocess);
    perf_metric_print(&input_set);
    perf_metric_print(&inference);
    perf_metric_print(&output_get);
    perf_metric_print(&postprocess);
    perf_metric_print(&total);

    yolov5_deinit();
    return 0;
}

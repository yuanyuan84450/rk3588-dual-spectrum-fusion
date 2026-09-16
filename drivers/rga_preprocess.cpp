#include "rga_preprocess.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

#include <rga/im2d.h>
#include <rga/rga.h>

namespace {

bool rga_status_ok(IM_STATUS status, const char *operation)
{
    if (status == IM_STATUS_SUCCESS || status == IM_STATUS_NOERROR) return true;
    std::cerr << "[RGA] " << operation << " failed: " << imStrError(status)
              << " (" << static_cast<int>(status) << ')' << std::endl;
    return false;
}

bool run_letterbox(rga_buffer_t src, int src_width, int src_height,
                   rga_buffer_t dst, int dst_width, int dst_height,
                   rga_letterbox_t *letterbox)
{
    if (!letterbox || src_width <= 0 || src_height <= 0 ||
        dst_width <= 0 || dst_height <= 0) {
        return false;
    }

    letterbox->scale = std::min(static_cast<float>(dst_width) / src_width,
                                static_cast<float>(dst_height) / src_height);
    int scaled_width = std::max(1, static_cast<int>(std::round(src_width * letterbox->scale)));
    int scaled_height = std::max(1, static_cast<int>(std::round(src_height * letterbox->scale)));

    // RGA YUV operations require even coordinates and dimensions.
    scaled_width &= ~1;
    scaled_height &= ~1;
    letterbox->pad_x = ((dst_width - scaled_width) / 2) & ~1;
    letterbox->pad_y = ((dst_height - scaled_height) / 2) & ~1;

    const im_rect full_dst = {0, 0, dst_width, dst_height};
    if (!rga_status_ok(imfill(dst, full_dst, 0x00727272, 1), "letterbox fill")) {
        return false;
    }

    const im_rect src_rect = {0, 0, src_width, src_height};
    const im_rect dst_rect = {letterbox->pad_x, letterbox->pad_y,
                              scaled_width, scaled_height};
    const im_rect empty_rect = {0, 0, 0, 0};
    rga_buffer_t empty_buffer{};

    IM_STATUS check = imcheck(src, dst, src_rect, dst_rect);
    if (!rga_status_ok(check, "capability check")) return false;

    IM_STATUS status = improcess(src, dst, empty_buffer,
                                 src_rect, dst_rect, empty_rect,
                                 -1, nullptr, nullptr, IM_SYNC);
    return rga_status_ok(status, "resize/color conversion");
}

}  // namespace

bool rga_letterbox_bgr_to_rgb(const void *src_virtual,
                              int src_width, int src_height, int src_width_stride,
                              int dst_dma_fd, int dst_width, int dst_height,
                              rga_letterbox_t *letterbox)
{
    if (!src_virtual || dst_dma_fd < 0 || src_width_stride < src_width) return false;

    rga_buffer_t src = wrapbuffer_virtualaddr(const_cast<void *>(src_virtual),
                                               src_width, src_height,
                                               RK_FORMAT_BGR_888,
                                               src_width_stride, src_height);
    rga_buffer_t dst = wrapbuffer_fd(dst_dma_fd, dst_width, dst_height,
                                     RK_FORMAT_RGB_888);
    return run_letterbox(src, src_width, src_height, dst, dst_width, dst_height,
                         letterbox);
}

bool rga_letterbox_nv12_to_rgb(int src_dma_fd,
                               int src_width, int src_height,
                               int src_width_stride, int src_height_stride,
                               int dst_dma_fd, int dst_width, int dst_height,
                               rga_letterbox_t *letterbox)
{
    if (src_dma_fd < 0 || dst_dma_fd < 0 || src_width_stride < src_width ||
        src_height_stride < src_height) {
        return false;
    }

    rga_buffer_t src = wrapbuffer_fd(src_dma_fd, src_width, src_height,
                                     RK_FORMAT_YCbCr_420_SP,
                                     src_width_stride, src_height_stride);
    rga_buffer_t dst = wrapbuffer_fd(dst_dma_fd, dst_width, dst_height,
                                     RK_FORMAT_RGB_888);
    return run_letterbox(src, src_width, src_height, dst, dst_width, dst_height,
                         letterbox);
}

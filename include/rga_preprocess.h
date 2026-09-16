#ifndef RGA_PREPROCESS_H
#define RGA_PREPROCESS_H

struct rga_letterbox_t {
    float scale;
    int pad_x;
    int pad_y;
};

bool rga_letterbox_bgr_to_rgb(const void *src_virtual,
                              int src_width, int src_height, int src_width_stride,
                              int dst_dma_fd, int dst_width, int dst_height,
                              rga_letterbox_t *letterbox);

bool rga_letterbox_nv12_to_rgb(int src_dma_fd,
                               int src_width, int src_height,
                               int src_width_stride, int src_height_stride,
                               int dst_dma_fd, int dst_width, int dst_height,
                               rga_letterbox_t *letterbox);

#endif

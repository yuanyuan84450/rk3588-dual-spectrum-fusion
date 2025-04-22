#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>

#include "mix415_drv.h"
#include "opencv_draw.h"

struct buffer {
    void *start;
    size_t length;
};

static struct buffer *buffers;

int cam_main(int argc, char *argv[]) {
    const char *device = (argc == 4) ? argv[3] : CAM_DEVICE;
    int fd = open(device, O_RDWR);
    if (fd == -1) {
        perror("Opening video device");
        return 1;
    }

    // 设置多平面图像格式
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = MIX_WIDTH;
    fmt.fmt.pix_mp.height = MIX_HEIGHT;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("Setting Pixel Format");
        close(fd);
        return 1;
    }

    // 请求缓冲区
    struct v4l2_requestbuffers req = {0};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("Requesting Buffers");
        close(fd);
        return 1;
    }

    // mmap 映射缓冲区
    buffers = calloc(req.count, sizeof(*buffers));
    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("Querying Buffer");
            close(fd);
            return 1;
        }

        buffers[i].length = buf.m.planes[0].length;
        buffers[i].start = mmap(NULL, buf.m.planes[0].length,
                                PROT_READ | PROT_WRITE, MAP_SHARED,
                                fd, buf.m.planes[0].m.mem_offset);
    }

    // 入队缓冲区
    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;

        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Queue Buffer");
            close(fd);
            return 1;
        }
    }

    // 启动采集
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("Stream On");
        close(fd);
        return 1;
    }

    while (1) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;

        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            perror("Dequeue Buffer");
            continue;
        }

        // 提取 Y 分量（灰度图像）并访问 ROI
        uint8_t *y_plane = (uint8_t *)buffers[buf.index].start;
        draw_roi_frame(y_plane);
        // for (int y = 0; y < ROI_H; y++) {
        //     uint8_t *line = y_plane + (ROI_Y + y) * MIX_WIDTH + ROI_X;
            
            // // 这里可处理 line 中的 ROI_W 宽度数据，比如融合热成像
            // for(int x=0; x<ROI_W; x++){
            //     uint8_t gray = line[x];
            //     if(x==ROI_W/2 && y ==ROI_H/2)
            //         printf("x[ROI_H/2][ROI_W/2]=%d\n",gray);
            // }

        // }

        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Requeue Buffer");
        }
        usleep(1);
    }

    // 停止采集
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    close(fd);
    return 0;
}

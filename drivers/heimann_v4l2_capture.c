#include "heimann_v4l2_capture.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#define HEIMANN_V4L2_WIDTH 32u
#define HEIMANN_V4L2_HEIGHT 32u
#define HEIMANN_V4L2_FRAME_SIZE 2580u
#define HEIMANN_V4L2_BUFFER_COUNT 4u
#define V4L2_PIX_FMT_HTPA v4l2_fourcc('H', 'T', 'P', 'A')

struct mapped_buffer {
    void *address;
    size_t length;
};

struct heimann_v4l2_capture {
    int fd;
    int streaming;
    unsigned int buffer_count;
    struct mapped_buffer *buffers;
};

static int xioctl(int fd, unsigned long request, void *argument)
{
    int ret;

    do {
        ret = ioctl(fd, request, argument);
    } while (ret < 0 && errno == EINTR);
    return ret;
}

void heimann_v4l2_capture_close(heimann_v4l2_capture_t *capture)
{
    unsigned int i;

    if (!capture)
        return;

    if (capture->streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(capture->fd, VIDIOC_STREAMOFF, &type);
    }

    for (i = 0; i < capture->buffer_count; ++i) {
        if (capture->buffers[i].address &&
            capture->buffers[i].address != MAP_FAILED) {
            munmap(capture->buffers[i].address,
                   capture->buffers[i].length);
        }
    }

    free(capture->buffers);
    if (capture->fd >= 0)
        close(capture->fd);
    free(capture);
}

int heimann_v4l2_capture_open(const char *device,
                              heimann_v4l2_capture_t **out_capture)
{
    heimann_v4l2_capture_t *capture;
    struct v4l2_capability capability = {0};
    struct v4l2_format format = {0};
    struct v4l2_requestbuffers request = {0};
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    unsigned int capabilities;
    unsigned int i;

    if (!device || !out_capture)
        return -EINVAL;
    *out_capture = NULL;

    capture = calloc(1, sizeof(*capture));
    if (!capture)
        return -ENOMEM;
    capture->fd = -1;

    capture->fd = open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (capture->fd < 0)
        goto fail;

    if (xioctl(capture->fd, VIDIOC_QUERYCAP, &capability) < 0)
        goto fail;

    capabilities = capability.capabilities & V4L2_CAP_DEVICE_CAPS
                       ? capability.device_caps
                       : capability.capabilities;
    if (!(capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(capabilities & V4L2_CAP_STREAMING)) {
        errno = ENOTSUP;
        goto fail;
    }

    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = HEIMANN_V4L2_WIDTH;
    format.fmt.pix.height = HEIMANN_V4L2_HEIGHT;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_HTPA;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(capture->fd, VIDIOC_S_FMT, &format) < 0)
        goto fail;
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_HTPA ||
        format.fmt.pix.sizeimage != HEIMANN_V4L2_FRAME_SIZE) {
        errno = EPROTO;
        goto fail;
    }

    request.count = HEIMANN_V4L2_BUFFER_COUNT;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(capture->fd, VIDIOC_REQBUFS, &request) < 0 ||
        request.count < 2) {
        goto fail;
    }

    capture->buffers = calloc(request.count, sizeof(*capture->buffers));
    if (!capture->buffers)
        goto fail;
    capture->buffer_count = request.count;

    for (i = 0; i < capture->buffer_count; ++i) {
        struct v4l2_buffer buffer = {0};

        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (xioctl(capture->fd, VIDIOC_QUERYBUF, &buffer) < 0)
            goto fail;

        capture->buffers[i].length = buffer.length;
        capture->buffers[i].address = mmap(NULL, buffer.length,
                                           PROT_READ | PROT_WRITE,
                                           MAP_SHARED, capture->fd,
                                           buffer.m.offset);
        if (capture->buffers[i].address == MAP_FAILED)
            goto fail;

        if (xioctl(capture->fd, VIDIOC_QBUF, &buffer) < 0)
            goto fail;
    }

    if (xioctl(capture->fd, VIDIOC_STREAMON, &type) < 0)
        goto fail;
    capture->streaming = 1;
    *out_capture = capture;
    return 0;

fail:
    {
        int saved_errno = errno ? errno : EIO;
        heimann_v4l2_capture_close(capture);
        return -saved_errno;
    }
}

ssize_t heimann_v4l2_capture_frame(heimann_v4l2_capture_t *capture,
                                   void *destination, size_t destination_size,
                                   uint64_t *timestamp_us,
                                   uint32_t *sequence)
{
    struct pollfd poll_fd;
    struct v4l2_buffer buffer = {0};

    if (!capture || !destination || destination_size < HEIMANN_V4L2_FRAME_SIZE)
        return -EINVAL;

    poll_fd.fd = capture->fd;
    poll_fd.events = POLLIN;
    poll_fd.revents = 0;
    for (;;) {
        int ret = poll(&poll_fd, 1, 1000);

        if (ret > 0)
            break;
        if (ret == 0)
            return -ETIMEDOUT;
        if (errno != EINTR)
            return -errno;
    }

    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (xioctl(capture->fd, VIDIOC_DQBUF, &buffer) < 0)
        return -errno;

    if (buffer.index >= capture->buffer_count ||
        buffer.bytesused != HEIMANN_V4L2_FRAME_SIZE) {
        xioctl(capture->fd, VIDIOC_QBUF, &buffer);
        return -EPROTO;
    }

    memcpy(destination, capture->buffers[buffer.index].address,
           HEIMANN_V4L2_FRAME_SIZE);
    if (timestamp_us) {
        *timestamp_us = (uint64_t)buffer.timestamp.tv_sec * 1000000ull +
                        buffer.timestamp.tv_usec;
    }
    if (sequence)
        *sequence = buffer.sequence;

    if (xioctl(capture->fd, VIDIOC_QBUF, &buffer) < 0)
        return -errno;
    return HEIMANN_V4L2_FRAME_SIZE;
}

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define HTPA_WIDTH 32u
#define HTPA_HEIGHT 32u
#define HTPA_FRAME_SIZE 2580u
#define V4L2_PIX_FMT_HTPA v4l2_fourcc('H', 'T', 'P', 'A')
#define BUFFER_COUNT 4u

struct mapped_buffer {
    void *address;
    size_t length;
};

static int xioctl(int fd, unsigned long request, void *argument)
{
    int ret;

    do {
        ret = ioctl(fd, request, argument);
    } while (ret < 0 && errno == EINTR);

    return ret;
}

static uint64_t monotonic_ns(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

int main(int argc, char **argv)
{
    const char *device = argc > 1 ? argv[1] : "/dev/video0";
    unsigned int frame_limit = argc > 2 ? (unsigned int)strtoul(argv[2], NULL, 10) : 100;
    struct mapped_buffer buffers[BUFFER_COUNT] = {0};
    struct v4l2_capability capability = {0};
    struct v4l2_format format = {0};
    struct v4l2_requestbuffers request = {0};
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uint64_t begin_ns;
    uint64_t previous_timestamp_us = 0;
    unsigned int sequence_gaps = 0;
    unsigned int received = 0;
    unsigned int mapped = 0;
    int streaming = 0;
    int fd = -1;
    int ret = EXIT_FAILURE;

    if (frame_limit == 0)
        frame_limit = 1;

    fd = open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", device, strerror(errno));
        goto cleanup;
    }

    if (xioctl(fd, VIDIOC_QUERYCAP, &capability) < 0) {
        perror("VIDIOC_QUERYCAP");
        goto cleanup;
    }

    printf("driver=%s card=%s bus=%s\n",
           capability.driver, capability.card, capability.bus_info);

    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = HTPA_WIDTH;
    format.fmt.pix.height = HTPA_HEIGHT;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_HTPA;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &format) < 0) {
        perror("VIDIOC_S_FMT");
        goto cleanup;
    }

    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_HTPA ||
        format.fmt.pix.sizeimage != HTPA_FRAME_SIZE) {
        fprintf(stderr, "unexpected format fourcc=%c%c%c%c size=%u\n",
                format.fmt.pix.pixelformat & 0xff,
                (format.fmt.pix.pixelformat >> 8) & 0xff,
                (format.fmt.pix.pixelformat >> 16) & 0xff,
                (format.fmt.pix.pixelformat >> 24) & 0xff,
                format.fmt.pix.sizeimage);
        goto cleanup;
    }

    request.count = BUFFER_COUNT;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &request) < 0 || request.count < 2) {
        perror("VIDIOC_REQBUFS");
        goto cleanup;
    }

    for (mapped = 0; mapped < request.count; ++mapped) {
        struct v4l2_buffer buffer = {0};

        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = mapped;
        if (xioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) {
            perror("VIDIOC_QUERYBUF");
            goto cleanup;
        }

        buffers[mapped].length = buffer.length;
        buffers[mapped].address = mmap(NULL, buffer.length,
                                      PROT_READ | PROT_WRITE, MAP_SHARED,
                                      fd, buffer.m.offset);
        if (buffers[mapped].address == MAP_FAILED) {
            buffers[mapped].address = NULL;
            perror("mmap");
            goto cleanup;
        }

        if (xioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
            perror("VIDIOC_QBUF");
            goto cleanup;
        }
    }

    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        goto cleanup;
    }
    streaming = 1;
    begin_ns = monotonic_ns();

    while (received < frame_limit) {
        struct pollfd poll_fd = {.fd = fd, .events = POLLIN};
        struct v4l2_buffer buffer = {0};
        uint64_t timestamp_us;

        if (poll(&poll_fd, 1, 3000) <= 0) {
            fprintf(stderr, "poll timeout/error after %u frames: %s\n",
                    received, strerror(errno));
            goto cleanup;
        }

        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN)
                continue;
            perror("VIDIOC_DQBUF");
            goto cleanup;
        }

        if (buffer.index >= request.count || buffer.bytesused != HTPA_FRAME_SIZE) {
            fprintf(stderr, "invalid buffer index=%u bytesused=%u\n",
                    buffer.index, buffer.bytesused);
            goto cleanup;
        }

        timestamp_us = (uint64_t)buffer.timestamp.tv_sec * 1000000ull +
                       buffer.timestamp.tv_usec;
        if (previous_timestamp_us && timestamp_us < previous_timestamp_us) {
            fprintf(stderr, "non-monotonic timestamp at sequence=%u\n",
                    buffer.sequence);
            goto cleanup;
        }
        previous_timestamp_us = timestamp_us;

        if (received && buffer.sequence != received)
            sequence_gaps++;
        received++;

        if (xioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
            perror("VIDIOC_QBUF(requeue)");
            goto cleanup;
        }
    }

    {
        double seconds = (monotonic_ns() - begin_ns) / 1000000000.0;
        printf("received=%u sequence_gaps=%u elapsed=%.3f s fps=%.2f\n",
               received, sequence_gaps, seconds, received / seconds);
    }
    ret = EXIT_SUCCESS;

cleanup:
    if (streaming)
        xioctl(fd, VIDIOC_STREAMOFF, &type);
    while (mapped > 0) {
        mapped--;
        if (buffers[mapped].address)
            munmap(buffers[mapped].address, buffers[mapped].length);
    }
    if (fd >= 0)
        close(fd);
    return ret;
}

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
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <sys/types.h>

#include "mix415_drv.h"
#include "opencv_draw.h"
#include "public_cfg.h"

#define TARGET_FPS 30
#define MAX_DEQUEUE_FAIL 10
#define BUFFER_COUNT 8 // 调大缓冲区数量，避免丢帧

/*static inline uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}*/

struct buffer {
    void *start;
    size_t length;
};

static struct buffer *buffers;

void* camera_thread(void *arg) {

    static uint64_t cam_frame_id = 0;
    pthread_setname_np(pthread_self(), "camera");
    pid_t tid = syscall(SYS_gettid);
    printf("camera_thread start, tid=%d\n", tid);

    thread_context_t* ctx = (thread_context_t*)arg;
    int argc = ctx->thread_args.argc;
    char **argv = ctx->thread_args.argv;

    const char *device = (argc >= 3) ? argv[2] : CAM_DEVICE;
    // int fd = open(device, O_RDWR | O_NONBLOCK);
    int fd = open(device, O_RDWR);
    if (fd == -1) {
        perror("Opening video device failed!");
        return (void*)-1;
    }

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
        return (void*)-1;
    }

    struct v4l2_requestbuffers req = {0};
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("Requesting Buffers");
        close(fd);
        return (void*)-1;
    }

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
            return (void*)-1;
        }

        buffers[i].length = buf.m.planes[0].length;
        buffers[i].start = mmap(NULL, buf.m.planes[0].length,
                                PROT_READ | PROT_WRITE, MAP_SHARED,
                                fd, buf.m.planes[0].m.mem_offset);
    }

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
            return (void*)-1;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("Stream On");
        close(fd);
        return (void*)-1;
    }

    int dequeue_fail_count = 0;

    size_t frame_size = MIX_WIDTH * MIX_HEIGHT * 3 / 2; // NV12大小
    uint8_t *local_frame_buffer = malloc(frame_size); // 本地拷贝缓冲区

    while(!ctx->cmd_req.exit_req) {

        static uint64_t cap_last_print = 0;
        static int cap_fps_cnt = 0;
        static uint64_t cap_last_frame_ts = 0;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int r = select(fd + 1, &fds, NULL, NULL, &tv);
        if (r == -1) {
            if (errno == EINTR)
                continue;
            perror("Select failed");
            break;
        }
        else if (r == 0) {
            fprintf(stderr, "Camera select timeout!\n");
            continue;
        }

        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;

        usleep(1000);

        //取满数据缓冲区
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            // perror("Dequeue Buffer failed");
            // if (++dequeue_fail_count > MAX_DEQUEUE_FAIL) {
            //     fprintf(stderr, "Dequeue failed too many times, exiting...\n");
            //     break;
            // }
            // continue; //对 V4L2 设备做非阻塞打开后，如果这时 buffer 还没准备好，ioctl(fd, VIDIOC_DQBUF, &buf) 就会立刻返回 -EAGAIN continue 直接跳回去，不会做任何延时，导致 tight‐loop。
            perror("Dequeue Buffer failed");
            break;  // 阻塞模式下一般不会走到这里，直接退出更安全
        }
        
        if (cap_last_print == 0) cap_last_print = now_us();

        //测试时间
        uint64_t dq_ts = now_us();
        cap_fps_cnt++;
        if (dq_ts - cap_last_print >= 1000000ULL) {
            //printf("[CAP] fps=%d\n", cap_fps_cnt);
            cap_fps_cnt = 0;
            cap_last_print = dq_ts;
        }

        if (cap_last_frame_ts != 0) {
            //printf("摄像头两针间隔 = %.2f ms\n", (dq_ts - cap_last_frame_ts) / 1000.0);
        }
        cap_last_frame_ts = dq_ts;
        //到这

        dequeue_fail_count = 0;

        uint64_t cam_ts = now_us();
        uint64_t next_cam_frame_id = cam_frame_id + 1;

        /* 1) 先更新老模式用的 yuv_buf */
        pthread_mutex_lock(&ctx->yuv_buf.mutex);

        memcpy(local_frame_buffer, buffers[buf.index].start, frame_size);
        ctx->yuv_buf.yuv_data = local_frame_buffer;
        ctx->yuv_buf.meta.frame_id = next_cam_frame_id;
        ctx->yuv_buf.meta.ts_us = cam_ts;
        ctx->yuv_buf.updated = 1;

        pthread_cond_signal(&ctx->yuv_buf.cond);
        pthread_mutex_unlock(&ctx->yuv_buf.mutex);

        /* 2) 再更新同步模式用的 cam_queue */
        pthread_mutex_lock(&ctx->cam_queue.mutex);

        cam_frame_t *slot = &ctx->cam_queue.frames[ctx->cam_queue.write_idx];
        memcpy(slot->data, buffers[buf.index].start, frame_size);
        slot->meta.frame_id = next_cam_frame_id;
        slot->meta.ts_us = cam_ts;
        slot->valid = 1;

        ctx->cam_queue.write_idx = (ctx->cam_queue.write_idx + 1) % CAM_QUEUE_SIZE;
        cam_frame_id = next_cam_frame_id;

        pthread_mutex_unlock(&ctx->cam_queue.mutex);

        // 重新入队
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Requeue Buffer failed");
            break;
        }

        
    }
    
    for (int i = 0; i < req.count; i++) {
        munmap(buffers[i].start, buffers[i].length);
    }
    free(buffers);
    free(local_frame_buffer);

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    close(fd);

    return NULL;
}

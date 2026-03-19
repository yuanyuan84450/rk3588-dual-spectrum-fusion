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

#include "mix415_drv.h"
#include "opencv_draw.h"
#include "public_cfg.h"

#define TARGET_FPS 30
#define MAX_DEQUEUE_FAIL 10
#define BUFFER_COUNT 8 // 调大缓冲区数量，避免丢帧

// 定义一个结构体来保存映射后的信息
struct buffer {
    void *start;
    size_t length;
};

static struct buffer *buffers;

void* camera_thread(void *arg) {
    //获取线程信息
    thread_context_t* ctx = (thread_context_t*)arg;
    int argc = ctx->thread_args.argc;
    char **argv = ctx->thread_args.argv;

    const char *device = (argc == 4) ? argv[3] : CAM_DEVICE;
    // int fd = open(device, O_RDWR | O_NONBLOCK);

    //用读写方式打开V4L2设备
    int fd = open(device, O_RDWR);
    if (fd == -1) {
        perror("Opening video device failed!");
        return (void*)-1;
    }

    //设置图像格式
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;  //视频采集类型 多平面
    fmt.fmt.pix_mp.width = MIX_WIDTH;   //宽640
    fmt.fmt.pix_mp.height = MIX_HEIGHT;     //高360
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12; //像素格式 NV12 每一帧压缩率高345kb
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;  //逐行扫描

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("Setting Pixel Format");
        close(fd);
        return (void*)-1;
    }

    //申请帧缓冲区，让内核在内核空间分配内存，用于存放图像数据
    struct v4l2_requestbuffers req = {0};
    req.count = BUFFER_COUNT;  //8个缓冲区
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;  //视频采集类型 多平面
    req.memory = V4L2_MEMORY_MMAP;    //内存映射方式
    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("Requesting Buffers");
        close(fd);
        return (void*)-1;
    }

    //内存映射  把内核空间的缓冲区映射到用户空间
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

        //映射每个平面
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("Querying Buffer");
            close(fd);
            return (void*)-1;
        }

        //执行mmap映射
        buffers[i].length = buf.m.planes[0].length;
        buffers[i].start = mmap(NULL,                       // 让内核选择映射地址
                                buf.m.planes[0].length,     // 长度
                                PROT_READ | PROT_WRITE,     // 可读可写
                                MAP_SHARED,                 // 与内核共享
                                fd,                         //设备文件描述符
                                buf.m.planes[0].m.mem_offset);  // 内核空间的偏移量
    }


    //把空的缓冲区放入 “输入队列”，让摄像头往里填数据 多平面比单平面要多
    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];   //这个数组来存单个平面的信息，因为好几个平面所以是数组
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;   //操作多平面
        buf.memory = V4L2_MEMORY_MMAP;                   //内存交换方式是内存映射模式
        buf.index = i;                                   //缓冲区编号
        buf.length = VIDEO_MAX_PLANES;                   //几个平面
        buf.m.planes = planes;                           //多平面指向v4l2_plane数组

        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Queue Buffer");
            close(fd);
            return (void*)-1;
        }
    }

    //启动摄像头数据流 开始采集
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("Stream On");
        close(fd);
        return (void*)-1;
    }

    int dequeue_fail_count = 0;

    size_t frame_size = MIX_WIDTH * MIX_HEIGHT * 3 / 2; // NV12大小
    uint8_t *local_frame_buffer = malloc(frame_size); // 本地拷贝缓冲区


    //主循环负责等待数据并取出
    while (!ctx->cmd_req.exit_req) {

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int r = select(fd + 1, &fds, NULL, NULL, &tv);
        //出错
        if (r == -1) {
            if (errno == EINTR)
                continue;
            perror("Select failed");
            break;
        }
        //超时
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
        dequeue_fail_count = 0;

        pthread_mutex_lock(&ctx->yuv_buf.mutex);

        //可以优化性能 应该
        memcpy(local_frame_buffer, buffers[buf.index].start, frame_size); // 拷贝一份，避免撕裂
        ctx->yuv_buf.yuv_data = local_frame_buffer; 
        ctx->yuv_buf.updated = 1;

        pthread_cond_signal(&ctx->yuv_buf.cond);
        pthread_mutex_unlock(&ctx->yuv_buf.mutex);

        // 重新入队
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Requeue Buffer failed");
            break;
        }

        usleep(100);
    }

    //这个很关键 对取消采集和解除内存映射进行调换
    //先让硬件停止工作，再释放硬件正在访问的资源。
    //取消采集
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    //解除内存映射
    for (int i = 0; i < req.count; i++) {
        munmap(buffers[i].start, buffers[i].length);
    }
    free(buffers);
    free(local_frame_buffer);

    //关闭设备
    close(fd);

    return NULL;
}

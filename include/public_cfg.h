#ifndef _PUBLIC_CFG_H
#define _PUBLIC_CFG_H


#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>

#define TH_THERMAL_ROWS 32
#define TH_THERMAL_COLS 32
#define TH_MIX_WIDTH 640
#define TH_MIX_HEIGHT 360
#define TH_YUV_FRAME_SIZE (TH_MIX_WIDTH * TH_MIX_HEIGHT *3/2)
#define CAM_QUEUE_SIZE 8 //新增

//新增
typedef struct {
    uint64_t frame_id; //这是第几帧   //还要帧号 看匹配到的是哪一帧
    uint64_t ts_us;  //这帧生成的时间
} frame_meta_t;

//可见光的一帧
typedef struct {
    uint8_t *data;          // 一帧 NV12 数据
    size_t size;            // 帧大小
    int valid;              // 这个槽位里当前有没有有效帧
    frame_meta_t meta;      // frame_id + ts 这帧的编号和时间戳
} cam_frame_t;

//最近几帧可见光组成的缓存队列
typedef struct {
    pthread_mutex_t mutex; //保护整个队列的访问
    cam_frame_t frames[CAM_QUEUE_SIZE]; //一排缓存槽位 结构体数组
    int write_idx; //下一帧应该写到哪里
} cam_queue_t;
//
typedef struct {
    uint16_t thermal_data[TH_THERMAL_ROWS][TH_THERMAL_COLS];
    pthread_mutex_t mutex; //互斥量
    pthread_cond_t cond;   //条件变量
    int updated;
    frame_meta_t meta; //新增
} thermal_buffer_t;

typedef struct {
    uint8_t *yuv_data;//[TH_YUV_FRAME_SIZE];
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int updated;
} yuv_buffer_t;

typedef struct {
    uint16_t fusion_data[256][256];
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int updated;
} fusion_buffer_t;

typedef struct {
    int argc;
    char **argv;
} thread_args_t;

typedef struct {
    volatile sig_atomic_t snapshot_request;
    volatile sig_atomic_t print_eeprom_header_req;
    volatile sig_atomic_t print_eeprom_hex_req;
    volatile sig_atomic_t exit_req;
    volatile sig_atomic_t colormap_ctrl;
    volatile sig_atomic_t yolo_req;
    volatile sig_atomic_t edge_req;
    volatile sig_atomic_t sync_req;

} cmd_request_t;

typedef struct {
    thread_args_t thread_args;
    thermal_buffer_t thermal_buf;
    yuv_buffer_t yuv_buf;
    fusion_buffer_t fusion_buf;
    cmd_request_t cmd_req;

    cam_queue_t cam_queue; //新增
} thread_context_t;




/**
 * @brief 初始化 thread_context_t 及其内部所有锁和条件变量
 * @param out_ctx 输出参数，返回初始化好的上下文指针
 * @return 成功返回 0，失败返回 -1
 */
int thread_context_init(thread_context_t **out_ctx);

/**
 * @brief 销毁 thread_context_t 及其内部所有锁和条件变量，并释放内存
 * @param ctx 要销毁的上下文指针
 * @note 如果 ctx->yuv_buf.yuv_data 是你自己 malloc 的，请在调用此函数前先 free
 */
void thread_context_destroy(thread_context_t *ctx);

static inline uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

#endif
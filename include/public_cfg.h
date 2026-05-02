#ifndef _PUBLIC_CFG_H
#define _PUBLIC_CFG_H

#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>

#define TH_THERMAL_ROWS 32
#define TH_THERMAL_COLS 32
#define TH_MIX_WIDTH 640
#define TH_MIX_HEIGHT 360
#define TH_YUV_FRAME_SIZE (TH_MIX_WIDTH * TH_MIX_HEIGHT * 3 / 2)
#define CAM_QUEUE_SIZE 15
#define THERMAL_QUEUE_SIZE 8

typedef struct {
    uint64_t frame_id;
    uint64_t ts_us;
} frame_meta_t;

typedef struct {
    uint8_t *data;
    size_t size;
    int valid;
    frame_meta_t meta;
} cam_frame_t;

typedef struct {
    pthread_mutex_t mutex;
    cam_frame_t frames[CAM_QUEUE_SIZE];
    int write_idx;
} cam_queue_t;

typedef struct {
    uint16_t data[TH_THERMAL_ROWS][TH_THERMAL_COLS];
    int valid;
    frame_meta_t meta;
} thermal_frame_t;

typedef struct {
    pthread_mutex_t mutex;
    thermal_frame_t frames[THERMAL_QUEUE_SIZE];
    int write_idx;
} thermal_queue_t;

typedef struct {
    uint16_t thermal_data[TH_THERMAL_ROWS][TH_THERMAL_COLS];
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int updated;
    frame_meta_t meta;
} thermal_buffer_t;

typedef struct {
    uint8_t *yuv_data;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int updated;
    frame_meta_t meta;
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
    cam_queue_t cam_queue;
    thermal_queue_t thermal_queue;
} thread_context_t;

int thread_context_init(thread_context_t **out_ctx);
void thread_context_destroy(thread_context_t *ctx);
int init_cam_queue(cam_queue_t *q, size_t frame_size);
void destroy_cam_queue(cam_queue_t *q);
int init_thermal_queue(thermal_queue_t *q);
void destroy_thermal_queue(thermal_queue_t *q);

static inline uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

#endif

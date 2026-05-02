#include "public_cfg.h"
#include <stdio.h>
#include <stdlib.h>

int init_cam_queue(cam_queue_t *q, size_t frame_size) {
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->mutex, NULL);
    q->write_idx = 0;

    for (int i = 0; i < CAM_QUEUE_SIZE; i++) {
        q->frames[i].data = (uint8_t *)malloc(frame_size);
        if (!q->frames[i].data) {
            perror("malloc cam queue frame failed");
            return -1;
        }
        q->frames[i].size = frame_size;
        q->frames[i].valid = 0;
        q->frames[i].meta.frame_id = 0;
        q->frames[i].meta.ts_us = 0;
    }
    return 0;
}

int init_thermal_queue(thermal_queue_t *q) {
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->mutex, NULL);
    q->write_idx = 0;

    for (int i = 0; i < THERMAL_QUEUE_SIZE; i++) {
        q->frames[i].valid = 0;
        q->frames[i].meta.frame_id = 0;
        q->frames[i].meta.ts_us = 0;
    }
    return 0;
}

void destroy_cam_queue(cam_queue_t *q) {
    for (int i = 0; i < CAM_QUEUE_SIZE; i++) {
        free(q->frames[i].data);
        q->frames[i].data = NULL;
    }
    pthread_mutex_destroy(&q->mutex);
}

void destroy_thermal_queue(thermal_queue_t *q) {
    pthread_mutex_destroy(&q->mutex);
}

int thread_context_init(thread_context_t **out_ctx) {
    if (out_ctx == NULL) {
        fprintf(stderr, "Error: out_ctx is NULL\n");
        return -1;
    }

    // 1. 分配内存并清零 (calloc 会将所有成员初始化为 0/NULL，更安全)
    thread_context_t *ctx = (thread_context_t *)calloc(1, sizeof(thread_context_t));
    if (ctx == NULL) {
        perror("calloc failed for thread_context_t");
        return -1;
    }

    // -------------------------------------------------------------------------
    // 2. 初始化 thermal_buf 的锁和条件变量
    // -------------------------------------------------------------------------
    if (pthread_mutex_init(&ctx->thermal_buf.mutex, NULL) != 0) {
        perror("init thermal_buf.mutex failed");
        free(ctx);
        return -1;
    }
    if (pthread_cond_init(&ctx->thermal_buf.cond, NULL) != 0) {
        perror("init thermal_buf.cond failed");
        pthread_mutex_destroy(&ctx->thermal_buf.mutex);
        free(ctx);
        return -1;
    }

    // -------------------------------------------------------------------------
    // 3. 初始化 yuv_buf 的锁和条件变量
    // -------------------------------------------------------------------------
    if (pthread_mutex_init(&ctx->yuv_buf.mutex, NULL) != 0) {
        perror("init yuv_buf.mutex failed");
        pthread_cond_destroy(&ctx->thermal_buf.cond);
        pthread_mutex_destroy(&ctx->thermal_buf.mutex);
        free(ctx);
        return -1;
    }
    if (pthread_cond_init(&ctx->yuv_buf.cond, NULL) != 0) {
        perror("init yuv_buf.cond failed");
        pthread_mutex_destroy(&ctx->yuv_buf.mutex);
        pthread_cond_destroy(&ctx->thermal_buf.cond);
        pthread_mutex_destroy(&ctx->thermal_buf.mutex);
        free(ctx);
        return -1;
    }

    // -------------------------------------------------------------------------
    // 4. 初始化 fusion_buf 的锁和条件变量
    // -------------------------------------------------------------------------
    if (pthread_mutex_init(&ctx->fusion_buf.mutex, NULL) != 0) {
        perror("init fusion_buf.mutex failed");
        pthread_cond_destroy(&ctx->yuv_buf.cond);
        pthread_mutex_destroy(&ctx->yuv_buf.mutex);
        pthread_cond_destroy(&ctx->thermal_buf.cond);
        pthread_mutex_destroy(&ctx->thermal_buf.mutex);
        free(ctx);
        return -1;
    }
    if (pthread_cond_init(&ctx->fusion_buf.cond, NULL) != 0) {
        perror("init fusion_buf.cond failed");
        pthread_mutex_destroy(&ctx->fusion_buf.mutex);
        pthread_cond_destroy(&ctx->yuv_buf.cond);
        pthread_mutex_destroy(&ctx->yuv_buf.mutex);
        pthread_cond_destroy(&ctx->thermal_buf.cond);
        pthread_mutex_destroy(&ctx->thermal_buf.mutex);
        free(ctx);
        return -1;
    }

    // -------------------------------------------------------------------------
    // 5. 初始化其他标志位 (calloc 已清零，这里显式初始化关键值增加可读性)
    // -------------------------------------------------------------------------
    ctx->thermal_buf.updated = 0;
    ctx->yuv_buf.updated = 0;
    ctx->fusion_buf.updated = 0;
    ctx->cmd_req.exit_req = 0;
    ctx->cmd_req.sync_req = 0;
    // 注意：yuv_buf.yuv_data 是 NULL，需要你在 camera_thread 里单独 malloc

    *out_ctx = ctx;
    printf("初始化成功\n");
    return 0;
}

void thread_context_destroy(thread_context_t *ctx) {
    if (ctx == NULL) {
        return;
    }

    // -------------------------------------------------------------------------
    // 按初始化的**反顺序**销毁资源
    // -------------------------------------------------------------------------
    
    // 1. 销毁 fusion_buf
    pthread_cond_destroy(&ctx->fusion_buf.cond);
    pthread_mutex_destroy(&ctx->fusion_buf.mutex);

    // 2. 销毁 yuv_buf
    pthread_cond_destroy(&ctx->yuv_buf.cond);
    pthread_mutex_destroy(&ctx->yuv_buf.mutex);
    
    // 【重要提示】如果 yuv_data 是你在其他地方 malloc 的，请在这里取消注释并 free：
    // if (ctx->yuv_buf.yuv_data) {
    //     free(ctx->yuv_buf.yuv_data);
    //     ctx->yuv_buf.yuv_data = NULL;
    // }

    // 3. 销毁 thermal_buf
    pthread_cond_destroy(&ctx->thermal_buf.cond);
    pthread_mutex_destroy(&ctx->thermal_buf.mutex);

    // 4. 释放结构体本身
    free(ctx);
    printf("thread_context destroyed successfully.\n");
}

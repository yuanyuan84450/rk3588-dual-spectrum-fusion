#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <pthread.h>

#include "heimann_drv.h"
#include "mix415_drv.h"
#include "public_cfg.h"
#include "opencv_draw.h"

void* cmd_thread(void *arg) {
    thread_context_t* ctx = (thread_context_t*)arg;
    char input[100];
    while (1) {
        if (fgets(input, sizeof(input), stdin)) {
            if (strncmp(input, "snap", 4) == 0) {
                ctx->cmd_req.snapshot_request = 1;
            }
        }
    }
}


int main(int argc, char *argv[]) 
{
    thread_context_t *ctx = malloc(sizeof(thread_context_t));
    ctx->thread_args.argc = argc;
    ctx->thread_args.argv = argv;

    pthread_t therm_tid, cam_tid, opencv_tid, cmd_tid;
    // thread_args_t *args = malloc(sizeof(thread_args_t));
    // args->argc = argc;
    // args->argv = argv;

    if (pthread_create(&therm_tid, NULL, thermal_thread, ctx) != 0) {
        perror("Failed to create thermal_thread");
        return -1;
    }

    if (pthread_create(&cam_tid, NULL, camera_thread, ctx) != 0) {
        perror("Failed to create camera_thread");
        return -1;
    }

    if (pthread_create(&opencv_tid, NULL, opencv_thread, ctx) != 0) {
        perror("Failed to create camera_thread");
        return -1;
    }

    if (pthread_create(&cmd_tid, NULL, cmd_thread, ctx) != 0) {
        perror("Failed to create camera_thread");
        return -1;
    }

    pthread_join(therm_tid, NULL);
    pthread_join(cam_tid, NULL);
    pthread_join(opencv_tid, NULL);
    pthread_join(cmd_tid, NULL);

    free(ctx);  // 线程结束后释放内存
    return 0;
}

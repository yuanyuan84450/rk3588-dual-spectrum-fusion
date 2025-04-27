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
#include "thread.h"
#include "opencv_draw.h"


// typedef struct {
//     int argc;
//     char **argv;
// } thread_args_t;

// void* thermal_thread(void* arg) 
// {
//     thread_args_t *args = (thread_args_t*)arg;
//     int argc = args->argc;
//     char **argv = args->argv;

//     sensor_main(argc, argv);

//     return NULL;
// }

// void* camera_thread(void* arg) 
// {
//     thread_args_t *args = (thread_args_t*)arg;
//     int argc = args->argc;
//     char **argv = args->argv;

//     cam_main(argc, argv);

//     return NULL;
// }

int main(int argc, char *argv[]) 
{
    thread_context_t *ctx = malloc(sizeof(thread_context_t));
    ctx->thread_args.argc = argc;
    ctx->thread_args.argv = argv;

    pthread_t therm_tid, cam_tid, opencv_tid;
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

    pthread_join(therm_tid, NULL);
    pthread_join(cam_tid, NULL);
    pthread_join(opencv_tid, NULL);

    free(ctx);  // 线程结束后释放内存
    return 0;
}

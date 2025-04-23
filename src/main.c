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




// int main(int argc, char *argv[]) {
//     pid_t pid = fork();

//     if (pid < 0) {
//         perror("fork failed");
//         exit(EXIT_FAILURE);
//     }

//     if (pid == 0) {
//         // 子进程：执行 sensor_main
//         printf("[Child] cam_main starting...\n");
//         // exit(sensor_main(argc, argv)); // 确保子进程退出时正确返回
//         exit(cam_main(argc, argv));
//     } else {
//         // 父进程：可以执行别的逻辑，或者等待子进程
//         printf("[Parent] cam_main forked, PID = %d\n", pid);

//         int status;
//         waitpid(pid, &status, 0);  // 等待子进程结束
//         if (WIFEXITED(status)) {
//             printf("[Parent] cam_main exited with status %d\n", WEXITSTATUS(status));
//         } else {
//             printf("[Parent] cam_main terminated abnormally.\n");
//         }
//     }

//     return 0;
// }

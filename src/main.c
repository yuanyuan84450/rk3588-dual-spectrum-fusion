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

#include "heimann_drv.h"
#include "mix415_drv.h"

// extern int sensor_main(int argc,char *argv[]);

int main(int argc, char *argv[]) {
    pid_t pid = fork();

    if (pid < 0) {
        perror("fork failed");
        exit(EXIT_FAILURE);
    }

    if (pid == 0) {
        // 子进程：执行 sensor_main
        printf("[Child] cam_main starting...\n");
        // exit(sensor_main(argc, argv)); // 确保子进程退出时正确返回
        exit(cam_main(argc, argv));
    } else {
        // 父进程：可以执行别的逻辑，或者等待子进程
        printf("[Parent] cam_main forked, PID = %d\n", pid);

        int status;
        waitpid(pid, &status, 0);  // 等待子进程结束
        if (WIFEXITED(status)) {
            printf("[Parent] cam_main exited with status %d\n", WEXITSTATUS(status));
        } else {
            printf("[Parent] cam_main terminated abnormally.\n");
        }
    }

    return 0;
}

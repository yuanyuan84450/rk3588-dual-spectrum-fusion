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
#include <readline/readline.h>
#include <readline/history.h>


#include "heimann_drv.h"
#include "mix415_drv.h"
#include "public_cfg.h"
#include "opencv_draw.h"

const char* commands[] = {
    "heimann-snap",
    "heimann-header",
    "heimann-hex",
    "exit",
    NULL
};

char* command_generator(const char* text, int state) {
    static int list_index, len;
    const char* name;

    if (!state) {
        list_index = 0;
        len = strlen(text);
    }

    while ((name = commands[list_index++])) {
        if (strncmp(name, text, len) == 0) {
            return strdup(name);  // readline 会 free 这个字符串
        }
    }

    return NULL;
}

char** command_completion(const char* text, int start, int end) {
    // 如果是第一个单词才触发命令补全
    if (start == 0)
        return rl_completion_matches(text, command_generator);
    return NULL;
}

void* cmd_thread(void *arg) {
    thread_context_t* ctx = (thread_context_t*)arg;

    rl_attempted_completion_function = command_completion;

    while (!ctx->cmd_req.exit_req) {
        // char* input = readline(">> ");  // 自动刷新提示符
        char* input = readline("\033[1;33m>> \033[0m");


        if (input == NULL) {
            // 可能是 Ctrl+D 或其他 EOF
            ctx->cmd_req.exit_req = 1;
            break;
        }

        if (strlen(input) > 0) {
            add_history(input);  // 可选：支持方向键历史命令
        }

        if (strncmp(input, "heimann-snap", 12) == 0) {
            ctx->cmd_req.snapshot_request = 1;
        } else if (strncmp(input, "heimann-header", 14) == 0) {
            ctx->cmd_req.print_eeprom_header_req = 1;
        } else if (strncmp(input, "heimann-hex", 11) == 0) {
            ctx->cmd_req.print_eeprom_hex_req = 1;
        } else if (strncmp(input, "exit", 4) == 0) {
            ctx->cmd_req.exit_req = 1;
        }

        free(input);  // readline 分配了内存，记得释放
    }

    return NULL;
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
    ctx->cmd_req.exit_req = 0;

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

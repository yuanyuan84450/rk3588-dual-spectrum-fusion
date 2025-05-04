#!/bin/bash

# 开启脚本严格模式：出错立即退出
set -e

# 捕获 SIGTSTP 信号并执行清理操作
trap 'echo "Ctrl+Z 被按下，释放 /dev"; sudo fuser -k /dev/video11' SIGTSTP
# 捕获 EXIT 信号并执行清理操作
trap 'echo "脚本退出，释放 /dev"; sudo fuser -k /dev/video11 /dev/i2c-5 /dev/i2c-6' EXIT

# 编译程序
make

# 运行程序
sudo ./app /dev/i2c-6 /dev/i2c-5 /dev/video11

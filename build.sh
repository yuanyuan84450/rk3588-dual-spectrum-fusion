#!/bin/bash

# 开启脚本严格模式：出错立即退出
set -e

# 设置目标文件名和目标路径
TARGET=app
DEST_DIR=/home/shenjiajie/nfs/

# 第一步：编译
echo "=== Building $TARGET ==="
make

# 第二步：复制到NFS目录
echo "=== Copying *.c *.h Makefile to $DEST_DIR ==="
# rm -rf "$DEST_DIR"/*
# cp *.c *.h Makefile "$DEST_DIR"
cp "$TARGET" "$DEST_DIR"

# 第三步：清理
echo "=== Cleaning build files ==="
make clean

echo "=== Build completed successfully ==="

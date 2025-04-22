# # 编译器使用本地 gcc
# CC := gcc

# # 源码目录
# SRCDIR := .
# TARGET := app
# SRC := $(wildcard $(SRCDIR)/*.c)

# # 静态/动态链接控制
# LINK ?= dynamic

# # 编译选项（注意：SDL2 的头文件应已安装到系统路径中）
# CFLAGS += -Wall -O2 -MMD -MP
# LDFLAGS +=
# ifeq ($(LINK),static)
#     LDFLAGS += -static
# endif

# # 依赖库
# LDLIBS := -lSDL2 -lgbm -lm

# # 默认目标
# all: $(TARGET)

# $(TARGET): $(SRC)
# 	@echo "[CC]  $@"
# 	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

# -include $(SRC:.c=.d)

# clean:
# 	@echo "[CLEAN]"
# 	rm -f *.o *.d $(TARGET)

# 编译器
CC := gcc

# 编译参数
CFLAGS := -Wall -O2 -Iinclude -MMD -MP
LDFLAGS :=
LDLIBS := -lSDL2 -lgbm -lm

# 搜索所有源文件
SRC := $(wildcard src/*.c drivers/*.c display/*.c utils/*.c)
OBJ := $(patsubst %.c, build/%.o, $(SRC))
DEP := $(OBJ:.o=.d)

# 目标名称
TARGET := app

# 默认目标
all: $(TARGET)

# 编译规则
build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS)

# 自动依赖
-include $(DEP)

# 清理
clean:
	@echo "[CLEAN]"
	rm -rf build app

.PHONY: all clean



# 编译器设置
CC := gcc
CXX := g++
CFLAGS := -Wall -O2 -Iinclude -Iopencv -MMD -MP
CXXFLAGS := -Wall -O2 -Iinclude -Iopencv -MMD -MP `pkg-config --cflags opencv4`

LDFLAGS :=
LDLIBS := -lm -lpthread -lreadline -lhistory `pkg-config --libs opencv4`

# C 源文件与目标文件
C_SRC := $(wildcard src/*.c drivers/*.c)
C_OBJ := $(patsubst %.c, build/%.o, $(C_SRC))

# C++ 源文件与目标文件
CPP_SRC := $(wildcard opencv/*.cpp)
CPP_OBJ := $(patsubst %.cpp, build/%.o, $(CPP_SRC))

# 所有对象文件
OBJ := $(C_OBJ) $(CPP_OBJ)
DEP := $(OBJ:.o=.d)

# 目标名
TARGET := app

# 默认目标
all: $(TARGET)

# C 编译规则
build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# C++ 编译规则
build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 链接目标
$(TARGET): $(OBJ)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

# 自动依赖
-include $(DEP)

# 清理
clean:
	@echo "[CLEAN]"
	rm -rf build app

.PHONY: all clean

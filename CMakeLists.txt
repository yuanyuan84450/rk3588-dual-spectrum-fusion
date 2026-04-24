# 1. CMake最低版本要求（鲁班猫4自带的CMake版本足够）
cmake_minimum_required(VERSION 3.10)

# 2. 项目名称 + 同时声明C和C++两种语言（核心！支持混合编译）
project(thermal_yolo C CXX)

# 3. 设置语言标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 4. 编译选项（Debug带调试信息，Release优化速度）
set(CMAKE_C_FLAGS_DEBUG "-g -Wall -O0")
set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# 5. 添加所有头文件搜索目录（按你的目录结构写）
include_directories(
    include
    drivers
    opencv
    yolov5
)

# 6. 自动收集所有源文件（按目录分别收集，清晰易维护）
# 收集drivers目录下的所有.c和.cpp
file(GLOB_RECURSE DRIVERS_SOURCES "drivers/*.c" "drivers/*.cpp")
# 收集opencv目录下的所有.c和.cpp
file(GLOB_RECURSE OPENCV_SOURCES "opencv/*.c" "opencv/*.cpp")
# 收集src目录下的所有.c
file(GLOB_RECURSE SRC_SOURCES "src/*.c")

file(GLOB_RECURSE YOLOV5_SOURCES "yolov5/*.c" "yolov5/*.cpp")

# 合并所有源文件
set(ALL_SOURCES
    ${DRIVERS_SOURCES}
    ${OPENCV_SOURCES}
    ${SRC_SOURCES}
    ${YOLOV5_SOURCES}
)

# 7. 查找并链接第三方库
# 7.1 查找OpenCV库
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# 7.2 查找RKNN库（鲁班猫4自带，路径固定）
find_library(RKNN_LIB rknnrt /usr/lib)
if(NOT RKNN_LIB)
    message(FATAL_ERROR "RKNN library not found! Please install rknn runtime on LubanCat 4")
endif()

# 8. 生成可执行文件
add_executable(app ${ALL_SOURCES})

# 9. 链接所有需要的库（修改后）
target_link_libraries(app PRIVATE
    ${OpenCV_LIBS}
    ${RKNN_LIB}
    pthread
    v4l2
    m
    readline    # 新增：命令行补全库
    ncurses     # 新增：readline依赖
    websockets  # 新增：WebSocket库
)
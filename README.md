# RK3588S 双光融合与边缘目标检测系统

运行在 LubanCat 4（RK3588S）上的嵌入式视觉系统。项目同步采集可见光摄像头和
Heimann 32x32 红外阵列数据，在板端完成帧同步、空间配准、伪彩融合、YOLOv5
目标检测，并通过 WebSocket 输出实时画面。

## 系统架构

```text
可见光摄像头 (V4L2/NV12) ----+
                              +--> 时间戳同步 --> 配准/融合 --> JPEG --> WebSocket
Heimann 32x32 (V4L2/HTPA) ----+                       |
                                                      +--> RKNN/NPU 目标检测
```

## 核心功能

- 双路图像采集：通过 V4L2 获取可见光和热成像数据。
- 帧同步：使用单调时间戳和有界队列匹配双路图像。
- 图像融合：完成热成像插值、伪彩映射、空间配准和透明度融合。
- 目标检测：在 RK3588S NPU 上运行 YOLOv5 RKNN 模型。
- 硬件加速：使用 RGA 完成颜色转换、缩放和 letterbox 预处理。
- 性能分析：统计采集、预处理、推理、融合和编码阶段的 P50/P95/P99 延迟。
- 实时输出：将检测与融合结果编码为 JPEG，通过 WebSocket 发送到浏览器。

## Heimann V4L2 驱动

项目包含独立的 Heimann 热成像内核驱动，主要实现：

- 将 32x32 热成像原始帧注册为自定义 `HTPA` V4L2 Capture 格式。
- 使用 videobuf2 管理 MMAP Buffer，支持 QBUF、DQBUF 和 STREAMON/OFF。
- 使用独立内核线程采集 I2C 数据，并写入 sequence 和单调时间戳。
- 保留 `/dev/heimann0` 字符设备，用于 EEPROM 控制和旧接口回退。
- 提供设备树 Overlay、udev 规则和用户态采集测试程序。

驱动源码位于 [`kernel/heimann_v4l2`](kernel/heimann_v4l2)。

## 技术栈

| 类别 | 技术 |
| --- | --- |
| 硬件 | LubanCat 4、RK3588S、Heimann 32x32、V4L2 摄像头 |
| 系统 | Debian 11、Linux 5.10、aarch64 |
| 开发语言 | C99、C++17 |
| 图像处理 | OpenCV、Rockchip RGA |
| AI 推理 | RKNN Runtime、RK3588S NPU、YOLOv5 |
| 视频与通信 | V4L2、videobuf2、JPEG、WebSocket |
| 构建工具 | CMake、Make |

## 目录结构

```text
.
├── drivers/                  # 采集、同步、融合、RGA 和 WebSocket
├── include/                  # 公共头文件与设备 UAPI
├── kernel/heimann_v4l2/      # Heimann V4L2 内核驱动
├── opencv/                   # 图像处理与绘制
├── src/                      # 主程序入口
├── tools/rknn_benchmark.cpp  # RKNN/RGA/DMA-BUF 性能基准
├── yolov5/                   # RKNN 推理封装与模型
├── docs/                     # 设计与测试文档
└── CMakeLists.txt
```

## 构建

板端需要安装 OpenCV、RKNN Runtime、RGA、libv4l2、libwebsockets、readline 和
ncurses 开发包。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

构建结果：

- `build/app`：双光融合与目标检测主程序。
- `build/rknn_benchmark`：独立的 RKNN/RGA 性能测试程序。

## 运行

```bash
sudo ./build/app /dev/heimann0 /dev/video11 ./yolov5/yolov5n.rknn
```

参数依次为 Heimann 设备、可见光 V4L2 设备和 RKNN 模型。

运行期间支持以下控制命令：

```text
yolo-req=0 / yolo-req=1
edge-req=0 / edge-req=1
sync-req=0 / sync-req=1
exit
```

## 性能基准

```bash
# RKNN 推理基准
./build/rknn_benchmark ./yolov5/yolov5n.rknn 100

# DMA-BUF -> RGA -> RKNN 链路
./build/rknn_benchmark ./yolov5/yolov5n.rknn 100 nv12-dmabuf
```

## 验证情况

- 双光应用和 RKNN 基准程序已在 RK3588S 板端完成 Release 编译。
- Heimann V4L2 驱动已完成板端编译和连续 100 帧采集。
- V4L2 compliance 基础及 streaming 共 54 项测试通过。
- STREAMON/OFF 100 次压力测试通过。
- 合成 NV12 DMA-BUF、RGA 和 RKNN 推理链路测试通过。

## 当前限制

- 可见光摄像头的 media pipeline 和设备节点需要根据实际板卡配置。
- 当前 RKNN 模型为 FP16，完整 I/O Memory 路径仍受输入类型转换和 DVFS 波动影响。
- DMA-BUF 与生产融合线程之间的 Buffer 生命周期仍需结合具体摄像头驱动继续联调。

更多设计和测试记录见 [`docs`](docs)。

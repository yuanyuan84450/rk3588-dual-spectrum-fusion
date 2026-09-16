# RK3588S 双光融合与边缘目标检测系统

运行在 LubanCat 4（RK3588S）上的嵌入式视觉系统。项目同步采集可见光摄像头与
Heimann 32x32 红外阵列数据，在板端完成时间戳匹配、空间配准、伪彩融合、
YOLOv5 RKNN 推理，并通过 WebSocket 输出 JPEG 画面。

## 项目亮点

- 设计可见光与热成像双路采集链路，使用单调时钟和有界队列完成帧同步。
- 将 Heimann 热成像接入 V4L2 Capture 框架，支持 MMAP、QBUF/DQBUF、时间戳和
  sequence；保留字符设备作为控制与回退通道。
- 使用 RGA 完成 NV12/BGR 到 RGB、等比例缩放和 letterbox，降低 CPU 图像预处理开销。
- 将 RKNN 模型、Context 和张量属性从逐帧初始化改为进程级复用，并提供 I/O Memory
  与传统接口的 A/B 测试开关。
- 内置端到端性能统计，记录采集、同步、预处理、推理、融合和 JPEG 编码的
  P50/P95/P99 延迟。
- 提供不依赖摄像头的 RKNN/DMA-BUF 基准程序，便于在板端复现和定位性能问题。

## 个人工作

项目历史提交曾使用 `yuanyuan` 和 `shenjiajie` 等本人的旧提交身份，现统一为
`yuanyuan84450 <maywillie@163.com>`。主要工作包括：

- 为双路采集增加单调时间戳、同步队列及运行状态控制，处理同步后闪烁与退出时资源
  回收问题。
- 重构 RKNN 生命周期与 letterbox 预处理，增加分阶段 P50/P95/P99 性能统计和独立
  benchmark。
- 接入 DMA-BUF、RGA 和 RKNN I/O Memory 实验路径，并根据板端实测提供可回退方案。
- 开发 Heimann V4L2/videobuf2 内核驱动、用户态采集测试和应用兼容层。
- 完成板端编译、V4L2 compliance、连续采集及 STREAMON/OFF 压力验证，并整理技术文档。

`.mailmap` 同时记录旧提交身份与当前 GitHub 身份的对应关系。

## 系统架构

```text
可见光摄像头 (V4L2/NV12) ----+
                              +--> 时间戳同步 --> 配准/融合 --> JPEG --> WebSocket
Heimann 32x32 (V4L2/HTPA) ----+                       |
                                                      +--> RKNN/NPU 目标检测
```

## 硬件与软件

| 类别 | 环境 |
| --- | --- |
| SoC | Rockchip RK3588S |
| 开发板 | LubanCat 4 |
| 系统 | Debian 11, Linux 5.10, aarch64 |
| 传感器 | V4L2 可见光摄像头、Heimann 32x32 热成像阵列 |
| 加速 | Rockchip RGA、RKNN Runtime / NPU |
| 主要语言 | C99、C++17 |
| 依赖 | CMake、OpenCV、libv4l2、libwebsockets、readline、ncurses |

## 目录结构

```text
.
├── drivers/                  # 采集、同步、融合、RGA 和 WebSocket 实现
├── include/                  # 公共头文件与 Heimann UAPI
├── kernel/heimann_v4l2/      # Heimann V4L2 内核驱动、DTS 与测试工具
├── opencv/                   # OpenCV 图像处理与绘制
├── src/                      # 主程序入口
├── tools/rknn_benchmark.cpp  # RKNN/RGA/DMA-BUF 独立基准
├── yolov5/                   # RKNN 推理封装与模型
├── docs/                     # 实现原理、测试记录与面试要点
└── CMakeLists.txt
```

## 构建

先在 RK3588S 板端安装 OpenCV、RKNN Runtime、RGA、libv4l2、libwebsockets、
readline 和 ncurses 的开发包，然后执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

生成目标：

- `build/app`：双光融合主程序。
- `build/rknn_benchmark`：可独立运行的 RKNN/RGA 性能基准。

## 运行

```bash
sudo ./build/app /dev/heimann0 /dev/video11 ./yolov5/yolov5n.rknn
```

参数依次为 Heimann 设备、可见光 V4L2 设备和 RKNN 模型。程序运行后可输入：

```text
yolo-req=0 / yolo-req=1
edge-req=0 / edge-req=1
sync-req=0 / sync-req=1
exit
```

## 独立性能基准

```bash
# 常规 RKNN 推理
./build/rknn_benchmark ./yolov5/yolov5n.rknn 100

# 使用 dma-heap 构造 NV12 DMA-BUF，验证 fd -> RGA -> RKNN 链路
./build/rknn_benchmark ./yolov5/yolov5n.rknn 100 nv12-dmabuf

# 强制使用 RKNN I/O Memory 做 A/B 对比
RKNN_FORCE_IO_MEM=1 ./build/rknn_benchmark \
  ./yolov5/yolov5n.rknn 100 nv12-dmabuf
```

可用诊断开关：`RKNN_DISABLE_RGA=1`、`RKNN_DISABLE_IO_MEM=1`、
`RKNN_FORCE_IO_MEM=1`。

## Heimann V4L2 驱动

`kernel/heimann_v4l2/` 包含驱动源码和部署说明。驱动将 32x32 热成像原始帧注册为
自定义 `HTPA` V4L2 Capture 格式，使用 videobuf2 管理 MMAP Buffer，并由独立
kthread 采集 I2C 数据。

已完成的板端验证包括：

- 连续采集 100 帧，检查 sequence 与单调时间戳。
- V4L2 compliance 基础及 streaming 共 54 项测试。
- STREAMON/STREAMOFF 100 次压力测试。
- 应用自动识别 V4L2 节点，并可回退到旧字符设备接口。

驱动的编译、设备树配置和验证命令见
[`docs/phase3_heimann_v4l2_driver_and_interview.md`](docs/phase3_heimann_v4l2_driver_and_interview.md)。

## 当前限制

- RKNN 模型为 FP16，完整零拷贝路径仍受输入类型转换和 DVFS 波动影响，因此默认使用
  实测更稳定的 RGA + legacy I/O 路径。
- 可见光 camera sensor 的 media pipeline 需要按实际板卡重新配置；设备节点不同
  时需修改运行参数。
- 生产融合线程仍包含 OpenCV 畸变校正与同步队列，DMA-BUF 生命周期需结合具体
  camera 驱动继续联调。

## 文档

- [阶段一：RKNN 生命周期、letterbox 与性能统计](docs/phase1_optimization_and_interview.md)
- [阶段二：DMA-BUF、RGA 与 RKNN I/O Memory](docs/phase2_dma_rga_rknn_and_interview.md)
- [阶段三：Heimann V4L2 内核驱动](docs/phase3_heimann_v4l2_driver_and_interview.md)

## 第三方组件

仓库中的 RKNN 接口、模型及第三方依赖分别遵循其原始许可证和使用条款。用于其他
硬件或商业场景前，请自行确认模型、SDK 与传感器资料的授权范围。

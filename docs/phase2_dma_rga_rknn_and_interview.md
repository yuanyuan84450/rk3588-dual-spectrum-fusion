# 第二阶段：DMA-BUF、RGA 与 RKNN 共享链路

## 1. 本阶段真正完成了什么

代码新增了两条预处理路径：

```text
兼容路径：BGR 虚拟地址 ─ RGA ─→ RKNN 分配的 DMA 内存 ─ rknn_inputs_set ─→ NPU

共享路径：NV12 DMA fd ───── RGA ─→ RKNN 分配并绑定的 DMA 内存 ─ rknn_run ─→ NPU
                         resize + CSC + letterbox
```

具体改动如下：

1. V4L2 仍使用 `V4L2_MEMORY_MMAP` 采集，但为每个 buffer 调用
   `VIDIOC_EXPBUF` 导出 DMA-BUF fd，并在退出时关闭 fd。
2. RGA 一次任务完成 NV12/BGR 到 RGB、等比例缩放和 letterbox，目标是
   `rknn_create_mem()` 得到的 DMA buffer，不再先生成一个普通 CPU Mat。
3. 新增 `yolov5_detect_nv12_dmabuf()`。调用者传入源 fd、有效尺寸和 stride；函数
   同步返回前会完成 RGA 和 RKNN 消费。
4. 增加 RKNN I/O Memory 输入和单输出绑定，并保留 legacy I/O 回退路径。
5. 基准程序可从 `/dev/dma_heap/system` 分配 NV12 DMA-BUF，填充测试图后直接把
   fd 交给 RGA，用于没有摄像头时的链路验证。

必须讲清楚边界：现在生产融合线程仍消费复制后的 OpenCV 图像，因为它还要做
畸变校正、双光时间同步和空间配准。摄像头 sensor 当前未连通，无法安全验证“延迟
QBUF 直到推理结束”的改造。因此目前可以说“完成并验证 DMA-BUF/RGA/RKNN 共享
通路和 V4L2 导出能力”，不能说“完整业务链路已经全程零拷贝”。

## 2. 为什么选择 EXPBUF，而不是把采集改成 V4L2_MEMORY_DMABUF

两者方向相反：

- `VIDIOC_EXPBUF`：buffer 由 V4L2 驱动分配，应用把它导出成 DMA-BUF fd，适合
  已经稳定工作的 MMAP 采集程序渐进改造。
- `V4L2_MEMORY_DMABUF`：应用先从 dma-heap/DRM 等处分配 buffer，再把 fd 导入
  V4L2 驱动。控制力更强，但要求摄像头驱动支持该内存模式。

本项目先采用 EXPBUF，改动较小，也方便在导出失败时继续使用原 MMAP 路径。

DMA-BUF fd 不是物理地址，也不保证物理连续。它是内核中的共享 buffer 句柄；
摄像头、RGA、NPU 等设备通过各自的 DMA/IOMMU 映射访问同一底层存储。

## 3. buffer 所有权是最容易出错的地方

V4L2 buffer 的状态可以简化为：

```text
QBUF：所有权交给摄像头驱动
  ↓ 摄像头写入
DQBUF：所有权回到应用
  ↓ RGA/RKNN 同步消费
QBUF：应用确认不再使用，才归还驱动
```

如果 RGA 尚未读完就 `QBUF`，摄像头可能覆盖同一块内存，结果会是偶发花屏、错帧，
而不是稳定崩溃，所以特别难查。当前 DMA-BUF 检测 API 是同步接口：调用返回后才能
重新 QBUF。未来若改为异步 RGA/NPU，需要传递 acquire/release fence 或建立引用计数，
不能只靠线程间“差不多执行完了”的假设。

fd 生命周期也不同于 buffer 所有权：导出的 fd 应在整个 streaming 周期保持打开，
`STREAMOFF`、解除 mmap 后再关闭；每帧反复 EXPBUF/close 没有必要。

## 4. stride、格式和缓存一致性

RGA 包装 buffer 时必须同时传有效宽高和实际 stride。`width=640` 不代表
`width_stride=640`；驱动可能为了总线和硬件对齐返回更大的 `bytesperline`。
忽略 stride 会导致错行、偏色或越界。

NV12 的 Y 平面大小通常是 `width_stride × height_stride`，UV 平面紧随其后，
总量约为前者的 1.5 倍。YUV420 的尺寸、裁剪坐标通常要求偶数，本实现对 RGA 的
缩放尺寸和 padding 做了偶数对齐。

合成测试由 CPU 写 dma-heap buffer，因此在写前后使用
`DMA_BUF_IOCTL_SYNC START/END | WRITE`。真实摄像头写、RGA 读时，设备驱动和
同步调用负责可见性；若改成异步链路，还必须处理 fence。DMA-BUF 解决的是共享，
不自动等价于“无缓存一致性问题”。

## 5. 为什么零拷贝不一定更快

当前模型的真实属性是：

```text
input : 1×640×640×3, FP16, NHWC, 2,457,600 bytes
output: 1×25200×85, FP16, 4,284,000 bytes
```

RGA 输出的是 UINT8 RGB。即使把这块内存用 `rknn_set_io_mem()` 绑定为输入，
Runtime 仍需将 UINT8 转成模型要求的 FP16。legacy 路径中，这部分通常记在
`rknn_inputs_set()`；I/O Memory 路径中，它可能移入 `rknn_run()`。因此只看到
`input_set` 从几十毫秒降到 0.x ms，不能直接声称推理整体快了几十毫秒，必须看
`yolo_total`。

一次同输入、同为 50 次的板端回测如下：

| NV12 DMA-BUF 路径 | 预处理 avg | input avg | run avg | output avg | total avg | total P95 |
|---|---:|---:|---:|---:|---:|---:|
| 强制 I/O Memory | 1.548 ms | 0.426 ms | 162.480 ms | 1.355 ms | 168.777 ms | 210.787 ms |
| legacy RKNN I/O | 1.661 ms | 50.881 ms | 109.999 ms | 28.131 ms | 193.873 ms | 241.318 ms |

该组回测中总平均延迟下降约 12.9%，但此前不同轮次出现过相反排序，说明 RK3588S
的 CPU/NPU DVFS、温度和 Runtime 转换会显著影响结果。本阶段不把单轮数字包装成
稳定收益：默认保留实测较稳的 legacy I/O；使用 `RKNN_FORCE_IO_MEM=1` 开启完整
共享模式做诊断。得到 INT8/UINT8 模型后，应锁定性能策略、交替运行多轮并重新决策。

## 6. 如何运行和验证

```bash
cmake -S . -B build_release -DCMAKE_BUILD_TYPE=Release
cmake --build build_release -j4

# 普通 BGR 输入
./build_release/rknn_benchmark ./yolov5/yolov5n.rknn 100

# 合成 NV12 dma-heap 输入，验证 DMA fd → RGA → RKNN
./build_release/rknn_benchmark ./yolov5/yolov5n.rknn 100 nv12-dmabuf

# 完整 I/O Memory 共享路径
RKNN_FORCE_IO_MEM=1 ./build_release/rknn_benchmark \
  ./yolov5/yolov5n.rknn 100 nv12-dmabuf

# 两个独立回退开关
RKNN_DISABLE_RGA=1 ./build_release/rknn_benchmark ./yolov5/yolov5n.rknn 100
RKNN_DISABLE_IO_MEM=1 ./build_release/rknn_benchmark ./yolov5/yolov5n.rknn 100
```

验证不能只有“程序没崩”：

1. 对同一张真实图分别运行 OpenCV 和 RGA 路径，比较 letterbox 像素误差。
2. 比较解码前输出 tensor 或最终框的类别、置信度、坐标，设置可解释的容差。
3. 用 100 次以上预热后测试，报告 avg、P50、P95/P99，而非只挑最好一次。
4. 同时看端到端 total 和各阶段耗时，确认耗时没有只是换了计时归属。
5. 恢复 sensor 后，用真实 `bytesperline/sizeimage`，长时间检查错帧、花屏及 fd 泄漏。

## 7. 你应该掌握的代码主线

面试前至少能不看代码讲出下面五步：

1. `REQBUFS/QUERYBUF/mmap` 建立 V4L2 buffer 池，`EXPBUF` 一次性导出 fd。
2. `DQBUF` 后应用拥有该帧，读取 `buf.index` 找到对应 fd。
3. RGA 用源 fd 包装 NV12 buffer，以 RKNN input mem fd 为 RGB 目标，执行同步
   CSC、resize 和 letterbox。
4. legacy 模式调用 `rknn_inputs_set()`；I/O Memory 模式初始化时只绑定一次，
   每帧同步后直接 `rknn_run()`。
5. 推理结束才允许 QBUF；退出时按顺序销毁 RKNN memory、关闭 DMA fd、munmap。

## 8. 高频面试问题与参考回答

### Q1：你这个项目是真正的零拷贝吗？

严谨回答：独立 NV12 通路实现了 Camera-compatible DMA-BUF → RGA → RKNN input
memory 的共享设计，合成 dma-heap fd 已在板端验证；但生产融合线程仍需 OpenCV
畸变校正和双光配准，真实 camera sensor 当时不可用，所以还没有宣称完整业务全程
零拷贝。FP16 模型还存在 Runtime 内部类型转换，下一步是 INT8 模型和真实 buffer
生命周期联调。

### Q2：DMA-BUF 和 mmap 有什么区别？

`mmap` 是把内存映射给 CPU 虚拟地址；DMA-BUF fd 是让不同内核设备/驱动共享同一
buffer 的句柄。一块 buffer 可以同时被 mmap 给 CPU，也可以通过 fd 给 RGA/NPU，
两者不是互斥关系。

### Q3：EXPBUF 和 V4L2_MEMORY_DMABUF 有什么区别？

EXPBUF 是把 V4L2 自己分配的 MMAP buffer 导出；DMABUF memory mode 是应用分配
buffer 后导入 V4L2。本项目为了兼容原采集驱动选择前者。

### Q4：为什么不能 DQBUF 后立刻 QBUF？

DQBUF 后 RGA 正在读取这块存储。提前 QBUF 会把所有权还给摄像头，摄像头可能在
RGA 读取期间覆盖数据。同步调用返回或 release fence 完成后才能归还。

### Q5：为什么 input_set 变成 0.4 ms，run 却变慢？

模型输入是 FP16，RGA 输出 UINT8。绑定 I/O Memory 后并没有消除类型转换，只是
Runtime 将转换从 `rknn_inputs_set` 延迟到 `rknn_run`。所以优化判断必须看 total。

### Q6：`size` 和 `size_with_stride` 为什么不能混用？

`size` 是有效 tensor 数据量；`size_with_stride` 包含硬件布局对齐。分配共享输入时
至少要满足后者，否则硬件按对齐步长访问可能越界。代码分配两者的较大值。

### Q7：RGA 相比 OpenCV 的价值只在速度吗？

不是。RGA 能直接消费 DMA fd 并写另一个设备可访问的 DMA buffer，把颜色转换和
缩放从 CPU 数据链路移出。对一张已经在 CPU cache 中的小图，调用 RGA 的固定开销
甚至可能使它不占优；真实价值要放在 camera fd 的端到端链路中评价。

### Q8：怎样保证 CPU、RGA、NPU 看到一致数据？

CPU mmap 访问 dma-buf 时使用 `DMA_BUF_IOCTL_SYNC` 划定访问区间；设备间依赖同步
ioctl 的完成语义，异步时传递 fence。还要严格遵守 DQBUF/QBUF 和 fd 生命周期。

### Q9：失败时怎么降级？

EXPBUF 失败不影响原 MMAP 采集；RGA 初始化/执行失败会切换 OpenCV 预处理；
I/O Memory 可通过环境变量关闭。生产系统的优化必须有可观测日志和可回退路径。

### Q10：下一阶段你会怎么做？

先找回 `.pt/.onnx` 和代表性校准集，生成 INT8 RKNN，验证精度；再把 V4L2 buffer
索引、fd 和时间戳作为帧描述符传入处理线程，用引用计数或 fence 延迟 QBUF；最后
锁定 CPU/NPU 性能策略，做真实双光业务的功耗、平均延迟和 P95/P99 对比。

## 9. 简历写法

当前事实范围内建议写：

> 基于 V4L2 `VIDIOC_EXPBUF`、DMA-BUF、RGA 与 RKNN I/O Memory 构建可回退的
> 图像共享预处理通路，融合完成 NV12→RGB、resize 与 letterbox；设计 dma-heap
> 合成基准并按阶段统计 avg/P95，定位 FP16 类型转换导致的耗时迁移，避免以单阶段
> 指标误判零拷贝收益。

暂时不要写“完整业务零拷贝”“帧率提升 X%”。等真实摄像头恢复、全链路接通并完成
多轮稳定测试后，再把可复现的端到端数据加到简历。

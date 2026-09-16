# 第三阶段：Heimann V4L2/videobuf2 内核驱动

## 0. 进度检查点（下次继续先看这里）

截至 2026-09-06 已完成：

- 从原 `/home/cat/heimann_driver/heimann_i2c_drv.c` 派生独立 V4L2 版本，未覆盖旧驱动。
- 新模块位于 `/home/cat/heimann_v4l2_driver/heimann_v4l2_drv.ko`。
- 实现 `video_device + vb2_queue + vb2_vmalloc + capture kthread`。
- 实现固定 `HTPA 32×32 / 2580 bytes` 格式。
- 实现 MMAP、READ、QBUF/DQBUF、STREAMON/OFF、sequence 和单调时间戳。
- 保留 `/dev/heimann0` EEPROM ioctl 和 legacy read，并与 V4L2 streaming 互斥。
- 新增独立 `test_heimann_v4l2` 测试程序。
- 主应用新增 V4L2 采集适配层，可自动回退旧 read()。
- 内核模块通过 Linux 5.10.160 头文件 `W=1` 编译。
- 测试程序通过 `-Wall -Wextra -Werror` 编译。
- DTS overlay 通过 `dtc -@` 编译。
- 完整双光应用 Release 编译通过。
- 修复 `QUERYCAP.bus_info` 小写 `i2c:` 不符合 v4l-utils 合规规则的问题，改为
  `I2C:5-001a`。
- 新模块已实际加载并 probe，生成 `/dev/video20`（HTPA）和 `/dev/heimann0`。
- `v4l2-compliance -d /dev/video20 -s 60`：54/54 通过，0 warning。
- 独立测试两轮各采集 100 帧：`sequence_gaps=0`，实测约 4.28 FPS。
- 新驱动的 legacy read 以 root 验证可正常返回 2580 字节。
- V4L2 streaming 期间调用 legacy read，正确返回 `-EBUSY`；V4L2 30 帧继续正常。
- 独立进程执行 100 轮 STREAMON→采集→STREAMOFF：全部通过，无卡死。

尚未完成：

1. 尚未进行 10000 帧和 I²C/EOC 错误注入测试；100 次反复启停已经通过。
2. `/dev/heimann0` 当前为 `root:root 0600`；普通 `cat` 用户需要 udev 规则或暂时
   以 root 运行完整应用。规则文件已准备，但尚未获准安装到 `/etc/udev/rules.d`。
3. 可见光 `/dev/video11` 的 sensor 链路未建立，STREAMON 返回
   `Operation not permitted`，因此完整双光应用仍无法端到端验证。
4. 暂未增加 I²C 错误、超时、丢帧等 debugfs/sysfs 统计。

下次继续顺序：安装并验证 udev 权限规则 → 运行 10000 帧与错误注入测试 → 修复
可见光 camera sensor/media graph → 运行完整融合应用 → 记录双光同步数据。

## 1. 为什么不是简单把 read 改名为 DQBUF

旧驱动的数据路径是：

```text
应用 read()
  → 驱动进入进程上下文
  → 同步等待多个 EOC
  → I²C 读取 2580 字节
  → copy_to_user
```

V4L2 streaming 改为：

```text
应用 REQBUFS/mmap/QBUF/STREAMON
                    ↓
              capture kthread
                    ↓
      从驱动 queued list 取一个 vb2 buffer
                    ↓
        EOC + I²C 采集 + 填充 buffer
                    ↓
      timestamp + sequence + buffer_done
                    ↓
               应用 DQBUF/QBUF
```

这次升级的价值不是让 2.58 KB memcpy 神奇消失，而是获得标准 Buffer 生命周期、
阻塞等待、时间戳、序号、流式启停和通用 V4L2 工具支持。

## 2. 为什么使用自定义 HTPA，而不是 Y16

传感器一帧大小为：

```text
(2 × 4 blocks + 2 electrical-offset blocks) × 258 = 2580 bytes
```

纯 32×32 Y16 只有 2048 字节。剩余数据包含温度补偿依赖的 PTAT/VDD、electrical
offset 及 block 信息。如果直接声明 Y16，会让画面看起来更“标准”，但破坏现有温度
算法。因此第一版定义：

```c
#define V4L2_PIX_FMT_HTPA v4l2_fourcc('H', 'T', 'P', 'A')
```

`sizeimage=2580`，用户态继续解析原始 RAMoutput，浮点温度补偿继续留在应用层。

未来可设计第二版带显式帧头的 UAPI，存储 ABI version、PTAT/VDD 标志和硬件状态；
不能复用 V4L2 reserved 字段偷偷传私有数据。

## 3. 核心对象及职责

| 对象 | 职责 |
|---|---|
| `i2c_driver` | probe/remove、传感器及 EEPROM 管理 |
| `v4l2_device` | 将设备挂入 V4L2 框架 |
| `video_device` | 创建可被应用打开的 `/dev/videoX` |
| `vb2_queue` | 管理 MMAP Buffer 状态和 streaming ioctl |
| `heimann_buffer` | 在 `vb2_v4l2_buffer` 外增加驱动链表节点 |
| capture kthread | 在可睡眠上下文等待 EOC、执行 I²C 并完成 Buffer |
| miscdevice | EEPROM ioctl 和旧应用回退 |

这里只有传感器和 I²C，没有 CSI bridge/ISP 为它创建 capture node，所以只注册
`v4l2_subdev` 不够；本项目直接注册完整 `video_device`。

## 4. Buffer 生命周期

```text
DEQUEUED ─QBUF→ QUEUED ─驱动取走→ ACTIVE
    ↑                               │
    └──────── DQBUF ← DONE/ERROR ←──┘
```

- `buf_queue()` 只在 spinlock 下把 Buffer 放入私有队列，不能在这里睡眠或做 I²C。
- kthread 从队列取 Buffer 后才执行传感器采集。
- 成功后设置 payload、timestamp、sequence，并调用 `vb2_buffer_done(DONE)`。
- 失败用 `VB2_BUF_STATE_ERROR` 返回，不能吞掉 Buffer。
- `stop_streaming()` 必须停止线程并归还私有队列中的所有 Buffer，否则 DQBUF 或关闭
  可能永久阻塞。

## 5. 锁和线程

| 同步对象 | 保护内容 | 是否可睡眠 |
|---|---|---|
| I²C mutex | 传感器和 EEPROM 传输 | 可以 |
| frame mutex | 一次完整采集及 legacy/V4L2 互斥 | 可以 |
| vb2 mutex | ioctl、队列配置和 stream 状态 | 可以 |
| queued spinlock | 待采集 Buffer 链表和计数 | 不可以 |
| wait queue | 无 Buffer 时让 kthread 睡眠 | — |

绝不能持有 spinlock 后调用 `i2c_transfer()` 或 `msleep()`。停止 streaming 时先设置
`capture_abort`，EOC 轮询检测该标志并返回 `-ECANCELED`，避免 STREAMOFF 最坏等待
多个完整超时周期。

## 6. 驱动时间戳为什么更适合双光同步

旧应用在 `read()` 返回后调用 `clock_gettime()`，这个时间包含 EOC 等待、所有 I²C
读取和调度延迟。新驱动在开始采集一帧时记录 `ktime_get_ns()`，通过 V4L2 Buffer
返回单调时间戳。

这仍不是严格的硬件曝光时间：最理想的时间戳应在 Data Ready IRQ 或传感器开始积分
时产生。当前 DTS 未定义 IRQ，因此第一版的时间戳是“驱动开始采集时间”。面试时必须
明确这个误差来源。

## 7. 用户态兼容策略

主应用会先对第一个参数尝试 `VIDIOC_QUERYCAP`：

- 若为 HTPA V4L2 节点，使用四个 MMAP Buffer 和 QBUF/DQBUF。
- EEPROM 从 `HEIMANN_CONTROL_DEVICE` 指定的节点读取，默认 `/dev/heimann0`。
- 若不是 V4L2 节点，自动回退原来的同步 `read()`。

示例：

```bash
# 新 V4L2 路径
HEIMANN_CONTROL_DEVICE=/dev/heimann0 \
  ./build_release/app /dev/video-heimann /dev/video11 ./yolov5/yolov5n.rknn

# 旧路径
./build_release/app /dev/heimann0 /dev/video11 ./yolov5/yolov5n.rknn
```

实际 video 编号可能变化，应使用 `/dev/v4l/by-path`、udev symlink 或设备名查找，
不要在最终部署中硬编码 `/dev/video0`。

## 8. 构建与待执行部署步骤

驱动源码：

```bash
cd /home/cat/heimann_v4l2_driver
make clean
make
```

目前不要盲目执行下面的加载步骤。先确认 overlay 路径和板卡启动配置，并保留串口或
SSH 恢复手段：

```bash
# 确认新旧模块不能同时绑定同一个 compatible
lsmod | grep heimann

# overlay 生效并确认 I²C 节点后再加载
sudo insmod /home/cat/heimann_v4l2_driver/heimann_v4l2_drv.ko

v4l2-ctl --list-devices
v4l2-ctl -d /dev/videoX --all
/home/cat/heimann_v4l2_driver/test_heimann_v4l2 /dev/videoX 100
```

完整应用构建：

```bash
cmake -S /home/cat/project_clean -B /tmp/project_clean_build_v4l2 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/project_clean_build_v4l2 -j4
```

## 9. 验收计划

1. 旧驱动连续采集 1000 帧，确认硬件基线。
2. 新模块生成 `/dev/videoX`，检查 HTPA、32×32、sizeimage=2580。
3. 测试工具采集 100 帧，验证 bytesused、单调时间戳和 sequence。
4. 对比新旧路径同类帧的 block 结构和温度范围。
5. 连续采集 10000 帧，统计错误、超时和序号间断。
6. STREAMON/OFF 循环 100 次。
7. 采集中发送 SIGKILL，随后重新打开设备。
8. 在 EOC/I²C 错误时确认 Buffer 以 ERROR 返回，应用不会永久阻塞。
9. 运行 `v4l2-compliance`，记录自定义 raw 格式相关的预期例外。
10. 完整双光应用对比驱动时间戳前后的同步误差 P50/P95/P99。

## 10. 高频面试问题

### Q1：为什么用 video_device 而不是只注册 v4l2_subdev？

因为该 I²C 传感器直接产生完整帧，没有 CSI bridge/ISP 提供 capture node。subdev
主要表示 media pipeline 内部组件，应用需要直接 QBUF/DQBUF，所以驱动创建完整
video_device。

### Q2：为什么不直接用 Y16？

Y16 只能容纳 2048 字节像素，而原始帧为 2580 字节，温度补偿还需要 PTAT/VDD 和
electrical offset。第一版用私有 HTPA 保持数据无损，温度算法留在用户态。

### Q3：vb2解决了什么？

它统一处理 Buffer 分配、mmap、QBUF/DQBUF、streaming 状态和文件操作，驱动只需
实现硬件相关的 queue/setup/start/stop/buffer-done 回调。

### Q4：为什么需要 kthread？

一次采集包含多次 EOC 等待和 I²C 传输，都可能睡眠，不能放在 spinlock 或硬中断
上下文。独立线程还能把采集生命周期与应用系统调用解耦。

### Q5：buf_queue 中为什么不能直接采集？

buf_queue 的职责是快速接收 Buffer；直接做长耗时 I²C 会阻塞 QBUF 路径并让锁关系
复杂。它只入链表并唤醒采集线程。

### Q6：stop_streaming 最容易出现什么错误？

忘记停止后台任务，或没有把驱动持有的 Buffer 全部用 DONE/ERROR 返回，最终导致
关闭、REQBUFS(0) 或下一次 streaming 卡死。

### Q7：为什么同时有 mutex 和 spinlock？

I²C 和帧采集会睡眠，用 mutex；短小的 Buffer 链表操作可能与不同执行上下文并发，
用 spinlock。持有 spinlock 时不能睡眠。

### Q8：驱动时间戳是否就是曝光时间？

目前不是。它是采集线程开始读取一帧的时间，比 read 返回后的应用时间更接近硬件，
但仍包含传感器积分/EOC 误差。接入 Data Ready IRQ 后可进一步逼近硬件事件。

### Q9：为什么选择 vb2-vmalloc，不做 DMA-BUF？

每帧只有 2580 字节，I²C 和 EOC 才是主要瓶颈；DMA 管理成本可能高于一次 memcpy。
vb2-vmalloc 实现简单、支持 MMAP/read，符合当前收益与复杂度平衡。

### Q10：新旧节点同时读取会怎样？

两者会修改同一个 PTAT/VDD 和 block 状态机，造成帧类型错乱。因此驱动在 V4L2
streaming 时让 legacy read 返回 `-EBUSY`，只保留 EEPROM ioctl。

### Q11：如何处理采集速度比应用消费速度快？

vb2 Buffer 数量形成有限背压。应用未及时 QBUF 时驱动没有可用 Buffer，采集线程在
wait queue 睡眠，不建立无界队列。后续可统计这种 starvation。

### Q12：这个改造提高了帧率吗？

不能先验声称。传感器 EOC 和 I²C 决定主要吞吐，本次核心收益是标准流式接口、并发
解耦、时间戳和可观测 Buffer 生命周期。真实帧率和同步误差需硬件测试后报告。

## 11. 简历写法

目前已经可以写，但必须注明是“设计并实现、完成编译验证”，不要编造实测帧率：

> 基于 Linux V4L2/videobuf2 重构 Heimann 32×32 热成像 I²C 驱动，设计 HTPA 私有
> 原始帧格式，使用 vb2 MMAP Buffer、采集 kthread 与 wait queue 实现标准
> STREAMON/QBUF/DQBUF 流程；为热帧注入单调时间戳和 sequence，并实现 legacy
> 字符设备互斥回退。模块通过 RK3588S Linux 5.10 `W=1` 编译验证。

完成真实长稳测试后可升级成：

> 完成 10000 帧连续采集及 100 次 STREAMON/OFF 压力测试，通过驱动侧时间戳重构
> 双光同步基准，将同步误差 P95 从 X ms 降至 Y ms，并实现 I²C 超时与异常 Buffer
> 回收，避免采集线程永久阻塞。

第二条中的 X/Y 必须来自真实测量。

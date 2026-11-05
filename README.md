# RV1126B 双链路 FEC 图传系统（网口 UDP + 串口 UART）

基于**正点原子 ATK-DL RV1126B** 开发板的实时视频传输系统。发送端通过 V4L2 采集摄像头画面，使用 Rockchip MPP 硬件编码 H.265，经 **zfec 前向纠错（RS 码）** 切包后，通过 **网口 UDP** 或 **串口 UART** 两条链路自适应传输；接收端重组、纠错、硬解码，并经 RGA 旋转后通过 DRM 直接送显。整套链路不依赖任何用户态第三方框架，全部直接操作底层设备。

## 目录

- [系统特性](#系统特性)
- [整体架构](#整体架构)
- [硬件平台与连线](#硬件平台与连线)
- [目录结构](#目录结构)
- [协议设计](#协议设计)
- [传输模式与自动切换](#传输模式与自动切换)
- [时钟同步机制](#时钟同步机制)
- [编译](#编译)
- [部署与运行](#部署与运行)
- [运行时统计与实时绘图](#运行时统计与实时绘图)
- [关键参数速查表](#关键参数速查表)
- [常见问题](#常见问题)

---

## 系统特性

| 特性 | 说明 |
|---|---|
| 视频采集 | V4L2 MPLANE，`/dev/video23`，1280x720 NV12，DMA-BUF 零拷贝导出 |
| 视频编码 | Rockchip MPP 硬编码 H.265（HEVC），CBR 1Mbps，30fps，GOP 3000，IPC 场景模式 + AQ |
| 前向纠错 | zfec（Vandermonde 矩阵 RS 码），按帧类型动态冗余：IDR 帧 K=4/M=7，P 帧 K=11/M=12 |
| 传输链路 | 网口 UDP（包大小贴合 MTU）与串口 UART（termios2 任意波特率，当前 3Mbps）双链路 |
| 链路自适应 | 启动时按网线 carrier 自动选择；运行中网口 15 秒无应答自动回退串口 |
| 反向控制 | 接收端通过反向包请求 IDR（0xFF）/ 上报接通（0xEE），双向闭环 |
| 视频显示 | MPP 硬解码 + RGA 硬件旋转 90° + DRM 三缓冲直显，无 GUI 依赖 |
| 时钟同步 | RTS/CTS GPIO 秒脉冲对时，接收端可测量端到端延迟（ms 级） |
| 实时性 | 关键线程 SCHED_FIFO 实时调度优先级 |

## 整体架构

```
                        发送端 (Sender)                         接收端 (Receiver)
                 ┌──────────────────────────┐           ┌───────────────────────────┐
                 │ V4L2 采集 (NV12, DMA)    │           │ 双 fd select 收包          │
                 │         │                │           │ (UDP + 串口同时监听)       │
                 │ MPP H.265 硬编码 (1Mbps)  │           │ 字节流状态机解析 FecHeader  │
                 │         │                │           │ FrameReassembler 组帧       │
                 │ 发送队列 (线程间解耦)      │           │ zfec 解码抢救受损包         │
                 │ zfec 编码切包 (K/M 动态)   │           │ MPP H.265 硬解码           │
                 │      ┌───┴───┐           │  正向码流  │ RGA 旋转90° + DRM 三缓冲    │
                 │   UDP     串口           │ ════════▶ │        │                   │
                 └──────┬───────┬───────────┘           └────────┼───────────────────┘
                        │       │                                 │
                        │       └──────── 反向 0xFF/0xEE ─────────┤
                        └──── (填充包：网口0xEE为30字节，其余256字节) ┘
```

两条物理链路同时初始化、谁可用用谁；接收端**不区分**码流来自哪条链路，反向信号则两条链路都发一份，保证发送端无论走哪条介质都能收到。

## 硬件平台与连线

- **开发板**：正点原子 ATK-DL RV1126B（发送端、接收端各一块，均运行本项目的收发程序）
- **摄像头**：接到发送端板子（`/dev/video23`，需支持 NV12 输出）
- **显示器**：接到接收端板子（HDMI/LVDS，程序通过 DRM 直接写帧缓冲）
- **串口线**：两板 `/dev/ttyS2` 交叉连接（TX↔RX、GND 共地；波特率由程序用 `termios2` 设为 3Mbps）
- **网线**：两板网口直连（静态 IP `192.168.144.109` ↔ `192.168.144.110`，见下表）
- **同步线**：发送端 GPIO3_A0（GPIO96，程序驱动为秒脉冲输出）→ 接收端 GPIO（GPIO97，程序配置为中断输入）。两者分别占用开发板的 RTS/CTS 引脚。

| 设备 | 发送端 | 接收端 |
|---|---|---|
| 串口 | `/dev/ttyS2` @ 3Mbps | `/dev/ttyS2` @ 3Mbps |
| UDP | 本地端口 8000 → 目标 `192.168.144.110:8001` | 本地端口 8001 → 目标 `192.168.144.109:8000` |
| 同步 GPIO | 96（输出，秒翻转） | 97（输入，沿中断） |
| 摄像头 | `/dev/video23` 1280x720 | — |
| 显示 | — | DRM 三缓冲 |

## 目录结构

```
ipc_cameratrans_zfec/
├── Sender/
│   ├── ipc_sender.cpp        # 发送端主程序（采集/编码/FEC/双链路发送/反向处理）
│   ├── Makefile              # 发送端交叉编译脚本
│   └── ipc_sender_uarteth    # 编译产物（发送端可执行文件）
├── Receiver/
│   ├── ipc_receiver.cpp      # 接收端主程序（收包/组帧/FEC解码/解码显示/统计）
│   ├── Makefile              # 接收端交叉编译脚本（额外链接 RGA/libdrm）
│   ├── live_plot.py          # 实时曲线绘图（延迟/码率/帧率，matplotlib）
│   └── ipc_receiver_uarteth  # 编译产物（接收端可执行文件）
├── fec.c / fec.h             # zfec 前向纠错库（RS 码，源自 zfec 开源项目）
└── osd_burner.h              # OSD 时间戳水印烧录接口（叠加到画面）
```

## 协议设计

### 正向码流（发送端 → 接收端）

编码后的每一帧 H.265 被按 `chunk_size` 切成 K 个数据块，zfec 生成 M−K 个校验块，以物理包形式发送。物理包头部（32 字节，`#pragma pack(1)`）：

```cpp
struct FecHeader {
    uint32_t sync;             // 同步字 0x55AA55AA
    uint32_t frame_id;         // 帧序列号
    uint64_t capture_ts;       // 采集时间戳（用于端到端延迟测量）
    uint32_t total_frame_len;  // 帧真实总长（用于剥离 FEC 对齐 Padding）
    uint16_t group_id;         // 本帧内第几个 FEC 组
    uint16_t total_groups;     // 本帧总组数
    uint8_t  block_idx;        // 块索引：0~K-1 数据块，K~M-1 校验块
    uint16_t payload_len;      // 本包实际携带数据长度
    uint16_t fec_block_size;   // 本组 FEC 运算对齐长度（组内最大 payload）
    uint8_t  fec_k;            // 本帧使用的 K
    uint8_t  fec_m;            // 本帧使用的 M
    uint16_t fec_chunk_size;   // 切块大小（880=网口 / 1024=串口）
    uint8_t  header_crc;       // 头部校验和（8 位累加）
};                              // 尾部附加 1 字节整包校验和
```

要点：

- **按帧类型动态冗余**：IDR 帧更大更关键，用 K=4/M=7（75% 冗余）；P 帧用 K=11/M=12（约 9% 冗余）。丢失任意 K 个块即可完整重建整组。
- **切块大小随介质动态调整**：网口 880 字节（贴合以太网 MTU，避免 IP 分片）；串口 1024 字节。接收端从包头动态读取，**发送端切换介质无需通知接收端**。
- **两级校验**：头部 CRC 通过才确定包长，整包校验通过才入组，串口误码不会污染组缓冲。
- 接收端用字节流状态机逐字节找同步字，串口/UDP 统一走同一套解析。

### 反向控制（接收端 → 发送端）

接收端用整包填充同一字节的包表达两种信号，两条链路各发一份：

| 内容 | 网口包长 | 串口包长 | 触发时机 | 发送端行为 |
|---|---|---|---|---|
| `0xFF` | 256 字节 | 256 字节 | 启动时一次；处于"等待 IDR"状态时每秒最多一次 | 强制编码器输出 IDR，并清空发送队列中的旧 P 帧 |
| `0xEE` | **30 字节**（短包） | 256 字节 | 从"等待 IDR"恢复（成功解码显示）的瞬间一次 | 标记接收端已接通，停止每 5 秒的 IDR 兜底 |

网口的 `0xEE` 采用 30 字节短包（UDP 数据报一次收全，30 > n/2 判定阈值足够）；`0xFF` 保持 256 字节、串口一律 256 字节不变——串口 `read()` 可能只读出半包、甚至与残余码流混在同一窗口，大包保证按"某字节占比超过 n/2"判定时的容错余量。

## 传输模式与自动切换

发送端对两条链路**都执行初始化**，然后按以下优先级选择（`main()` 中）：

1. 命令行 `--serial`：强制串口
2. 命令行 `--udp`：强制网口
3. 网线已插（扫描 `/sys/class/net/*/carrier`）：自动选网口（带宽优先）
4. 网线未插且串口可用：自动选串口

**运行时自动回退**：网口模式下若 15 秒收不到任何 `0xEE` 接通应答，自动切换到串口继续图传。每帧在发送入口对介质做一次快照，保证单帧数据只走一条介质，不会半帧切链路。

接收端没有"模式"概念：两条链路同时监听、反向信号两条都发，因此发送端怎么切换，接收端都不需要重启。

## 时钟同步机制

端到端延迟测量不依赖网络对时：

1. 发送端 `sender_sync_thread` 用 `timerfd`（1ms 精度）维护虚拟主时钟，每满 1 秒翻转一次 RTS GPIO（GPIO96）电平；
2. 接收端 GPIO97 配置为双边沿中断（`cts_interrupt_thread`，SCHED_FIFO 优先级 60），首次收到沿即对齐本地从时钟（`slave_timer_thread` 同样用 1ms timerfd 走秒）；
3. 每帧的 `capture_ts` 即发送端主时钟值，接收端显示完成时刻的从时钟与之相减得到**实时延迟**（统计输出中的 `Latency (ms)`）。

## 编译

### 依赖

在 Ubuntu 交叉编译主机上需要：

- 正点原子 RV1126B 交叉工具链（默认路径 `/opt/atk-dlrv1126b-toolchain`）
- Rockchip MPP 开发文件（头文件/库，路径见 Makefile：`/home/alientek/mpp`）
- 接收端还需要 RGA（`/home/alientek/librga-main`、`/home/alientek/rv1126_rga`）和 libdrm

工具链/MPP 路径若不同，改两个 Makefile 顶部的变量即可。

### 命令

```bash
# 发送端
cd Sender && make

# 接收端
cd Receiver && make
```

产物：

- `Sender/ipc_sender_uarteth`（实际参数以源码为准：当前源码为 720p、串口 3Mbps、1Mbps 码率，详见[关键参数速查表](#关键参数速查表)）
- `Receiver/ipc_receiver_uarteth`

清理：`make clean`。注意 `fec.o` 生成在上级目录，清理发送端或接收端任一都会删掉它，下次编译会自动重新生成。

## 部署与运行

两块板子通过 USB 连到主机，用 adb 部署：

```bash
# 查看设备
adb devices

# 推送到板子（示例，序列号按实际设备替换）
adb -s <发送端序列号> push Sender/ipc_sender_uarteth /userdata/
adb -s <接收端序列号> push Receiver/ipc_receiver_uarteth        /userdata/
```

板上运行（建议 root，否则 SCHED_FIFO 实时调度会失败）：

```bash
# 发送端（可加 --serial 或 --udp 强制介质；不加则按网线状态自动选择）
adb -s <发送端序列号> shell /userdata/ipc_sender_uarteth

# 接收端
adb -s <接收端序列号> shell /userdata/ipc_receiver_uarteth
```

启动顺序不限：接收端启动即发送一次 `0xFF` 请求 IDR，随后进入按需重发逻辑；发送端启动后若未收到 `0xEE`，每 5 秒主动输出 IDR 兜底，保证两端任意时刻上电都能在 1~5 秒内出图。

## 运行时统计与实时绘图

接收端每 100ms 输出一行机器可读的 CSV 统计（供 `live_plot.py` 解析）：

```
STAT,<运行秒数>,<平均延迟ms>,<码率Kbps>,<帧率fps>,<累计丢帧>
```

同时每周期打印人类可读的"实时统计"块：延迟/帧率/码率/帧大小分布/收发 I 帧与 P 帧数/丢帧明细（校验失败、等待 IDR 丢弃、解码失败）/**FEC 抢救成功的受损包数**等。

发送端每 10 秒汇总一次发送侧统计（帧数、字节数、IDR/P 数、最大最小帧、平均 QP、数据包/校验包数）。

主机端实时绘图（需 matplotlib）：

```bash
adb -s <接收端序列号> shell /userdata/ipc_receiver_uarteth | python3 live_plot.py
```

绘制三条实时曲线：端到端延迟、码率、帧率，并显示平均值。

## 关键参数速查表

修改下列参数后需重新编译并推送对应端：

| 参数 | 当前值 | 位置 |
|---|---|---|
| 采集分辨率 | 1280x720 | `Sender/ipc_sender.cpp` `CAPTURE_WIDTH/HEIGHT` |
| 摄像头设备 | `/dev/video23` | `Sender/ipc_sender.cpp` `VIDEO_DEV` |
| 编码码率/帧率/GOP | 1Mbps / 30fps / 3000 | `Sender/ipc_sender.cpp` `init_mpp_encoder()` |
| FEC 冗余 | IDR: K=4,M=7；P: K=11,M=12 | `Sender/ipc_sender.cpp` `fec_send_frame()` |
| 切块大小 | 网口 880 / 串口 1024 | `Sender/ipc_sender.cpp` `main()` 各模式分支 |
| 串口设备/波特率 | `/dev/ttyS2` @ 3,000,000 | 两端 `init_serial()`（`termios2` BOTHER 任意波特率） |
| 反向信号包长 | 网口 `0xEE` 30 字节，其余 256 字节 | `Receiver/ipc_receiver.cpp` `send_reverse_byte()` |
| UDP 地址/端口 | 109:8000 ↔ 110:8001 | 两端文件头宏 |
| 网口无应答回退 | 15 秒 | `Sender/ipc_sender.cpp` `main()` 主循环 |
| IDR 兜底周期 | 5 秒 | `Sender/ipc_sender.cpp` `main()` 主循环 |
| 反向请求 0xFF 限速 | 1 次/秒 | `Receiver/ipc_receiver.cpp` `decode_and_display()` |
| 同步 GPIO | 发送 96 / 接收 97 | 两端 `init_rts_gpio()` / `init_cts_gpio()` |
| 旋转角度 | 90° | `Receiver/ipc_receiver.cpp` `imrotate(..., IM_HAL_TRANSFORM_ROT_90)` |

## 常见问题

**Q：`live_plot.py` 报 `ModuleNotFoundError: No module named 'matplotlib'`**

主机缺 matplotlib。注意：若你 source 过交叉编译工具链的环境脚本，`python3` 可能指向工具链自带的、没有 pip 的解释器。此时显式用系统 Python：

```bash
sudo apt install -y python3-matplotlib python3-tk
adb -s <序列号> shell /userdata/ipc_receiver_uarteth | /usr/bin/python3 live_plot.py
```

**Q：接收端画面迟迟不出（一直"等待 IDR"）**

- 确认发送端进程在跑且已成功初始化摄像头与编码器（看发送端日志）；
- 串口模式检查 TX/RX 是否交叉、是否共地；
- 网口模式检查两端静态 IP 与 `REMOTE_IP` 宏是否一致、网线 carrier 是否为 1。

**Q：串口模式卡顿/丢帧多**

串口有效载荷速率有限（3Mbps 下约 300KB/s），720p@1Mbps 码率已接近上限，P 帧冗余只有 1/11，突发误码超过冗余能力即丢帧。可降低码率/分辨率，或提高串口波特率（两端同步修改 `init_serial()`）。

**Q：`SCHED_FIFO` 设置失败的警告**

程序需要 root 权限才能使用实时调度，用 `sudo` 运行即可；非 root 下仅是调度延迟略增，不影响功能。

**Q：发送端为什么还在发 IDR（日志里频繁 `[IDR]`）？**

未收到接收端 `0xEE` 前发送端每 5 秒强制 IDR 兜底。若接收端明明已出图但仍频繁 IDR，检查反向链路（反向信号走的路径）是否通畅。

---

*FEC 算法源自开源 [zfec](https://github.com/tahoe-lafs/zfec) 项目（Zooko Wilcox-O'Hearn，基于 Luigi Rizzo 的 Vandermonde RS 码实现），遵循其原始许可条款。*

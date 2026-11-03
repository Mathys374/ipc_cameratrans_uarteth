#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <errno.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <ctime>
#include <rockchip/rk_mpi.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_packet.h>
#include <rockchip/mpp_meta.h>
#include <sys/timerfd.h>
#include <thread>
#include <poll.h>
#include <sched.h>
#include <pthread.h>
#include <queue>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <sys/time.h>
#include <stdarg.h>
#include <atomic>
#include <dirent.h>

extern "C" {
    #include "fec.h"
}

#include "../osd_burner.h"

static fec_t* g_fecCtx = nullptr;

/* ==========================================================
 * 宏定义与常量
 * ========================================================== */
#define VIDEO_DEV       "/dev/video23"
#define SERIAL_DEV      "/dev/ttyS2"
#define LOCAL_PORT 8000
#define REMOTE_IP "192.168.144.110"
#define REMOTE_PORT 8001
#define CAPTURE_WIDTH   1280
#define CAPTURE_HEIGHT  720
#define VIDEO_BUFFER_COUNT 2

/* FEC 协议相关结构体定义 */
#define FEC_K 11                // 默认 K（实际按帧类型动态调整：IDR=4 / P=11）
#define FEC_M 12                // 默认 M（实际按帧类型动态调整：IDR=7 / P=12）

#pragma pack(push, 1)
struct FecHeader {
    uint32_t sync;             // 0x55AA55AA
    uint32_t frame_id;         // 帧序列号
    uint64_t capture_ts;       // 采集时间戳
    uint32_t total_frame_len;  // 该视频帧的真实总长度（用于剥离 Padding）
    uint16_t group_id;         // 属于该帧的第几个 FEC 组
    uint16_t total_groups;     // 该帧总组数
    uint8_t  block_idx;        // 块索引：0~K-1 是真实数据，K~M-1 是校验包
    uint16_t payload_len;      // 当前这个物理包真正携带的数据长度
    uint16_t fec_block_size;   // 本组 FEC 矩阵运算时的统一对齐长度 (max_payload_len)
    uint8_t  fec_k;            // 当前包使用的 K
    uint8_t  fec_m;            // 当前包使用的 M
    uint16_t fec_chunk_size;   // 动态 FEC 原始切块大小 (880=网口MTU / 1024=串口大块)
    uint8_t  header_crc;       // 头部校验

};
#pragma pack(pop)

/* 发送队列变量：用于在编码线程和物理发送线程间传递数据 */
struct SendPacket {
    std::vector<uint8_t> data;
    uint64_t capture_ts;
};

/* 摄像头缓冲结构体：管理 V4L2 映射的内存及 DMA 句柄 */
struct CameraBuffer {
    void *start[3];
    size_t length[3];
    int dma_fd;
};

/* ==========================================================
 * 全局变量
 * ========================================================== */
/* 系统运行控制 */
static volatile bool g_running = true;

/* 传输介质控制（atomic：主线程可动态切换，发送线程随时读取） */
static std::atomic<bool> g_useUdp{false};       // 发送介质：网口
static std::atomic<bool> g_useSerial{false};    // 发送介质：串口
static std::atomic<uint16_t> g_fecChunkSize{0}; // 动态 FEC 切块大小 (880=网口 / 1024=串口)

/* 线程间通信与队列 */
static std::queue<SendPacket> g_sendQueue;
static std::mutex g_sendMutex;
static std::condition_variable g_sendCond;

/* 硬件句柄与上下文 */
static int g_rtsGpioFd = -1;
static int g_syncTfd = -1;
static int g_camFd = -1;
static int g_serialFd = -1;
static int g_udpFd = -1;
static struct sockaddr_in g_remoteAddr;
static CameraBuffer g_buffers[VIDEO_BUFFER_COUNT];
static MppCtx g_mppCtx = nullptr;
static MppApi* g_mppApi = nullptr;
static MppBufferGroup g_frmGroup = nullptr;

/* 时钟同步与编码状态 */
static volatile uint64_t g_masterTimestampMs = 0;  // 当前虚拟时间戳 (ms)
static RK_S64 g_pts = 0;                           // 视频帧显示时间戳
static uint32_t g_frame_seq = 0;                   // 发送帧序列号

/* IDR 强制请求标志：接收端通过 0xFF 反向请求时置位，
 * 下一帧必须编码输出（即使发送队列堆积也不跳过），并清空队列中的旧 P 帧 */
static std::atomic<bool> g_idrRequested{false};

/* 接收端接通状态：false=未接通，发送端每5秒主动输出IDR兜底；
 * true=已接通，恢复gop=3000自然IDR频率。
 * 收到 0xEE 信号置 true，收到 0xFF 呼救置 false */
static std::atomic<bool> g_receiverReady{false};

/* ==========================================================
 * 发送端统计变量
 * 每 10 秒汇总一次，用于观察实际帧率、码率、帧大小分布
 * ========================================================== */
static int g_statSentBytes = 0;
static int g_statSentFrames = 0;
static int g_statSentIDR = 0;
static int g_statSentP = 0;
static int g_statMinSize = 0;
static int g_statMaxSize = 0;
static time_t g_statLastTime = 0;
static time_t g_startTime = 0;         // 程序启动时间戳，用于计算相对时间
static int g_statCurFrameSize = 0;     // 当前帧大小（字节）
static int g_statCurFrameType = 0;     // 当前帧类型：0=未知 1=P 2=IDR
static int g_statSumQP = 0;            // 周期内 QP 累加总和
static int g_statCountQP = 0;          // 周期内成功获取 QP 的帧数
static int g_statCurQP = 0;            // 当前最后一帧的 QP 值
static int g_statSentDataSlices = 0;   // 发送的真实视频数据包数量
static int g_statSentParitySlices = 0; // 发送的异或冗余包数量

static int g_statInjectedDropData = 0;   // 故意丢弃的真实数据包数量
static int g_statInjectedDropParity = 0; // 故意丢弃的冗余校验包数量


/* ==========================================================
 * 基础工具函数
 * ========================================================== */
static void sigint_handler(int) {
    printf("\nSender stopping...\n");
    g_running = false;
}

// 提升线程为实时优先级，降低系统调度延迟
static void set_realtime_priority(int priority = 50) {
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        perror("警告: 设置 SCHED_FIFO 失败 (请确保使用 sudo 运行程序)");
    }
}

// 计算简单的 8 位校验和
static uint8_t calc_checksum(const uint8_t* data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

/* 检测网线物理连接状态：扫描 /sys/class/net 下所有网卡（排除 lo）的 carrier
 * carrier=1 表示网线已插入且链路协商成功，与 IP 是否配置无关 */
static bool net_link_up() {
    DIR* dir = opendir("/sys/class/net");
    if (!dir) return false;
    bool up = false;
    struct dirent* e;
    while ((e = readdir(dir)) != nullptr) {
        if (e->d_name[0] == '.') continue;
        if (strcmp(e->d_name, "lo") == 0) continue;
        char path[320];
        snprintf(path, sizeof(path), "/sys/class/net/%s/carrier", e->d_name);
        int fd = open(path, O_RDONLY);
        if (fd < 0) continue; // 网卡未启用，读不到 carrier
        char val[8] = {0};
        ssize_t n = read(fd, val, sizeof(val) - 1);
        close(fd);
        if (n > 0 && val[0] == '1') { up = true; break; }
    }
    closedir(dir);
    return up;
}

/* ==========================================================
 * 硬件外设初始化与清理
 * ========================================================== */

/* --- GPIO (RTS) 时钟同步 --- */
static bool init_rts_gpio() {
    const int gpioNum = 96;  // GPIO3 A0
    char path[128];
    
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd >= 0) { dprintf(fd, "%d\n", gpioNum); close(fd); usleep(100000); }
    
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpioNum);
    fd = open(path, O_WRONLY);
    if (fd < 0) { perror("RTS direction"); return false; }
    write(fd, "out", 3); close(fd);
    
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpioNum);
    g_rtsGpioFd = open(path, O_WRONLY);
    return g_rtsGpioFd >= 0;
}

/* --- V4L2 摄像头 --- */
static bool init_camera() {
    g_camFd = open(VIDEO_DEV, O_RDWR | O_NONBLOCK);
    if (g_camFd < 0) { perror("open camera"); return false; }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = CAPTURE_WIDTH;
    fmt.fmt.pix_mp.height = CAPTURE_HEIGHT;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.num_planes = 1;
    if (ioctl(g_camFd, VIDIOC_S_FMT, &fmt) < 0) { perror("S_FMT"); return false; }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = VIDEO_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(g_camFd, VIDIOC_REQBUFS, &req) < 0) { perror("REQBUFS"); return false; }

    for (int i = 0; i < VIDEO_BUFFER_COUNT; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[3];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.m.planes = planes;
        buf.length = 1;
        ioctl(g_camFd, VIDIOC_QUERYBUF, &buf);
        g_buffers[i].length[0] = planes[0].length;
        g_buffers[i].start[0] = mmap(NULL, planes[0].length, PROT_READ | PROT_WRITE,
                                       MAP_SHARED, g_camFd, planes[0].m.mem_offset);
        struct v4l2_exportbuffer expbuf;
        memset(&expbuf, 0, sizeof(expbuf));
        expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        expbuf.index = i;
        expbuf.plane = 0;
        expbuf.flags = O_CLOEXEC | O_RDWR;
        if (ioctl(g_camFd, VIDIOC_EXPBUF, &expbuf) == 0) {
            g_buffers[i].dma_fd = expbuf.fd;
        } else {
            perror("警告: 无法导出 DMA_FD，可能不支持零拷贝");
            g_buffers[i].dma_fd = -1;
        }
    }

    for (int i = 0; i < VIDEO_BUFFER_COUNT; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[3];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.m.planes = planes;
        buf.length = 1;
        ioctl(g_camFd, VIDIOC_QBUF, &buf);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(g_camFd, VIDIOC_STREAMON, &type) < 0) { perror("STREAMON"); return false; }

    printf("Camera OK: %dx%d NV12\n", CAPTURE_WIDTH, CAPTURE_HEIGHT);
    return true;
}

static void cleanup_camera() {
    if (g_camFd >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ioctl(g_camFd, VIDIOC_STREAMOFF, &type);
        for (int i = 0; i < VIDEO_BUFFER_COUNT; i++) {
            if (g_buffers[i].start[0]) {
                munmap(g_buffers[i].start[0], g_buffers[i].length[0]);
                g_buffers[i].start[0] = nullptr;
            }
            if (g_buffers[i].dma_fd >= 0) {
                close(g_buffers[i].dma_fd);
                g_buffers[i].dma_fd = -1;
            }
        }
        close(g_camFd);
        g_camFd = -1;
    }
}

/* --- MPP H265 编码器 --- */
static bool init_mpp_encoder() {
    MPP_RET ret = mpp_create(&g_mppCtx, &g_mppApi);
    if (ret != MPP_OK) { printf("mpp_create failed\n"); return false; }

    ret = mpp_init(g_mppCtx, MPP_CTX_ENC, MPP_VIDEO_CodingHEVC);
    if (ret != MPP_OK) { printf("mpp_init failed\n"); return false; }

    MppEncCfg cfg;
    ret = mpp_enc_cfg_init(&cfg);
    if (ret != MPP_OK) { printf("cfg_init failed\n"); return false; }

    int bps = 1024 * 1024; // 目标码率 1Mbps

    mpp_enc_cfg_set_s32(cfg, "prep:width", CAPTURE_WIDTH);
    mpp_enc_cfg_set_s32(cfg, "prep:height", CAPTURE_HEIGHT);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", CAPTURE_WIDTH);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", CAPTURE_HEIGHT);
    mpp_enc_cfg_set_s32(cfg, "prep:format", MPP_FMT_YUV420SP);

    mpp_enc_cfg_set_s32(cfg, "rc:mode", MPP_ENC_RC_MODE_CBR);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_target", bps);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_max", bps * 1.5);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_min", bps * 0.5);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num", 30);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denorm", 1);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", 30);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denorm", 1);
    mpp_enc_cfg_set_s32(cfg, "rc:gop", 3000);

    mpp_enc_cfg_set_s32(cfg, "rc:refresh_en", 1);
    mpp_enc_cfg_set_s32(cfg, "rc:refresh_mode", 0);
    mpp_enc_cfg_set_s32(cfg, "rc:refresh_num", 30);

    mpp_enc_cfg_set_s32(cfg, "tune:scene_mode", MPP_ENC_SCENE_MODE_IPC);

    mpp_enc_cfg_set_s32(cfg, "rc:qp_init", 26);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 40);   
    mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 16);   
    mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 35); 
    mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 14);

    mpp_enc_cfg_set_s32(cfg, "tune:aq_en", 1);

    mpp_enc_cfg_set_u32(cfg, "rc:super_mode", 2);
    mpp_enc_cfg_set_u32(cfg, "rc:priority", 1);
    mpp_enc_cfg_set_u32(cfg, "rc:max_reenc_times", 3);
    mpp_enc_cfg_set_u32(cfg, "rc:super_i_thd", 10000);
    mpp_enc_cfg_set_u32(cfg, "rc:super_p_thd", 5000);

    mpp_enc_cfg_set_s32(cfg, "codec:type", MPP_VIDEO_CodingHEVC);

    ret = g_mppApi->control(g_mppCtx, MPP_ENC_SET_CFG, cfg);
    mpp_enc_cfg_deinit(cfg);
    if (ret != MPP_OK) { printf("SET_CFG failed\n"); return false; }

    MppEncHeaderMode headerMode = MPP_ENC_HEADER_MODE_EACH_IDR;
    g_mppApi->control(g_mppCtx, MPP_ENC_SET_HEADER_MODE, &headerMode);

    mpp_buffer_group_get_internal(&g_frmGroup, MPP_BUFFER_TYPE_ION);

    g_pts = 0;
    printf("MPP Encoder OK: %dx%d H265 @ %dbps\n", CAPTURE_WIDTH, CAPTURE_HEIGHT, bps);
    return true;
}

static void cleanup_mpp() {
    if (g_frmGroup) { mpp_buffer_group_put(g_frmGroup); g_frmGroup = nullptr; }
    if (g_mppCtx) { mpp_destroy(g_mppCtx); g_mppCtx = nullptr; g_mppApi = nullptr; }
}

/* --- 串口配置（termios2 自定义 4Mbps） --- */
static bool init_serial() {
    g_serialFd = open(SERIAL_DEV, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (g_serialFd < 0) { perror("open serial"); return false; }

    tcflush(g_serialFd, TCIOFLUSH);

    struct termios2 {
        tcflag_t c_iflag;
        tcflag_t c_oflag;
        tcflag_t c_cflag;
        tcflag_t c_lflag;
        cc_t c_line;
        cc_t c_cc[19];
        speed_t c_ispeed;
        speed_t c_ospeed;
    };

    #ifndef TCGETS2
    #define TCGETS2 _IOR('T', 0x2A, struct termios2)
    #endif
    #ifndef TCSETS2
    #define TCSETS2 _IOW('T', 0x2B, struct termios2)
    #endif
    #ifndef BOTHER
    #define BOTHER 0x00001000
    #endif

    struct termios2 tio;
    if (ioctl(g_serialFd, TCGETS2, &tio) < 0) {
        perror("TCGETS2");
        close(g_serialFd); g_serialFd = -1; return false;
    }

    tio.c_iflag = IGNBRK | IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cflag &= ~(CBAUD | CBAUDEX | CSIZE | PARENB | CSTOPB| CRTSCTS);
    tio.c_cflag |= CS8 | CLOCAL | CREAD | BOTHER ;
    tio.c_ispeed = 4000000;
    tio.c_ospeed = 4000000;
    
    if (ioctl(g_serialFd, TCSETS2, &tio) < 0) {
        perror("TCSETS2");
        close(g_serialFd); g_serialFd = -1; return false;
    }

    tcflush(g_serialFd, TCOFLUSH);
    printf("Serial OK: %s @ 4Mbps\n", SERIAL_DEV);
    return true;
}

static void cleanup_serial() {
    if (g_serialFd >= 0) 
    { 
        tcflush(g_serialFd, TCIOFLUSH);
        close(g_serialFd); 
        g_serialFd = -1; 
    }
}

/* --- UDP 网口配置 --- */
static bool init_udp() {
    g_udpFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udpFd < 0) { perror("socket"); return false; }

    int flags = fcntl(g_udpFd, F_GETFL, 0);
    fcntl(g_udpFd, F_SETFL, flags | O_NONBLOCK);

    int buf_size = 2 * 1024 * 1024; // 2MB
    setsockopt(g_udpFd, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size));
    setsockopt(g_udpFd, SOL_SOCKET, SO_SNDBUF, &buf_size, sizeof(buf_size));

    struct sockaddr_in localAddr = {};
    localAddr.sin_family = AF_INET;
    localAddr.sin_port = htons(LOCAL_PORT);
    localAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(g_udpFd, (struct sockaddr*)&localAddr, sizeof(localAddr)) < 0) {
        perror("bind udp"); return false;
    }

    g_remoteAddr.sin_family = AF_INET;
    g_remoteAddr.sin_port = htons(REMOTE_PORT);
    inet_pton(AF_INET, REMOTE_IP, &g_remoteAddr.sin_addr);

    printf("UDP Ready: Local Port %d, Target %s:%d\n", LOCAL_PORT, REMOTE_IP, REMOTE_PORT);
    return true;
}

static void cleanup_udp() {
    if (g_udpFd >= 0) { close(g_udpFd); g_udpFd = -1; }
}

/* ==========================================================
 * 核心处理逻辑与独立线程
 * ========================================================== */


/* --- 时钟同步线程：驱动发送端的系统节拍，控制 RTS 电平翻转 --- */
static void sender_sync_thread() {
    g_syncTfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    struct itimerspec its = {};
    its.it_value.tv_nsec = 1000000;
    its.it_interval.tv_nsec = 1000000;
    timerfd_settime(g_syncTfd, 0, &its, nullptr);
    
    bool rtsLevel = false;
    uint32_t sec = 0, ms = 0;
    
    dprintf(g_rtsGpioFd, "0\n");
    
    while (g_running) {
        struct pollfd pfd = { g_syncTfd, POLLIN, 0 };
        int ret = poll(&pfd, 1, 100);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ret == 0) continue;
        
        uint64_t exp = 0;
        if (read(g_syncTfd, &exp, sizeof(exp)) <= 0) continue;
        
        ms += (uint32_t)exp;  
        while (ms >= 1000) {
            ms -= 1000;
            sec++;
            rtsLevel = !rtsLevel;
            write(g_rtsGpioFd, rtsLevel ? "1" : "0", 1);
        }
        g_masterTimestampMs = (uint64_t)sec * 1000ULL + ms;
    }
    close(g_syncTfd);
    g_syncTfd = -1;
}

/* --- 平滑发送：串口专用，分片 + 背压 + 熔断 --- */
static bool write_all_paced(const uint8_t* data, size_t len) {
    size_t written = 0;
    const size_t CHUNK = 1024;
    
    auto start_time = std::chrono::steady_clock::now();

    while (written < len && g_running) {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (duration > 500) {
            printf("\n[熔断警告] 串口发送严重阻塞 (>500ms)！模组可能假死！\n");
            tcflush(g_serialFd, TCOFLUSH);
            return false;
        }

        int outq = 0;
        if (ioctl(g_serialFd, TIOCOUTQ, &outq) == 0) {
            while (outq > 1024 && g_running) {
                usleep(100); 
                ioctl(g_serialFd, TIOCOUTQ, &outq);
                auto inner_now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(inner_now - start_time).count() > 500) {
                    printf("\n[死锁熔断] 模组硬件假死，强行打破死锁！\n");
                    tcflush(g_serialFd, TCOFLUSH);
                    return false;
                }
            }
        }
        size_t to_write = (len - written > CHUNK) ? CHUNK : (len - written);
        ssize_t n = write(g_serialFd, data + written, to_write);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) { usleep(100); continue; }
            perror("serial write");
            return false;
        }
        if (n == 0) { usleep(100); continue; }
        written += n;
        
        if (written < len) {
            usleep(50);
        }
    }
    return true;
}

/* --- 介质无关的物理发送：use_udp 由每帧入口快照传入，保证一帧只走一条介质 --- */
static bool transport_send(const uint8_t* data, size_t len, bool use_udp) {
    if (use_udp && g_udpFd >= 0) {
        ssize_t n = sendto(g_udpFd, data, len, 0, (struct sockaddr*)&g_remoteAddr, sizeof(g_remoteAddr));
        if (n < 0) return false;
        usleep(50);
        return true;
    }
    if (!use_udp && g_serialFd >= 0) {
        return write_all_paced(data, len);
    }
    return false;
}

/* --- FEC 前向纠错发送：使用动态 chunk_size --- */
static bool fec_send_frame(const uint8_t* data, size_t len, uint64_t capture_ts) {
    // 帧入口快照介质：主线程切换介质的瞬间，保证一帧完整走同一条通路
    bool use_udp = g_useUdp.load();
    if ((!use_udp && !g_useSerial.load()) || !data || len == 0) return false;

    // 1. 侦测帧类型
    bool is_idr = false;
    for (size_t i = 0; i + 5 < len; ) {
        if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 1) {
            uint8_t nal_type = (data[i+3] >> 1) & 0x3F;
            if (nal_type == 19 || nal_type == 20 || nal_type == 32) { is_idr = true; break; }
            i += 4;
        } else if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1) {
            uint8_t nal_type = (data[i+4] >> 1) & 0x3F;
            if (nal_type == 19 || nal_type == 20 || nal_type == 32) { is_idr = true; break; }
            i += 5;
        } else {
            i++;
        }
    }

    bool use_fec = true;

    uint32_t frame_id = g_frame_seq++;
    uint32_t total_frame_len = len;
    uint16_t chunk_size = g_fecChunkSize.load(); // 使用全局动态值

    uint8_t current_k = is_idr ? 4 : 11;
    uint8_t current_m = is_idr ? 7 : 12;
    
    size_t group_capacity = current_k * chunk_size;
    uint16_t total_groups = (len + group_capacity - 1) / group_capacity;

    fec_t* fec = fec_new(current_k, current_m);

    for (uint16_t gid = 0; gid < total_groups; gid++) {
        size_t group_offset = gid * group_capacity;

        uint16_t max_payload_len = 0;
        std::vector<uint16_t> actual_lens(current_k, 0);
        
        for (int i = 0; i < current_k; i++) {
            size_t block_offset = group_offset + i * chunk_size;
            if (block_offset < len) {
                actual_lens[i] = std::min((size_t)chunk_size, len - block_offset);
                if (actual_lens[i] > max_payload_len) max_payload_len = actual_lens[i];
            }
        }
        if (max_payload_len == 0) continue;
        
        std::vector<const gf*> src_blocks(current_k, nullptr);
        std::vector<std::vector<uint8_t>> padded_data(current_k, std::vector<uint8_t>(max_payload_len, 0));

        for (int i = 0; i < current_k; i++) {
            if (actual_lens[i] > 0) {
                memcpy(padded_data[i].data(), data + group_offset + i * chunk_size, actual_lens[i]);
            }
            src_blocks[i] = (const gf*)padded_data[i].data();
        }

        // 3. 发送前 K 个真实数据包
        for (int i = 0; i < current_k; i++) {
            if (actual_lens[i] == 0) continue;

            size_t pkt_size = sizeof(FecHeader) + actual_lens[i] + 1;
            std::vector<uint8_t> pkt_buf(pkt_size, 0);
            FecHeader* hdr = (FecHeader*)pkt_buf.data();

            hdr->sync = 0x55AA55AA;
            hdr->frame_id = frame_id;
            hdr->capture_ts = capture_ts;
            hdr->total_frame_len = total_frame_len;
            hdr->group_id = gid;
            hdr->total_groups = total_groups;
            hdr->block_idx = i;
            hdr->payload_len = actual_lens[i];
            hdr->fec_block_size = max_payload_len;
            hdr->fec_k = current_k;
            hdr->fec_m = current_m;
            hdr->fec_chunk_size = chunk_size;
            hdr->header_crc = calc_checksum((uint8_t*)hdr, sizeof(FecHeader) - 1);

            memcpy(pkt_buf.data() + sizeof(FecHeader), src_blocks[i], actual_lens[i]);
            pkt_buf.back() = calc_checksum(pkt_buf.data(), pkt_size - 1);

            if (!transport_send(pkt_buf.data(), pkt_size, use_udp)) {
                fec_free(fec);
                return false;
            }
            g_statSentDataSlices++;
        }

        // 4. 发送冗余包
        if (use_fec) {
            int num_parities = current_m - current_k;
            std::vector<gf*> parity_blocks(num_parities);
            std::vector<unsigned> block_nums(num_parities);
            std::vector<std::vector<uint8_t>> parity_data(num_parities, std::vector<uint8_t>(max_payload_len, 0));

            for (int i = 0; i < num_parities; i++) {
                parity_blocks[i] = (gf*)parity_data[i].data();
                block_nums[i] = current_k + i;
            }

            fec_encode(fec, src_blocks.data(), parity_blocks.data(), block_nums.data(), num_parities, max_payload_len);

            for (int i = 0; i < num_parities; i++) {
                size_t pkt_size = sizeof(FecHeader) + max_payload_len + 1;
                std::vector<uint8_t> pkt_buf(pkt_size, 0);
                FecHeader* hdr = (FecHeader*)pkt_buf.data();

                hdr->sync = 0x55AA55AA;
                hdr->frame_id = frame_id;
                hdr->capture_ts = capture_ts;
                hdr->total_frame_len = total_frame_len;
                hdr->group_id = gid;
                hdr->total_groups = total_groups;
                hdr->block_idx = current_k + i;
                hdr->payload_len = max_payload_len;
                hdr->fec_block_size = max_payload_len;
                hdr->fec_k = current_k;
                hdr->fec_m = current_m;
                hdr->fec_chunk_size = chunk_size;
                hdr->header_crc = calc_checksum((uint8_t*)hdr, sizeof(FecHeader) - 1);

                memcpy(pkt_buf.data() + sizeof(FecHeader), parity_blocks[i], max_payload_len);
                pkt_buf.back() = calc_checksum(pkt_buf.data(), pkt_size - 1);

                if (!transport_send(pkt_buf.data(), pkt_size, use_udp)) {
                    fec_free(fec);
                    return false;
                }
                g_statSentParitySlices++;
            }
        }
    }

    fec_free(fec);
    g_statSentBytes += len;
    g_statSentFrames++;
    g_statCurFrameSize = (int)len;
    g_statCurFrameType = 0;
    if (len >= 6) {
        uint8_t nal_type = (data[4] >> 1) & 0x3F;
        if (nal_type == 19 || nal_type == 20 || nal_type == 32) { g_statSentIDR++; g_statCurFrameType = 2; }
        else if (nal_type == 1) { g_statSentP++; g_statCurFrameType = 1; }
    }
    if (g_statMinSize == 0 || (int)len < g_statMinSize) g_statMinSize = (int)len;
    if ((int)len > g_statMaxSize) g_statMaxSize = (int)len;
    return true;
}

/* --- 编码模块 --- */
static bool encode_and_send(const uint8_t* nv12, int dma_fd, int width, int height, uint64_t capture_ts) {
    int buf_size = width * height * 3 / 2;
    MppBuffer frmBuf = nullptr;

    if (dma_fd >= 0) {
        MppBufferInfo info;
        memset(&info, 0, sizeof(info));
        info.type = MPP_BUFFER_TYPE_EXT_DMA;
        info.fd = dma_fd;
        info.size = buf_size;
        mpp_buffer_import(&frmBuf, &info);
    } else {
        mpp_buffer_get(g_frmGroup, &frmBuf, buf_size);
        memcpy(mpp_buffer_get_ptr(frmBuf), nv12, buf_size);
    }

    if (!frmBuf) return false;

    MppFrame frame = nullptr;
    mpp_frame_init(&frame);
    mpp_frame_set_width(frame, width);
    mpp_frame_set_height(frame, height);
    mpp_frame_set_hor_stride(frame, width);
    mpp_frame_set_ver_stride(frame, height);
    mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
    mpp_frame_set_buffer(frame, frmBuf);
    mpp_frame_set_pts(frame, g_pts++);
    mpp_frame_set_eos(frame, 0);

    MPP_RET ret = g_mppApi->encode_put_frame(g_mppCtx, frame);
    mpp_frame_deinit(&frame);
    mpp_buffer_put(frmBuf);
    if (ret != MPP_OK) return false;

    uint8_t* frame_data = nullptr;
    size_t frame_len = 0;
    bool any_packet = false;

    while (1) {
        MppPacket packet = nullptr;
        ret = g_mppApi->encode_get_packet(g_mppCtx, &packet);
        if (ret != MPP_OK || !packet) break;

        MppMeta meta = mpp_packet_get_meta(packet);
        if (meta) {
            RK_S32 qp = 0;
            mpp_meta_get_s32(meta, KEY_ENC_AVERAGE_QP, &qp);
            if (qp > 0) {
                g_statCurQP = qp;
                g_statSumQP += qp;
                g_statCountQP++;
            }
        }

        void* data = mpp_packet_get_data(packet);
        size_t len = mpp_packet_get_length(packet);
        if (data && len > 0) {
            uint8_t* new_buf = (uint8_t*)realloc(frame_data, frame_len + len);
            if (new_buf) {
                frame_data = new_buf;
                memcpy(frame_data + frame_len, data, len);
                frame_len += len;
                any_packet = true;
            }
        }
        mpp_packet_deinit(&packet);
    }

    bool sent = false;
    if (any_packet && frame_data && frame_len > 0) {
        SendPacket pkt;
        pkt.data.assign(frame_data, frame_data + frame_len);
        pkt.capture_ts = capture_ts;
        
        {
            std::lock_guard<std::mutex> lock(g_sendMutex);
            g_sendQueue.push(std::move(pkt));
            g_sendCond.notify_one();
        }
        sent = true;
    }
    free(frame_data);
    return sent;
}

/* --- 独立发送线程 --- */
static void sender_send_thread() {
    while (g_running) {
        SendPacket pkt;
        {
            std::unique_lock<std::mutex> lock(g_sendMutex);
            g_sendCond.wait(lock, []{ return !g_sendQueue.empty() || !g_running; });
            if (!g_running && g_sendQueue.empty()) break;
            
            pkt = std::move(g_sendQueue.front());
            g_sendQueue.pop();
        }
        if (!pkt.data.empty()) {
            fec_send_frame(pkt.data.data(), pkt.data.size(), pkt.capture_ts);
        }
    }
}

/* --- 反向通信检测：统一处理 0xFF/0xEE（select 报告哪个 fd 就绪就读哪个） --- */
static void handle_reverse_comm(int fd, bool is_udp) {
    if (fd < 0) return;

    uint8_t buf[256];
    ssize_t n;
    if (is_udp) {
        n = recv(fd, buf, sizeof(buf), 0);
    } else {
        n = read(fd, buf, sizeof(buf));
    }

    if (n > 0) {
        int cnt_ff = 0, cnt_ee = 0, cnt_zero = 0;
        for (ssize_t i = 0; i < n; i++) {
            if (buf[i] == 0xFF) cnt_ff++;
            else if (buf[i] == 0xEE) cnt_ee++;
            else if (buf[i] == 0x00) cnt_zero++;
        }
        if (cnt_ff > n / 2) {
            printf("[IDR] 收到接收端 0xFF 反向请求 (n=%zd, ff=%d)，强制输出 IDR\n", n, cnt_ff);
            RK_S32 force_idr = 1;
            g_mppApi->control(g_mppCtx, MPP_ENC_SET_IDR_FRAME, &force_idr);
            g_idrRequested.store(true);
            g_receiverReady.store(false);
            std::lock_guard<std::mutex> lock(g_sendMutex);
            std::queue<SendPacket> empty;
            g_sendQueue.swap(empty);
        } else if (cnt_ee > n / 2) {
            if (!g_receiverReady.load()) {
                printf("[IDR] 收到接收端 0xEE 已接通信号 (n=%zd, ee=%d)，停止定期 IDR\n", n, cnt_ee);
            }
            g_receiverReady.store(true);
        }
        if (cnt_zero > n / 2) {
            usleep(5000);
        }
    } else if (n < 0 && errno != EAGAIN) {
        usleep(10000); 
    }
}

/* ==========================================================
 * 主函数
 * ========================================================== */
int main(int argc, char** argv) {
    fec_init();
    g_fecCtx = fec_new(FEC_K, FEC_M);
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);
    signal(SIGINT, sigint_handler);
    signal(SIGPIPE, SIG_IGN);

    srand((unsigned int)time(nullptr));

    system("/usr/bin/rkaiq_3A_server > /dev/null 2>&1 &");
    usleep(1000000);
    
    printf("等待摄像头就绪...\n");
    bool camera_ok = false;
    for (int i = 0; i < 20; i++) {
        if (init_camera()) {
            camera_ok = true;
            break;
        }
        usleep(100000); 
    }
    if (!camera_ok) {
        printf("Failed to init camera after retries\n");
        return 1;
    }

    if (!init_mpp_encoder()) { cleanup_camera(); return 1; }

    // ===== 双初始化传输层，谁成功用谁 =====
    bool serial_ok = init_serial();                                                                                                                                                                                                               
    bool udp_ok    = init_udp();

    // 都没开 → 报错退出
    if (!serial_ok && !udp_ok) {
        printf("ERROR: 串口和网口初始化全部失败！\n");
        cleanup_mpp(); cleanup_camera(); return 1;
    }

    // ===== 自动选择通路：检测网线物理连接 (carrier) =====
    bool link = net_link_up();
    bool force_serial = (argc >= 2 && strcmp(argv[1], "--serial") == 0);
    bool force_udp    = (argc >= 2 && strcmp(argv[1], "--udp") == 0);

    if (force_serial) {
        if (!serial_ok) {
            printf("ERROR: 串口不可用，无法强制串口模式\n");
            cleanup_mpp(); cleanup_camera(); cleanup_udp(); return 1;
        }
        g_useSerial = true;
        g_fecChunkSize = 1024;
        printf("[模式] 强制使用串口\n");
    } else if (force_udp) {
        if (!udp_ok) {
            printf("ERROR: 网口不可用，无法强制网口模式\n");
            cleanup_mpp(); cleanup_camera(); cleanup_serial(); return 1;
        }
        g_useUdp = true;
        g_fecChunkSize = 880;
        printf("[模式] 强制使用网口\n");
    } else if (udp_ok && link) {
        // 网线插着 → 网口（带宽优先）
        g_useUdp = true;
        g_fecChunkSize = 880;
        printf("[模式] 检测到网线已连接，自动选择网口\n");
    } else if (serial_ok) {
        // 网线没插 → 串口
        g_useSerial = true;
        g_fecChunkSize = 1024;
        printf("[模式] 网线未连接，自动选择串口\n");
    } else {
        // 只有网口但网线没插：尽力而为
        g_useUdp = true;
        g_fecChunkSize = 880;
        printf("[警告] 网线未连接且串口不可用，仅网口尽力发送\n");
    }

    printf("传输模式: %s, FEC chunk_size: %d, 网线状态: %s\n",
           g_useUdp.load() ? "UDP 网口" : "串口 UART",
           (int)g_fecChunkSize.load(), link ? "已连接" : "未连接");

    if (!init_rts_gpio()) { printf("Failed to init RTS GPIO\n"); return 1; }

    std::thread syncThread(sender_sync_thread);
    std::thread sendThread(sender_send_thread);

    printf("Sender running. Press Ctrl+C to stop.\n");
    g_startTime = time(nullptr);
    
    // 主循环
    while (g_running) {
        // 运行时自动回退：网口模式下 15 秒收不到接收端 0xEE 应答 → 切换串口
        {
            static bool s_switched = false;
            if (!s_switched && g_useUdp.load() && serial_ok && !g_receiverReady.load()
                && time(nullptr) - g_startTime >= 15) {
                s_switched = true;
                g_useUdp = false;
                g_useSerial = true;
                g_fecChunkSize = 1024;
                printf("[自动切换] 网口 15 秒无任何接收端应答，切换到串口图传\n");
            }
        }

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(g_camFd, &fds);
        int max_fd = g_camFd;

        // 监听所有已打开的传输层 fd（无论当前发送走哪条，反向信号两条都收）
        if (g_udpFd >= 0) {
            FD_SET(g_udpFd, &fds);
            if (g_udpFd > max_fd) max_fd = g_udpFd;
        }
        if (g_serialFd >= 0) {
            FD_SET(g_serialFd, &fds);
            if (g_serialFd > max_fd) max_fd = g_serialFd;
        }

        struct timeval tv = {0, 10000}; // 10ms
        if (select(max_fd + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;

        // 反向通信检测：哪个 fd 有数据就处理哪个
        if (g_udpFd >= 0 && FD_ISSET(g_udpFd, &fds)) {
            handle_reverse_comm(g_udpFd, true);
        }
        if (g_serialFd >= 0 && FD_ISSET(g_serialFd, &fds)) {
            handle_reverse_comm(g_serialFd, false);
        }

        // 定期 IDR 兜底
        {
            static time_t s_last_periodic_idr = 0;
            time_t now = time(nullptr);
            if (!g_receiverReady.load() && now - s_last_periodic_idr >= 5) {
                s_last_periodic_idr = now;
                RK_S32 force_idr = 1;
                g_mppApi->control(g_mppCtx, MPP_ENC_SET_IDR_FRAME, &force_idr);
                g_idrRequested.store(true);
                printf("[IDR] 未接通，定期主动输出 IDR 兜底\n");
            }
        }
        
        // 摄像头采集处理
        if (FD_ISSET(g_camFd, &fds)) {
            struct v4l2_buffer buf;
            struct v4l2_plane planes[3];
            memset(&buf, 0, sizeof(buf));
            memset(planes, 0, sizeof(planes));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.m.planes = planes;
            buf.length = 1;

            if (ioctl(g_camFd, VIDIOC_DQBUF, &buf) < 0) {
                usleep(10000);
                continue;
            }

            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            uint64_t sys_now_ms = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
            uint64_t buf_ts_ms = buf.timestamp.tv_sec * 1000 + buf.timestamp.tv_usec / 1000;
            uint64_t age_ms = (sys_now_ms >= buf_ts_ms) ? (sys_now_ms - buf_ts_ms) : 0;
            uint64_t exact_capture_ts = g_masterTimestampMs - age_ms;

            bool skip_encode = false;
            bool idr_needed = g_idrRequested.load();
            {
                std::lock_guard<std::mutex> lock(g_sendMutex);
                if (g_sendQueue.size() >= 2 && !idr_needed) {
                    skip_encode = true;
                }
            }

            if (skip_encode) {
                ioctl(g_camFd, VIDIOC_QBUF, &buf);
            } else {
                uint8_t* nv12_data = (uint8_t*)g_buffers[buf.index].start[0];
                int dma_fd = g_buffers[buf.index].dma_fd;

                encode_and_send(nv12_data, dma_fd, CAPTURE_WIDTH, CAPTURE_HEIGHT, exact_capture_ts);
                if (idr_needed) {
                    g_idrRequested.store(false);
                }
                ioctl(g_camFd, VIDIOC_QBUF, &buf);
            }
        }

        // 定期数据统计
        time_t now = time(nullptr);
        if (g_statLastTime == 0) g_statLastTime = now;
        if (now - g_statLastTime >= 10) {
            int elapsed = now - g_statLastTime;
            float fps = (float)g_statSentFrames / elapsed;
            float kbps = (float)g_statSentBytes * 8.0f / 1000.0f / elapsed;
            float avg = g_statSentFrames > 0 ? (float)g_statSentBytes / g_statSentFrames / 1024.0f : 0;
            int period = (int)(now - g_startTime);
            float max_kb = g_statMaxSize / 1024.0f;
            float min_kb = g_statMinSize / 1024.0f;
            float cur_kb = g_statCurFrameSize / 1024.0f;
            float avg_qp = g_statCountQP > 0 ? (float)g_statSumQP / g_statCountQP : 0;
            const char* cur_type = (g_statCurFrameType == 2) ? "IDR帧" : (g_statCurFrameType == 1) ? "P帧" : "未知";
            
            printf("========== 实时统计 [第 %d 秒] ==========\n", period);
            printf("帧率 FPS: \"%.1f\" fps\n", fps);
            printf("码率: \"%.0f\" Kbps\n", kbps);
            printf("平均帧大小: \"%.1f\" KB\n", avg);
            printf("最大帧: \"%.1f\"KB  最小帧: \"%.1f\"KB\n", max_kb, min_kb);
            printf("I帧: %d   P帧: %d   总帧: %d\n", g_statSentIDR, g_statSentP, g_statSentFrames);
            printf("画面质量(QP): 平均 \"%.1f\"  当前帧: %d\n", avg_qp, g_statCurQP);
            float fec_overhead = 0.0f;
            if (g_statSentDataSlices > 0) {
                fec_overhead = (float)g_statSentParitySlices / g_statSentDataSlices * 100.0f;
            }
            printf("FEC统计: 发送真实包:%d 校验包:%d (带宽开销: %.1f%%)\n", 
                   g_statSentDataSlices, g_statSentParitySlices, fec_overhead);
            printf("当前帧: %s 大小: %.1f KB\n", cur_type, cur_kb);
            printf("=================================\n");
            
            g_statSentBytes = 0;
            g_statSentFrames = 0;
            g_statSentIDR = 0;
            g_statSentP = 0;
            g_statMinSize = 0;
            g_statMaxSize = 0;
            g_statSumQP = 0;
            g_statCountQP = 0;
            g_statSentDataSlices = 0;
            g_statSentParitySlices = 0;
            g_statLastTime = now;
        }
    }

    g_running = false;
    if (g_syncTfd >= 0) { close(g_syncTfd); }
    
    syncThread.join();
    g_sendCond.notify_all(); 
    sendThread.join();
    
    cleanup_mpp();
    cleanup_camera();
    cleanup_serial();
    cleanup_udp();
    
    printf("Sender exit.\n");
    return 0;
}

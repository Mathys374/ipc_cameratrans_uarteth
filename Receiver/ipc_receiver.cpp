#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <ctime>
#include <drm/drm.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <rockchip/rk_mpi.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_packet.h>
#include <im2d.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <sys/timerfd.h>
#include <poll.h>
#include <sched.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdarg.h>
#include <dirent.h>

extern "C" {
    #include "fec.h"
}
#include "../osd_burner.h"

static fec_t* g_fecCtx = nullptr;

/* ==========================================================
 * 宏定义与常量
 * ========================================================== */
#define SERIAL_DEV      "/dev/ttyS2"
#define MAX_FRAME_SIZE  (2 * 1024 * 1024)
#define LOCAL_PORT 8001
#define REMOTE_IP "192.168.144.109"
#define REMOTE_PORT 8000

/* FEC 协议数据结构 */
#define FEC_K 11
#define FEC_M 12

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
    uint16_t fec_chunk_size;   // 动态 FEC 原始切块大小 (从发送端 header 读取)
    uint8_t  header_crc;       // 头部校验

};
#pragma pack(pop)

struct FecGroup {
    std::vector<std::vector<uint8_t>> blocks;
    std::vector<uint8_t> block_indices;
    bool decoded = false;
};

struct FrameReassembler {
    uint32_t frame_id = 0xFFFFFFFF;
    uint64_t capture_ts = 0;
    uint32_t total_frame_len = 0;
    uint16_t total_groups = 0;
    std::vector<FecGroup> groups;
    uint16_t ready_groups = 0;
    uint8_t  fec_k = 0;
    uint8_t  fec_m = 0;
    uint16_t fec_chunk_size = 0;  // 从 header 动态获取的 chunk_size
    bool pushed = false;

    void reset(uint32_t id, uint16_t tgroups, uint32_t tflen, uint64_t ts,
               uint8_t k, uint8_t m, uint16_t chunk_sz) {
        frame_id = id;
        total_groups = tgroups;
        total_frame_len = tflen;
        capture_ts = ts;
        fec_k = k;
        fec_m = m;
        fec_chunk_size = chunk_sz;
        groups.assign(total_groups, FecGroup());
        ready_groups = 0;
        pushed = false;
    }
};

/* 多线程队列变量：用于存放完整重组的帧数据 */
struct FramePacket {
    std::vector<uint8_t> data;
    int64_t capture_ts;
};

/* DRM 硬件显示缓冲结构体 */
struct DrmBuffer {
    int drm_fd;
    uint32_t width, height, pitch;
    uint32_t handle[3], fb_id[3];
    uint32_t crtc_id, conn_id;
    unsigned int size;
    uint8_t *vaddr[3];
    int dma_fd[3];
};


/* ==========================================================
 * 全局变量
 * ========================================================== */
static volatile bool g_running = true;
static std::queue<FramePacket> g_frameQueue;
static std::mutex g_queueMutex;
static std::condition_variable g_queueCond;

/* 传输介质：接收端同时监听所有已打开的 fd 并在所有通路上发反向信号，无需二选一 */

/* 时钟同步变量 */
static int g_ctsGpioFd = -1;
static volatile uint32_t g_slaveSec = 0;
static volatile uint32_t g_slaveMs = 0;
static volatile uint32_t g_cachedSec = 0;
static volatile bool g_slaveSynced = false;

/* 硬件句柄与协议流 */
static int g_serialFd = -1;
static int g_udpFd = -1;
static struct sockaddr_in g_remoteAddr;
static DrmBuffer g_drm = {};
static MppCtx g_mppCtx = nullptr;
static MppApi* g_mppApi = nullptr;
static RK_S64 g_pts = 0;
static FrameReassembler g_rxFrame;

/* 协议解析状态 */
static uint32_t g_frameLen = 0;
static uint8_t* g_frameBuf = nullptr;
static uint8_t g_pktBuf[2048];
static size_t  g_pktIdx = 0;
static size_t  g_targetLen = sizeof(FecHeader);

/* 显示与丢帧控制 */
static RK_S64 g_last_display_pts = -1;
static bool g_waitIdr = true;

/* 统计变量 */
static float g_statAvgLatency = 0.0f;
static int g_totalBytes = 0;
static int g_totalReads = 0;
static int g_rxFrames = 0;
static int g_decodeOk = 0;
static int g_decodeErr = 0;
static time_t g_lastStatTime = 0;
static time_t g_startTime = 0;  

static uint64_t g_lastStatTimeMs = 0;
static uint64_t g_startTimeMs = 0;

static int g_statRecvIDR = 0;
static int g_statRecvP = 0;
static int g_statMinSize = 0;
static int g_statMaxSize = 0;
static int g_statDecodeBytes = 0;
static int g_statDecodeIDR = 0;
static int g_statDecodeP = 0;
static int g_statCurFrameSize = 0;
static int g_statCurFrameType = 0;

static int g_statDropFrames = 0;
static int g_statChecksumFail = 0;
static int g_statWaitIdrDrop = 0;
static int g_statFecRecoveredSlices = 0;

static uint64_t g_zeroFpsStartMs = 0;
static uint64_t g_zeroFpsMaxDurationMs = 0;


/* ==========================================================
 * 基础工具函数
 * ========================================================== */
static void sigint_handler(int) {
    printf("\nReceiver stopping...\n");
    g_running = false;
}

static void set_realtime_priority(int priority = 50) {
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        perror("警告: 设置 SCHED_FIFO 失败 (请确保使用 sudo 运行程序)");
    }
}

static uint8_t calc_checksum(const uint8_t* data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

/* 检测网线物理连接状态：扫描 /sys/class/net 下所有网卡（排除 lo）的 carrier */
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
        if (fd < 0) continue;
        char val[8] = {0};
        ssize_t n = read(fd, val, sizeof(val) - 1);
        close(fd);
        if (n > 0 && val[0] == '1') { up = true; break; }
    }
    closedir(dir);
    return up;
}

static bool is_idr_packet(const uint8_t* data, size_t len) {
    if (!data || len < 6) return false;
    for (size_t i = 0; i + 5 < len; ) {
        if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 1) {
            uint8_t nal_type = (data[i+3] >> 1) & 0x3F;
            if (nal_type == 19 || nal_type == 20 || nal_type == 32) return true;
            i += 4;
        } else if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1) {
            uint8_t nal_type = (data[i+4] >> 1) & 0x3F;
            if (nal_type == 19 || nal_type == 20 || nal_type == 32) return true;
            i += 5;
        } else {
            i++;
        }
    }
    return false;
}

/* ==========================================================
 * 反向通信发送：统一封装 0xFF / 0xEE
 * ========================================================== */
static void send_reverse_byte(uint8_t val) {
    uint8_t req[256];
    memset(req, val, sizeof(req));

    if (g_udpFd >= 0) {
        // 网口是 UDP 数据报，反向信号（0xFF/0xEE）统一用 30 字节短包
        sendto(g_udpFd, req, 30, 0, (struct sockaddr*)&g_remoteAddr, sizeof(g_remoteAddr));
    }
    if (g_serialFd >= 0) {
        write(g_serialFd, req, sizeof(req));   // 串口仍发 256 字节
    }
}

/* ==========================================================
 * 硬件外设初始化与清理
 * ========================================================== */

/* --- GPIO (CTS) --- */
static bool init_cts_gpio() {
    const int gpioNum = 97;
    char path[128];
    
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd >= 0) { dprintf(fd, "%d\n", gpioNum); close(fd); usleep(100000); }
    
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpioNum);
    fd = open(path, O_WRONLY);
    if (fd < 0) return false;
    write(fd, "in", 2); close(fd);
    
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/edge", gpioNum);
    fd = open(path, O_WRONLY);
    if (fd >= 0) { write(fd, "both", 4); close(fd); }
    
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpioNum);
    g_ctsGpioFd = open(path, O_RDONLY);
    return g_ctsGpioFd >= 0;
}

/* --- DRM --- */
static bool init_drm() {
    g_drm.drm_fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    if (g_drm.drm_fd < 0) { perror("open /dev/dri/card0"); return false; }

    drmModeRes *res = drmModeGetResources(g_drm.drm_fd);
    if (!res) { close(g_drm.drm_fd); g_drm.drm_fd = -1; return false; }

    drmModeConnector *conn = nullptr;
    drmModeModeInfo mode;
    for (int i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(g_drm.drm_fd, res->connectors[i]);
        if (conn && conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0) {
            g_drm.conn_id = conn->connector_id;
            mode = conn->modes[0];
            g_drm.width = mode.hdisplay;
            g_drm.height = mode.vdisplay;
            break;
        }
        if (conn) { drmModeFreeConnector(conn); conn = nullptr; }
    }

    if (!conn) {
        printf("No connected DRM connector\n");
        drmModeFreeResources(res);
        close(g_drm.drm_fd); g_drm.drm_fd = -1;
        return false;
    }

    drmModeEncoder *enc = drmModeGetEncoder(g_drm.drm_fd, conn->encoder_id);
    if (enc) { g_drm.crtc_id = enc->crtc_id; drmModeFreeEncoder(enc); }
    else if (res->count_crtcs > 0) { g_drm.crtc_id = res->crtcs[0]; }
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);

    if (g_drm.crtc_id == 0) { printf("No CRTC\n"); return false; }

    for (int i = 0; i < 3; i++) {
        struct drm_mode_create_dumb creq = {};
        creq.width = g_drm.width; creq.height = g_drm.height; creq.bpp = 32;
        if (drmIoctl(g_drm.drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0) {
            perror("CREATE_DUMB"); return false;
        }
        g_drm.handle[i] = creq.handle; 
        g_drm.pitch = creq.pitch; 
        g_drm.size = creq.size;

        struct drm_mode_map_dumb mreq = {};
        mreq.handle = g_drm.handle[i];
        if (drmIoctl(g_drm.drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0) {
            perror("MAP_DUMB"); return false;
        }

        g_drm.vaddr[i] = (uint8_t*)mmap(0, g_drm.size, PROT_READ | PROT_WRITE,
                                       MAP_SHARED, g_drm.drm_fd, mreq.offset);
        if (g_drm.vaddr[i] == MAP_FAILED) { perror("mmap"); return false; }

        if (drmPrimeHandleToFD(g_drm.drm_fd, g_drm.handle[i], DRM_CLOEXEC, &g_drm.dma_fd[i]) < 0) {
            perror("警告: drmPrimeHandleToFD 失败");
            g_drm.dma_fd[i] = -1;
        }

        if (drmModeAddFB(g_drm.drm_fd, g_drm.width, g_drm.height, 24, 32,
                         g_drm.pitch, g_drm.handle[i], &g_drm.fb_id[i]) < 0) {
            perror("AddFB"); return false;
        }

        memset(g_drm.vaddr[i], 0x80, g_drm.size);
    }

    if (drmModeSetCrtc(g_drm.drm_fd, g_drm.crtc_id, g_drm.fb_id[0],
                       0, 0, &g_drm.conn_id, 1, &mode) < 0) {
        printf("ERROR: SetCrtc failed: %s\n", strerror(errno));
        return false;
    }

    printf("DRM OK (Triple Buffered): %dx%d\n", g_drm.width, g_drm.height);
    return true;
}

static void cleanup_drm() {
    if (g_drm.drm_fd < 0) return;
    for (int i = 0; i < 3; i++) {
        if (g_drm.vaddr[i] && g_drm.vaddr[i] != (uint8_t*)MAP_FAILED) {
            memset(g_drm.vaddr[i], 0, g_drm.size);
            munmap(g_drm.vaddr[i], g_drm.size); 
            g_drm.vaddr[i] = nullptr;
        }
        if (g_drm.fb_id[i]) { drmModeRmFB(g_drm.drm_fd, g_drm.fb_id[i]); g_drm.fb_id[i] = 0; }
        if (g_drm.handle[i]) {
            struct drm_mode_destroy_dumb dreq = {}; dreq.handle = g_drm.handle[i];
            drmIoctl(g_drm.drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
            g_drm.handle[i] = 0;
        }
    }
    close(g_drm.drm_fd); g_drm.drm_fd = -1;
}

/* --- MPP H265 硬解码器 --- */
static bool init_mpp_decoder() {
    MPP_RET ret = mpp_create(&g_mppCtx, &g_mppApi);
    if (ret != MPP_OK) { printf("mpp_create failed\n"); return false; }

    ret = mpp_init(g_mppCtx, MPP_CTX_DEC, MPP_VIDEO_CodingHEVC);
    if (ret != MPP_OK) { printf("mpp_init dec failed\n"); return false; }

    RK_U32 fast_mode = 1;
    g_mppApi->control(g_mppCtx, MPP_DEC_SET_PARSER_FAST_MODE, &fast_mode);

    RK_U32 output_block = 0;
    g_mppApi->control(g_mppCtx, MPP_SET_OUTPUT_BLOCK, &output_block);

    printf("MPP Decoder OK: H265 (low-latency)\n");
    return true;
}

static void cleanup_mpp() {
    if (g_mppCtx) { mpp_destroy(g_mppCtx); g_mppCtx = nullptr; g_mppApi = nullptr; }
}

/* --- 串口配置 --- */
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

    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cflag &= ~(CBAUD | CBAUDEX | CSIZE | PARENB | CSTOPB| CRTSCTS);
    tio.c_cflag |= CS8 | CLOCAL | CREAD | BOTHER ;
    tio.c_ispeed = 3000000;
    tio.c_ospeed = 3000000;

    if (ioctl(g_serialFd, TCSETS2, &tio) < 0) {
        perror("TCSETS2");
        close(g_serialFd); g_serialFd = -1; return false;
    }

    tcflush(g_serialFd, TCOFLUSH);
    printf("Serial OK: %s @ 3Mbps\n", SERIAL_DEV);
    return true;
}

static void cleanup_serial() {
    if (g_serialFd >= 0) { close(g_serialFd); g_serialFd = -1; }
}

/* --- UDP 网口配置 --- */
static bool init_udp() {
    g_udpFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udpFd < 0) { perror("socket"); return false; }

    int flags = fcntl(g_udpFd, F_GETFL, 0);
    fcntl(g_udpFd, F_SETFL, flags | O_NONBLOCK);

    int buf_size = 2 * 1024 * 1024;
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
 * 同步时钟与中断线程
 * ========================================================== */

static void slave_timer_thread() {
    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    struct itimerspec its = {};
    its.it_value.tv_nsec = 1000000;
    its.it_interval.tv_nsec = 1000000;
    timerfd_settime(tfd, 0, &its, nullptr);
    
    while (g_running) {
        uint64_t exp;
        if (read(tfd, &exp, sizeof(exp)) <= 0) continue;
        
        if (!g_slaveSynced) continue;
        
        g_slaveMs += (uint32_t)exp;
        if (g_slaveMs >= 1000) {
            g_slaveMs = 0;
            g_slaveSec++;
        }
    }
    close(tfd);
}

static void cts_interrupt_thread() {
    set_realtime_priority(60);
    struct pollfd pfd = { g_ctsGpioFd, POLLPRI, 0 };
    char buf[4];
    
    while (g_running) {
        if (poll(&pfd, 1, 1000) <= 0) continue;
        
        lseek(g_ctsGpioFd, 0, SEEK_SET);
        read(g_ctsGpioFd, buf, sizeof(buf));
        
        g_slaveSec = g_cachedSec+1;
        g_slaveMs = 0;
        g_slaveSynced = true;
    }
}

/* ==========================================================
 * 数据协议处理与组帧逻辑
 * ========================================================== */

static void check_and_push_frame() {
    if (g_rxFrame.pushed || g_rxFrame.frame_id == 0xFFFFFFFF) return;

    fec_t* fec = nullptr;
    uint8_t current_k = g_rxFrame.fec_k;
    uint8_t current_m = g_rxFrame.fec_m;
    uint16_t chunk_size = g_rxFrame.fec_chunk_size; // 使用从 header 读取的动态值
    
    if (current_k == 0 || current_m == 0 || current_k >= current_m) return;

    for (uint16_t gid = 0; gid < g_rxFrame.total_groups; gid++) {
        FecGroup& grp = g_rxFrame.groups[gid];
        if (grp.decoded) continue;
        if (grp.blocks.empty()) continue; 

        size_t dynamic_block_size = grp.blocks[0].size();

        for (uint8_t i = 0; i < current_k; i++) {
            // 用动态 chunk_size 计算偏移
            size_t block_offset = gid * current_k * chunk_size + i * chunk_size;
            if (block_offset >= g_rxFrame.total_frame_len) {
                bool found = false;
                for (uint8_t idx : grp.block_indices) {
                    if (idx == i) { found = true; break; }
                }
                if (!found) {
                    grp.blocks.push_back(std::vector<uint8_t>(dynamic_block_size, 0));
                    grp.block_indices.push_back(i);
                }
            }
        }

        if (grp.blocks.size() >= current_k) {
            if (!fec) fec = fec_new(current_k, current_m);

            std::vector<const gf*> inpkts(current_k);
            std::vector<unsigned> index(current_k, 999);
            std::vector<gf*> outpkts(current_k);

            for (size_t b = 0; b < grp.blocks.size(); b++) {
                uint8_t b_idx = grp.block_indices[b];
                if (b_idx < current_k) {
                    inpkts[b_idx] = (const gf*)grp.blocks[b].data();
                    index[b_idx] = b_idx;
                }
            }

            size_t used_block_ptr = 0;
            int recovered_count = 0;
            std::vector<std::vector<uint8_t>> recovered_data;
            recovered_data.reserve(current_k);

            for (unsigned i = 0; i < current_k; i++) {
                if (index[i] == 999) {
                    while (used_block_ptr < grp.blocks.size() && 
                           grp.block_indices[used_block_ptr] < current_k) {
                        used_block_ptr++;
                    }
                    inpkts[i] = (const gf*)grp.blocks[used_block_ptr].data();
                    index[i] = grp.block_indices[used_block_ptr];
                    used_block_ptr++;

                    recovered_data.emplace_back(std::vector<uint8_t>(dynamic_block_size, 0));
                    outpkts[recovered_count++] = (gf*)recovered_data.back().data();
                    g_statFecRecoveredSlices++;
                }
            }

            if (recovered_count > 0) {
                fec_decode(fec, inpkts.data(), outpkts.data(), index.data(), dynamic_block_size);
                
                int rec_idx = 0;
                for (unsigned i = 0; i < current_k; i++) {
                    if (index[i] != i) { 
                        grp.blocks.push_back(std::move(recovered_data[rec_idx++]));
                        grp.block_indices.push_back(i);
                    }
                }
            }
            grp.decoded = true;
            g_rxFrame.ready_groups++;
        }
    }

    if (fec) fec_free(fec);

    if (g_rxFrame.ready_groups == g_rxFrame.total_groups) {
        FramePacket pkt;
        pkt.capture_ts = g_rxFrame.capture_ts;
        // 用动态 chunk_size
        pkt.data.reserve(g_rxFrame.total_groups * current_k * chunk_size);

        for (uint16_t gid = 0; gid < g_rxFrame.total_groups; gid++) {
            FecGroup& grp = g_rxFrame.groups[gid];
            for (uint8_t i = 0; i < current_k; i++) {
                for (size_t b = 0; b < grp.blocks.size(); b++) {
                    if (grp.block_indices[b] == i) {
                        pkt.data.insert(pkt.data.end(), grp.blocks[b].begin(), grp.blocks[b].end());
                        break;
                    }
                }
            }
        }

        if (pkt.data.size() > g_rxFrame.total_frame_len) {
            pkt.data.resize(g_rxFrame.total_frame_len);
        }

        g_statCurFrameSize = pkt.data.size();
        if (g_statMinSize == 0 || g_statCurFrameSize < g_statMinSize) g_statMinSize = g_statCurFrameSize;
        if (g_statCurFrameSize > g_statMaxSize) g_statMaxSize = g_statCurFrameSize;
        g_cachedSec = (uint32_t)(g_rxFrame.capture_ts / 1000);
        
        {
            std::lock_guard<std::mutex> lock(g_queueMutex);
            if (g_frameQueue.size() < 10) {
                g_frameQueue.push(std::move(pkt));
                g_queueCond.notify_one();
            }
        }
        g_rxFrame.pushed = true;
        g_rxFrames++; 
    }
}

/* --- 字节流解析状态机 --- */
static void process_rx_byte(uint8_t byte) {
    const uint8_t SYNC[4] = {0xAA, 0x55, 0xAA, 0x55}; 
    
    if (g_pktIdx < 4) {
        if (byte == SYNC[g_pktIdx]) {
            g_pktBuf[g_pktIdx++] = byte;
        } else {
            g_pktIdx = (byte == SYNC[0]) ? 1 : 0;
            if (g_pktIdx == 1) g_pktBuf[0] = SYNC[0];
        }
        return;
    }

    g_pktBuf[g_pktIdx++] = byte;

    if (g_pktIdx == sizeof(FecHeader)) {
        FecHeader* hdr = (FecHeader*)g_pktBuf;
        
        if (calc_checksum(g_pktBuf, sizeof(FecHeader) - 1) != hdr->header_crc) {
            g_statChecksumFail++;
            g_pktIdx = 0; 
            g_targetLen = sizeof(FecHeader);
            return;
        }
        
        g_targetLen = sizeof(FecHeader) + hdr->payload_len + 1;
    }

    if (g_pktIdx == g_targetLen) {
        size_t full_pkt_len = g_targetLen;  
        
        g_pktIdx = 0; 
        g_targetLen = sizeof(FecHeader);

        if (calc_checksum(g_pktBuf, full_pkt_len - 1) != g_pktBuf[full_pkt_len - 1]) {
            g_statChecksumFail++; 
            return;
        }

        FecHeader* hdr = (FecHeader*)g_pktBuf;
        uint8_t* payload = g_pktBuf + sizeof(FecHeader);

        if (hdr->frame_id != g_rxFrame.frame_id) {
            if (g_rxFrame.frame_id != 0xFFFFFFFF && !g_rxFrame.pushed) {
                g_statDropFrames++;
                g_waitIdr = true;
            }
            // 用 header 中的 fec_chunk_size 替代编译宏
            g_rxFrame.reset(hdr->frame_id, hdr->total_groups, hdr->total_frame_len,
                            hdr->capture_ts, hdr->fec_k, hdr->fec_m, hdr->fec_chunk_size);
        }

        uint16_t gid = hdr->group_id;

        if (gid < g_rxFrame.total_groups && !g_rxFrame.groups[gid].decoded) {
            
            bool duplicate = false;
            for (uint8_t idx : g_rxFrame.groups[gid].block_indices) {
                if (idx == hdr->block_idx) { 
                    duplicate = true; 
                    break; 
                }
            }
            
            if (!duplicate) {
                std::vector<uint8_t> block_data(hdr->fec_block_size, 0);
                
                if (hdr->payload_len > 0) {
                    memcpy(block_data.data(), payload, hdr->payload_len);
                }

                g_rxFrame.groups[gid].blocks.push_back(std::move(block_data));
                g_rxFrame.groups[gid].block_indices.push_back(hdr->block_idx);
                
                check_and_push_frame();
            }
        }
    }
}

/* ==========================================================
 * 解码与显示流水线线程
 * ========================================================== */

static bool decode_and_display(const uint8_t* h265_data, size_t len, uint64_t capture_ts) {
    if (!g_mppCtx || !h265_data || len == 0) return false;

    if (is_idr_packet(h265_data, len)) {
        g_waitIdr = false;
    } else if (g_waitIdr && len >= 6) {
        for (size_t i = 0; i + 5 < len && i < 20; ) {
            if (h265_data[i]==0 && h265_data[i+1]==0 && h265_data[i+2]==1) {
                printf("[DIAG] 等待IDR期间收到非IDR帧, NAL type=%d, len=%zu\n", (h265_data[i+3]>>1)&0x3F, len);
                break;
            } else if (h265_data[i]==0 && h265_data[i+1]==0 && h265_data[i+2]==0 && h265_data[i+3]==1) {
                printf("[DIAG] 等待IDR期间收到非IDR帧, NAL type=%d, len=%zu\n", (h265_data[i+4]>>1)&0x3F, len);
                break;
            } else {
                i++;
            }
        }
    }

    // 从等待 IDR 状态恢复时，发送 0xEE
    {
        static bool s_lastWaitIdr = true;
        if (s_lastWaitIdr && !g_waitIdr) {
            send_reverse_byte(0xEE);
            printf("[IDR] 已接通，发送 0xEE 通知发送端停止定期 IDR\n");
        }
        s_lastWaitIdr = g_waitIdr;
    }

    if (g_waitIdr) {
        g_statWaitIdrDrop++;
        g_statDropFrames++;
        static time_t s_last_req_time = 0;
        time_t now = time(nullptr);
        if (now - s_last_req_time >= 1) {
            s_last_req_time = now;
            send_reverse_byte(0xFF);
        }
        return false;
    }

    MppPacket packet = nullptr;
    mpp_packet_init(&packet, (void*)h265_data, len);
    mpp_packet_set_pts(packet, capture_ts);
    mpp_packet_set_length(packet, len);

    MPP_RET ret;
    int retry = 0;
    while ((ret = g_mppApi->decode_put_packet(g_mppCtx, packet)) != MPP_OK) {
        if (ret == MPP_ERR_BUFFER_FULL && retry < 10) {
            usleep(2000);
            retry++;
            continue;
        }
        g_waitIdr = true;
        mpp_packet_deinit(&packet);
        g_decodeErr++;
        return false;
    }
    mpp_packet_deinit(&packet);

    MppFrame last_frame = nullptr;
    RK_S64 last_pts = -1;
    int last_w = 0, last_h = 0, last_hs = 0, last_vs = 0;
    MppBuffer last_buf = nullptr;

    while (1) {
        MppFrame frame = nullptr;
        ret = g_mppApi->decode_get_frame(g_mppCtx, &frame);
        if (ret != MPP_OK || !frame) break;

        if (mpp_frame_get_info_change(frame)) {
            RK_U32 info_change_ready = 1;
            g_mppApi->control(g_mppCtx, MPP_DEC_SET_INFO_CHANGE_READY, &info_change_ready);
            mpp_frame_deinit(&frame);
            continue;
        }

        MppBuffer buf = mpp_frame_get_buffer(frame);
        if (!buf) {
            mpp_frame_deinit(&frame);
            continue;
        }

        if (mpp_frame_get_errinfo(frame) != 0) {
            mpp_frame_deinit(&frame);
            g_decodeErr++;
            g_mppApi->reset(g_mppCtx);
            g_waitIdr = true;
            continue;
        }

        RK_S64 frame_pts = mpp_frame_get_pts(frame);

        if (g_last_display_pts != -1 && (g_last_display_pts - frame_pts) > 5000) {
            printf("[警告] 检测到发送端时间戳重置，恢复显示流水线...\n");
            g_last_display_pts = -1;
        }

        if (frame_pts <= g_last_display_pts) {
            mpp_frame_deinit(&frame);
            continue;
        }

        if (last_frame) mpp_frame_deinit(&last_frame);

        last_frame = frame;
        last_pts = frame_pts;
        last_w = mpp_frame_get_width(frame);
        last_h = mpp_frame_get_height(frame);
        last_hs = mpp_frame_get_hor_stride(frame);
        last_vs = mpp_frame_get_ver_stride(frame);
        last_buf = buf;
    }

    bool displayed = false;
    if (last_frame && last_buf) {
        void* ptr = mpp_buffer_get_ptr(last_buf);
        if (ptr) {
            if (last_w < 16 || last_h < 16 || last_w > 1920 || last_h > 1080 || last_hs > 4096) {
            } else {

                static int back_buf_idx = 1;
                
                int src_fd = mpp_buffer_get_fd(last_buf);
                rga_buffer_t src = wrapbuffer_fd(src_fd, last_w, last_h, RK_FORMAT_YCbCr_420_SP, last_hs, last_vs);
                
                int drm_stride_pixels = g_drm.pitch / 4; 
                rga_buffer_t dst;
                if (g_drm.dma_fd[back_buf_idx] >= 0) {
                    dst = wrapbuffer_fd(g_drm.dma_fd[back_buf_idx], g_drm.width, g_drm.height, 
                                        RK_FORMAT_BGRA_8888, drm_stride_pixels, (int)g_drm.height);
                } else {
                    dst = wrapbuffer_virtualaddr(g_drm.vaddr[back_buf_idx], g_drm.width, g_drm.height, 
                                          RK_FORMAT_BGRA_8888, drm_stride_pixels, (int)g_drm.height);
                }

                IM_STATUS status = imrotate(src, dst, IM_HAL_TRANSFORM_ROT_90);
                if (status == IM_STATUS_SUCCESS) {
                    displayed = true;
                }
                
                if (displayed) {
                    uint64_t sendTsMs = (uint64_t)last_pts;  
                    uint64_t slaveTsMs = (uint64_t)g_slaveSec * 1000ULL + g_slaveMs;
                    float latency_ms = (float)((int64_t)slaveTsMs - (int64_t)sendTsMs);
                    
                    if (g_statAvgLatency == 0) g_statAvgLatency = latency_ms;
                    else g_statAvgLatency = g_statAvgLatency * 0.9f + latency_ms * 0.1f;
                    
                    g_decodeOk++;
                    g_waitIdr = false;
                    g_last_display_pts = last_pts; 
                    
                    int ret = drmModePageFlip(g_drm.drm_fd, g_drm.crtc_id, g_drm.fb_id[back_buf_idx], 0, nullptr);

                    if (ret == 0) {
                        back_buf_idx = (back_buf_idx + 1) % 3;
                    }

                    g_statDecodeBytes += last_hs * last_vs * 3 / 2;
                }
            }
        }
        mpp_frame_deinit(&last_frame);
    }
    return displayed;
}

static void decode_thread_func() {
    set_realtime_priority(50);
    while (g_running) {
        FramePacket pkt; 
        {
            std::unique_lock<std::mutex> lock(g_queueMutex);
            g_queueCond.wait(lock, []{ return !g_frameQueue.empty() || !g_running; });
            
            if (!g_running && g_frameQueue.empty()) break;
            
            pkt = std::move(g_frameQueue.front());
            g_frameQueue.pop();
        }

        if (!pkt.data.empty()) {
            decode_and_display(pkt.data.data(), pkt.data.size(), pkt.capture_ts);
        }
    }
}

/* ==========================================================
 * 主函数
 * ========================================================== */
int main(int argc, char** argv) {
    fec_init();
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);
    signal(SIGINT, sigint_handler);
    signal(SIGPIPE, SIG_IGN);
    g_startTime = time(nullptr);
    g_lastStatTime = g_startTime;

    if (!init_drm()) { printf("Failed to init DRM\n"); return 1; }
    if (!init_mpp_decoder()) { cleanup_drm(); return 1; }

    // ===== 双初始化传输层 =====
    bool serial_ok = init_serial();
    bool udp_ok    = init_udp();

    if (!serial_ok && !udp_ok) {
        printf("ERROR: 串口和网口初始化全部失败！\n");
        cleanup_mpp(); cleanup_drm(); return 1;
    }

    // ===== 自动选择通路：检测网线物理连接 (carrier) =====
    printf("正在唤醒网卡并检测物理链路 (最多等待 5 秒)...\n");
    system("ifconfig eth0 up 2>/dev/null"); // 强制唤醒网卡，如果网卡是 down 状态，carrier 是读不到的

    bool link = false;
    for (int i = 0; i < 50; i++) { // 循环 50 次，每次 100ms，最多等 5 秒
        if (net_link_up()) {
            link = true;
            break;
        }
        usleep(100000); // 停顿 100 毫秒
    }

    const char* mode_str;
    if (argc >= 2 && strcmp(argv[1], "--serial") == 0) {
        mode_str = "串口 (强制)";
    } else if (argc >= 2 && strcmp(argv[1], "--udp") == 0) {
        mode_str = "网口 (强制)";
    } else if (udp_ok && link) {
        mode_str = "网口优先 (自动: 网线已连接)";
    } else if (serial_ok) {
        mode_str = "串口优先 (自动: 网线未连接)";
    } else {
        mode_str = "网口 (自动: 仅网口可用)";
    }
    printf("接收模式: %s, 网线状态: %s\n", mode_str, link ? "已连接" : "未连接");

    if (!init_cts_gpio()) { printf("Failed to init CTS GPIO\n"); return 1; }

    // 启动时发送 0xFF 请求一次 IDR
    send_reverse_byte(0xFF);

    std::thread timerThread(slave_timer_thread);
    std::thread ctsThread(cts_interrupt_thread);
    
    std::thread decode_thread(decode_thread_func);

    printf("Receiver running.\n");
    set_realtime_priority(55);

    uint8_t rx_tmp[4096];
    
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        g_startTimeMs = (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
        g_lastStatTimeMs = g_startTimeMs;
    }

    // 主循环：双 fd 同时监听
    while (g_running) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        int max_fd = -1;

        if (g_udpFd >= 0) {
            FD_SET(g_udpFd, &read_fds);
            max_fd = g_udpFd;
        }
        if (g_serialFd >= 0) {
            FD_SET(g_serialFd, &read_fds);
            if (g_serialFd > max_fd) max_fd = g_serialFd;
        }

        if (max_fd < 0) { usleep(1000); continue; }
        
        struct timeval tv = {0, 10000};
        int ret = select(max_fd + 1, &read_fds, nullptr, nullptr, &tv);

        // 统计
        struct timeval stat_tv;
        gettimeofday(&stat_tv, NULL);
        uint64_t now_ms = (uint64_t)stat_tv.tv_sec * 1000 + stat_tv.tv_usec / 1000;

        if (now_ms - g_lastStatTimeMs >= 1000) {
            uint64_t elapsed_ms = now_ms - g_lastStatTimeMs;
            float elapsed_sec = elapsed_ms / 1000.0f;
            float period_sec = (now_ms - g_startTimeMs) / 1000.0f;
            
            float fps = (float)g_decodeOk / elapsed_sec;
            float kbps = (float)g_totalBytes * 8.0f / 1000.0f / elapsed_sec;
            float avgRecv = g_rxFrames > 0 ? (float)g_totalBytes / g_rxFrames / 1024.0f : 0;
            float avgDec = g_decodeOk > 0 ? (float)g_statDecodeBytes / g_decodeOk / 1024.0f : 0;
            float max_kb = g_statMaxSize / 1024.0f;
            float min_kb = g_statMinSize / 1024.0f;
            float cur_kb = g_statCurFrameSize / 1024.0f;
            const char* cur_type = (g_statCurFrameType == 2) ? "IDR帧" : (g_statCurFrameType == 1) ? "P帧" : "未知";
            
            printf("STAT,%.1f,%.1f,%.0f,%.1f,%d\n", period_sec, g_statAvgLatency, kbps, fps, g_statDropFrames);
            fflush(stdout); 
            
            printf("========== 实时统计 [第 %.1f 秒] ==========\n", period_sec);
            printf("实时延迟: \"%.1f\" ms\n", g_statAvgLatency);
            printf("帧率 FPS: \"%.1f\" fps\n", fps);
            printf("码率: \"%.0f\" Kbps\n", kbps);
            printf("平均帧大小: \"%.1f\" KB\n", avgRecv);
            printf("解码后平均帧大小: \"%.1f\" KB\n", avgDec);
            printf("最大帧: \"%.1f\"KB  最小帧: \"%.1f\"KB\n", max_kb, min_kb);
            printf("I帧: %d   P帧: %d   总帧: %d\n", g_statRecvIDR, g_statRecvP, g_rxFrames);
            printf("丢帧: %d\n", g_statDropFrames);
            printf("  └─ 丢帧明细: 校验失败:%d  等待IDR丢弃:%d  解码失败:%d\n", g_statChecksumFail, g_statWaitIdrDrop, g_decodeErr);
            printf("  └─ FEC 成功抢救回 %d 个受损包！\n", g_statFecRecoveredSlices);
            printf("当前帧: %s 大小: %.1f KB\n", cur_type, cur_kb);
            if (fps == 0.0f) {
                if (g_zeroFpsStartMs == 0) g_zeroFpsStartMs = g_lastStatTimeMs;
                uint64_t dur = now_ms - g_zeroFpsStartMs;
                if (dur > g_zeroFpsMaxDurationMs) g_zeroFpsMaxDurationMs = dur;
            } else {
                g_zeroFpsStartMs = 0;
            }
            printf("零帧率最大持续时间: \"%.1f\" 秒\n", g_zeroFpsMaxDurationMs / 1000.0f);
            printf("=================================\n");
            
            g_totalBytes = 0;          
            g_rxFrames = 0;            
            g_decodeOk = 0;            
            g_statMinSize = 0;       
            g_statMaxSize = 0;       
            g_statRecvIDR = 0;       
            g_statRecvP = 0;         
            g_statDecodeBytes = 0;   
            g_statDecodeIDR = 0;     
            g_statDecodeP = 0;       
            g_statCurFrameSize = 0;  
            g_statCurFrameType = 0;  
            
            g_lastStatTimeMs = now_ms;    
        }

        if (ret < 0 && errno != EINTR) {
            perror("select error");
            break;
        } else if (ret == 0) {
            continue;
        }

        // 从激活的 fd 读取数据喂给状态机
        if (g_udpFd >= 0 && FD_ISSET(g_udpFd, &read_fds)) {
            ssize_t n = recv(g_udpFd, rx_tmp, sizeof(rx_tmp), 0);
            if (n < 0) {
                if (errno == EAGAIN || errno == EINTR) {
                } else {
                    perror("udp recv");
                }
            } else if (n > 0) {
                g_totalBytes += n;
                g_totalReads++;
                for (ssize_t i = 0; i < n; i++) {
                    process_rx_byte(rx_tmp[i]);
                }
            }
        }
        if (g_serialFd >= 0 && FD_ISSET(g_serialFd, &read_fds)) {
            ssize_t n = read(g_serialFd, rx_tmp, sizeof(rx_tmp));
            if (n < 0) {
                if (errno == EAGAIN || errno == EINTR) {
                } else {
                    perror("serial read");
                }
            } else if (n > 0) {
                g_totalBytes += n;
                g_totalReads++;
                for (ssize_t i = 0; i < n; i++) {
                    process_rx_byte(rx_tmp[i]);
                }
            }
        }
    }

    g_running = false;
    g_queueCond.notify_all();
    if (decode_thread.joinable()) {
        decode_thread.join();
    }

    timerThread.join();
    ctsThread.join();
    free(g_frameBuf);
    
    cleanup_serial();
    cleanup_udp();
    cleanup_mpp();
    cleanup_drm();
    printf("Receiver exit.\n");
    return 0;
}

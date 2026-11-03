import sys
import threading
import queue
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 数据存储列表
times = []
latencies = []
bitrates = []
fps_data = []  # 新增 FPS 列表

# 创建上下三个子图：图1为延迟，图2为码率，图3为帧率
# 调整 figsize 高度以容纳三张图
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9))
fig.canvas.manager.set_window_title('Video Stream Monitor')

# 初始化折线
line_lat, = ax1.plot([], [], 'r-', marker='o', label='Latency (ms)')
line_bit, = ax2.plot([], [], 'b-', marker='o', label='Bitrate (Kbps)')
line_fps, = ax3.plot([], [], 'g-', marker='o', label='FPS') # 绿色的帧率折线

# ================= 初始化右上角的平均值文本对象 ============================
# 使用 transform=ax.transAxes 将坐标锁定为相对比例 (0~1)，0.98, 0.90 代表右上角
# bbox 添加一个半透明的白色背景，防止文本被折线或网格遮挡
text_props = dict(boxstyle='round', facecolor='white', alpha=0.7, edgecolor='gray')
avg_text_lat = ax1.text(0.98, 0.90, '', transform=ax1.transAxes, ha='right', va='top', fontsize=10, bbox=text_props)
avg_text_bit = ax2.text(0.98, 0.90, '', transform=ax2.transAxes, ha='right', va='top', fontsize=10, bbox=text_props)
avg_text_fps = ax3.text(0.98, 0.90, '', transform=ax3.transAxes, ha='right', va='top', fontsize=10, bbox=text_props)
# ======================================================================

# 配置图表样式
ax1.set_title("Real-time Latency")
ax1.set_ylabel("ms")
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.legend(loc='upper left')

ax2.set_title("Real-time Bitrate")
ax2.set_ylabel("Kbps")
ax2.grid(True, linestyle='--', alpha=0.6)
ax2.legend(loc='upper left')

ax3.set_title("Real-time FPS")
ax3.set_ylabel("FPS")
ax3.set_xlabel("Time (s)")
ax3.grid(True, linestyle='--', alpha=0.6)
ax3.legend(loc='upper left')

MAX_POINTS = 300 # 图表上最多显示最近 30 秒的数据点 (30s * 10Hz = 300)

# ================= 线程安全的数据队列 =================
data_queue = queue.Queue()

def read_from_stdin():
    """后台线程函数：不受定时器限制，全速无阻塞读取 C++ 输出"""
    for line in sys.stdin:
        data_queue.put(line)

# 启动后台守护线程
reader_thread = threading.Thread(target=read_from_stdin, daemon=True)
reader_thread.start()
# ==========================================================

def update(frame):
    # 每次界面刷新时，一口气把队列里所有积压的数据全部处理完
    while not data_queue.empty():
        try:
            line = data_queue.get_nowait()
        except queue.Empty:
            break
            
        # 过滤并解析带有 STAT 前缀的数据行
        if line.startswith("STAT,"):
            parts = line.strip().split(',')
            try:
                t = float(parts[1])
                lat = float(parts[2])
                kbps = float(parts[3])
                fps_val = float(parts[4]) # 解析我们刚加进去的 FPS 数据
                
                times.append(t)
                latencies.append(lat)
                bitrates.append(kbps)
                fps_data.append(fps_val)
                
                # 维持滑动窗口
                if len(times) > MAX_POINTS:
                    times.pop(0)
                    latencies.pop(0)
                    bitrates.pop(0)
                    fps_data.pop(0)
            except ValueError:
                pass
        else:
            # 人类可读的统计文本，直接原样全速打印
            print(line, end='', flush=True)

    # 在处理完所有积压数据后，统一更新一次折线图表
    if times:
        line_lat.set_data(times, latencies)
        line_bit.set_data(times, bitrates)
        line_fps.set_data(times, fps_data)

        # ================= 新增：计算并更新最近 30s 的平均值文本 =================
        if latencies:
            avg_lat = sum(latencies) / len(latencies)
            avg_text_lat.set_text(f'30s Avg: {avg_lat:.1f} ms')
            
        if bitrates:
            avg_bit = sum(bitrates) / len(bitrates)
            avg_text_bit.set_text(f'30s Avg: {avg_bit:.0f} Kbps')
            
        if fps_data:
            avg_fps = sum(fps_data) / len(fps_data)
            avg_text_fps.set_text(f'30s Avg: {avg_fps:.1f} FPS')
        # ========================================================================
        
        # 动态调整坐标轴范围
        TIME_WINDOW_SEC = 30 
        
        # 三张图同步 X 轴进度
        ax1.set_xlim(max(0, times[-1] - TIME_WINDOW_SEC), times[-1] + 0.5)
        ax2.set_xlim(max(0, times[-1] - TIME_WINDOW_SEC), times[-1] + 0.5)
        ax3.set_xlim(max(0, times[-1] - TIME_WINDOW_SEC), times[-1] + 0.5)
        
        # ================= 自适应 Y 轴缩放 =================
        if latencies:
            if len(latencies) > 3:
                normal_max_lat = max(sorted(latencies)[:-2])
            else:
                normal_max_lat = max(latencies)
            ax1.set_ylim(0, normal_max_lat * 1.5 + 10)
            
        if bitrates:
            ax2.set_ylim(0, max(bitrates) * 1.2 + 100)
            
        if fps_data:
            # 帧率一般波动不大，留出 20% 余量即可，防止顶格
            ax3.set_ylim(0, max(fps_data) * 1.2 + 5)
        # ==========================================================
            
    # 注意把 line_fps 也返回，让 matplotlib 知道它被更新了
    return line_lat, line_bit, line_fps

# 依然是每 100ms 刷新一次画面
ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)

plt.tight_layout()
plt.show()
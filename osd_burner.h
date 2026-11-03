#pragma once
#include <stdint.h>
#include <stdio.h>

/**
 * 帧内时间戳“烧录”工具
 * 
 * 原理：将 0-999 的毫秒值转换为 10 位的二进制。
 *      在图像的指定位置画 10 个方块，白块代表 1，黑块代表 0。
 */

static const int BLOCK_SIZE = 30; // 每个方块的像素边长
static const int BLOCK_GAP = 10;  // 方块之间的间隔

/**
 * 在 NV12 格式的 Y 分量上画条码 (用于 Sender 端)
 * 
 * @param nv12_data  NV12 缓冲区的起始地址
 * @param width      视频宽度
 * @param height     视频高度
 * @param ts_ms      要烧录的毫秒级时间戳
 * @param is_top     true 画在顶部，false 画在底部
 */
inline void draw_osd_nv12(uint8_t* nv12_data, int width, int height, uint64_t ts_ms, bool is_top) {
    if (!nv12_data) return;

    // 取毫秒的后三位 (0-999)
    uint32_t val = ts_ms % 1000;
    
    // 确定 Y 坐标的起点
    int start_y = is_top ? 10 : (height - BLOCK_SIZE - 10);

    for (int i = 0; i < 10; i++) {
        // 从高位到低位提取二进制 (第 9 位到第 0 位)
        bool bit = (val >> (9 - i)) & 1;
        
        // NV12 的 Y 分量：255 为纯白，0 为纯黑
        uint8_t color = bit ? 255 : 0;
        
        int start_x = 10 + i * (BLOCK_SIZE + BLOCK_GAP);

        // 暴力覆写 Y 亮度平面的像素内存
        for (int y = start_y; y < start_y + BLOCK_SIZE; y++) {
            for (int x = start_x; x < start_x + BLOCK_SIZE; x++) {
                if (x < width && y < height) {
                    nv12_data[y * width + x] = color;
                }
            }
        }
    }
}

/**
 * 在 BGRA 格式上画条码 (用于 Receiver 端的 DRM 显存直写)
 * 
 * @param bgra_data  BGRA 缓冲区的起始地址 (DRM vaddr)
 * @param width      屏幕宽度
 * @param height     屏幕高度
 * @param pitch      DRM 的真实内存跨度 (Pitch / Line stride)
 * @param ts_ms      要烧录的毫秒级时间戳
 * @param is_top     true 画在顶部，false 画在底部
 */
inline void draw_osd_bgra(uint8_t* bgra_data, int width, int height, int pitch, uint64_t ts_ms, bool is_top) {
    if (!bgra_data) return;

    uint32_t val = ts_ms % 1000;
    int start_y = is_top ? 10 : (height - BLOCK_SIZE - 10);

    for (int i = 0; i < 10; i++) {
        bool bit = (val >> (9 - i)) & 1;
        
        // BGRA 像素颜色：255,255,255,255 是白；0,0,0,255 是黑
        uint8_t b = bit ? 255 : 0;
        uint8_t g = bit ? 255 : 0;
        uint8_t r = bit ? 255 : 0;
        uint8_t a = 255; 
        
        int start_x = 10 + i * (BLOCK_SIZE + BLOCK_GAP);

        for (int y = start_y; y < start_y + BLOCK_SIZE; y++) {
            // DRM 的内存对齐必须通过 pitch 来计算实际行偏移
            uint8_t* line_ptr = bgra_data + (y * pitch);
            
            for (int x = start_x; x < start_x + BLOCK_SIZE; x++) {
                if (x < width && y < height) {
                    int px_offset = x * 4; // BGRA 占用 4 个字节
                    line_ptr[px_offset + 0] = b;
                    line_ptr[px_offset + 1] = g;
                    line_ptr[px_offset + 2] = r;
                    line_ptr[px_offset + 3] = a;
                }
            }
        }
    }
}
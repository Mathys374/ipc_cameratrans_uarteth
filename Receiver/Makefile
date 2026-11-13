# 交叉编译器前缀
CROSS_COMPILE ?= /opt/atk-dlrv1126b-toolchain/bin/aarch64-linux-

# MPP 路径
MPP_INC = /home/alientek/mpp/inc
MPP_LIB = /home/alientek/rv1126_mpp/lib

# RGA 路径
RGA_INC = /home/alientek/librga-main/include
RGA_LIB = /home/alientek/rv1126_rga/lib

# 分别定义 C 和 C++ 编译器
CC       = $(CROSS_COMPILE)gcc
CXX      = $(CROSS_COMPILE)g++

# 抽取公共编译选项
COMMON_FLAGS = -O3 -Wall -march=armv8-a+simd -ftree-vectorize \
               -I/opt/atk-dlrv1126b-toolchain/aarch64-buildroot-linux-gnu/sysroot/usr/include/libdrm \
               -I$(MPP_INC) \
               -I$(RGA_INC) \
               -I..  # 确保能找到上一级目录下的 fec.h

# 分别定义 C 和 C++ 的特有选项
CFLAGS   = $(COMMON_FLAGS) -std=gnu99
CXXFLAGS = $(COMMON_FLAGS) -std=c++11 -pthread -Drestrict=__restrict__

LDFLAGS  = -L$(MPP_LIB) -Wl,-rpath-link,$(MPP_LIB) \
           -L$(RGA_LIB) -Wl,-rpath-link,$(RGA_LIB) \
           -lpthread -ldrm -lrockchip_mpp -lrga

TARGET   = ipc_receiver_uarteth

# 区分 C 源文件和 C++ 源文件
C_SRCS   = ../fec.c
CXX_SRCS = ipc_receiver.cpp

# 生成对应的目标文件 (.o)
OBJS     = $(C_SRCS:.c=.o) $(CXX_SRCS:.cpp=.o)

all: $(TARGET)

# 最终链接步骤 (使用 g++ 统一链接)
$(TARGET): $(OBJS)
	$(CXX) $^ -o $@ $(LDFLAGS)

# C 文件的编译规则
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# C++ 文件的编译规则
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)

.PHONY: all clean
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <math.h>

/**
 * @file autobright_daemon.c
 * @brief 极致优化的变频自动亮度守护进程 (生产级/零 IO/整型版)
 * 
 * 本版本集成了所有最新的调优特性：
 * 1. 动态变频逻辑：当目标亮度与当前物理亮度差距 >= 20% 时触发 10Hz 爆发采样。
 * 2. 纯整型高效算法：核心像素处理移除所有 float 运算，采用 256 字节 LUT 查表。
 * 3. 测光对齐优化：支持 Stride 处理与 Y 信号范围校正 (Limited -> Full Range)。
 * 4. 极致静默运行：彻底移除终端 IO，仅保留核心控制逻辑，专为后台服务设计。
 */

// --- 硬件与测光区域配置 (同步自用户微调值) ---
#define DEV "/dev/video0"
#define EXP 2000          // 固定测光曝光绝对值 (测光基准)
#define GAIN 100          // 固定测光增益绝对值
#define FW 40             // 关注区宽度占比 (40%)
#define FH 40             // 关注区高度占比 (40%)
#define FG 10             // 关注区距底部的间隙 (10%, 避开键盘)
#define WEIGHT 80         // 关注区的测光决策权重 (80%)

// --- 映射逻辑与变频阈值 ---
#define MIN_L 25          // 测光下限：低于此值时屏幕设为最低亮度
#define MAX_L 200         // 测光上限：高于此值时屏幕设为最高亮度
#define MIN_B 10          // 屏幕允许的最低亮度 (%)
#define MAX_B 100         // 屏幕允许的最高亮度 (%)
#define GAMMA 1.8f        // 视觉感知校正 Gamma 系数
#define SMOOTH 25         // 指数平滑系数 (25% 目标 + 75% 现状)
#define BR_THRESHOLD 20   // 爆发模式触发阈值：亮度差距 >= 20%

// --- 频率控制配置 ---
#define BASE_SLEEP  1000000 // 基础休眠时长 (1s, 即 1Hz)
#define BURST_SLEEP 100000  // 爆发休眠时长 (0.1s, 即 10Hz)
#define BURST_COUNT 25      // 爆发调节持续帧数 (约 2.5 秒)

// 环境亮度(0-255) -> 屏幕亮度目标(0-100) 的预计算查找表
unsigned char lut[256];

typedef struct {
    int fd;
    void* buf;
    size_t len;
    int w, h, stride;
} Cam;

/**
 * 封装 ioctl 系统调用，自动处理信号中断
 */
static int xioctl(int fh, int req, void *arg) {
    int r; do r = ioctl(fh, req, arg); while (-1 == r && EINTR == errno); return r;
}

/**
 * 初始化 Gamma 查找表：将环境 Luma 映射到符合人眼感官的屏幕亮度值
 * 仅在程序启动时运行一次浮点运算，消除运行时的计算开销。
 */
void init_lut() {
    for (int i=0; i<256; i++) {
        if (i <= MIN_L) lut[i] = MIN_B;
        else if (i >= MAX_L) lut[i] = MAX_B;
        else {
            float r = (float)(i - MIN_L) / (MAX_L - MIN_L);
            // 应用幂律函数进行非线性视觉校正
            lut[i] = (unsigned char)(MIN_B + (powf(r, 1.0f/GAMMA) * (MAX_B - MIN_B)));
        }
    }
}

/**
 * 轻量级获取当前系统物理亮度：解析 brightnessctl -m 的百分比输出
 */
int get_sys() {
    FILE *fp = popen("brightnessctl -m", "r");
    if (!fp) return 50;
    char b[128]; int v = 50;
    if (fgets(b, sizeof(b), fp)) {
        char *p = strchr(b, '%');
        if (p) {
            char *s = p;
            while(s > b && *(s-1) != ',') s--;
            v = atoi(s);
        }
    }
    pclose(fp); return v;
}

/**
 * 开启摄像头会话并强制锁定 V4L2 底层控制参数
 */
int open_cam(Cam *c) {
    c->fd = open(DEV, O_RDWR); if (c->fd < 0) return -1;
    struct v4l2_control k;
    // 锁定硬件曝光与增益，确保测光结果在不同环境下具有一致性
    k.id=V4L2_CID_EXPOSURE_AUTO; k.value=1; xioctl(c->fd, VIDIOC_S_CTRL, &k);
    k.id=V4L2_CID_EXPOSURE_ABSOLUTE; k.value=EXP; xioctl(c->fd, VIDIOC_S_CTRL, &k);
    k.id=V4L2_CID_GAIN; k.value=GAIN; xioctl(c->fd, VIDIOC_S_CTRL, &k);
    
    struct v4l2_format f = { .type=V4L2_BUF_TYPE_VIDEO_CAPTURE };
    xioctl(c->fd, VIDIOC_G_FMT, &f); 
    c->w=f.fmt.pix.width; 
    c->h=f.fmt.pix.height;
    c->stride=f.fmt.pix.bytesperline; // 用于 Stride 步长处理
    
    struct v4l2_requestbuffers rb = { .count=1, .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory=V4L2_MEMORY_MMAP };
    xioctl(c->fd, VIDIOC_REQBUFS, &rb);
    struct v4l2_buffer b = { .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory=V4L2_MEMORY_MMAP, .index=0 };
    xioctl(c->fd, VIDIOC_QUERYBUF, &b);
    c->len = b.length;
    c->buf = mmap(NULL, b.length, PROT_READ|PROT_WRITE, MAP_SHARED, c->fd, b.m.offset);
    
    int t=V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(c->fd, VIDIOC_STREAMON, &t);
    return 0;
}

/**
 * 核心测光算法：执行区域加权采样
 * 引入 Stride 偏移处理和 Limited-to-Full 像素映射逻辑。
 */
int grab(Cam *c) {
    struct v4l2_buffer b = { .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory=V4L2_MEMORY_MMAP, .index=0 };
    xioctl(c->fd, VIDIOC_QBUF, &b); xioctl(c->fd, VIDIOC_DQBUF, &b);
    
    int fw = c->w * FW / 100, fh = c->h * FH / 100;
    int x1 = (c->w - fw)/2, x2 = x1 + fw;
    int y2 = c->h * (100 - FG) / 100, y1 = y2 - fh;
    
    unsigned long long fs=0, bs=0; long fc=0, bc=0;
    unsigned char *p = (unsigned char*)c->buf;
    
    for (int y=0; y<c->h; y++) {
        // 基于驱动 Stride 准确定位行首
        unsigned char* row = p + (y * c->stride);
        for (int x=0; x<c->w; x++) {
            int raw_y = row[x * 2]; // YUYV：Y 在偶数位置
            // 对齐校正：(raw_y - 16) * 255 / 219 映射至全范围
            int y_f = (raw_y - 16) * 255 / 219;
            if (y_f < 0) y_f = 0; else if (y_f > 255) y_f = 255;

            if (x>=x1 && x<x2 && y>=y1 && y<y2) { fs+=y_f; fc++; } 
            else { bs+=y_f; bc++; }
        }
    }
    // 纯整型加权合成，避免像素级浮点加法
    return (int)(((fs/fc)*WEIGHT + (bs/bc)*(100-WEIGHT)) / 100);
}

/**
 * 彻底释放摄像头资源 (物理指示灯熄灭)
 */
void close_cam(Cam *c) {
    int t=V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(c->fd, VIDIOC_STREAMOFF, &t);
    munmap(c->buf, c->len); close(c->fd);
}

int main() {
    init_lut();
    Cam c; int curr_br = get_sys();
    
    // 静默运行：关闭 stdout/stderr 可进一步降低系统调用开销 (可选)
    close(STDOUT_FILENO); close(STDERR_FILENO);

    while (1) {
        if (open_cam(&c) == 0) {
            int luma = grab(&c);
            int target = lut[luma];
            int sys_now = get_sys();

            // 变频自适应：如果目标与现状差距显著，开启高频爆发调节直到接近目标
            if (abs(target - sys_now) >= BR_THRESHOLD) {
                for (int i=0; i<BURST_COUNT; i++) {
                    luma = grab(&c);
                    target = lut[luma];
                    // 指数平滑：实现具有呼吸感的亮度调节
                    curr_br = (target * SMOOTH + curr_br * (100 - SMOOTH)) / 100;
                    char cmd[48]; sprintf(cmd, "brightnessctl s %d%% -q", curr_br); system(cmd);
                    usleep(BURST_SLEEP); // 10Hz
                }
            } else {
                // 标准节能模式：每秒采样一次
                curr_br = (target * SMOOTH + sys_now * (100 - SMOOTH)) / 100;
                char cmd[48]; sprintf(cmd, "brightnessctl s %d%% -q", curr_br); system(cmd);
            }
            close_cam(&c);
        }
        usleep(BASE_SLEEP); // 基础采样间隔
    }
    return 0;
}

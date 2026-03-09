#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <math.h>

/**
 * @file autobright_fixed_point.c
 * @brief 变频自动亮度 - 纯整型算法对比测试版 (逻辑增强)
 * 
 * 技术特点：
 * 1. 移除运行时浮点运算：所有 Luma 映射和权重融合均采用 int 计算。
 * 2. 预计算 Gamma 查找表 (LUT)：启动时计算一次 powf，运行时仅查表。
 * 3. 动态变频：当目标亮度与当前物理亮度差距超过 20% 时，触发 10Hz 爆发模式。
 */

// --- 硬件与测光区域配置 (相对于全画面的比例) ---
#define CAMERA_DEVICE    "/dev/video0"
#define EXPOSURE_VAL     2000          // 测光基准曝光绝对值
#define GAIN_VAL         100           // 测光基准增益绝对值
#define FOCUS_W          0.4f          // 关注区宽度占比
#define FOCUS_H          0.4f          // 关注区高度占比
#define FOCUS_BOTTOM_GAP 0.1f          // 关注区底部间隙 (避开键盘反光)
#define FOCUS_WEIGHT_INT 80            // 关注区测光决策权重 (80%)

// --- 算法映射配置 ---
#define MIN_LUMA         25            // 传感器读数下限 (25 -> 最低屏幕亮度)
#define MAX_LUMA         200           // 传感器读数上限 (200 -> 最高屏幕亮度)
#define MIN_BRIGHT       10            // 屏幕最低亮度百分比 (%)
#define MAX_BRIGHT       100           // 屏幕最高亮度百分比 (%)
#define GAMMA            1.8f          // 视觉校正 Gamma 值
#define SMOOTH_INT       25            // 指数平滑系数 (25% 目标 + 75% 现状)

// --- 变频控制配置 ---
#define BASE_SLEEP       1000000       // 基础模式休眠 (1s, 即 1Hz)
#define BURST_SLEEP      100000        // 爆发模式休眠 (0.1s, 即 10Hz)
#define BURST_FRAMES     25            // 爆发模式持续帧数 (约 2.5 秒)
#define BRIGHT_THRESHOLD 20            // 触发爆发模式的亮度差值阈值 (%)

// 全局预计算查找表：环境加权 Luma (0-255) -> 目标屏幕亮度 (0-100)
unsigned char lut[256];

typedef struct {
    int fd;
    void* buffer;
    size_t buf_len;
    int width, height, stride;
} Session;

/**
 * 封装 ioctl 系统调用以处理信号中断
 */
static int xioctl(int fh, int request, void *arg) {
    int r; do r = ioctl(fh, request, arg); while (-1 == r && EINTR == errno); return r;
}

/**
 * 预计算视觉校正查找表 (LUT)
 * 仅在程序启动时运行一次，利用浮点幂运算生成匹配人眼感官的非线性映射曲线。
 */
void init_lut() {
    for (int i = 0; i < 256; i++) {
        if (i <= MIN_LUMA) lut[i] = MIN_BRIGHT;
        else if (i >= MAX_LUMA) lut[i] = MAX_BRIGHT;
        else {
            float ratio = (float)(i - MIN_LUMA) / (MAX_LUMA - MIN_LUMA);
            // 使用 Gamma 校正使亮度增长在低光下更缓慢，更符合心理物理学感受
            lut[i] = (unsigned char)(MIN_BRIGHT + powf(ratio, 1.0f / GAMMA) * (MAX_BRIGHT - MIN_BRIGHT));
        }
    }
}

/**
 * 通过 brightnessctl 获取系统当前的物理背光百分比
 */
int get_sys_brightness() {
    FILE *fp = popen("brightnessctl -m", "r");
    if (!fp) return 50;
    char res[128]; int val = 50;
    if (fgets(res, sizeof(res), fp)) {
        // 解析格式: device,class,curr,percent,max
        char *p = strchr(res, '%');
        if (p) {
            char *s = p;
            while (s > res && *(s-1) != ',') s--;
            sscanf(s, "%d%%", &val);
        }
    }
    pclose(fp); return val;
}

/**
 * 摄像头初始化：强制手动模式并申请 MMAP 缓冲区
 */
int camera_init(Session *s) {
    s->fd = open(CAMERA_DEVICE, O_RDWR); if (s->fd < 0) return -1;
    struct v4l2_control ctrl;
    // 锁定测光基准，确保不同光照下的像素值具有可比性
    ctrl.id = V4L2_CID_EXPOSURE_AUTO; ctrl.value = 1; xioctl(s->fd, VIDIOC_S_CTRL, &ctrl);
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE; ctrl.value = EXPOSURE_VAL; xioctl(s->fd, VIDIOC_S_CTRL, &ctrl);
    ctrl.id = V4L2_CID_GAIN; ctrl.value = GAIN_VAL; xioctl(s->fd, VIDIOC_S_CTRL, &ctrl);

    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    xioctl(s->fd, VIDIOC_G_FMT, &fmt);
    s->width = fmt.fmt.pix.width; 
    s->height = fmt.fmt.pix.height;
    s->stride = fmt.fmt.pix.bytesperline; // 记录 Stride，用于准确像素定位

    struct v4l2_requestbuffers req = { .count = 1, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
    xioctl(s->fd, VIDIOC_REQBUFS, &req);
    struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = 0 };
    xioctl(s->fd, VIDIOC_QUERYBUF, &buf);
    s->buf_len = buf.length;
    s->buffer = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, s->fd, buf.m.offset);

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(s->fd, VIDIOC_STREAMON, &type);
    return 0;
}

/**
 * 核心测光算法：纯整型实现 Limited->Full 校正与区域加权
 * 性能优化：通过 Stride 处理解决驱动对齐导致的像素偏移。
 */
int grab_luma_fixed(Session *s) {
    struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = 0 };
    xioctl(s->fd, VIDIOC_QBUF, &buf); xioctl(s->fd, VIDIOC_DQBUF, &buf);

    int fw = (int)(s->width * FOCUS_W), fh = (int)(s->height * FOCUS_H);
    int x1 = (s->width - fw) / 2, x2 = x1 + fw;
    int y2 = s->height - (int)(s->height * FOCUS_BOTTOM_GAP), y1 = y2 - fh;

    unsigned long long f_sum = 0, b_sum = 0;
    long f_cnt = 0, b_cnt = 0;
    unsigned char* ptr = (unsigned char*)s->buffer;

    for (int y = 0; y < s->height; y++) {
        unsigned char* row = ptr + (y * s->stride); // 基于步长定位行
        for (int x = 0; x < s->width; x++) {
            int raw_y = row[x * 2]; // YUYV 格式：亮度在每像素对的首位
            // 范围校正：Limited Range (16-235) 映射到 Full Range (0-255)
            int y_full = (raw_y - 16) * 255 / 219;
            if (y_full < 0) y_full = 0; else if (y_full > 255) y_full = 255;

            if (x >= x1 && x < x2 && y >= y1 && y < y2) { f_sum += y_full; f_cnt++; }
            else { b_sum += y_full; b_cnt++; }
        }
    }
    // 纯整型权重融合逻辑
    return (int)(( (f_sum / f_cnt) * FOCUS_WEIGHT_INT + (b_sum / b_cnt) * (100 - FOCUS_WEIGHT_INT) ) / 100);
}

void camera_release(Session *s) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(s->fd, VIDIOC_STREAMOFF, &type);
    munmap(s->buffer, s->buf_len); close(s->fd);
}

int main() {
    init_lut();
    Session s; 
    int current_br = get_sys_brightness(); // 记录当前亮度的软件内部状态
    printf("[对比测试] 逻辑增强版启动 | 爆发模式触发阈值: 亮度差 >= %d%%\n", BRIGHT_THRESHOLD);

    while (1) {
        if (camera_init(&s) == 0) {
            int luma = grab_luma_fixed(&s);
            int target = lut[luma];
            int sys_now = get_sys_brightness();

            // 触发逻辑：如果目标亮度与现状差距过大 (如环境剧变导致差距 > 20%)，启动爆发调节
            if (abs(target - sys_now) >= BRIGHT_THRESHOLD) {
                printf("\n[爆发调节] 差距过大 (%d%% -> %d%%)，开启 10Hz 高频响应...\n", sys_now, target);
                for (int i = 0; i < BURST_FRAMES; i++) {
                    luma = grab_luma_fixed(&s);
                    target = lut[luma];
                    // 指数平滑滤波：实现呼吸般的平滑过渡
                    current_br = (target * SMOOTH_INT + current_br * (100 - SMOOTH_INT)) / 100;
                    char cmd[64]; snprintf(cmd, sizeof(cmd), "brightnessctl s %d%% >/dev/null 2>&1", current_br); system(cmd);
                    printf("\r[高频] 测光: %3d | 屏幕目标: %3d%% | 调节进度: %d%%    ", luma, target, current_br); fflush(stdout);
                    usleep(BURST_SLEEP);
                }
                printf("\n[恢复基础模式]\n");
            } else {
                // 基础 1Hz 采样调整逻辑
                current_br = (target * SMOOTH_INT + sys_now * (100 - SMOOTH_INT)) / 100;
                char cmd[64]; snprintf(cmd, sizeof(cmd), "brightnessctl s %d%% >/dev/null 2>&1", current_br); system(cmd);
                printf("\r[基础] 测光: %3d | 屏幕目标: %3d%% | 调节进度: %d%%    ", luma, target, current_br); fflush(stdout);
            }
            camera_release(&s);
        }
        usleep(BASE_SLEEP);
    }
    return 0;
}

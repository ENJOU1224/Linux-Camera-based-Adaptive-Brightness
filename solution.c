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
 * @file solution.c
 * @brief 变频自适应区域加权自动亮度控制系统 - C 语言高性能版
 * 
 * 本程序旨在为 Linux 笔记本提供接近原生的自动亮度调节体验。
 * 
 * 核心技术路线：
 * 1. 变频采样 (Adaptive Sampling)：
 *    - 基础模式 (1Hz)：环境稳定时每秒采样一次，指示灯瞬间闪烁，极低功耗。
 *    - 爆发模式 (10Hz)：光照突变时触发，保持摄像头长连接 2 秒，实现极速平滑过渡。
 * 2. 区域加权测光 (Weighted Metering)：
 *    - 针对人脸区域（画面中心及下部）进行重点采样，权重设为 80%。
 * 3. 视觉 Gamma 校正：
 *    - 应用 Gamma = 1.8 的幂律曲线，使亮度调整符合 Weber-Fechner 人眼感官定律。
 * 4. 跨运行周期平滑：
 *    - 实时读取系统亮度作为基准，应用指数平滑滤波，消除调节过程中的跳变感。
 */

// --- 硬件底层参数 (根据调试反馈锁定) ---
#define CAMERA_DEVICE    "/dev/video0"
#define EXPOSURE_VAL     2000          // 测光基准曝光时间
#define GAIN_VAL         100           // 测光基准硬件增益

// --- 测光区域配置 (相对于全画面的比例 0.0 - 1.0) ---
#define FOCUS_W          0.4f          // 关注区宽度占比
#define FOCUS_H          0.4f          // 关注区高度占比
#define FOCUS_BOTTOM_GAP 0.1f          // 关注区距画面底部的间隙 (避开键盘反光)
#define FOCUS_WEIGHT     0.8f          // 关注区的测光决策权重 (80%)

// --- 映射与算法配置 ---
#define MIN_LUMA         25.0f         // 环境亮度下限 (对应最低屏幕亮度)
#define MAX_LUMA         200.0f        // 环境亮度上限 (对应最高屏幕亮度)
#define MIN_BRIGHT       10.0f         // 屏幕最低允许亮度 (%)
#define MAX_BRIGHT       100.0f        // 屏幕最高允许亮度 (%)
#define GAMMA            1.8f          // 视觉感知校正 Gamma 值
#define SMOOTH_FACTOR    0.25f         // 指数平滑系数 (越小过渡越柔和)

// --- 变频自适应配置 ---
#define BASE_SLEEP       1000000       // 基础休眠时长 (1s, 对应 1Hz)
#define BURST_SLEEP      100000        // 爆发休眠时长 (0.1s, 对应 10Hz)
#define BURST_FRAMES     20            // 突变后进入爆发模式的持续帧数
#define CHANGE_THRESHOLD 25.0f         // 触发爆发模式的测光变化阈值


/**
 * 摄像头会话状态结构体，用于在长连接采样中维持硬件状态
 */
typedef struct {
    int fd;
    void* buffer;
    size_t buf_len;
    int width, height;
    int stride; // 每行数据的字节数 (bytesperline)
} CameraSession;

/**
 * 封装 ioctl 系统调用以处理信号中断
 */
static int xioctl(int fh, int request, void *arg) {
    int r;
    do r = ioctl(fh, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

/**
 * 通过 brightnessctl 获取当前系统的物理屏幕亮度百分比
 * 用于作为平滑滤波的起始状态基准
 */
float get_current_system_brightness() {
    FILE *fp = popen("brightnessctl -m", "r");
    if (!fp) return 50.0f;
    char res[128];
    float val = 50.0f;
    if (fgets(res, sizeof(res), fp)) {
        char *p = strchr(res, '%');
        if (p) {
            char *start = p;
            while (start > res && *(start-1) != ',') start--;
            sscanf(start, "%f%%", &val);
        }
    }
    pclose(fp);
    return val;
}

/**
 * 初始化摄像头会话并锁定测光基准
 */
int camera_session_init(CameraSession *s) {
    s->fd = open(CAMERA_DEVICE, O_RDWR);
    if (s->fd < 0) return -1;

    // 锁定曝光和增益，确保环境光变化能线性反映在像素亮度上
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_AUTO; ctrl.value = 1; xioctl(s->fd, VIDIOC_S_CTRL, &ctrl);
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE; ctrl.value = EXPOSURE_VAL; xioctl(s->fd, VIDIOC_S_CTRL, &ctrl);
    ctrl.id = V4L2_CID_GAIN; ctrl.value = GAIN_VAL; xioctl(s->fd, VIDIOC_S_CTRL, &ctrl);

    // 获取驱动默认采集分辨率
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    xioctl(s->fd, VIDIOC_G_FMT, &fmt);
    s->width = fmt.fmt.pix.width;
    s->height = fmt.fmt.pix.height;
    s->stride = fmt.fmt.pix.bytesperline;

    // 配置单缓冲区 MMAP 流
    struct v4l2_requestbuffers req = { .count = 1, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
    xioctl(s->fd, VIDIOC_REQBUFS, &req);
    struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = 0 };
    xioctl(s->fd, VIDIOC_QUERYBUF, &buf);
    s->buf_len = buf.length;
    s->buffer = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, s->fd, buf.m.offset);

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(s->fd, VIDIOC_STREAMON, &type);
    return 0;
}

/**
 * 在已有会话中抓取一帧并执行区域加权测光
 */
float camera_session_grab_luma(CameraSession *s) {
    struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = 0 };
    xioctl(s->fd, VIDIOC_QBUF, &buf);
    xioctl(s->fd, VIDIOC_DQBUF, &buf);

    // 计算关注区域边界
    int fw = (int)(s->width * FOCUS_W);
    int fh = (int)(s->height * FOCUS_H);
    int bgap = (int)(s->height * FOCUS_BOTTOM_GAP);
    int x1 = (s->width - fw) / 2, x2 = x1 + fw;
    int y2 = s->height - bgap, y1 = y2 - fh;

    unsigned long long f_sum = 0, bg_sum = 0;
    long f_cnt = 0, b_cnt = 0;
    unsigned char* ptr = (unsigned char*)s->buffer;

    // 遍历像素进行区域加权求和
    for (int y = 0; y < s->height; y++) {
        // 使用 stride (bytesperline) 准确定位行首，解决某些驱动下的对齐偏移问题
        unsigned char* row = ptr + (y * s->stride);
        for (int x = 0; x < s->width; x++) {
            // YUYV 格式：亮度 Y 位于每像素 2 字节的首位
            int raw_y = row[x * 2];
            
            // 关键修正：将 Limited Range (16-235) 映射到 Full Range (0-255)
            // 这一步是让 C 读数对齐 Python (FFmpeg) 读数的关键
            float y_val = (raw_y - 16.0f) * (255.0f / 219.0f);
            if (y_val < 0) y_val = 0; 
            if (y_val > 255) y_val = 255;

            if (x >= x1 && x < x2 && y >= y1 && y < y2) { f_sum += y_val; f_cnt++; }
            else { bg_sum += y_val; b_cnt++; }
        }
    }
    // 融合关注区与背景区的平均亮度
    return (f_cnt > 0 ? (float)f_sum/f_cnt : 0) * FOCUS_WEIGHT + 
           (b_cnt > 0 ? (float)bg_sum/b_cnt : 0) * (1.0f - FOCUS_WEIGHT);
}

/**
 * 释放会话并关闭摄像头（指示灯熄灭）
 */
void camera_session_release(CameraSession *s) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(s->fd, VIDIOC_STREAMOFF, &type);
    munmap(s->buffer, s->buf_len);
    close(s->fd);
}

int main() {
    CameraSession s;
    float last_luma = -1.0f;
    float current_target_bright = get_current_system_brightness();

    printf("[服务] 变频自适应自动亮度控制已启动 (1Hz/10Hz模式)\n");
    printf("[配置] 关注权重: %.0f%% | Gamma: %.1f | 阈值: %.1f-%.1f\n", 
           FOCUS_WEIGHT * 100, GAMMA, MIN_LUMA, MAX_LUMA);

    while (1) {
        // --- 1Hz 基础测光逻辑 (离散采样，指示灯闪烁) ---
        if (camera_session_init(&s) == 0) {
            float luma = camera_session_grab_luma(&s);
            
            // 检测光照突变：如果变化量超过阈值，触发爆发模式
            if (last_luma >= 0 && fabsf(luma - last_luma) > CHANGE_THRESHOLD) {
                printf("\n[触发] 光照突变 (%.1f -> %.1f)，开启 10Hz 高频过渡...\n", last_luma, luma);
                
                // 爆发模式：保持摄像头长连接，极速采样调整
                for (int i = 0; i < BURST_FRAMES; i++) {
                    luma = camera_session_grab_luma(&s);
                    
                    // 1. 归一化环境亮度
                    float ratio = 0.0f;
                    if (luma >= MAX_LUMA) ratio = 1.0f;
                    else if (luma > MIN_LUMA) ratio = (luma - MIN_LUMA) / (MAX_LUMA - MIN_LUMA);
                    
                    // 2. 视觉非线性 Gamma 校正
                    float corrected_ratio = powf(ratio, 1.0f / GAMMA);
                    float target = MIN_BRIGHT + corrected_ratio * (MAX_BRIGHT - MIN_BRIGHT);
                    
                    // 3. 指数平滑滤波，模拟瞳孔适应过程
                    current_target_bright = (target * SMOOTH_FACTOR) + (current_target_bright * (1.0f - SMOOTH_FACTOR));
                    
                    char cmd[64];
                    snprintf(cmd, sizeof(cmd), "brightnessctl s %d%% >/dev/null 2>&1", (int)current_target_bright);
                    system(cmd);
                    
                    printf("\r[爆发] 测光指数: %5.1f | 实时目标: %3d%%    ", luma, (int)current_target_bright);
                    fflush(stdout);
                    usleep(BURST_SLEEP);
                }
                printf("\n[恢复] 环境已平稳，切换至 1Hz 节能采样模式。\n");
            } else {
                // 标准 1Hz 节能调节逻辑
                float ratio = 0.0f;
                if (luma >= MAX_LUMA) ratio = 1.0f;
                else if (luma > MIN_LUMA) ratio = (luma - MIN_LUMA) / (MAX_LUMA - MIN_LUMA);
                float corrected_ratio = powf(ratio, 1.0f / GAMMA);
                float target = MIN_BRIGHT + corrected_ratio * (MAX_BRIGHT - MIN_BRIGHT);
                
                // 读取系统亮度作为基准进行微调平滑
                float sys_now = get_current_system_brightness();
                current_target_bright = (target * SMOOTH_FACTOR) + (sys_now * (1.0f - SMOOTH_FACTOR));
                
                char cmd[64];
                snprintf(cmd, sizeof(cmd), "brightnessctl s %d%% >/dev/null 2>&1", (int)current_target_bright);
                system(cmd);
                printf("\r[基础] 测光指数: %5.1f | 屏幕目标: %3d%%    ", luma, (int)current_target_bright);
                fflush(stdout);
            }
            
            last_luma = luma;
            camera_session_release(&s); // 采样结束，立即释放设备并灭灯
        }
        
        usleep(BASE_SLEEP); // 等待下一次基础采样
    }

    return 0;
}

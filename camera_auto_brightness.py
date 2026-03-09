import subprocess
import time
import sys
import os
import argparse

# --- 全局路径配置 ---
FIFO_PATH = "/tmp/camera_fifo"  # 命名管道，用于 FFmpeg 视频流分流

def run_v4l2_ctrl(device, args):
    """
    调用 v4l2-ctl 命令行工具直接控制摄像头的底层硬件寄存器。
    """
    try:
        subprocess.run(["v4l2-ctl", "-d", device] + args, capture_output=True)
    except Exception:
        pass

def get_system_brightness():
    """
    通过 brightnessctl 获取当前系统的物理屏幕亮度百分比。
    同步自 solution.c：确保平滑基准一致。
    """
    try:
        res = subprocess.check_output(["brightnessctl", "-m"]).decode()
        # 格式: device,class,curr,percent,max
        parts = res.split(',')
        if len(parts) >= 4:
            return float(parts[3].replace('%', ''))
    except:
        pass
    return 50.0

def main():
    # --- 命令行参数定义 (同步自 solution.c 的核心逻辑) ---
    parser = argparse.ArgumentParser(description="对齐 C 语言版的区域加权自动亮度控制 (Python 调试版)")
    
    # 基础设备配置
    parser.add_argument("--device", default="/dev/video0", help="摄像头设备路径")
    parser.add_argument("--view", "-v", action="store_true", help="开启带标记的预览窗口")
    parser.add_argument("--fps", type=int, default=2, help="采样频率")
    
    # 核心算法参数
    parser.add_argument("--gamma", type=float, default=1.8, help="Gamma 校正系数")
    parser.add_argument("--smooth", type=float, default=0.25, help="平滑系数 (对齐 C 版)")
    parser.add_argument("--min-luma", type=float, default=25.0, help="测光下限")
    parser.add_argument("--max-luma", type=float, default=200.0, help="测光上限")
    parser.add_argument("--min-bright", type=int, default=10, help="最低屏幕亮度 %")
    parser.add_argument("--max-bright", type=int, default=100, help="最高屏幕亮度 %")
    
    # 区域权重配置
    parser.add_argument("--focus-weight", type=float, default=0.8, help="人脸区权重")
    parser.add_argument("--focus-w", type=float, default=0.4, help="关注区宽度比例")
    parser.add_argument("--focus-h", type=float, default=0.4, help="关注区高度比例")
    parser.add_argument("--focus-bottom-gap", type=float, default=0.1, help="距底部间隙比例")
    
    # 硬件感光参数
    parser.add_argument("--exposure", type=int, default=2000, help="固定曝光值")
    parser.add_argument("--gain", type=int, default=100, help="固定增益值")
    
    args = parser.parse_args()

    # 1. 硬件层初始化
    run_v4l2_ctrl(args.device, ["-c", "auto_exposure=1", "-c", f"exposure_time_absolute={args.exposure}", "-c", f"gain={args.gain}"])

    # 2. 预览窗口 UI 坐标计算 (640x480)
    W, H = 640, 480
    fw, fh = int(W * args.focus_w), int(H * args.focus_h)
    bgap = int(H * args.focus_bottom_gap)
    fx, fy = (W - fw) // 2, H - bgap - fh

    # 3. 构造 FFmpeg 处理流水线
    ffmpeg_cmd = [
        "ffmpeg", "-hide_banner", "-loglevel", "quiet",
        "-probesize", "32", "-analyzeduration", "0",
        "-f", "video4linux2", "-r", str(args.fps), "-i", args.device,
        "-filter_complex", 
        f"split=2[v1][v2]; "
        f"[v1]scale={W}:{H},drawbox=x={fx}:y={fy}:w={fw}:h={fh}:color=green@0.5:t=2[gui]; "
        f"[v2]scale=32:32,format=gray[data]",
        "-map", "[gui]", "-f", "nut", "-y", FIFO_PATH if args.view else "/dev/null",
        "-map", "[data]", "-f", "rawvideo", "-"
    ]

    ffplay_proc = None
    if args.view:
        if os.path.exists(FIFO_PATH): os.remove(FIFO_PATH)
        os.mkfifo(FIFO_PATH)
        ffplay_cmd = ["ffplay", "-hide_banner", "-loglevel", "quiet", "-fflags", "nobuffer", "-flags", "low_delay", "-framedrop", FIFO_PATH]
        ffplay_proc = subprocess.Popen(ffplay_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print(f"[*] 同步模式启动 | Gamma: {args.gamma} | 阈值: {args.min_luma}-{args.max_luma}")
    
    try:
        proc = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        RES = 32
        
        # 预计算 32x32 网格边界
        dfw, dfh = int(RES * args.focus_w), int(RES * args.focus_h)
        dbgap = int(RES * args.focus_bottom_gap)
        dx1, dx2 = (RES - dfw) // 2, (RES + dfw) // 2
        dy2 = RES - dbgap
        dy1 = dy2 - dfh

        while True:
            raw_data = proc.stdout.read(RES * RES)
            if not raw_data or len(raw_data) < RES * RES:
                break
            
            f_sum, f_cnt, b_sum, b_cnt = 0, 0, 0, 0
            for y in range(RES):
                for x in range(RES):
                    val = raw_data[y * RES + x]
                    if dx1 <= x < dx2 and dy1 <= y < dy2:
                        f_sum += val; f_cnt += 1
                    else:
                        b_sum += val; b_cnt += 1
            
            avg_f = f_sum / f_cnt if f_cnt > 0 else 0
            avg_b = b_sum / b_cnt if b_cnt > 0 else 0
            weighted_luma = (avg_f * args.focus_weight) + (avg_b * (1 - args.focus_weight))
            
            # --- 视觉核心算法 (严格对齐 solution.c) ---
            if weighted_luma <= args.min_luma:
                ratio = 0.0
            elif weighted_luma >= args.max_luma:
                ratio = 1.0
            else:
                ratio = (weighted_luma - args.min_luma) / (args.max_luma - args.min_luma)
            
            # 1. 应用 Gamma 校正
            corrected_ratio = ratio ** (1.0 / args.gamma)
            
            # 2. 映射到亮度百分比
            target_bright = args.min_bright + corrected_ratio * (args.max_bright - args.min_bright)
            
            # 3. 以物理亮度为基准的指数平滑
            sys_now = get_system_brightness()
            final_bright = (target_bright * args.smooth) + (sys_now * (1.0 - args.smooth))
            
            sys.stdout.write(f"\r测光: {weighted_luma:5.1f} | 目标: {target_bright:3.0f}% | 物理基准: {sys_now:3.0f}% -> 调整: {final_bright:3.0f}%    ")
            sys.stdout.flush()
            
            subprocess.run(["brightnessctl", "s", f"{int(final_bright)}%"], capture_output=True)

    except KeyboardInterrupt:
        print("\n退出...")
    finally:
        if 'proc' in locals(): proc.terminate()
        if ffplay_proc: ffplay_proc.terminate()
        if os.path.exists(FIFO_PATH): os.remove(FIFO_PATH)
        run_v4l2_ctrl(args.device, ["-c", "auto_exposure=3"])

if __name__ == "__main__":
    main()

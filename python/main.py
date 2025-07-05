# SPDX-FileCopyrightText: 2025 LICO-META
#
# SPDX-License-Identifier: GPL-3.0-only

# pip install pyserial matplotlib numpy
import os
import serial
import json
import time
import datetime as dt
import csv
import math
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.dates as mdates
from matplotlib.widgets import Button
from matplotlib import rcParams
from matplotlib.font_manager import FontProperties


# --- 中文显示配置 ---
font_path = "/System/Library/Fonts/PingFang.ttc"  # 使用你系统默认的中文字体路径
custom_font = FontProperties(fname=font_path)
rcParams['font.family'] = custom_font.get_name()
rcParams['axes.unicode_minus'] = False

# --- 配置 ---
SERIAL_PORT = 'COM1'  # 根据实际情况修改串口号
BAUD_RATE = 115200 # 波特率, 根据实际情况修改
PROFILE_FILE = 'profile.json' # 温度曲线配置文件
MAX_DATA_POINTS = 1500 # 最大数据点数，防止内存溢出

# --- PID 和控制配置 ---
# 你需要根据烤箱的实际情况调整这些值 (自整定)
# 在运行自整定后，用推荐值更新这里
PID_KP = 2
PID_KI = 0.02
PID_KD = 33
PWM_WINDOW_SECONDS = 1 # PWM控制周期（秒）

# --- PID 自整定配置 ---
AUTOTUNE_SETPOINT = 120.0 # 自整定目标温度 (°C)，应高于室温且在安全范围内
AUTOTUNE_HYSTERESIS = 2.0 # 迟滞范围(°C)，防止继电器在设定点附近快速切换
AUTOTUNE_CYCLES = 200       # 需要稳定的振荡周期数来计算

# --- 全局状态变量 ---
ser = None
control_mode = 'MANUAL' # 'AUTO', 'MANUAL', 或 'AUTOTUNE'
process_running = False
process_start_time = 0
relay_state = False
temperatures = deque(maxlen=MAX_DATA_POINTS)
times = deque(maxlen=MAX_DATA_POINTS)
target_temps_history = deque(maxlen=MAX_DATA_POINTS)

# --- 自整定状态变量 ---
autotune_running = False
autotune_state_is_rising = True
autotune_peaks = []
autotune_peak_times = []
autotune_troughs = []


# --- PID 控制器类 ---
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time == 0:
            return 0

        error = self.setpoint - current_value
        self.integral += error * delta_time
        derivative = (error - self.last_error) / delta_time
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        self.last_error = error
        self.last_time = current_time
        
        # 将输出限制在 0-100 范围内，代表PWM占空比
        return max(0, min(100, output))

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        # 重置积分项以避免积分饱和
        self.integral = 0


# 获取脚本所在目录并拼接 profile.json 的绝对路径
base_dir = os.path.dirname(os.path.abspath(__file__))
profile_file_path = os.path.join(base_dir, PROFILE_FILE)
# --- 加载温度曲线 (指定 UTF-8 编码以避免 GBK 解码错误) ---
with open(profile_file_path, 'r', encoding='utf-8-sig') as f:
    profile_data = json.load(f)
profile_points = profile_data['points']
profile_times = np.array([p['time'] for p in profile_points])
profile_temps = np.array([p['temp'] for p in profile_points])

# 初始化PID控制器，初始设定点为室温
pid = PID(PID_KP, PID_KI, PID_KD, profile_temps[0])

# --- 串口通信函数 ---
def connect_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"成功连接到串口 {SERIAL_PORT}")
        return True
    except serial.SerialException:
        print(f"错误: 无法打开串口 {SERIAL_PORT}。")
        return False

def send_relay_command(state):
    global relay_state
    if ser and ser.is_open:
        command = '1' if state else '0'
        ser.write(command.encode('utf-8'))
        relay_state = state

# --- 绘图设置 ---
fig, ax = plt.subplots(figsize=(12, 7))
plt.subplots_adjust(bottom=0.2, top=0.9, left=0.1, right=0.9)
line_real, = ax.plot([], [], 'b-', marker='o', markersize=2, label='实际温度')
line_target, = ax.plot([], [], 'r--', label='目标温度')
ax.set_title('回流焊温度监控')
ax.set_ylabel('温度 (°C)')
ax.set_xlabel('时间')
ax.legend()
ax.grid(True)
ax.set_ylim(0, 300)
ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
info_text = fig.suptitle('', fontsize=12)

# --- 动画更新函数 ---
last_pwm_update_time = 0
on_duration = 0  # 初始化 on_duration

def animate(i):
    global process_running, process_start_time, last_pwm_update_time, on_duration
    global autotune_running, autotune_state_is_rising, autotune_peaks, autotune_peak_times, autotune_troughs

    try:
        line_data = ser.readline().decode('utf-8').strip()
        if not line_data:
            return
        
        current_temp = float(line_data)
        now = dt.datetime.now()
        
        temperatures.append(current_temp)
        times.append(now)
        
        target_temp = pid.setpoint

        # --- 控制逻辑 ---
        if control_mode == 'AUTOTUNE' and autotune_running:
            # 1. 继电器控制 (Bang-Bang Control)
            if current_temp < AUTOTUNE_SETPOINT:
                send_relay_command(True)
            elif current_temp > AUTOTUNE_SETPOINT:
                send_relay_command(False)

            # 2. 波峰/波谷检测 (ESP32已做平滑，可简化检测)
            if len(temperatures) > 2:
                # 如果之前在上升且当前温度开始下降，则检测到波峰
                if autotune_state_is_rising and temperatures[-1] < temperatures[-2]:
                    # 增加迟滞判断，防止噪声干扰
                    if abs(temperatures[-2] - AUTOTUNE_SETPOINT) > AUTOTUNE_HYSTERESIS:
                        peak_temp = temperatures[-2]
                        autotune_peaks.append(peak_temp)
                        autotune_peak_times.append(times[-2])
                        autotune_state_is_rising = False
                        print(f"自整定: 检测到波峰 {peak_temp:.2f}°C")
                # 如果之前在下降且当前温度开始上升，则检测到波谷
                elif not autotune_state_is_rising and temperatures[-1] > temperatures[-2]:
                     if abs(temperatures[-2] - AUTOTUNE_SETPOINT) > AUTOTUNE_HYSTERESIS:
                        trough_temp = temperatures[-2]
                        autotune_troughs.append(trough_temp)
                        autotune_state_is_rising = True
                        print(f"自整定: 检测到波谷 {trough_temp:.2f}°C")

            # 3. 计算与终止
            if len(autotune_peaks) >= AUTOTUNE_CYCLES:
                stop_autotune(calculate=True) # 停止并执行计算

        elif control_mode == 'AUTO' and process_running:
            elapsed_time = (now - process_start_time).total_seconds()
            target_temp = np.interp(elapsed_time, profile_times, profile_temps)
            pid.set_setpoint(target_temp)
            target_temps_history.append(target_temp)
            
            current_time = time.time()
            if current_time - last_pwm_update_time >= PWM_WINDOW_SECONDS:
                last_pwm_update_time = current_time
                pid_output = pid.update(current_temp)
                on_duration = (pid_output / 100.0) * PWM_WINDOW_SECONDS
                send_relay_command(True)
            
            if time.time() - last_pwm_update_time > on_duration:
                 send_relay_command(False)
        else: # MANUAL or idle
             target_temps_history.append(None)

        # 更新绘图数据
        line_real.set_data(times, temperatures)
        valid_targets = [(t, temp) for t, temp in zip(times, target_temps_history) if temp is not None]
        if valid_targets:
            t_valid, temp_valid = zip(*valid_targets)
            line_target.set_data(t_valid, temp_valid)
        else:
            line_target.set_data([], [])

        if times:
            ax.set_xlim(times[0], now + dt.timedelta(seconds=15))

        # 更新信息文本
        autotune_status = ""
        if autotune_running:
            autotune_status = f"| 自整定中 (目标: {AUTOTUNE_SETPOINT}°C, 周期: {len(autotune_peaks)}/{AUTOTUNE_CYCLES})"
        
        info_string = (
            f"模式: {control_mode} | 状态: {'运行中' if process_running else '已停止'} | 继电器: {'开' if relay_state else '关'}{autotune_status}\n"
            f"当前温度: {current_temp:.2f}°C | 目标温度: {target_temp:.2f}°C"
        )
        info_text.set_text(info_string)

    except (ValueError, UnicodeDecodeError):
        pass
    except Exception as e:
        print(f"发生错误: {e}")

# --- 按钮回调函数 ---
def start_autotune(event):
    global control_mode, autotune_running, process_running
    global autotune_peaks, autotune_peak_times, autotune_troughs, autotune_state_is_rising
    if autotune_running or process_running:
        print("错误: 请先停止当前流程。")
        return
    
    print("\n--- 开始 PID 自整定 ---")
    print(f"目标温度: {AUTOTUNE_SETPOINT}°C")
    print("烤箱将围绕目标温度振荡以确定系统特性。")
    print("请等待流程自动完成...")

    control_mode = 'AUTOTUNE'
    autotune_running = True
    process_running = False

    autotune_peaks.clear()
    autotune_peak_times.clear()
    autotune_troughs.clear()
    autotune_state_is_rising = True
    
    temperatures.clear()
    times.clear()
    target_temps_history.clear()

def stop_autotune(calculate=False):
    global autotune_running, control_mode
    if not autotune_running:
        return

    print("自整定流程已停止。")
    autotune_running = False
    control_mode = 'MANUAL'
    send_relay_command(False)

    if calculate:
        print("开始计算 PID 参数...")
        # 为保证稳定，舍弃第一个周期的数据
        stable_peaks = autotune_peaks[1:]
        stable_troughs = autotune_troughs[1:len(stable_peaks)+1]
        stable_peak_times = autotune_peak_times[1:]

        if len(stable_peaks) < 2 or len(stable_troughs) == 0:
            print("自整定错误: 未能捕获到足够的稳定振荡。请检查烤箱或增加整定周期数。")
            return

        # 计算振荡周期 Tu (秒)
        time_diffs = np.diff([t.timestamp() for t in stable_peak_times])
        Tu = np.mean(time_diffs)

        # 计算振荡幅度 a (°C)
        a = (np.mean(stable_peaks) - np.mean(stable_troughs)) / 2.0
        
        # 继电器输出幅度 d (这里是100%占空比)
        d_control = 100 
        if a == 0:
            print("自整定错误: 振荡幅度为零，无法计算。")
            return

        # 根据继电器法公式计算 Ku
        Ku = (4 * d_control) / (math.pi * a)

        # 根据 Ziegler-Nichols 规则计算 PID 参数 (带一些超调的经典规则)
        new_Kp = 0.6 * Ku
        Ti = Tu / 2.0
        Td = Tu / 8.0
        new_Ki = new_Kp / Ti
        new_Kd = new_Kp * Td

        print("\n" + "="*30)
        print("--- PID 自整定完成 ---")
        print(f"振荡周期 (Tu): {Tu:.2f} s")
        print(f"振荡幅度 (a): {a:.2f} °C")
        print(f"临界增益 (Ku): {Ku:.2f}")
        print("\n请将以下推荐值更新到脚本的配置部分:")
        print(f"PID_KP = {new_Kp:.4f}")
        print(f"PID_KI = {new_Ki:.4f}")
        print(f"PID_KD = {new_Kd:.4f}")
        print("="*30 + "\n")

def start_process(event):
    global process_running, process_start_time, control_mode
    if not process_running:
        print("开始回流焊流程...")
        process_running = True
        process_start_time = dt.datetime.now()
        control_mode = 'AUTO'
        # 清空历史数据以开始新的绘图
        temperatures.clear()
        times.clear()
        target_temps_history.clear()

def stop_process(event):
    global process_running, control_mode
    if autotune_running:
        stop_autotune(calculate=False)
    else:
        print("停止流程，切换到手动模式。")
        process_running = False
        control_mode = 'MANUAL'
        send_relay_command(False)

def manual_on(event):
    global control_mode
    if not process_running:
        control_mode = 'MANUAL'
        print("手动开启继电器")
        send_relay_command(True)

def manual_off(event):
    global control_mode
    if not process_running:
        control_mode = 'MANUAL'
        print("手动关闭继电器")
        send_relay_command(False)

# --- 创建按钮 ---
ax_start = plt.axes([0.1, 0.05, 0.1, 0.075])
btn_start = Button(ax_start, '开始流程')
btn_start.on_clicked(start_process)

ax_stop = plt.axes([0.21, 0.05, 0.1, 0.075])
btn_stop = Button(ax_stop, '停止/复位')
btn_stop.on_clicked(stop_process)

ax_autotune = plt.axes([0.32, 0.05, 0.15, 0.075])
btn_autotune = Button(ax_autotune, 'PID 自整定')
btn_autotune.on_clicked(start_autotune)

ax_man_on = plt.axes([0.7, 0.05, 0.1, 0.075])
btn_man_on = Button(ax_man_on, '手动开')
btn_man_on.on_clicked(manual_on)

ax_man_off = plt.axes([0.81, 0.05, 0.1, 0.075])
btn_man_off = Button(ax_man_off, '手动关')
btn_man_off.on_clicked(manual_off)

# --- 主程序 ---
if connect_serial():
    print(f"--- LICO-RFS 回流焊控制程序---\n")
    ani = animation.FuncAnimation(fig, animate, interval=250, blit=False)
    plt.show()
    # 关闭窗口后执行清理
    send_relay_command(False) # 确保退出时关闭继电器
    ser.close()
    print("程序结束，串口已关闭。")
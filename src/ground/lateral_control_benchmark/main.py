"""
地面小车横向路径跟踪控制器对比评测实验（纯 Python 运动学仿真）

依赖: numpy, matplotlib, Pillow
使用: python main.py
说明: 以独轮（阿克曼等效）运动学模型为被控对象，小车以恒定纵向速度沿一条
      双移线（lane change）参考路径行驶。在同一参考路径、同一初始位姿下，
      依次接入 P 比例、PID、Stanley 三种横向控制器，分别在「无扰动」和
      「定位噪声 + 侧风偏航扰动」两组条件下完成跟随，对每段轨迹统计最大横向
      误差、横向误差均方根、收敛时间、最大控制量四个指标，量化不同控制器在
      理想工况与扰动工况下的路径跟随能力。
      所有结果写入本脚本同级的 results/ 目录，与运行时的工作目录无关。
"""
import os
import csv
import math
import random

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 无图形界面环境下后台出图
import matplotlib.pyplot as plt
from PIL import Image

# 结果目录固定在脚本同级，避免从其他目录运行时把结果写到别处
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

# ---------- 仿真参数 ----------
DT = 0.05                 # 仿真步长 (s)
V = 5.0                   # 恒定纵向速度 (m/s)
WHEELBASE = 2.5           # 轴距 (m)
X_END = 40.0              # 路径终点纵向距离 (m)
MAX_TIME = 60.0           # 仿真最长时长 (s)
SEED = 20261111           # 随机种子，保证噪声序列可复现

# 扰动强度：定位噪声标准差 (m)、侧风带来的恒定偏航角速度 (rad/s)
NOISE_STD = 0.06
WIND_YAW_RATE = 0.12

# 收敛判定：横向误差进入 ±0.15 m 并保持在内即认为收敛
SETTLE_BAND = 0.15


def reference_path(x):
    """双移线参考路径：x∈[10,30] 内用正弦平方过渡到 2 m 横向偏移，其余保持。

    返回 (y_ref, dy_dx)，dy_dx 用于推算参考航向角。
    """
    if x < 10.0 or x > 30.0:
        return 0.0, 0.0
    u = (x - 10.0) / 20.0
    y = 2.0 * math.sin(math.pi * u) ** 2
    dy_dx = 2.0 * math.pi / 20.0 * math.sin(2.0 * math.pi * u)
    return y, dy_dx


# ---------- 三种横向控制器 ----------
class PController:
    """横向误差比例控制：直接用横向误差比例输出角速度。"""

    name = 'P'

    def __init__(self, kp=0.9):
        self.kp = kp

    def step(self, e_y, e_theta, v, dt):
        return self.kp * e_y


class PIDController:
    """横向误差 PID 控制：比例 + 积分消稳态偏差 + 微分抑制超调。"""

    name = 'PID'

    def __init__(self, kp=1.0, ki=0.05, kd=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_e = None

    def step(self, e_y, e_theta, v, dt):
        self.integral += e_y * dt
        deriv = 0.0 if self.prev_e is None else (e_y - self.prev_e) / dt
        self.prev_e = e_y
        return self.kp * e_y + self.ki * self.integral + self.kd * deriv


class StanleyController:
    """Stanley 横向控制：航向误差 + 基于横向偏差的前轮几何反馈。"""

    name = 'Stanley'

    def __init__(self, k=1.0, wheelbase=WHEELBASE):
        self.k = k
        self.wheelbase = wheelbase

    def step(self, e_y, e_theta, v, dt):
        heading_term = e_theta
        cross_term = math.atan2(self.k * e_y, max(v, 0.1))
        delta = heading_term + cross_term
        delta = max(-0.6, min(0.6, delta))  # 限制前轮转角
        return v / self.wheelbase * math.tan(delta)


# ---------- 单段仿真 ----------
def simulate(controller, disturbed, rng):
    """运行一段跟随仿真，返回记录的轨迹与误差序列。"""
    x, y, theta = 0.0, 0.0, 0.0
    records = {'t': [], 'x': [], 'y': [], 'e_y': [], 'omega': []}
    settled_at = None
    t = 0.0

    while x <= X_END and t <= MAX_TIME:
        y_ref, dy_dx = reference_path(x)
        theta_ref = math.atan(dy_dx)

        e_y_true = y_ref - y
        # 控制器只拿到带噪声的横向测量值；扰动工况下再叠加侧风偏航
        e_y = e_y_true + (rng.gauss(0.0, NOISE_STD) if disturbed else 0.0)
        e_theta = theta_ref - theta

        omega = controller.step(e_y, e_theta, V, DT)
        omega = max(-1.2, min(1.2, omega))  # 限制角速度

        if disturbed:
            theta += WIND_YAW_RATE * DT
        theta += omega * DT
        x += V * math.cos(theta) * DT
        y += V * math.sin(theta) * DT
        t += DT

        records['t'].append(t)
        records['x'].append(x)
        records['y'].append(y)
        records['e_y'].append(e_y_true)
        records['omega'].append(omega)

        if settled_at is None and abs(e_y_true) <= SETTLE_BAND:
            # 进入带宽后需保持到本段结束，否则不算收敛
            tail = records['e_y'][-1:]
            if all(abs(v) <= SETTLE_BAND for v in tail):
                settled_at = t

    # 收敛时间复核：找到最后一次越出带宽的时刻，其后即收敛
    settle_time = MAX_TIME
    for i, err in enumerate(records['e_y']):
        if abs(err) > SETTLE_BAND:
            settle_time = records['t'][i]
    if all(abs(v) <= SETTLE_BAND for v in records['e_y']):
        settle_time = records['t'][0]

    e_arr = np.array(records['e_y'])
    metrics = {
        'max_err': round(float(np.max(np.abs(e_arr))), 3),
        'rmse': round(float(np.sqrt(np.mean(e_arr ** 2))), 3),
        'settle_time': round(float(settle_time), 2),
        'max_steer': round(float(np.max(np.abs(records['omega']))), 3),
    }
    return records, metrics


def evaluate(controllers, conditions):
    """对每种控制器 × 每组条件跑仿真，汇总指标与轨迹。"""
    summary = []
    trajectories = {}
    for cond_name, disturbed in conditions:
        for ctrl in controllers:
            rng = random.Random(SEED + hash(cond_name) % 1000)
            records, metrics = simulate(ctrl, disturbed, rng)
            row = {'controller': ctrl.name, 'condition': cond_name}
            row.update(metrics)
            summary.append(row)
            trajectories[(ctrl.name, cond_name)] = records
    return summary, trajectories


# ---------- 绘图 ----------
def plot_trajectory(trajectories, cond_name, out_path):
    """俯视图：参考路径与各控制器实际轨迹。"""
    xs = np.linspace(0, X_END, 400)
    y_ref = [reference_path(x)[0] for x in xs]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(xs, y_ref, 'k--', lw=1.5, label='reference')
    colors = {'P': '#1f77b4', 'PID': '#2ca02c', 'Stanley': '#d62728'}
    for (name, cond), rec in trajectories.items():
        if cond != cond_name:
            continue
        ax.plot(rec['x'], rec['y'], lw=1.4, color=colors.get(name), label=name)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'Trajectory — {cond_name}')
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_error(trajectories, cond_name, out_path):
    """横向误差随时间曲线。"""
    fig, ax = plt.subplots(figsize=(10, 4.5))
    colors = {'P': '#1f77b4', 'PID': '#2ca02c', 'Stanley': '#d62728'}
    for (name, cond), rec in trajectories.items():
        if cond != cond_name:
            continue
        ax.plot(rec['t'], rec['e_y'], lw=1.3, color=colors.get(name), label=name)
    ax.axhline(SETTLE_BAND, color='gray', ls=':', lw=1)
    ax.axhline(-SETTLE_BAND, color='gray', ls=':', lw=1)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('lateral error (m)')
    ax.set_title(f'Lateral error — {cond_name}')
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_metrics_bar(summary, out_path):
    """RMSE 与最大误差分组柱状图。"""
    controllers = ['P', 'PID', 'Stanley']
    conditions = sorted({r['condition'] for r in summary})
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    width = 0.35
    x = np.arange(len(controllers))
    for j, cond in enumerate(conditions):
        rmse = [next(r['rmse'] for r in summary
                      if r['controller'] == c and r['condition'] == cond)
                for c in controllers]
        axes[0].bar(x + (j - 0.5) * width, rmse, width, label=cond)
    for j, cond in enumerate(conditions):
        maxerr = [next(r['max_err'] for r in summary
                       if r['controller'] == c and r['condition'] == cond)
                  for c in controllers]
        axes[1].bar(x + (j - 0.5) * width, maxerr, width, label=cond)
    axes[0].set_title('RMSE lateral error (m)')
    axes[1].set_title('Max lateral error (m)')
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(controllers)
        ax.legend()
        ax.grid(alpha=0.3, axis='y')
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def render_demo_frame(records, idx, colors):
    """渲染一帧演示画面（参考路径 + 已走轨迹 + 当前小车），返回 PIL 图像。"""
    fig, ax = plt.subplots(figsize=(8, 4.5))
    xs = np.linspace(0, X_END, 400)
    y_ref = [reference_path(x)[0] for x in xs]
    ax.plot(xs, y_ref, 'k--', lw=1.5, label='reference')
    for (name, cond), rec in records.items():
        if cond != 'clean':
            continue
        cur = min(idx, len(rec['x']))
        ax.plot(rec['x'][:cur], rec['y'][:cur], lw=1.4,
                color=colors.get(name), label=name)
        cx, cy = rec['x'][cur - 1], rec['y'][cur - 1]
        ax.plot(cx, cy, 'o', color=colors.get(name), markersize=6)
    ax.set_xlim(-1, X_END + 1)
    ax.set_ylim(-1.5, 3.5)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Lateral tracking demo (clean)')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    img = Image.fromarray(buf.reshape(fig.canvas.get_width_height()[::-1] + (4,)))
    plt.close(fig)
    return img.convert('RGB')


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    controllers = [PController(), PIDController(), StanleyController()]
    conditions = [('clean', False), ('disturbed', True)]

    summary, trajectories = evaluate(controllers, conditions)

    # ---------- 写指标 CSV ----------
    csv_path = os.path.join(RESULTS_DIR, 'metrics.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(
            f, fieldnames=['controller', 'condition', 'max_err', 'rmse',
                           'settle_time', 'max_steer'])
        writer.writeheader()
        writer.writerows(summary)

    # ---------- 轨迹与误差图 ----------
    colors = {'P': '#1f77b4', 'PID': '#2ca02c', 'Stanley': '#d62728'}
    for cond, _ in conditions:
        plot_trajectory(trajectories, cond,
                        os.path.join(RESULTS_DIR, f'lateral_control_trajectory_{cond}.png'))
        plot_error(trajectories, cond,
                   os.path.join(RESULTS_DIR, f'lateral_control_error_{cond}.png'))
    plot_metrics_bar(summary, os.path.join(RESULTS_DIR, 'lateral_control_metrics.png'))

    # ---------- 演示动图：clean 条件下三控制器跟随过程 ----------
    demo_records = {k: v for k, v in trajectories.items() if k[1] == 'clean'}
    n_steps = len(next(iter(demo_records.values()))['t'])
    frames = []
    step_every = max(1, n_steps // 30)
    for idx in range(step_every, n_steps + 1, step_every):
        frames.append(render_demo_frame(demo_records, idx, colors))
    if frames:
        frame_duration = max(60, int(1000 / len(frames) * 8))
        frames[0].save(os.path.join(RESULTS_DIR, 'lateral_control_demo.gif'),
                       save_all=True, append_images=frames[1:],
                       duration=frame_duration, loop=0, optimize=True)

    # ---------- 控制台输出 ----------
    for row in summary:
        print(f"{row['controller']:8s} {row['condition']:10s} "
              f"max_err={row['max_err']:.3f} rmse={row['rmse']:.3f} "
              f"settle={row['settle_time']:.2f}s max_steer={row['max_steer']:.3f}")
    print(f'实验完成，结果保存在 {RESULTS_DIR} 目录')


if __name__ == '__main__':
    main()

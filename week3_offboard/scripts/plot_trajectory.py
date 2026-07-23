#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 rosbag，绘制飞行轨迹图：俯视轨迹 + 高度曲线，标注起点/航点/终点/最大误差"""
import math
import rosbag
import matplotlib
matplotlib.use('Agg')          # 不弹窗，直接存文件
import matplotlib.pyplot as plt

BAG = '/workspace/catkin_ws/src/week3_offboard/bags/survey.bag'
WAYPOINTS = [(3.5, 0), (0, 3.5), (-3.5, 0)]   # 巡视航点（与 survey_node.py 一致）

# ---------- 读 bag ----------
xs, ys, zs, ts = [], [], [], []
t0 = None
with rosbag.Bag(BAG) as bag:
    for topic, msg, t in bag.read_messages(topics=['/mavros/local_position/pose']):
        if t0 is None:
            t0 = t.to_sec()
        ts.append(t.to_sec() - t0)
        xs.append(msg.pose.position.x)
        ys.append(msg.pose.position.y)
        zs.append(msg.pose.position.z)

print(f"共 {len(xs)} 个位置点，时长 {ts[-1]:.1f}s")

# ---------- 计算每个航点的最大偏差（轨迹到航点的最近距离） ----------
print("\n航点误差：")
worst = 0
for i, (wx, wy) in enumerate(WAYPOINTS, 1):
    d = min(math.hypot(x - wx, y - wy) for x, y in zip(xs, ys))
    worst = max(worst, d)
    status = "PASS" if d < 0.5 else "FAIL"
    print(f"  航点{i} ({wx},{wy}): 最近距离 {d:.3f} m  [{status}] (<0.5m 达标)")
print(f"  最大误差: {worst:.3f} m")

# ---------- 画图 ----------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6))

# 左图：俯视轨迹
ax1.plot(xs, ys, 'b-', linewidth=1.5, label='trajectory')
ax1.plot(xs[0], ys[0], 'go', markersize=12, label=f'start ({xs[0]:.1f},{ys[0]:.1f})')
ax1.plot(xs[-1], ys[-1], 'rs', markersize=12, label=f'end ({xs[-1]:.1f},{ys[-1]:.1f})')
for i, (wx, wy) in enumerate(WAYPOINTS, 1):
    ax1.plot(wx, wy, 'rx', markersize=14, markeredgewidth=3)
    ax1.annotate(f'WP{i}', (wx, wy), textcoords='offset points', xytext=(8, 8), fontsize=11)
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_title('Top-down Trajectory')
ax1.legend()
ax1.grid(True)
ax1.axis('equal')

# 右图：高度曲线
ax2.plot(ts, zs, 'b-')
ax2.axhline(y=2.0, color='r', linestyle='--', alpha=0.5, label='target 2m')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Z (m)')
ax2.set_title('Altitude vs Time')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
out = '/workspace/catkin_ws/src/week3_offboard/bags/trajectory.png'
plt.savefig(out, dpi=150)
print(f"\n图已保存: {out}")
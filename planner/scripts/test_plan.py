#!/usr/bin/env python3
"""
测试修改后的A*规划，检查是否还有浮空问题
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config

cfg = Config()

# 使用与plan.py相同的设置
tomo_file = '567_-ready-to-pctplanner'
start_pos = np.array([-0.68, -1.0, 0.1], dtype=np.float32)
end_pos = np.array([21.6, -29.3, -4.0], dtype=np.float32)

print("=" * 60)
print("测试A*规划...")
print("=" * 60)
print(f"起点: {start_pos}")
print(f"终点: {end_pos}")

# 创建规划器
planner = TomogramPlanner(cfg)
planner.loadTomogram(tomo_file)

# 规划
traj_3d = planner.plan(start_pos, end_pos)

if traj_3d is None:
    print("\n规划失败！")
    sys.exit(1)

print(f"\n规划成功！轨迹点数: {len(traj_3d)}")

# 分析轨迹
print("\n轨迹分析:")
print(f"  起点: [{traj_3d[0, 0]:.2f}, {traj_3d[0, 1]:.2f}, {traj_3d[0, 2]:.2f}]")
print(f"  终点: [{traj_3d[-1, 0]:.2f}, {traj_3d[-1, 1]:.2f}, {traj_3d[-1, 2]:.2f}]")
print(f"  Z高度范围: [{np.min(traj_3d[:, 2]):.2f}, {np.max(traj_3d[:, 2]):.2f}]")

# 检查轨迹是否贴近tomogram地面
print("\n检查轨迹与地面高度的贴合度:")

# 加载tomogram数据
import pickle
tomo_dir = '../../rsc/tomogram/'
with open(tomo_dir + tomo_file + '.pickle', 'rb') as handle:
    data_dict = pickle.load(handle)
    tomogram = np.asarray(data_dict['data'], dtype=np.float32)
    resolution = float(data_dict['resolution'])
    center = np.asarray(data_dict['center'], dtype=np.double)
    n_slice = tomogram.shape[1]
    slice_h0 = float(data_dict['slice_h0'])
    slice_dh = float(data_dict['slice_dh'])
    map_dim = [tomogram.shape[2], tomogram.shape[3]]

elev_g = tomogram[3]
elev_g_clean = np.nan_to_num(elev_g, nan=-100)

# 检查每个轨迹点
offset = np.array([int(map_dim[0] / 2), int(map_dim[1] / 2)], dtype=np.int32)
height_diffs = []
floating_count = 0

for i in range(len(traj_3d)):
    pt = traj_3d[i]
    # 转换为地图索引
    pos_2d = np.asarray(pt[:2], dtype=np.float32) - center[:2]
    idx = np.round(pos_2d / resolution).astype(np.int32) + offset
    x_idx, y_idx = int(np.clip(idx[0], 0, map_dim[0] - 1)), int(np.clip(idx[1], 0, map_dim[1] - 1))
    
    # 找到该点最合适的slice
    pt_z = pt[2]
    min_diff = 1e9
    closest_ground = None
    
    for slice_idx in range(n_slice):
        ground_h = elev_g_clean[slice_idx, x_idx, y_idx]
        if ground_h > -50:  # 有效地面
            diff = abs(pt_z - ground_h)
            if diff < min_diff:
                min_diff = diff
                closest_ground = ground_h
    
    if closest_ground is not None:
        height_diffs.append(min_diff)
        if min_diff > 0.5:  # 认为大于0.5m为浮空
            floating_count += 1
            if floating_count <= 5:  # 只打印前5个浮空点
                print(f"  点 {i}: 位置=[{pt[0]:.2f}, {pt[1]:.2f}], "
                      f"轨迹高度={pt_z:.2f}, 地面高度={closest_ground:.2f}, "
                      f"偏差={min_diff:.2f}m")

if height_diffs:
    print(f"\n统计:")
    print(f"  平均偏差: {np.mean(height_diffs):.3f}m")
    print(f"  最大偏差: {np.max(height_diffs):.3f}m")
    print(f"  浮空点数 (>0.5m): {floating_count}/{len(traj_3d)} ({floating_count/len(traj_3d)*100:.1f}%)")
    
    if floating_count == 0:
        print("\n✓ 太好了！轨迹已经紧贴地面，没有浮空现象！")
    elif floating_count < len(traj_3d) * 0.1:
        print("\n✓ 轨迹基本贴合地面，只有少量点浮空。")
    else:
        print("\n✗ 仍有较多浮空点，需要进一步调整。")

# 可视化
fig = plt.figure(figsize=(15, 5))

# 1. 2D轨迹俯视图
ax1 = fig.add_subplot(131)
ax1.plot(traj_3d[:, 0], traj_3d[:, 1], 'b-', linewidth=2, label='轨迹')
ax1.plot(start_pos[0], start_pos[1], 'go', markersize=10, label='起点')
ax1.plot(end_pos[0], end_pos[1], 'ro', markersize=10, label='终点')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_title('轨迹俯视图')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.axis('equal')

# 2. Z高度变化
ax2 = fig.add_subplot(132)
ax2.plot(traj_3d[:, 2], 'b-', linewidth=2)
ax2.axhline(y=start_pos[2], color='g', linestyle='--', alpha=0.5, label='起点高度')
ax2.axhline(y=end_pos[2], color='r', linestyle='--', alpha=0.5, label='终点高度')
ax2.set_xlabel('轨迹点索引')
ax2.set_ylabel('高度 (m)')
ax2.set_title('轨迹高度变化')
ax2.legend()
ax2.grid(True, alpha=0.3)

# 3. 3D轨迹
ax3 = fig.add_subplot(133, projection='3d')
ax3.plot(traj_3d[:, 0], traj_3d[:, 1], traj_3d[:, 2], 'b-', linewidth=2, label='轨迹')
ax3.scatter(start_pos[0], start_pos[1], start_pos[2], c='g', s=100, label='起点')
ax3.scatter(end_pos[0], end_pos[1], end_pos[2], c='r', s=100, label='终点')
ax3.set_xlabel('X (m)')
ax3.set_ylabel('Y (m)')
ax3.set_zlabel('Z (m)')
ax3.set_title('3D轨迹')
ax3.legend()

plt.tight_layout()
plt.savefig('/home/cjsg/PCT_planner/test_trajectory.png', dpi=150)
print(f"\n轨迹可视化已保存到: /home/cjsg/PCT_planner/test_trajectory.png")


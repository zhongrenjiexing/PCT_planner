#!/usr/bin/env python3
"""
详细分析A*路径，找出为什么会绕大弯
"""

import os
import sys
import pickle
import numpy as np
import matplotlib.pyplot as plt

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))
sys.path.insert(0, os.path.join(script_dir, '../lib'))

from config import Config
from planner_wrapper import TomogramPlanner
import a_star

# 设置
tomo_file = '2026-01-02_22-26-52_colorized_L567_FT_AddLift_AddPoints_XYZ'
tomo_dir = '../../rsc/tomogram/'

# 测试用例
start_pos = np.array([18.8, -6.79, 4.7], dtype=np.float32)
end_pos = np.array([19.3, -10.9, 4.7], dtype=np.float32)

print("=" * 70)
print("详细分析A*路径绕弯原因")
print("=" * 70)

# 加载tomogram数据
with open(tomo_dir + tomo_file + '.pickle', 'rb') as handle:
    data_dict = pickle.load(handle)
    tomogram = np.asarray(data_dict['data'], dtype=np.float32)
    resolution = float(data_dict['resolution'])
    center = np.asarray(data_dict['center'], dtype=np.double)
    n_slice = tomogram.shape[1]
    map_dim = [tomogram.shape[2], tomogram.shape[3]]

offset = np.array([int(map_dim[0] / 2), int(map_dim[1] / 2)], dtype=np.int32)
trav = tomogram[0]
elev_g = tomogram[3]
elev_g_clean = np.nan_to_num(elev_g, nan=-100)

def world_to_grid(pos):
    pos_2d = np.asarray(pos[:2], dtype=np.float32) - center[:2]
    idx = np.round(pos_2d / resolution).astype(np.int32) + offset
    return np.array([idx[1], idx[0]], dtype=np.int32)  # row, col

def grid_to_world(row, col):
    x = (col - offset[0]) * resolution + center[0]
    y = (row - offset[1]) * resolution + center[1]
    return np.array([x, y])

# 执行规划
cfg = Config()
planner = TomogramPlanner(cfg)
planner.loadTomogram(tomo_file)
traj_3d = planner.plan(start_pos, end_pos)

if traj_3d is None:
    print("规划失败！")
    sys.exit(1)

# 获取A*原始路径
path_finder = planner.planner.get_path_finder()
path_raw = path_finder.get_result_matrix()

print(f"\n路径总点数: {len(path_raw)}")
print("\n完整A*路径:")
print("-" * 90)

for i in range(len(path_raw)):
    layer = int(path_raw[i, 0])
    row = int(path_raw[i, 1])
    col = int(path_raw[i, 2])
    world_pos = grid_to_world(row, col)
    ground_h = elev_g_clean[layer, row, col]
    cost = trav[layer, row, col]
    
    # 计算与直线的偏差
    t = i / (len(path_raw) - 1) if len(path_raw) > 1 else 0
    expected_x = start_pos[0] + t * (end_pos[0] - start_pos[0])
    expected_y = start_pos[1] + t * (end_pos[1] - start_pos[1])
    deviation = np.sqrt((world_pos[0] - expected_x)**2 + (world_pos[1] - expected_y)**2)
    
    marker = ""
    if deviation > 0.5:
        marker = f"  <-- 偏离直线 {deviation:.2f}m"
    
    print(f"[{i:3d}] slice={layer}, grid=({row:3d},{col:3d}), "
          f"world=({world_pos[0]:7.2f},{world_pos[1]:7.2f}), "
          f"z={ground_h:6.2f}, cost={cost:5.1f}{marker}")

print("-" * 90)

# 分析直线路径上的障碍
print("\n分析直线路径上各位置的情况 (slice 7):")
print("-" * 90)

start_grid = world_to_grid(start_pos)
end_grid = world_to_grid(end_pos)

n_samples = 50
for i, t in enumerate(np.linspace(0, 1, n_samples)):
    row = int(start_grid[0] + t * (end_grid[0] - start_grid[0]))
    col = int(start_grid[1] + t * (end_grid[1] - start_grid[1]))
    
    if 0 <= row < map_dim[1] and 0 <= col < map_dim[0]:
        world_pos = grid_to_world(row, col)
        
        # 检查slice 7和相邻slice
        costs = []
        elevs = []
        for s in [6, 7, 8]:
            if s < n_slice:
                costs.append(trav[s, row, col])
                elevs.append(elev_g_clean[s, row, col])
        
        blocked = any(c >= 35 for c in costs)
        status = "BLOCKED" if blocked else "OK"
        
        print(f"[{i:2d}] grid=({row:3d},{col:3d}), world=({world_pos[0]:6.2f},{world_pos[1]:6.2f}), "
              f"cost_s6={costs[0]:5.1f}, cost_s7={costs[1]:5.1f}, cost_s8={costs[2]:5.1f} [{status}]")

print("-" * 90)

# 可视化
fig, axes = plt.subplots(1, 2, figsize=(16, 8))

# 1. 路径和障碍物地图
ax = axes[0]
# 显示slice 7的cost地图（放大到问题区域）
margin = 60
center_row = (start_grid[0] + end_grid[0]) // 2
center_col = (start_grid[1] + end_grid[1]) // 2
row_min = max(0, center_row - margin)
row_max = min(map_dim[1], center_row + margin)
col_min = max(0, center_col - margin)
col_max = min(map_dim[0], center_col + margin)

cost_region = trav[7, row_min:row_max, col_min:col_max]
extent = [
    (col_min - offset[0]) * resolution + center[0],
    (col_max - offset[0]) * resolution + center[0],
    (row_min - offset[1]) * resolution + center[1],
    (row_max - offset[1]) * resolution + center[1]
]

im = ax.imshow(cost_region.T, origin='lower', extent=extent,
               cmap='RdYlGn_r', vmin=0, vmax=50, alpha=0.8)

# 画出A*路径
path_world = []
for i in range(len(path_raw)):
    row = int(path_raw[i, 1])
    col = int(path_raw[i, 2])
    world_pos = grid_to_world(row, col)
    path_world.append([world_pos[0], world_pos[1]])
path_world = np.array(path_world)

ax.plot(path_world[:, 0], path_world[:, 1], 'b.-', linewidth=2, markersize=4,
        label='A*实际路径', alpha=0.9)

# 画出直线路径
ax.plot([start_pos[0], end_pos[0]], [start_pos[1], end_pos[1]], 
        'r--', linewidth=2, alpha=0.7, label='直线路径')

ax.plot(start_pos[0], start_pos[1], 'go', markersize=12, label='起点')
ax.plot(end_pos[0], end_pos[1], 'ro', markersize=12, label='终点')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Slice 7 成本地图与A*路径')
ax.legend(loc='upper right')
plt.colorbar(im, ax=ax, label='Cost')

# 2. X坐标随路径变化
ax = axes[1]
path_indices = np.arange(len(path_world))
expected_x = start_pos[0] + path_indices / (len(path_indices)-1) * (end_pos[0] - start_pos[0])
ax.plot(path_indices, path_world[:, 0], 'b.-', linewidth=2, markersize=4, label='A*路径X坐标')
ax.plot(path_indices, expected_x, 'r--', linewidth=2, alpha=0.7, label='直线路径X坐标')
ax.set_xlabel('路径点索引')
ax.set_ylabel('X坐标 (m)')
ax.set_title('X坐标对比')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/home/cjsg/PCT_planner/path_analysis.png', dpi=150)
print(f"\n图表已保存到: /home/cjsg/PCT_planner/path_analysis.png")


#!/usr/bin/env python3
"""
检查A*原始输出（未优化前）与优化后的区别
"""

import sys
import numpy as np
import pickle
import matplotlib.pyplot as plt

from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config
from lib import a_star

cfg = Config()

# 使用与plan.py相同的设置
tomo_file = '567_-ready-to-pctplanner'
start_pos = np.array([-0.68, -1.0, 0.1], dtype=np.float32)
end_pos = np.array([21.6, -29.3, -4.0], dtype=np.float32)

print("=" * 60)
print("检查A*原始路径 vs 优化后路径")
print("=" * 60)

# 创建规划器
planner = TomogramPlanner(cfg)
planner.loadTomogram(tomo_file)

# 规划（这会同时生成A*路径和优化后的轨迹）
traj_3d = planner.plan(start_pos, end_pos)

if traj_3d is None:
    print("\n规划失败！")
    sys.exit(1)

# 获取A*原始路径
path_finder: a_star.Astar = planner.planner.get_path_finder()
path_raw = path_finder.get_result_matrix()  # [layer, row, col]

print(f"\nA*原始路径点数: {len(path_raw)}")
print(f"优化后轨迹点数: {len(traj_3d)}")

# 加载tomogram数据
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
trav = tomogram[0]

# 转换A*原始路径为世界坐标
offset = np.array([int(map_dim[0] / 2), int(map_dim[1] / 2)], dtype=np.int32)
path_world = []

print("\nA*原始路径前10个点的详细信息:")
for i in range(min(10, len(path_raw))):
    layer = int(path_raw[i, 0])
    row = int(path_raw[i, 1])
    col = int(path_raw[i, 2])
    
    # 转换为世界坐标
    x_idx = col - offset[0]
    y_idx = row - offset[1]
    x_world = x_idx * resolution + center[0]
    y_world = y_idx * resolution + center[1]
    
    # 获取该位置的地面高度
    # 注意：elev_g的形状是 [n_slice, map_dim_x, map_dim_y]
    # 而row对应y方向，col对应x方向
    # A*返回的path_raw格式是[layer, row, col]，其中row和col是grid_map的索引
    # grid_map的维度是[layer][row][col]，对应[n_slice][max_y_][max_x_]
    # 所以应该用row作为y索引，col作为x索引
    ground_h = elev_g_clean[layer, row, col]  # 修正：使用row, col而不是col, row
    cost = trav[layer, row, col]
    
    path_world.append([x_world, y_world, ground_h])
    
    print(f"  点{i}: layer={layer}, grid=[{row},{col}], "
          f"world=[{x_world:.2f},{y_world:.2f}], "
          f"ground_h={ground_h:.2f}, cost={cost:.1f}")

path_world = np.array(path_world)

# 检查A*原始路径的高度是否贴合地面
print("\n检查A*原始路径与地面高度的贴合度:")
height_diffs = []
floating_count = 0

for i in range(len(path_world)):
    layer = int(path_raw[i, 0])
    row = int(path_raw[i, 1])
    col = int(path_raw[i, 2])
    
    # A*使用的高度就是该层该位置的地面高度
    astar_height = elev_g_clean[layer, row, col]  # 修正索引顺序
    
    # 找到该位置所有层的最接近高度
    min_diff = 1e9
    for test_layer in range(n_slice):
        test_ground = elev_g_clean[test_layer, row, col]  # 修正索引顺序
        if test_ground > -50:
            diff = abs(astar_height - test_ground)
            if diff < min_diff:
                min_diff = diff
    
    height_diffs.append(min_diff)
    if min_diff > 0.5:
        floating_count += 1

if height_diffs:
    print(f"  平均偏差: {np.mean(height_diffs):.3f}m")
    print(f"  最大偏差: {np.max(height_diffs):.3f}m")
    print(f"  浮空点数 (>0.5m): {floating_count}/{len(path_world)} ({floating_count/len(path_world)*100:.1f}%)")
    
    if floating_count == 0:
        print("\n✓ A*原始路径紧贴地面，没有浮空！")
        print("问题可能出在轨迹优化阶段。")
    else:
        print("\n✗ A*原始路径就有浮空问题。")
        print("需要进一步改进A*的层选择逻辑。")

# 可视化对比
fig, axes = plt.subplots(2, 2, figsize=(15, 10))

# 1. XY平面对比
ax = axes[0, 0]
ax.plot(path_world[:, 0], path_world[:, 1], 'r.-', linewidth=2, markersize=3, alpha=0.7, label='A*原始路径')
ax.plot(traj_3d[:, 0], traj_3d[:, 1], 'b-', linewidth=1, alpha=0.7, label='优化后轨迹')
ax.plot(start_pos[0], start_pos[1], 'go', markersize=10, label='起点')
ax.plot(end_pos[0], end_pos[1], 'ro', markersize=10, label='终点')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('XY平面对比')
ax.legend()
ax.grid(True, alpha=0.3)
ax.axis('equal')

# 2. 高度对比
ax = axes[0, 1]
ax.plot(path_world[:, 2], 'r.-', linewidth=2, markersize=3, alpha=0.7, label='A*原始')
ax.plot(traj_3d[:, 2], 'b-', linewidth=1, alpha=0.7, label='优化后')
ax.axhline(y=start_pos[2], color='g', linestyle='--', alpha=0.5)
ax.axhline(y=end_pos[2], color='r', linestyle='--', alpha=0.5)
ax.set_xlabel('点索引')
ax.set_ylabel('高度 (m)')
ax.set_title('高度对比')
ax.legend()
ax.grid(True, alpha=0.3)

# 3. 层变化
ax = axes[1, 0]
layers = path_raw[:, 0]
ax.plot(layers, 'r.-', linewidth=2, markersize=3)
ax.set_xlabel('A*路径点索引')
ax.set_ylabel('Slice索引')
ax.set_title('A*路径的层变化')
ax.grid(True, alpha=0.3)
ax.set_yticks(range(n_slice))

# 4. 高度差分布
ax = axes[1, 1]
if len(path_world) > 1:
    astar_diffs = np.abs(np.diff(path_world[:, 2]))
    traj_diffs = np.abs(np.diff(traj_3d[:, 2]))
    
    ax.hist(astar_diffs, bins=30, alpha=0.5, color='r', label='A*原始')
    ax.hist(traj_diffs, bins=30, alpha=0.5, color='b', label='优化后')
    ax.set_xlabel('相邻点高度差 (m)')
    ax.set_ylabel('频数')
    ax.set_title('相邻点高度差分布')
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/home/cjsg/PCT_planner/astar_vs_optimized.png', dpi=150)
print(f"\n对比图已保存到: /home/cjsg/PCT_planner/astar_vs_optimized.png")


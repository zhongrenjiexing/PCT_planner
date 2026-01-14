#!/usr/bin/env python3
"""
检查障碍带的范围
"""

import os
import sys
import pickle
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))

# 设置
tomo_file = '2026-01-02_22-26-52_colorized_L567_FT_AddLift_AddPoints_XYZ'
tomo_dir = '../../rsc/tomogram/'

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

def grid_to_world(row, col):
    x = (col - offset[0]) * resolution + center[0]
    y = (row - offset[1]) * resolution + center[1]
    return x, y

print("=" * 70)
print("检查 y ≈ -7.3 附近的障碍带范围 (slice 7)")
print("=" * 70)

# 检查 y = -7.3 这一行，x 从 17 到 25 的范围
target_y = -7.3
target_row = int(round((target_y - center[1]) / resolution + offset[1]))

print(f"\n固定 y ≈ {target_y} (row={target_row})，检查不同x位置的cost:")
print("-" * 60)

for x in np.arange(16, 25, 0.2):
    col = int(round((x - center[0]) / resolution + offset[0]))
    if 0 <= col < map_dim[0] and 0 <= target_row < map_dim[1]:
        cost = trav[7, target_row, col]
        wx, wy = grid_to_world(target_row, col)
        status = "BLOCKED" if cost >= 35 else "OK"
        if status == "BLOCKED":
            print(f"x={wx:6.2f}, col={col:3d}, cost={cost:5.1f} [{status}]")

print("\n" + "=" * 70)
print("检查不同y位置的障碍情况 (x ≈ 19.0, slice 7)")
print("=" * 70)

target_x = 19.0
target_col = int(round((target_x - center[0]) / resolution + offset[0]))

print(f"\n固定 x ≈ {target_x} (col={target_col})，检查不同y位置的cost:")
print("-" * 60)

for y in np.arange(-6.5, -8.5, -0.1):
    row = int(round((y - center[1]) / resolution + offset[1]))
    if 0 <= row < map_dim[1] and 0 <= target_col < map_dim[0]:
        cost = trav[7, row, target_col]
        wx, wy = grid_to_world(row, target_col)
        status = "BLOCKED" if cost >= 35 else "OK"
        print(f"y={wy:6.2f}, row={row:3d}, cost={cost:5.1f} [{status}]")

print("\n" + "=" * 70)
print("检查是否有绕行的缺口 (在障碍带区域搜索可通行点)")
print("=" * 70)

# 在 y=-7.2 到 y=-7.5 的范围内，找出所有可通行的点
y_range = np.arange(-7.0, -8.0, -0.1)
passable_points = []

for y in y_range:
    row = int(round((y - center[1]) / resolution + offset[1]))
    for x in np.arange(16, 25, 0.1):
        col = int(round((x - center[0]) / resolution + offset[0]))
        if 0 <= row < map_dim[1] and 0 <= col < map_dim[0]:
            cost = trav[7, row, col]
            if cost < 35:
                wx, wy = grid_to_world(row, col)
                passable_points.append((wx, wy, cost))

if passable_points:
    # 找到最接近直线路径的可通行点
    start_pos = np.array([18.8, -6.79])
    end_pos = np.array([19.3, -10.9])
    
    print(f"\n在障碍区域找到 {len(passable_points)} 个可通行点")
    
    # 按x坐标分组统计
    x_min = min(p[0] for p in passable_points)
    x_max = max(p[0] for p in passable_points)
    print(f"可通行区域 x 范围: [{x_min:.2f}, {x_max:.2f}]")
    
    # 找出x最接近19的可通行点
    closest = min(passable_points, key=lambda p: abs(p[0] - 19.0))
    print(f"最接近 x=19.0 的可通行点: ({closest[0]:.2f}, {closest[1]:.2f}), cost={closest[2]:.1f}")
else:
    print("\n在障碍区域没有找到可通行点！")
    print("这说明障碍带完全阻断了直线路径，A*必须绕行。")


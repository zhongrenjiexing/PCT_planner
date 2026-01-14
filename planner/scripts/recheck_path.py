#!/usr/bin/env python3
"""
使用正确的索引顺序重新检查直线路径
"""

import os
import sys
import pickle
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))
sys.path.insert(0, os.path.join(script_dir, '../lib'))

# 加载tomogram
tomo_file = '2026-01-02_22-26-52_colorized_L567_FT_AddLift_AddPoints_XYZ'
tomo_dir = '../../rsc/tomogram/'

with open(tomo_dir + tomo_file + '.pickle', 'rb') as handle:
    data_dict = pickle.load(handle)
    tomogram = np.asarray(data_dict['data'], dtype=np.float32)
    resolution = float(data_dict['resolution'])
    center = np.asarray(data_dict['center'], dtype=np.double)
    n_slice = tomogram.shape[1]
    map_dim = [tomogram.shape[2], tomogram.shape[3]]

trav = tomogram[0]
elev_g = tomogram[3]
elev_g_clean = np.nan_to_num(elev_g, nan=-100)
offset = np.array([int(map_dim[0] / 2), int(map_dim[1] / 2)], dtype=np.int32)

print("=" * 70)
print("使用正确索引重新检查直线路径")
print("=" * 70)

# 起点和终点
start_pos = np.array([18.8, -6.79, 4.7])
end_pos = np.array([19.3, -10.9, 4.7])

def world_to_idx(x, y):
    """世界坐标转索引，返回 (col, row) 即 (x_idx, y_idx)"""
    col = int(round((x - center[0]) / resolution + offset[0]))
    row = int(round((y - center[1]) / resolution + offset[1]))
    return col, row

start_col, start_row = world_to_idx(start_pos[0], start_pos[1])
end_col, end_row = world_to_idx(end_pos[0], end_pos[1])

print(f"\n起点: ({start_pos[0]}, {start_pos[1]}) -> col={start_col}, row={start_row}")
print(f"终点: ({end_pos[0]}, {end_pos[1]}) -> col={end_col}, row={end_row}")

# 正确的索引顺序是 trav[slice, col, row]
print(f"\n使用正确的索引 trav[slice, col, row]:")
print(f"起点: trav[7, {start_col}, {start_row}] = {trav[7, start_col, start_row]:.2f}")
print(f"终点: trav[7, {end_col}, {end_row}] = {trav[7, end_col, end_row]:.2f}")

print(f"\n检查直线路径上的cost (slice 7):")
print("-" * 70)

n_samples = 50
blocked_count = 0
blocked_points = []

for i, t in enumerate(np.linspace(0, 1, n_samples)):
    x = start_pos[0] + t * (end_pos[0] - start_pos[0])
    y = start_pos[1] + t * (end_pos[1] - start_pos[1])
    col, row = world_to_idx(x, y)
    
    # 正确的索引: trav[slice, col, row]
    cost = trav[7, col, row]
    elev = elev_g_clean[7, col, row]
    
    status = "BLOCKED" if cost >= 35 else "OK"
    if cost >= 35:
        blocked_count += 1
        blocked_points.append((x, y, cost))
    
    print(f"[{i:2d}] ({x:6.2f}, {y:6.2f}) col={col:3d}, row={row:3d}, "
          f"cost={cost:5.1f}, elev={elev:5.2f} [{status}]")

print("-" * 70)
print(f"\n总结: 直线路径上 {blocked_count}/{n_samples} 个点被阻挡")

if blocked_count > 0:
    print(f"\n被阻挡的点:")
    for bp in blocked_points:
        print(f"  ({bp[0]:.2f}, {bp[1]:.2f}): cost={bp[2]:.1f}")
else:
    print("\n✓ 直线路径完全畅通！")
    print("如果A*仍然绕行，问题可能在其他地方...")


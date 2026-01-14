#!/usr/bin/env python3
"""
检查tomogram的slice和高度映射是否正确
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
    slice_h0 = float(data_dict['slice_h0'])
    slice_dh = float(data_dict['slice_dh'])
    map_dim = [tomogram.shape[2], tomogram.shape[3]]

offset = np.array([int(map_dim[0] / 2), int(map_dim[1] / 2)], dtype=np.int32)
trav = tomogram[0]
elev_g = tomogram[3]
elev_g_clean = np.nan_to_num(elev_g, nan=-100)

print("=" * 70)
print("Tomogram 高度映射检查")
print("=" * 70)

print(f"\n基本信息:")
print(f"  分辨率: {resolution}m")
print(f"  地图中心: {center}")
print(f"  slice数量: {n_slice}")
print(f"  slice_h0 (起始高度): {slice_h0}")
print(f"  slice_dh (高度间隔): {slice_dh}")
print(f"  tomogram shape: {tomogram.shape}")
print(f"  - 通道数: {tomogram.shape[0]}")
print(f"  - slice数: {tomogram.shape[1]}")
print(f"  - 维度1: {tomogram.shape[2]}")
print(f"  - 维度2: {tomogram.shape[3]}")

# 检查目标区域
target_x = 19.0
target_y = -7.3

# 计算网格索引
col = int(round((target_x - center[0]) / resolution + offset[0]))
row = int(round((target_y - center[1]) / resolution + offset[1]))

print(f"\n目标位置: x={target_x}, y={target_y}")
print(f"网格索引: row={row}, col={col}")
print(f"offset: {offset}")

print(f"\n检查该位置各slice的数据:")
print("-" * 70)
print(f"{'Slice':>5} | {'理论高度':>10} | {'elev_g[s,row,col]':>15} | {'elev_g[s,col,row]':>15} | {'cost[s,row,col]':>15} | {'cost[s,col,row]':>15}")
print("-" * 70)

for s in range(n_slice):
    theoretical_h = slice_h0 + s * slice_dh
    
    # 尝试两种索引顺序
    try:
        elev_rc = elev_g_clean[s, row, col]
        cost_rc = trav[s, row, col]
    except IndexError:
        elev_rc = "IndexError"
        cost_rc = "IndexError"
    
    try:
        elev_cr = elev_g_clean[s, col, row]
        cost_cr = trav[s, col, row]
    except IndexError:
        elev_cr = "IndexError"
        cost_cr = "IndexError"
    
    print(f"{s:>5} | {theoretical_h:>10.2f} | {str(elev_rc):>15} | {str(elev_cr):>15} | {str(cost_rc):>15} | {str(cost_cr):>15}")

print("-" * 70)

# 检查 z=4.5 附近的实际数据
print(f"\n\n检查用户说的'z=4.5附近cost很小'的位置:")
print("=" * 70)

# 找到 elev_g 最接近 4.5 的 slice 和位置
print(f"\n在目标位置 ({target_x}, {target_y}) 查找 elev_g ≈ 4.5 的slice:")
for s in range(n_slice):
    elev = elev_g_clean[s, row, col]
    cost = trav[s, row, col]
    if 4.0 < elev < 5.0:
        print(f"  Slice {s}: elev_g={elev:.2f}, cost={cost:.1f}")

# 也检查 [s, col, row] 顺序
print(f"\n使用 [s, col, row] 索引顺序:")
for s in range(n_slice):
    try:
        elev = elev_g_clean[s, col, row]
        cost = trav[s, col, row]
        if 4.0 < elev < 5.0:
            print(f"  Slice {s}: elev_g={elev:.2f}, cost={cost:.1f}")
    except IndexError:
        pass

# 检查A*实际使用的索引方式
print(f"\n\n检查A*代码中的索引方式:")
print("=" * 70)
print("""
在 a_star_search.cc 中:
  grid_map_[layer][row][col] - 三维数组索引
  
在初始化时 (line 40-44):
  for (size_t j = 0; j < max_y_; ++j) {      // j = row
    for (size_t k = 0; k < max_x_; ++k) {    // k = col
      grid_map_[i][j][k] = Node(...)
      grid_map_[i][j][k].cost = cost_map(j + row_offset, k);
      
这里 cost_map(j + row_offset, k) 表示:
  - 第一个索引: j + row_offset (row方向)
  - 第二个索引: k (col方向)
  
但在Python中, 我们传入的 cost_map 的shape是什么？
""")

# 检查实际传入A*的数据格式
print(f"\n传入A*的数据格式:")
print(f"  trav.shape = {trav.shape}")
print(f"  解释: trav[slice, ?, ?]")
print(f"  维度: [{n_slice}, {trav.shape[1]}, {trav.shape[2]}]")

# 在planner_wrapper.py中，数据是怎么reshape的
print(f"\n在planner_wrapper.py中的reshape:")
print(f"  trav.reshape(-1, trav.shape[-1]) 后的shape = {trav.reshape(-1, trav.shape[-1]).shape}")
print(f"  这意味着: [{n_slice * trav.shape[1]}, {trav.shape[2]}]")

# 验证
print(f"\n验证 map_dim:")
print(f"  map_dim = {map_dim}")
print(f"  max_x_ (应该是 cost_map.cols()) = {trav.shape[2]}")
print(f"  max_y_ (应该是 cost_map.rows() / num_layers) = {trav.shape[1]}")

# 关键检查：tomogram维度顺序
print(f"\n\n关键检查: tomogram的维度顺序")
print("=" * 70)
print(f"tomogram.shape = {tomogram.shape}")
print(f"  [0]: 数据通道 (trav, grad_x, grad_y, elev_g, elev_c)")
print(f"  [1]: slice数量 = {tomogram.shape[1]}")
print(f"  [2]: ??? = {tomogram.shape[2]}")
print(f"  [3]: ??? = {tomogram.shape[3]}")

# 检查中心点附近不同位置的值
print(f"\n检查中心点附近的数据，确定维度顺序:")
center_row = offset[1]  # y方向的中心
center_col = offset[0]  # x方向的中心
print(f"  offset = {offset}")
print(f"  如果 tomogram[channel, slice, row, col]:")
print(f"    中心点: trav[0, {center_row}, {center_col}]")

# 检查A*传入的数据索引
print(f"\n\n检查实际A*中使用的索引:")
print("=" * 70)

# 从起点位置检查
start_x, start_y, start_z = 18.8, -6.79, 4.7
start_col = int(round((start_x - center[0]) / resolution + offset[0]))
start_row = int(round((start_y - center[1]) / resolution + offset[1]))

print(f"起点: ({start_x}, {start_y}, {start_z})")
print(f"网格: row={start_row}, col={start_col}")

print(f"\n使用 [slice, row, col] 索引:")
for s in range(n_slice):
    try:
        elev = elev_g_clean[s, start_row, start_col]
        cost = trav[s, start_row, start_col]
        if abs(elev - start_z) < 1.0:
            print(f"  Slice {s}: elev_g={elev:.2f} (差{abs(elev-start_z):.2f}m), cost={cost:.1f}")
    except IndexError as e:
        print(f"  Slice {s}: IndexError - {e}")

print(f"\n使用 [slice, col, row] 索引:")
for s in range(n_slice):
    try:
        elev = elev_g_clean[s, start_col, start_row]
        cost = trav[s, start_col, start_row]
        if abs(elev - start_z) < 1.0:
            print(f"  Slice {s}: elev_g={elev:.2f} (差{abs(elev-start_z):.2f}m), cost={cost:.1f}")
    except IndexError as e:
        print(f"  Slice {s}: IndexError - {e}")


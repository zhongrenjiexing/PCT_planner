#!/usr/bin/env python3
"""
验证A*中的索引问题
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
offset = np.array([int(map_dim[0] / 2), int(map_dim[1] / 2)], dtype=np.int32)

print("=" * 70)
print("验证索引问题")
print("=" * 70)

print(f"\ntomogram.shape = {tomogram.shape}")
print(f"trav.shape = {trav.shape}")
print(f"map_dim = {map_dim}")
print(f"offset = {offset}")
print(f"center = {center}")

# 目标位置
target_x, target_y = 19.0, -7.3

# 方法1：按照当前代码的方式计算索引
# pos2idx 中的计算
pos_2d = np.array([target_x, target_y]) - center[:2]
idx_raw = np.round(pos_2d / resolution).astype(np.int32) + offset
# pos2idx 返回 [idx[1], idx[0]]
idx_pos2idx = np.array([idx_raw[1], idx_raw[0]])

print(f"\n目标位置: ({target_x}, {target_y})")
print(f"pos_2d = {pos_2d}")
print(f"idx_raw (先x后y) = {idx_raw}")
print(f"pos2idx返回 (先y后x) = {idx_pos2idx}")

# 这意味着传给C++的 start_idx[1:] = [idx_raw[1], idx_raw[0]] = [y方向索引, x方向索引]
start_slice = 7
start_idx_cpp = np.array([start_slice, idx_pos2idx[0], idx_pos2idx[1]], dtype=np.int32)
print(f"\n传给C++的 start_idx = {start_idx_cpp}")
print(f"  start_idx[0] = layer = {start_idx_cpp[0]}")
print(f"  start_idx[1] = {start_idx_cpp[1]} (从pos2idx的第一个元素)")
print(f"  start_idx[2] = {start_idx_cpp[2]} (从pos2idx的第二个元素)")

# 在C++中访问 grid_map_[start[0]][start[2]][start[1]]
cpp_access_1 = start_idx_cpp[2]  # 第一个索引（除layer外）
cpp_access_2 = start_idx_cpp[1]  # 第二个索引（除layer外）

print(f"\nC++访问 grid_map_[{start_idx_cpp[0]}][{cpp_access_1}][{cpp_access_2}]")

# 在C++初始化时，grid_map_[layer][j][k].cost = cost_map(j + layer*max_y, k)
# cost_map 是从 trav.reshape(-1, trav.shape[-1]) 来的
# 所以 grid_map_[layer][j][k].cost = trav[layer, j, k]

# C++访问的是 grid_map_[layer][cpp_access_1][cpp_access_2]
# 对应 trav[layer, cpp_access_1, cpp_access_2]

print(f"\n这对应 trav[{start_idx_cpp[0]}, {cpp_access_1}, {cpp_access_2}]")
cost_cpp_gets = trav[start_idx_cpp[0], cpp_access_1, cpp_access_2]
print(f"C++获取的cost = {cost_cpp_gets}")

# 正确的访问应该是什么？
# 从之前的检查，trav[s, col, row] 给出正确的值
# col = x方向索引 = idx_raw[0]
# row = y方向索引 = idx_raw[1]

col = idx_raw[0]
row = idx_raw[1]
print(f"\n直接计算: col(x方向)={col}, row(y方向)={row}")
print(f"trav[7, col, row] = trav[7, {col}, {row}] = {trav[7, col, row]}")
print(f"trav[7, row, col] = trav[7, {row}, {col}] = {trav[7, row, col]}")

# 问题分析
print("\n" + "=" * 70)
print("问题分析:")
print("=" * 70)

print(f"""
1. pos2idx 返回 [{idx_pos2idx[0]}, {idx_pos2idx[1]}] = [y索引, x索引] = [row, col]

2. 传给C++的 start_idx = [layer, pos2idx[0], pos2idx[1]] = [layer, row, col]

3. C++访问 grid_map_[start[0]][start[2]][start[1]] 
   = grid_map_[layer][start[2]][start[1]]
   = grid_map_[layer][col][row]
   
4. 由于初始化时 grid_map_[layer][j][k].cost = trav[layer, j, k]
   所以 C++访问的是 trav[layer, col, row]

5. 从测试来看:
   - trav[7, col, row] = trav[7, {col}, {row}] = {trav[7, col, row]}  <-- C++获取的
   - trav[7, row, col] = trav[7, {row}, {col}] = {trav[7, row, col]}  <-- 如果不交换
""")

# 检查是否真的是这个问题
if cost_cpp_gets == trav[7, col, row]:
    print("✓ 确认: C++获取的值与 trav[layer, col, row] 一致")
else:
    print("✗ 不一致！")

if trav[7, col, row] < 35:
    print("✓ trav[7, col, row] < 35，说明该位置可通行")
else:
    print("✗ trav[7, col, row] >= 35，说明该位置不可通行")

if trav[7, row, col] < 35:
    print("✓ trav[7, row, col] < 35，说明该位置可通行")
else:
    print("✗ trav[7, row, col] >= 35，说明该位置不可通行（这是索引错误时获取的）")


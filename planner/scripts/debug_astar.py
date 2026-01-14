#!/usr/bin/env python3
"""
调试A*算法，检查它实际遍历的节点
"""

import os
import sys
import pickle
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))
sys.path.insert(0, os.path.join(script_dir, '../lib'))

from config import Config
from planner_wrapper import TomogramPlanner
import a_star

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
print("调试A*算法")
print("=" * 70)

# 起点和终点
start_pos = np.array([18.8, -6.79, 4.7], dtype=np.float32)
end_pos = np.array([19.3, -10.9, 4.7], dtype=np.float32)

# 执行规划
cfg = Config()
planner = TomogramPlanner(cfg)
planner.loadTomogram(tomo_file)

print(f"\n起点: {start_pos}")
print(f"终点: {end_pos}")

# 查看 planner 计算的索引
print(f"\n检查 planner 内部的索引计算:")
print(f"  center = {planner.center}")
print(f"  offset = {planner.offset}")
print(f"  resolution = {planner.resolution}")

# 手动计算 pos2idx
pos_2d = start_pos[:2] - planner.center[:2]
idx_raw = np.round(pos_2d / planner.resolution).astype(np.int32) + planner.offset
idx_returned = np.array([idx_raw[1], idx_raw[0]])  # pos2idx 返回的顺序

print(f"\n起点的索引计算:")
print(f"  pos_2d = {pos_2d}")
print(f"  idx_raw (x,y顺序) = {idx_raw}")
print(f"  pos2idx返回 (y,x顺序) = {idx_returned}")

# 计算传给C++的索引
# start_idx = [slice, pos2idx[0], pos2idx[1]] = [slice, y_idx, x_idx]
slice_idx = 7
start_idx_cpp = np.array([slice_idx, idx_returned[0], idx_returned[1]])
print(f"\n传给C++的 start_idx = {start_idx_cpp}")

# C++访问 grid_map_[start[0]][start[2]][start[1]]
# = grid_map_[slice][x_idx][y_idx]
cpp_dim1 = start_idx_cpp[2]  # start[2] = x_idx
cpp_dim2 = start_idx_cpp[1]  # start[1] = y_idx
print(f"C++访问 grid_map_[{slice_idx}][{cpp_dim1}][{cpp_dim2}]")
print(f"对应 trav[{slice_idx}, {cpp_dim1}, {cpp_dim2}] = {trav[slice_idx, cpp_dim1, cpp_dim2]:.4f}")

# 但是！让我检查 grid_map_ 的初始化
print(f"\n检查 grid_map_ 初始化时的对应关系:")
print(f"  在初始化循环中: grid_map_[layer][j][k].cost = cost_map(j + row_offset, k)")
print(f"  j 遍历 [0, max_y_) = [0, {map_dim[1]})")
print(f"  k 遍历 [0, max_x_) = [0, {map_dim[0]})")
print(f"  cost_map(j + layer*max_y, k) 对应 trav.reshape(-1, {trav.shape[2]})[j + layer*{map_dim[1]}, k]")

# 验证 reshape 的对应关系
trav_reshaped = trav.reshape(-1, trav.shape[-1])
print(f"\n  trav.shape = {trav.shape}")
print(f"  trav_reshaped.shape = {trav_reshaped.shape}")

# 验证: trav[layer, i, j] == trav_reshaped[layer*dim1 + i, j]
test_layer, test_i, test_j = 7, 100, 200
val1 = trav[test_layer, test_i, test_j]
val2 = trav_reshaped[test_layer * trav.shape[1] + test_i, test_j]
print(f"\n  验证: trav[{test_layer}, {test_i}, {test_j}] = {val1:.4f}")
print(f"         trav_reshaped[{test_layer}*{trav.shape[1]}+{test_i}, {test_j}] = {val2:.4f}")
print(f"         相等: {np.isclose(val1, val2)}")

# 现在检查初始化时的维度对应
print(f"\n关键问题: max_x_ 和 max_y_ 与 trav.shape 的对应关系")
print(f"  max_x_ = cost_map.cols() = trav_reshaped.shape[1] = {trav_reshaped.shape[1]}")
print(f"  max_y_ = cost_map.rows() / num_layers = {trav_reshaped.shape[0]} / {n_slice} = {trav_reshaped.shape[0] // n_slice}")

# 这意味着:
# max_x_ = trav.shape[2] = 500
# max_y_ = trav.shape[1] = 508
# grid_map_[layer][j][k].cost = trav[layer, j, k]
# 其中 j < max_y_ = 508, k < max_x_ = 500

# 在 Search 中访问 grid_map_[start[0]][start[2]][start[1]]
# start = [layer, row, col] (从Python传入，pos2idx返回[row, col])
# start[0] = layer
# start[1] = row
# start[2] = col
# 访问的是 grid_map_[layer][col][row]
# 对应 trav[layer, col, row]

print(f"\n在 C++ Search 中:")
print(f"  start = [layer, row, col] = [{slice_idx}, {idx_returned[0]}, {idx_returned[1]}]")
print(f"  访问 grid_map_[start[0]][start[2]][start[1]]")
print(f"       = grid_map_[{slice_idx}][{idx_returned[1]}][{idx_returned[0]}]")
print(f"  这对应 trav[{slice_idx}, {idx_returned[1]}, {idx_returned[0]}]")
print(f"       = {trav[slice_idx, idx_returned[1], idx_returned[0]]:.4f}")

# 但是这里有个问题！
# idx_returned = [row, col] = [y_idx, x_idx]
# 其中 row < 500 (y方向), col < 508 (x方向)
# 但 grid_map_[layer][j][k] 要求 j < max_y_ = 508, k < max_x_ = 500

# 如果 row=297, col=259
# 访问 grid_map_[7][259][297]
# 这要求 259 < 508 ✓, 297 < 500 ✓

# 对应 trav[7, 259, 297]
# 但正确的值应该是 trav[7, col, row] = trav[7, 259, 297]

print(f"\n再次验证索引:")
row_idx = idx_returned[0]  # y方向索引
col_idx = idx_returned[1]  # x方向索引
print(f"  row_idx (y方向) = {row_idx}, 范围应该 < {map_dim[1]} = {map_dim[1]}")
print(f"  col_idx (x方向) = {col_idx}, 范围应该 < {map_dim[0]} = {map_dim[0]}")

# 从前面的分析，正确的访问是 trav[slice, col, row]
# col 对应 tomogram 的第三维 (range [0, 508))
# row 对应 tomogram 的第四维 (range [0, 500))

# 等等，让我再看看 verify_index.py 的输出:
# idx_raw (先x后y) = [261 292]
# 这表示 x方向索引=261, y方向索引=292
# 正确的访问是 trav[7, 261, 292] = 0

# 但在当前计算中:
# col_idx = 259 (从 pos2idx 的第二个元素)
# row_idx = 297 (从 pos2idx 的第一个元素)

# 现在让我直接检查起点位置的 cost
print(f"\n直接检查起点位置的cost:")
print(f"  trav[{slice_idx}, {col_idx}, {row_idx}] = {trav[slice_idx, col_idx, row_idx]:.4f}")
print(f"  trav[{slice_idx}, {row_idx}, {col_idx}] = {trav[slice_idx, row_idx, col_idx]:.4f}")

# 关键：C++ 访问 grid_map_[layer][start[2]][start[1]] = grid_map_[layer][col_idx][row_idx]
# 对应 trav[layer, col_idx, row_idx]
print(f"\nC++实际访问的值: trav[{slice_idx}, {col_idx}, {row_idx}] = {trav[slice_idx, col_idx, row_idx]:.4f}")


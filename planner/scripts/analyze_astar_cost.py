#!/usr/bin/env python3
"""
模拟A*的成本计算，分析为什么会绕路
"""

import os
import sys
import pickle
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))
sys.path.insert(0, os.path.join(script_dir, '../lib'))

tomo_file = '2026-01-02_22-26-52_colorized_L567_FT_AddLift_AddPoints_XYZ'
tomo_dir = '../../rsc/tomogram/'

with open(tomo_dir + tomo_file + '.pickle', 'rb') as handle:
    data_dict = pickle.load(handle)
    tomogram = np.asarray(data_dict['data'], dtype=np.float32)
    trav = tomogram[0]
    elev_g = np.nan_to_num(tomogram[3], nan=-100)
    resolution = float(data_dict['resolution'])

print("模拟A*成本计算")
print("=" * 70)

# 起点和终点
start_j, start_k = 259, 296
end_j, end_k = 264, 256

# 实际路径上的关键点
actual_path_key_points = [
    (259, 296),  # 起点
    (265, 290),  # 开始偏离
    (279, 269),  # 最大偏离点
    (264, 256),  # 终点
]

# 直线路径上的采样点
straight_path = []
for i in range(50):
    t = i / 49
    j = int(round(start_j + t * (end_j - start_j)))
    k = int(round(start_k + t * (end_k - start_k)))
    straight_path.append((j, k))

# 计算路径成本的函数
def calc_path_cost(path, layer=7, step_cost_weight=0.2):
    """计算路径的 g 值（累积成本）"""
    total_g = 0
    for i in range(1, len(path)):
        prev_j, prev_k = path[i-1]
        curr_j, curr_k = path[i]
        
        # 获取高度
        prev_h = elev_g[layer, prev_j, prev_k]
        curr_h = elev_g[layer, curr_j, curr_k]
        
        # 计算 z 值（A*代码中使用 round）
        prev_z = round(prev_h / resolution)
        curr_z = round(curr_h / resolution)
        
        # 计算步进距离
        diff = np.array([curr_z - prev_z, curr_j - prev_j, curr_k - prev_k])
        step_dist = np.linalg.norm(diff)
        
        # 计算步进成本
        cost = trav[layer, curr_j, curr_k]
        step_cost = step_cost_weight * cost
        if step_cost < 5:
            step_cost = 0.0
        
        total_g += step_dist + step_cost
    
    return total_g

# 计算启发式值
def calc_heuristic(j1, k1, j2, k2, layer=7):
    """计算 Diagonal 启发式"""
    h1 = elev_g[layer, j1, k1]
    h2 = elev_g[layer, j2, k2]
    z1 = round(h1 / resolution)
    z2 = round(h2 / resolution)
    
    dx = abs(z1 - z2)
    dy = abs(j1 - j2)
    dz = abs(k1 - k2)
    
    dmin = min(dx, dy, dz)
    dmax = max(dx, dy, dz)
    dmid = dx + dy + dz - dmin - dmax
    
    h = np.sqrt(3) * dmin + np.sqrt(2) * (dmid - dmin) + (dmax - dmid)
    return h

# 分析直线路径
print("\n直线路径分析:")
print("-" * 70)
straight_g = calc_path_cost(straight_path)
straight_h = calc_heuristic(straight_path[-1][0], straight_path[-1][1], end_j, end_k)
print(f"直线路径 g 值: {straight_g:.2f}")
print(f"终点启发式值: {straight_h:.2f}")
print(f"直线路径 f 值: {straight_g + straight_h:.2f}")

# 构建类似实际绕路的路径
# 从起点到最大偏离点，再到终点
detour_path = []
# 第一段：从起点到最大偏离点 (259, 296) -> (279, 269)
for i in range(30):
    t = i / 29
    j = int(round(259 + t * (279 - 259)))
    k = int(round(296 + t * (269 - 296)))
    detour_path.append((j, k))

# 第二段：从最大偏离点到终点 (279, 269) -> (264, 256)
for i in range(20):
    t = i / 19
    j = int(round(279 + t * (264 - 279)))
    k = int(round(269 + t * (256 - 269)))
    detour_path.append((j, k))

print("\n绕路路径分析:")
print("-" * 70)
detour_g = calc_path_cost(detour_path)
detour_h = calc_heuristic(detour_path[-1][0], detour_path[-1][1], end_j, end_k)
print(f"绕路路径 g 值: {detour_g:.2f}")
print(f"终点启发式值: {detour_h:.2f}")
print(f"绕路路径 f 值: {detour_g + detour_h:.2f}")

print("\n比较:")
print("-" * 70)
print(f"直线 g: {straight_g:.2f}, 绕路 g: {detour_g:.2f}")
print(f"差异: {detour_g - straight_g:.2f} ({(detour_g/straight_g - 1)*100:.1f}% 更长)")

if detour_g < straight_g:
    print("\n*** 问题：绕路 g 值更小！这解释了为什么 A* 选择绕路 ***")
else:
    print("\n*** 绕路 g 值更大，A* 应该选择直线路径。问题可能在搜索过程中 ***")

# 检查 z 值差异的影响
print("\n\n检查 z 值差异的影响:")
print("-" * 70)

print("\n直线路径上的 z 值:")
for i in range(0, len(straight_path), 10):
    j, k = straight_path[i]
    h = elev_g[7, j, k]
    z = round(h / resolution)
    print(f"  ({j}, {k}): h={h:.4f}, z={z}")

print("\n绕路路径上的 z 值:")
for i in range(0, len(detour_path), 10):
    j, k = detour_path[i]
    h = elev_g[7, j, k]
    z = round(h / resolution)
    print(f"  ({j}, {k}): h={h:.4f}, z={z}")


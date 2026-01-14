#!/usr/bin/env python3
"""
完全模拟A*搜索，找出为什么会绕路
"""

import os
import sys
import pickle
import numpy as np
import heapq

tomo_file = '2026-01-02_22-26-52_colorized_L567_FT_AddLift_AddPoints_XYZ'
tomo_dir = '../../rsc/tomogram/'

with open(tomo_dir + tomo_file + '.pickle', 'rb') as handle:
    data_dict = pickle.load(handle)
    tomogram = np.asarray(data_dict['data'], dtype=np.float32)
    trav = tomogram[0]
    elev_g = np.nan_to_num(tomogram[3], nan=-100)
    resolution = 0.1

max_layers = 10
max_y, max_x = 508, 500
cost_threshold = 35
step_cost_weight = 0.2

# 起点和终点
start = (7, 259, 296)  # (layer, j, k)
goal = (7, 264, 256)

# 8方向邻居
neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

def get_height(layer, j, k):
    return elev_g[layer, j, k]

def get_cost(layer, j, k):
    return trav[layer, j, k]

def get_z(layer, j, k):
    return round(get_height(layer, j, k) / resolution)

def calc_heuristic(j1, k1, j2, k2, z1, z2):
    """Diagonal启发式"""
    dx = abs(z1 - z2)
    dy = abs(j1 - j2)
    dz = abs(k1 - k2)
    dmin = min(dx, dy, dz)
    dmax = max(dx, dy, dz)
    dmid = dx + dy + dz - dmin - dmax
    return np.sqrt(3) * dmin + np.sqrt(2) * (dmid - dmin) + (dmax - dmid)

def astar_search():
    """模拟A*搜索"""
    start_layer, start_j, start_k = start
    goal_layer, goal_j, goal_k = goal
    
    start_z = get_z(start_layer, start_j, start_k)
    goal_z = get_z(goal_layer, goal_j, goal_k)
    
    # g值和父节点
    g_score = {}
    parent = {}
    
    # 优先队列: (f, g, j, k, layer)
    open_set = []
    closed_set = set()
    
    start_h = calc_heuristic(start_j, start_k, goal_j, goal_k, start_z, goal_z)
    start_key = (start_layer, start_j, start_k)
    g_score[start_key] = 0
    heapq.heappush(open_set, (start_h, 0, start_j, start_k, start_layer))
    
    iteration = 0
    expanded_nodes = []
    
    while open_set:
        f, g, cur_j, cur_k, cur_layer = heapq.heappop(open_set)
        cur_key = (cur_layer, cur_j, cur_k)
        
        if cur_key in closed_set:
            continue
        
        closed_set.add(cur_key)
        iteration += 1
        
        # 记录前20个扩展的节点
        if iteration <= 20:
            expanded_nodes.append((cur_j, cur_k, g, f))
        
        # 到达目标
        if cur_j == goal_j and cur_k == goal_k:
            print(f"找到路径！迭代次数: {iteration}")
            print(f"路径成本 g: {g:.2f}")
            
            # 重建路径
            path = []
            key = cur_key
            while key in parent:
                path.append(key)
                key = parent[key]
            path.append(start_key)
            path.reverse()
            
            return path, expanded_nodes
        
        # 扩展邻居
        layer = cur_layer  # 简化：不使用DecideLayer
        cur_z = get_z(cur_layer, cur_j, cur_k)
        
        for dj, dk in neighbors:
            nj, nk = cur_j + dj, cur_k + dk
            
            if nj < 0 or nj >= max_y or nk < 0 or nk >= max_x:
                continue
            
            # 检查有效性和成本
            nh = get_height(layer, nj, nk)
            if nh < -50:  # 无效
                continue
            
            ncost = get_cost(layer, nj, nk)
            if ncost > cost_threshold:
                continue
            
            neighbor_key = (layer, nj, nk)
            
            # 计算步进成本
            nz = get_z(layer, nj, nk)
            diff = np.array([nz - cur_z, dj, dk])
            step_dist = np.linalg.norm(diff)
            step_cost = step_cost_weight * ncost
            if step_cost < 5:
                step_cost = 0.0
            
            tentative_g = g + step_dist + step_cost
            
            if neighbor_key in g_score and tentative_g >= g_score[neighbor_key]:
                continue
            
            g_score[neighbor_key] = tentative_g
            parent[neighbor_key] = cur_key
            
            nh_val = calc_heuristic(nj, nk, goal_j, goal_k, nz, goal_z)
            nf = tentative_g + nh_val
            
            heapq.heappush(open_set, (nf, tentative_g, nj, nk, layer))
    
    print("未找到路径")
    return None, expanded_nodes

print("模拟A*搜索")
print("=" * 70)
print(f"起点: (layer={start[0]}, j={start[1]}, k={start[2]})")
print(f"终点: (layer={goal[0]}, j={goal[1]}, k={goal[2]})")
print()

path, expanded = astar_search()

print()
print("前20个扩展的节点:")
print("-" * 70)
for i, (j, k, g, f) in enumerate(expanded):
    x = (j - 254) * resolution + 18.26
    y = (k - 250) * resolution + (-11.52)
    print(f"[{i+1:2d}] (j={j:3d}, k={k:3d}) -> ({x:6.2f}, {y:6.2f}), g={g:5.2f}, f={f:5.2f}")

if path:
    print()
    print("路径详情:")
    print("-" * 70)
    for i, (layer, j, k) in enumerate(path):
        x = (j - 254) * resolution + 18.26
        y = (k - 250) * resolution + (-11.52)
        if i < 10 or i >= len(path) - 5:
            print(f"[{i:2d}] ({x:6.2f}, {y:6.2f})")
        elif i == 10:
            print("...")
    
    # 检查路径是否接近直线
    total_len = 0
    max_dev = 0
    for i in range(1, len(path)):
        _, j1, k1 = path[i-1]
        _, j2, k2 = path[i]
        total_len += np.sqrt((j2-j1)**2 + (k2-k1)**2) * resolution
        
        t = i / (len(path) - 1)
        exp_j = start[1] + t * (goal[1] - start[1])
        exp_k = start[2] + t * (goal[2] - start[2])
        dev = np.sqrt((j2 - exp_j)**2 + (k2 - exp_k)**2) * resolution
        max_dev = max(max_dev, dev)
    
    direct_len = np.sqrt((goal[1]-start[1])**2 + (goal[2]-start[2])**2) * resolution
    
    print()
    print(f"直线距离: {direct_len:.2f}m")
    print(f"路径长度: {total_len:.2f}m")
    print(f"最大偏离: {max_dev:.2f}m")


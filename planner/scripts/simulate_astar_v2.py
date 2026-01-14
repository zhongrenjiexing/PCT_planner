#!/usr/bin/env python3
"""
加入DecideLayer逻辑的A*模拟
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
    ele = tomogram[4] if tomogram.shape[0] > 4 else np.zeros_like(trav)
    resolution = 0.1

max_layers = 10
max_y, max_x = 508, 500
cost_threshold = 35
step_cost_weight = 0.2

start = (7, 259, 296)
goal = (7, 264, 256)

neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

def get_height(layer, j, k):
    return elev_g[layer, j, k]

def get_cost(layer, j, k):
    return trav[layer, j, k]

def get_z(layer, j, k):
    return round(get_height(layer, j, k) / resolution)

def decide_layer(cur_layer, cur_j, cur_k, cur_height):
    """模拟C++中的DecideLayer函数"""
    best_layer = cur_layer
    min_height_diff = float('inf')
    found_valid = False
    
    # 检查当前层是否有效
    current_layer_invalid = get_height(cur_layer, cur_j, cur_k) < -50
    
    # 在所有层中找最佳层
    for layer in range(max_layers):
        layer_height = get_height(layer, cur_j, cur_k)
        if layer_height < -50:  # 无效
            continue
        
        height_diff = abs(layer_height - cur_height)
        
        should_update = False
        if current_layer_invalid:
            should_update = (height_diff < min_height_diff)
        else:
            should_update = (height_diff < min_height_diff and height_diff < 2.0)
        
        if should_update:
            min_height_diff = height_diff
            best_layer = layer
            found_valid = True
    
    if found_valid:
        true_layer = best_layer
    else:
        true_layer = cur_layer
    
    # 检查ele标记进行层切换（gateway）
    for offset in [-1, 0, 1]:
        test_layer = best_layer + offset
        if test_layer < 0 or test_layer >= max_layers:
            continue
        
        test_height = get_height(test_layer, cur_j, cur_k)
        if test_height < -50:
            continue
        
        # 简化：检查ele值
        ele_val = ele[test_layer, cur_j, cur_k]
        if abs(ele_val) > 0.5:
            # Gateway标记，可能切换层
            if ele_val > 0.5:
                true_layer = min(best_layer + 1, max_layers - 1)
            elif ele_val < -0.5:
                true_layer = max(best_layer - 1, 0)
            break
    
    return true_layer

def calc_heuristic(j1, k1, j2, k2, z1, z2):
    dx = abs(z1 - z2)
    dy = abs(j1 - j2)
    dz = abs(k1 - k2)
    dmin = min(dx, dy, dz)
    dmax = max(dx, dy, dz)
    dmid = dx + dy + dz - dmin - dmax
    return np.sqrt(3) * dmin + np.sqrt(2) * (dmid - dmin) + (dmax - dmid)

def astar_with_decide_layer():
    """模拟加入DecideLayer的A*搜索"""
    start_layer, start_j, start_k = start
    goal_layer, goal_j, goal_k = goal
    
    start_z = get_z(start_layer, start_j, start_k)
    goal_z = get_z(goal_layer, goal_j, goal_k)
    
    g_score = {}
    parent = {}
    
    open_set = []
    closed_set = set()
    
    start_h = calc_heuristic(start_j, start_k, goal_j, goal_k, start_z, goal_z)
    start_key = (start_layer, start_j, start_k)
    g_score[start_key] = 0
    heapq.heappush(open_set, (start_h, 0, start_j, start_k, start_layer))
    
    iteration = 0
    layer_changes = []
    
    while open_set:
        f, g, cur_j, cur_k, cur_layer = heapq.heappop(open_set)
        cur_key = (cur_layer, cur_j, cur_k)
        
        if cur_key in closed_set:
            continue
        
        closed_set.add(cur_key)
        iteration += 1
        
        if cur_j == goal_j and cur_k == goal_k:
            print(f"找到路径！迭代次数: {iteration}")
            print(f"路径成本 g: {g:.2f}")
            
            path = []
            key = cur_key
            while key in parent:
                path.append(key)
                key = parent[key]
            path.append(start_key)
            path.reverse()
            
            return path, layer_changes
        
        # 使用DecideLayer确定邻居的层
        cur_height = get_height(cur_layer, cur_j, cur_k)
        layer = decide_layer(cur_layer, cur_j, cur_k, cur_height)
        
        if layer != cur_layer and iteration <= 100:
            layer_changes.append((iteration, cur_j, cur_k, cur_layer, layer))
        
        cur_z = get_z(cur_layer, cur_j, cur_k)
        
        for dj, dk in neighbors:
            nj, nk = cur_j + dj, cur_k + dk
            
            if nj < 0 or nj >= max_y or nk < 0 or nk >= max_x:
                continue
            
            nh = get_height(layer, nj, nk)  # 使用DecideLayer返回的层
            if nh < -50:
                continue
            
            ncost = get_cost(layer, nj, nk)
            if ncost > cost_threshold:
                continue
            
            neighbor_key = (layer, nj, nk)  # 邻居的层是DecideLayer返回的
            
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
    return None, layer_changes

print("模拟带DecideLayer的A*搜索")
print("=" * 70)

path, layer_changes = astar_with_decide_layer()

if layer_changes:
    print()
    print(f"DecideLayer改变了层的次数: {len(layer_changes)}")
    print("前10次层切换:")
    for i, (iter, j, k, old_l, new_l) in enumerate(layer_changes[:10]):
        print(f"  迭代{iter}: ({j}, {k}) 从层{old_l}切换到层{new_l}")

if path:
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
    
    # 打印路径前后几个点
    print()
    print("路径前5个点:")
    for i, (layer, j, k) in enumerate(path[:5]):
        x = (j - 254) * resolution + 18.26
        y = (k - 250) * resolution + (-11.52)
        print(f"  [{i}] layer={layer}, ({x:.2f}, {y:.2f})")


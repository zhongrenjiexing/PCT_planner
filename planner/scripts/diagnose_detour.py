#!/usr/bin/env python3
"""
诊断脚本：深入分析A*路径规划在特定位置绕路的问题
目标：分析 (19.3, -9.89, 4.51) 附近的绕路原因
"""

import os
import sys
import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 设置正确的路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))
sys.path.insert(0, os.path.join(script_dir, '../lib'))

from config import Config
from planner_wrapper import TomogramPlanner
import a_star

# 设置
tomo_file = '2026-01-02_22-26-52_colorized_L567_FT_AddLift_AddPoints_XYZ'
tomo_dir = '../../rsc/tomogram/'

# 用户报告的问题点
problem_point = np.array([19.3, -9.89, 4.51])
# 测试用例
start_pos = np.array([18.8, -6.79, 4.7], dtype=np.float32)
end_pos = np.array([19.3, -10.9, 4.7], dtype=np.float32)

print("=" * 70)
print("诊断A*绕路问题")
print(f"问题位置: {problem_point}")
print(f"起点: {start_pos}")
print(f"终点: {end_pos}")
print("=" * 70)

# 加载tomogram数据
print("\n1. 加载tomogram数据...")
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

print(f"   分辨率: {resolution}m, 地图尺寸: {map_dim}, slice数量: {n_slice}")

# 提取数据层
trav = tomogram[0]  # 可通行性成本
elev_g = tomogram[3]  # 地面高度
elev_g_clean = np.nan_to_num(elev_g, nan=-100)
elev_c = tomogram[4]  # 天花板高度
elev_c_clean = np.nan_to_num(elev_c, nan=1e6)

def world_to_grid(pos, center, resolution, offset):
    """世界坐标转网格索引 (row, col)"""
    pos_2d = np.asarray(pos[:2], dtype=np.float32) - center[:2]
    idx = np.round(pos_2d / resolution).astype(np.int32) + offset
    return np.array([idx[1], idx[0]], dtype=np.int32)  # row, col

def grid_to_world(row, col, center, resolution, offset):
    """网格索引转世界坐标"""
    x = (col - offset[0]) * resolution + center[0]
    y = (row - offset[1]) * resolution + center[1]
    return np.array([x, y])

# 2. 分析问题点附近的数据
print("\n2. 分析问题点附近的tomogram数据...")
problem_grid = world_to_grid(problem_point, center, resolution, offset)
start_grid = world_to_grid(start_pos, center, resolution, offset)
end_grid = world_to_grid(end_pos, center, resolution, offset)

print(f"   问题点网格: row={problem_grid[0]}, col={problem_grid[1]}")
print(f"   起点网格: row={start_grid[0]}, col={start_grid[1]}")
print(f"   终点网格: row={end_grid[0]}, col={end_grid[1]}")

# 检查问题点周围区域
print("\n3. 检查问题点附近各slice的数据:")
search_radius = 5
for row_off in range(-search_radius, search_radius + 1, 2):
    for col_off in range(-search_radius, search_radius + 1, 2):
        row = problem_grid[0] + row_off
        col = problem_grid[1] + col_off
        if 0 <= row < map_dim[1] and 0 <= col < map_dim[0]:
            world_pos = grid_to_world(row, col, center, resolution, offset)
            print(f"\n   位置 ({world_pos[0]:.2f}, {world_pos[1]:.2f}), grid=({row}, {col}):")
            for s in range(n_slice):
                elev = elev_g_clean[s, row, col]
                ceiling = elev_c_clean[s, row, col]
                cost = trav[s, row, col]
                valid = elev > -50
                passable = cost < 35  # A*的cost阈值通常是35
                print(f"      slice {s}: elev_g={elev:7.2f}, elev_c={ceiling:7.2f}, "
                      f"cost={cost:6.1f}, valid={valid}, passable={passable}")

# 4. 执行A*规划并分析路径
print("\n4. 执行A*规划...")
cfg = Config()
planner = TomogramPlanner(cfg)
planner.loadTomogram(tomo_file)
traj_3d = planner.plan(start_pos, end_pos)

if traj_3d is None:
    print("   规划失败！")
else:
    print(f"   规划成功，轨迹点数: {len(traj_3d)}")
    
    # 获取A*原始路径
    path_finder: a_star.Astar = planner.planner.get_path_finder()
    path_raw = path_finder.get_result_matrix()  # [layer, row, col]
    
    print(f"\n5. 分析A*原始路径 ({len(path_raw)} 个点):")
    
    # 找出路径中最接近问题点的部分
    path_world = []
    for i in range(len(path_raw)):
        layer = int(path_raw[i, 0])
        row = int(path_raw[i, 1])
        col = int(path_raw[i, 2])
        world_pos = grid_to_world(row, col, center, resolution, offset)
        ground_h = elev_g_clean[layer, row, col]
        cost = trav[layer, row, col]
        path_world.append([world_pos[0], world_pos[1], ground_h, layer, row, col, cost])
    
    path_world = np.array(path_world)
    
    # 找到最接近问题点的路径段
    distances = np.sqrt((path_world[:, 0] - problem_point[0])**2 + 
                        (path_world[:, 1] - problem_point[1])**2)
    closest_idx = np.argmin(distances)
    
    print(f"\n   最接近问题点的路径点 (索引 {closest_idx}):")
    for i in range(max(0, closest_idx - 5), min(len(path_world), closest_idx + 6)):
        wp = path_world[i]
        dist = distances[i]
        marker = "***" if i == closest_idx else ""
        print(f"      [{i:3d}] world=({wp[0]:7.2f}, {wp[1]:7.2f}), z={wp[2]:6.2f}, "
              f"slice={int(wp[3])}, grid=({int(wp[4])}, {int(wp[5])}), "
              f"cost={wp[6]:5.1f}, dist={dist:.2f}m {marker}")
    
    # 检查路径中是否有大转弯
    print(f"\n6. 检测路径中的大转弯:")
    if len(path_world) > 2:
        angles = []
        for i in range(1, len(path_world) - 1):
            v1 = path_world[i, :2] - path_world[i-1, :2]
            v2 = path_world[i+1, :2] - path_world[i, :2]
            
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)
            
            if norm1 > 0 and norm2 > 0:
                cos_angle = np.dot(v1, v2) / (norm1 * norm2)
                cos_angle = np.clip(cos_angle, -1, 1)
                angle = np.arccos(cos_angle) * 180 / np.pi
                angles.append((i, angle))
                
                if angle > 45:  # 大于45度的转弯
                    wp = path_world[i]
                    print(f"      [{i:3d}] 转弯角度 {angle:5.1f}°, "
                          f"world=({wp[0]:7.2f}, {wp[1]:7.2f}), "
                          f"slice={int(wp[3])}")
    
    # 7. 检查直线路径上的障碍物
    print(f"\n7. 检查起点到终点直线路径上的障碍:")
    n_samples = 50
    for s in range(n_slice):
        print(f"\n   Slice {s}:")
        blocked_points = []
        for t in np.linspace(0, 1, n_samples):
            row = int(start_grid[0] + t * (end_grid[0] - start_grid[0]))
            col = int(start_grid[1] + t * (end_grid[1] - start_grid[1]))
            if 0 <= row < map_dim[1] and 0 <= col < map_dim[0]:
                cost = trav[s, row, col]
                elev = elev_g_clean[s, row, col]
                if cost >= 35 or elev < -50:
                    world_pos = grid_to_world(row, col, center, resolution, offset)
                    blocked_points.append((world_pos[0], world_pos[1], cost, elev))
        
        if blocked_points:
            print(f"      障碍点数: {len(blocked_points)}/{n_samples}")
            for bp in blocked_points[:5]:  # 只显示前5个
                print(f"         ({bp[0]:.2f}, {bp[1]:.2f}): cost={bp[2]:.1f}, elev={bp[3]:.2f}")
        else:
            print(f"      无障碍 (所有点cost<35且elev>-50)")

    # 8. 可视化
    print(f"\n8. 生成可视化图...")
    fig = plt.figure(figsize=(18, 12))
    
    # 8.1 XY平面路径
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(path_world[:, 0], path_world[:, 1], 'g.-', linewidth=2, markersize=3, 
             label='A*路径', alpha=0.7)
    ax1.plot(start_pos[0], start_pos[1], 'go', markersize=12, label='起点')
    ax1.plot(end_pos[0], end_pos[1], 'ro', markersize=12, label='终点')
    ax1.plot(problem_point[0], problem_point[1], 'rx', markersize=15, 
             markeredgewidth=3, label='问题点')
    ax1.plot([start_pos[0], end_pos[0]], [start_pos[1], end_pos[1]], 
             'k--', alpha=0.5, linewidth=2, label='直线路径')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('XY平面路径')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 8.2 显示问题区域附近的cost地图
    ax2 = fig.add_subplot(2, 2, 2)
    
    # 只显示问题区域附近
    margin = 30
    row_min = max(0, problem_grid[0] - margin)
    row_max = min(map_dim[1], problem_grid[0] + margin)
    col_min = max(0, problem_grid[1] - margin)
    col_max = min(map_dim[0], problem_grid[1] + margin)
    
    # 使用路径经过的主要slice
    main_slice = int(np.median(path_world[:, 3]))
    cost_region = trav[main_slice, row_min:row_max, col_min:col_max]
    
    # 转换坐标用于显示
    extent = [
        (col_min - offset[0]) * resolution + center[0],
        (col_max - offset[0]) * resolution + center[0],
        (row_min - offset[1]) * resolution + center[1],
        (row_max - offset[1]) * resolution + center[1]
    ]
    
    im = ax2.imshow(cost_region.T, origin='lower', extent=extent,
                    cmap='RdYlGn_r', vmin=0, vmax=50, alpha=0.8)
    
    ax2.plot(path_world[:, 0], path_world[:, 1], 'b.-', linewidth=2, markersize=3,
             label='A*路径', alpha=0.9)
    ax2.plot(start_pos[0], start_pos[1], 'go', markersize=10)
    ax2.plot(end_pos[0], end_pos[1], 'ro', markersize=10)
    ax2.plot(problem_point[0], problem_point[1], 'rx', markersize=15, markeredgewidth=3)
    
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title(f'Slice {main_slice} 成本地图 (问题区域)')
    plt.colorbar(im, ax=ax2, label='Cost')
    
    # 8.3 slice变化
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(path_world[:, 3], 'b.-', linewidth=2, markersize=3)
    ax3.axhline(y=np.median(path_world[:, 3]), color='r', linestyle='--', 
                alpha=0.5, label=f'中位数slice={int(np.median(path_world[:, 3]))}')
    ax3.set_xlabel('路径点索引')
    ax3.set_ylabel('Slice索引')
    ax3.set_title('路径的slice变化')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 8.4 高度变化
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(path_world[:, 2], 'b.-', linewidth=2, markersize=3, label='A*路径高度')
    ax4.axhline(y=start_pos[2], color='g', linestyle='--', alpha=0.5, label='起点z')
    ax4.axhline(y=end_pos[2], color='r', linestyle='--', alpha=0.5, label='终点z')
    ax4.axhline(y=problem_point[2], color='orange', linestyle='--', alpha=0.5, 
                label='问题点z')
    ax4.set_xlabel('路径点索引')
    ax4.set_ylabel('高度 (m)')
    ax4.set_title('路径高度变化')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/cjsg/PCT_planner/diagnose_detour.png', dpi=150)
    print(f"   可视化图已保存到: /home/cjsg/PCT_planner/diagnose_detour.png")

print("\n" + "=" * 70)
print("可能的问题原因分析:")
print("=" * 70)
print("""
1. DecideLayer函数的bug：
   - 在a_star_search.cc中，DecideLayer使用当前节点的位置(i,j)来决定layer
   - 但这个layer被用于访问所有邻居节点
   - 如果当前位置和邻居位置在不同layer上的情况不同，会导致错误的层选择

2. 邻居节点的层使用了错误的位置决策：
   - Search函数中: int layer = DecideLayer(current_node);
   - 然后所有邻居都使用这个layer: grid_map_[layer][i][j]
   - 应该为每个邻居单独决定合适的层

3. 检查GetHash函数：
   - hash = idx[0] * 10000000 + idx[1] * max_x_ + idx[2]
   - idx[0]是高度/分辨率，不是layer索引
   - 这可能导致不同layer相同位置的节点有不同hash，重复评估
""")


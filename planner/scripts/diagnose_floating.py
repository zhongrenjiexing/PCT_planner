#!/usr/bin/env python3
"""
诊断脚本：检查A*规划轨迹浮空的问题
"""

import sys
import pickle
import numpy as np
import matplotlib.pyplot as plt

sys.path.append('../')
from config import Config

# 设置
tomo_file = '567_-ready-to-pctplanner'
tomo_dir = '../../rsc/tomogram/'

print("=" * 60)
print("加载 tomogram 数据...")
print("=" * 60)

with open(tomo_dir + tomo_file + '.pickle', 'rb') as handle:
    data_dict = pickle.load(handle)
    
    tomogram = np.asarray(data_dict['data'], dtype=np.float32)
    resolution = float(data_dict['resolution'])
    center = np.asarray(data_dict['center'], dtype=np.double)
    n_slice = tomogram.shape[1]
    slice_h0 = float(data_dict['slice_h0'])
    slice_dh = float(data_dict['slice_dh'])
    map_dim = [tomogram.shape[2], tomogram.shape[3]]

print(f"\n基本信息:")
print(f"  - 分辨率: {resolution} m")
print(f"  - 地图中心: {center}")
print(f"  - 切片数量: {n_slice}")
print(f"  - 切片起始高度: {slice_h0} m")
print(f"  - 切片高度间隔: {slice_dh} m")
print(f"  - 地图尺寸: {map_dim[0]} x {map_dim[1]}")

# 提取各层数据
trav = tomogram[0]  # 可通行性成本
trav_gx = tomogram[1]  # x方向梯度
trav_gy = tomogram[2]  # y方向梯度
elev_g = tomogram[3]  # 地面高度
elev_c = tomogram[4]  # 天花板高度

# 处理NaN值
elev_g_clean = np.nan_to_num(elev_g, nan=-100)
elev_c_clean = np.nan_to_num(elev_c, nan=1e6)

print(f"\n地面高度统计 (elev_g):")
for i in range(n_slice):
    valid_mask = elev_g_clean[i] > -50  # 有效地面数据
    if np.any(valid_mask):
        valid_heights = elev_g_clean[i][valid_mask]
        print(f"  Slice {i}: 有效点数={np.sum(valid_mask)}, "
              f"高度范围=[{np.min(valid_heights):.2f}, {np.max(valid_heights):.2f}], "
              f"平均={np.mean(valid_heights):.2f}")
    else:
        print(f"  Slice {i}: 无有效数据")

print(f"\n可通行性成本统计 (trav):")
for i in range(n_slice):
    valid_cost = trav[i][trav[i] > 0]
    passable = trav[i][trav[i] < 35]  # 成本阈值35
    print(f"  Slice {i}: 最小成本={np.min(trav[i]):.1f}, "
          f"最大成本={np.max(trav[i]):.1f}, "
          f"可通行点数={len(passable)} ({len(passable)/trav[i].size*100:.1f}%)")

# 检查高度连续性
print(f"\n检查相邻slice之间的高度连续性:")
for i in range(n_slice - 1):
    valid_mask = (elev_g_clean[i] > -50) & (elev_g_clean[i+1] > -50)
    if np.any(valid_mask):
        height_diff = np.abs(elev_g_clean[i+1][valid_mask] - elev_g_clean[i][valid_mask])
        large_gaps = np.sum(height_diff > 0.3)  # A*的高度约束是0.3m
        if large_gaps > 0:
            print(f"  Slice {i} -> {i+1}: {large_gaps} 个位置高度差>0.3m "
                  f"(最大差={np.max(height_diff):.2f}m)")

# 检查特定测试路径
print(f"\n检查测试路径的起点和终点:")
start_pos = np.array([-0.68, -1.0, 0.1], dtype=np.float32)
end_pos = np.array([21.6, -29.3, -4.0], dtype=np.float32)

def pos2idx(pos, center, resolution, map_dim):
    """将世界坐标转换为地图索引"""
    offset = np.array([int(map_dim[0] / 2), int(map_dim[1] / 2)], dtype=np.int32)
    pos_2d = np.asarray(pos[:2], dtype=np.float32) - center[:2]
    idx = np.round(pos_2d / resolution).astype(np.int32) + offset
    idx = np.array([idx[1], idx[0]], dtype=np.int32)  # [y, x]
    return idx

start_idx = pos2idx(start_pos, center, resolution, map_dim)
end_idx = pos2idx(end_pos, center, resolution, map_dim)

print(f"\n起点: {start_pos} -> 地图索引 {start_idx}")
print(f"  检查各层数据:")
for i in range(n_slice):
    y_idx, x_idx = start_idx
    if 0 <= x_idx < map_dim[0] and 0 <= y_idx < map_dim[1]:
        try:
            elev_g_val = elev_g_clean[i, x_idx, y_idx]
            elev_c_val = elev_c_clean[i, x_idx, y_idx]
            trav_val = trav[i, x_idx, y_idx]
            in_range = (elev_g_val <= start_pos[2] <= elev_c_val) if elev_g_val > -50 else False
            print(f"    Slice {i}: elev_g={elev_g_val:.2f}, elev_c={elev_c_val:.2f}, "
                  f"trav={trav_val:.1f}, 起点z在范围内={in_range}")
        except:
            print(f"    Slice {i}: 索引错误")

print(f"\n终点: {end_pos} -> 地图索引 {end_idx}")
print(f"  检查各层数据:")
for i in range(n_slice):
    y_idx, x_idx = end_idx
    if 0 <= x_idx < map_dim[0] and 0 <= y_idx < map_dim[1]:
        try:
            elev_g_val = elev_g_clean[i, x_idx, y_idx]
            elev_c_val = elev_c_clean[i, x_idx, y_idx]
            trav_val = trav[i, x_idx, y_idx]
            in_range = (elev_g_val <= end_pos[2] <= elev_c_val) if elev_g_val > -50 else False
            print(f"    Slice {i}: elev_g={elev_g_val:.2f}, elev_c={elev_c_val:.2f}, "
                  f"trav={trav_val:.1f}, 终点z在范围内={in_range}")
        except:
            print(f"    Slice {i}: 索引错误")

# 可视化地面高度的一个slice
print(f"\n生成可视化图...")
fig, axes = plt.subplots(2, 2, figsize=(15, 12))

# 1. 显示第一层的地面高度
ax = axes[0, 0]
elev_g_vis = np.copy(elev_g_clean[0])
elev_g_vis[elev_g_vis <= -50] = np.nan
im = ax.imshow(elev_g_vis.T, origin='lower', cmap='terrain')
ax.plot(start_idx[1], start_idx[0], 'go', markersize=10, label='起点')
ax.plot(end_idx[1], end_idx[0], 'ro', markersize=10, label='终点')
ax.set_title(f'Slice 0 地面高度 (elev_g)')
ax.legend()
plt.colorbar(im, ax=ax, label='高度 (m)')

# 2. 显示第一层的可通行性
ax = axes[0, 1]
trav_vis = np.copy(trav[0])
trav_vis[trav_vis > 100] = 100
im = ax.imshow(trav_vis.T, origin='lower', cmap='RdYlGn_r', vmin=0, vmax=50)
ax.plot(start_idx[1], start_idx[0], 'go', markersize=10, label='起点')
ax.plot(end_idx[1], end_idx[0], 'ro', markersize=10, label='终点')
ax.axhline(y=start_idx[0], color='g', alpha=0.3, linestyle='--')
ax.axhline(y=end_idx[0], color='r', alpha=0.3, linestyle='--')
ax.set_title(f'Slice 0 可通行性成本 (trav)')
ax.legend()
plt.colorbar(im, ax=ax, label='成本')

# 3. 沿着路径方向的剖面图
ax = axes[1, 0]
# 简单线性插值路径
n_samples = 100
path_x = np.linspace(start_idx[1], end_idx[1], n_samples).astype(int)
path_y = np.linspace(start_idx[0], end_idx[0], n_samples).astype(int)
path_x = np.clip(path_x, 0, map_dim[0] - 1)
path_y = np.clip(path_y, 0, map_dim[1] - 1)

for i in range(min(n_slice, 3)):  # 只显示前3层
    heights = elev_g_clean[i, path_x, path_y]
    heights[heights <= -50] = np.nan
    ax.plot(heights, label=f'Slice {i}', alpha=0.7)

ax.axhline(y=start_pos[2], color='g', linestyle='--', alpha=0.5, label='起点高度')
ax.axhline(y=end_pos[2], color='r', linestyle='--', alpha=0.5, label='终点高度')
ax.set_xlabel('路径采样点')
ax.set_ylabel('高度 (m)')
ax.set_title('沿路径的地面高度剖面')
ax.legend()
ax.grid(True, alpha=0.3)

# 4. 高度差统计
ax = axes[1, 1]
height_diffs = []
for i in range(n_slice - 1):
    valid_mask = (elev_g_clean[i] > -50) & (elev_g_clean[i+1] > -50)
    if np.any(valid_mask):
        diffs = np.abs(elev_g_clean[i+1][valid_mask] - elev_g_clean[i][valid_mask])
        height_diffs.append(diffs)

if height_diffs:
    ax.hist(np.concatenate(height_diffs), bins=50, alpha=0.7)
    ax.axvline(x=0.2, color='orange', linestyle='--', label='DecideLayer阈值 (0.2m)')
    ax.axvline(x=0.3, color='r', linestyle='--', label='A*邻居阈值 (0.3m)')
    ax.set_xlabel('相邻slice高度差 (m)')
    ax.set_ylabel('频数')
    ax.set_title('相邻slice之间的高度差分布')
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/home/cjsg/PCT_planner/diagnose_floating.png', dpi=150)
print(f"可视化图已保存到: /home/cjsg/PCT_planner/diagnose_floating.png")

print("\n" + "=" * 60)
print("可能的问题原因:")
print("=" * 60)
print("1. 高度约束过严: A*在搜索时要求相邻节点高度差<0.3m (a_star_search.cc:152)")
print("2. 层切换逻辑: DecideLayer函数要求同位置不同层高度差<0.2m才切换 (a_star_search.cc:214)")
print("3. tomogram数据: 如果某些slice的地面高度不连续，会导致路径浮空")
print("4. 初始化问题: height_map传入A*时可能有误差")
print("\n建议:")
print("1. 检查tomogram质量，特别是浮空路径对应的区域")
print("2. 放宽A*的高度约束阈值")
print("3. 改进层切换逻辑，使用更智能的策略")
print("4. 增加调试输出，记录每个规划步骤的slice和高度")


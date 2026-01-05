# -*- coding: utf-8 -*-
import os
import sys
import pickle
import numpy as np

# Set LD_LIBRARY_PATH for GTSAM libraries before importing lib modules
# This must be done before importing any modules that depend on GTSAM
rsg_root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../..'))
gtsam_lib_path = os.path.join(rsg_root, 'planner/lib/3rdparty/gtsam-4.1.1/install/lib')
smoothing_lib_path = os.path.join(rsg_root, 'planner/lib/build/src/common/smoothing')

# Update LD_LIBRARY_PATH - this must be set before any dynamic library imports
if 'LD_LIBRARY_PATH' in os.environ:
    os.environ['LD_LIBRARY_PATH'] = f"{gtsam_lib_path}:{smoothing_lib_path}:" + os.environ['LD_LIBRARY_PATH']
else:
    os.environ['LD_LIBRARY_PATH'] = f"{gtsam_lib_path}:{smoothing_lib_path}"

# For Linux, we also need to ensure the library loader can find the libraries
# by using ctypes to preload the library search paths
if sys.platform.startswith('linux'):
    try:
        import ctypes
        # Preload libmetis-gtsam.so to ensure it's available
        metis_lib_path = os.path.join(gtsam_lib_path, 'libmetis-gtsam.so')
        if os.path.exists(metis_lib_path):
            ctypes.CDLL(metis_lib_path, mode=ctypes.RTLD_GLOBAL)
    except Exception as e:
        # If preloading fails, continue anyway - the import might still work
        pass

from utils import *

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt


class TomogramPlanner(object):
    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.elev_g = None  # 地面高度 [n_slice, map_dim_x, map_dim_y]
        self.elev_c = None  # 天花板高度 [n_slice, map_dim_x, map_dim_y]

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    def loadTomogram(self, tomo_file):
        with open(self.tomo_dir + tomo_file + '.pickle', 'rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        elev_g = tomogram[3]
        elev_g = np.nan_to_num(elev_g, nan=-100)
        elev_c = tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)

        # 保存 elev_g 和 elev_c 用于查找有效 slice
        self.elev_g = elev_g
        self.elev_c = elev_c

        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        self.planner.init_map(
            20, 15, self.resolution, self.n_slice, 0.2,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )

    def plan(self, start_pos, end_pos, start_z=None, end_z=None):
        """
        start_pos/end_pos 支持 2D(x, y) 或 3D(x, y, z)，也可以通过 start_z/end_z 额外指定高度。
        slice 选取：根据给定位置的 z 坐标，查找在 elev_g 和 elev_c 范围内的有效 slice。
        未提供 z 时保持兼容，默认使用 0 层。
        """
        start_z_val = self._extract_z(start_pos, start_z)
        end_z_val = self._extract_z(end_pos, end_z)

        self.start_idx[0] = self.findValidSlice(start_pos, start_z_val)
        self.end_idx[0] = self.findValidSlice(end_pos, end_z_val)
        self.start_idx[1:] = self.pos2idx(start_pos)
        self.end_idx[1:] = self.pos2idx(end_pos)

        self.planner.plan(self.start_idx, self.end_idx, True)
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d

    def get_raw_path(self):
        """
        获取 A* 算法找到的原始路径的 3D 坐标版本
        """
        if not hasattr(self, 'start_idx') or not hasattr(self, 'end_idx'):
            return None

        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        # 构建原始轨迹
        # path格式: [layer, y, x]，需要转换为[x, y, z]格式给transTrajGrid2Map
        # transTrajGrid2Map期望输入格式: [x, y, z]

        # 首先将路径坐标转换为地图坐标（只转换x,y，高度后面设置）
        temp_traj = np.stack([path[:, 2], path[:, 1], np.zeros(len(path))], axis=1)  # x, y, z(临时)
        raw_traj_map = transTrajGrid2Map(self.map_dim, self.center, self.resolution, temp_traj)

        # 使用地图坐标来查询高度信息
        z_coords = np.zeros(len(path))
        for i, (layer_idx, grid_y, grid_x) in enumerate(path.astype(int)):
            # 将地图坐标转换回网格坐标来查询elev_g
            map_x, map_y = raw_traj_map[i, 0], raw_traj_map[i, 1]

            # 转换回网格索引
            pos = np.array([map_x, map_y]) - self.center[:2]
            idx = np.round(pos / self.resolution).astype(int) + self.offset
            grid_x_check = idx[1]  # x索引
            grid_y_check = idx[0]  # y索引

            # 确保索引在有效范围内
            if (0 <= layer_idx < self.elev_g.shape[0] and
                0 <= grid_y_check < self.elev_g.shape[1] and
                0 <= grid_x_check < self.elev_g.shape[2]):
                # 使用地面高度作为z坐标
                z_coords[i] = self.elev_g[layer_idx, grid_y_check, grid_x_check]
            else:
                # 如果索引超出范围，使用基于层的近似值
                z_coords[i] = self.slice_h0 + layer_idx * self.slice_dh

        raw_traj_3d = np.stack([raw_traj_map[:, 0], raw_traj_map[:, 1], z_coords], axis=1)

        return raw_traj_3d
    
    def pos2idx(self, pos):
        pos = np.asarray(pos[:2], dtype=np.float32) - self.center[:2]
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        idx = np.array([idx[1], idx[0]], dtype=np.float32)
        return idx

    def findValidSlice(self, pos, z):
        """
        根据给定的位置 (x, y) 和高度 z，找到该点应该位于的有效 slice。
        检查每个 slice 在 (x, y) 位置的 elev_g 和 elev_c，找到 z 在 [elev_g, elev_c] 范围内的 slice。
        如果有多个 slice 满足条件，选择最接近 z 的 slice。
        如果 z 未提供或没有找到有效 slice，返回基于 slice_h0 和 slice_dh 计算的 slice。
        
        注意：elev_g 和 elev_c 的形状是 [n_slice, map_dim_x, map_dim_y]
        """
        if z is None:
            return 0

        # 将 (x, y) 转换为地图索引
        # pos 是 [x, y] 或 [x, y, z]
        pos_2d = np.asarray(pos[:2], dtype=np.float32)
        pos_grid = pos_2d - self.center[:2]
        idx = np.round(pos_grid / self.resolution).astype(np.int32) + self.offset
        # idx 是 [x_idx, y_idx]，对应 map_dim[0] 和 map_dim[1]
        x_idx = int(np.clip(idx[0], 0, self.map_dim[0] - 1))
        y_idx = int(np.clip(idx[1], 0, self.map_dim[1] - 1))

        z = float(z)
        valid_slices = []
        
        # 调试信息
        print(f"[DEBUG] findValidSlice: pos={pos[:2]}, z={z}")
        print(f"[DEBUG] x_idx={x_idx}, y_idx={y_idx}, map_dim={self.map_dim}")
        print(f"[DEBUG] elev_g shape: {self.elev_g.shape}, elev_c shape: {self.elev_c.shape}")
        print(f"[DEBUG] slice_h0={self.slice_h0}, slice_dh={self.slice_dh}, n_slice={self.n_slice}")
        
        # 遍历所有 slice，查找 z 在 [elev_g, elev_c] 范围内的 slice
        # elev_g 和 elev_c 的形状是 [n_slice, map_dim_x, map_dim_y]
        # 注意：需要尝试两种索引方式，因为可能索引顺序不同
        for slice_idx in range(self.n_slice):
            try:
                # 尝试第一种索引方式：[slice, x, y]
                elev_g_val = self.elev_g[slice_idx, x_idx, y_idx]
                elev_c_val = self.elev_c[slice_idx, x_idx, y_idx]
            except IndexError:
                try:
                    # 尝试第二种索引方式：[slice, y, x]
                    elev_g_val = self.elev_g[slice_idx, y_idx, x_idx]
                    elev_c_val = self.elev_c[slice_idx, y_idx, x_idx]
                except IndexError as e:
                    print(f"[DEBUG] IndexError at slice_idx={slice_idx}, x_idx={x_idx}, y_idx={y_idx}: {e}")
                    continue
            
            # 检查是否是有效值
            # elev_g 的默认值是 -100，如果接近 -100 说明没有有效数据
            # elev_c 的默认值是 1e6，如果接近 1e6 表示没有天花板限制（仍然有效）
            is_valid_g = elev_g_val > -50  # 允许一些容差
            has_ceiling = elev_c_val < 1e5  # 是否有明确的天花板限制
            
            # 调试信息：打印每个 slice 的高度范围
            if slice_idx < self.n_slice:  # 打印所有 slice
                print(f"[DEBUG] slice {slice_idx}: elev_g={elev_g_val:.2f} (valid={is_valid_g}), elev_c={elev_c_val:.2f} (has_ceiling={has_ceiling}), z={z:.2f}")
            
            # 只有当 elev_g 有效时才检查范围
            if is_valid_g:
                # 检查 z 是否在有效范围内
                # 如果有天花板限制，检查 z <= elev_c；如果没有天花板限制，只要 z >= elev_g 即可
                if has_ceiling:
                    # 有明确的天花板限制
                    if elev_g_val <= z <= elev_c_val:
                        center_height = (elev_g_val + elev_c_val) / 2
                        valid_slices.append((slice_idx, abs(z - center_height)))
                        print(f"[DEBUG] Found valid slice {slice_idx}: elev_g={elev_g_val:.2f}, elev_c={elev_c_val:.2f}, center={center_height:.2f}")
                else:
                    # 没有天花板限制，只要 z >= elev_g 即可
                    if z >= elev_g_val:
                        # 使用 elev_g 作为参考高度来计算距离
                        valid_slices.append((slice_idx, abs(z - elev_g_val)))
                        print(f"[DEBUG] Found valid slice {slice_idx}: elev_g={elev_g_val:.2f}, elev_c=inf (no ceiling), z={z:.2f}")
        
        # 如果找到有效 slice，选择最接近 z 的
        if valid_slices:
            # 按距离排序，选择最接近的
            valid_slices.sort(key=lambda x: x[1])
            print(f"[DEBUG] Selected slice {valid_slices[0][0]} from {len(valid_slices)} valid slices")
            return valid_slices[0][0]
        
        # 如果没有找到有效 slice，使用基于 slice_h0 和 slice_dh 的近似计算
        print(f"[DEBUG] No valid slice found, using approximate calculation")
        print(f"[DEBUG] slice_h0={self.slice_h0}, slice_dh={self.slice_dh}")
        slice_idx = np.round((z - self.slice_h0) / self.slice_dh)
        slice_idx = int(np.clip(slice_idx, 0, self.n_slice - 1))
        print(f"[DEBUG] Approximate slice_idx={slice_idx}")
        return slice_idx

    def height2slice(self, z):
        """
        将高度映射到切片索引，按最近邻选择并限制在 [0, n_slice-1]。
        如果 z 未提供则返回 0 以保持旧行为。
        注意：此方法已弃用，请使用 findValidSlice(pos, z) 来获取更准确的 slice。
        """
        if z is None:
            return 0

        slice_idx = np.round((float(z) - self.slice_h0) / self.slice_dh)
        slice_idx = int(np.clip(slice_idx, 0, self.n_slice - 1))
        return slice_idx

    def _extract_z(self, pos, z_override):
        if z_override is not None:
            return z_override
        pos = np.asarray(pos)
        if pos.shape[0] >= 3:
            return pos[2]
        return None
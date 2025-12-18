from .scene import ScenePCD, SceneMap, SceneTrav


class SceneSiiL6():
    pcd = ScenePCD()
    # PCD only need to change here
    # pcd.file_name = 'scans_0908_L10L9L8_leveled_clean.pcd'
    # pcd.file_name = '6l7l_mid360_leveled_clear.ply'
    '''
    给PCT-planner的点云要密集(不subsample)，手动剔除大片噪声
    新添加点云只添加对应区域，否则易引入噪声点。
    '''
    pcd.file_name = '6L7L_Add613_-ready-to-pctplanner.ply' 
    # pcd.file_name = '6L7L_Add613_align_ds.ply' 

    map = SceneMap()
    map.resolution = 0.1 # 0.1 good, why?
    map.ground_h = 0.05
    map.slice_dh = 0.6 # >1.2 玻璃门ok, <0.7 栏杆ok

    trav = SceneTrav()
    trav.kernel_size = 1 # 可通行性计算的核大小。1 表示使用 1×1 的核（默认通常是 7，表示 7×7）
    trav.interval_min = 1.5 # 1.5 最小垂直间隔（米）。如果地面到天花板的高度小于此值，视为不可通行
    trav.interval_free = 0.65 # 0.65 自由间隔（米）。用于计算通行成本，间隔越大成本越低
    trav.slope_max = 0.5 # 最大坡度（弧度或比例）。用于计算可站立的高度差阈值
    trav.step_max = 0.2 # 最大步高（米）。机器人可跨越的最大高度差
    trav.standable_ratio = 0.40 # 可站立比例。核内需要至少 40% 的网格是可站立的才视为可通行
    trav.cost_barrier = 50 # 50.0 成本障碍值。通行成本小于此值的区域被视为不可通行
    trav.safe_margin = 0.05 # 1.2 # 安全边距（米）。用于路径膨胀，确保路径与障碍物保持安全距离
    trav.inflation = 0.1 # 膨胀半径（米）。在障碍物周围膨胀的距离，用于路径规划


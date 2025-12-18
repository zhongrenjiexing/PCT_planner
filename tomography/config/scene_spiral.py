from .scene import ScenePCD, SceneMap, SceneTrav


class SceneSpiral():
    pcd = ScenePCD()
    pcd.file_name = 'spiral0.3_2.pcd'

    map = SceneMap()
    # map.resolution = 0.05 # 对应pcd地图的分辨率
    map.resolution = 0.20 # default 
    # map.ground_h = 0.2
    map.ground_h = 0.0 # default
    map.slice_dh = 0.5

    trav = SceneTrav()
    trav.kernel_size = 7
    trav.interval_min = 0.50
    trav.interval_free = 0.65
    trav.slope_max = 0.40
    trav.step_max = 0.30
    trav.standable_ratio = 0.40
    trav.cost_barrier = 60.0 # 最大 cost, 减小这个参数会显著减小膨胀半径，大于40就不会影响了
    trav.safe_margin = 1.2
    trav.inflation = 0.2


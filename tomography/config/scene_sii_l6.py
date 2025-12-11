from .scene import ScenePCD, SceneMap, SceneTrav


class SceneSiiL6():
    pcd = ScenePCD()
    # PCD only need to change here
    # pcd.file_name = 'scans_0908_L10L9L8_leveled_clean.pcd'
    pcd.file_name = '6l7l_mid360_leveled_clear.ply'

    map = SceneMap()
    map.resolution = 0.1 # 0.1 good, why?
    map.ground_h = 0.1
    map.slice_dh = 0.4 # 0.5

    trav = SceneTrav()
    trav.kernel_size = 1 # 7
    trav.interval_min = 0.2 # 0.5
    trav.interval_free = 0.3 # 0.65
    trav.slope_max = 0.5
    trav.step_max = 0.30
    trav.standable_ratio = 0.40
    trav.cost_barrier = 50 # 50.0
    trav.safe_margin = 0.2 # 1.2
    trav.inflation = 0.1


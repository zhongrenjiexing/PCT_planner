# -*- coding: utf-8 -*-
import sys
import argparse
import numpy as np

import rospy
from nav_msgs.msg import Path

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config

parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=str, default='sii_l6', help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
args = parser.parse_args()

cfg = Config()

if args.scene == 'Spiral':
    tomo_file = 'spiral0.3_2'
    # start_pos = np.array([-16.0, -6.0], dtype=np.float32)
    start_pos = np.array([-16.0, -6.0, 0], dtype=np.float32)
    # end_pos = np.array([-26.0, -5.0], dtype=np.float32)
    end_pos = np.array([-43.0, -3.8, 16.5], dtype=np.float32) # 0.5 / 8.5 / 16.5
elif args.scene == 'Building':
    tomo_file = 'building2_9'
    start_pos = np.array([5.0, 5.0], dtype=np.float32)
    end_pos = np.array([-6.0, -1.0], dtype=np.float32)
elif args.scene == 'sii_l6':
    # tomo_file = '6L7L_Add613_-ready-to-pctplanner'
    tomo_file = '567_-ready-to-pctplanner'
    # tomo_file = '6L7L_Add613_align_ds'

    # start_pos = np.array([4.42, -16.0, 0.1], dtype=np.float32) # from L6-614
    # end_pos = np.array([6.32, -20.5, 4.65], dtype=np.float32) # Maker Club in L7
    # end_pos = np.array([5.56, 2.01, 0.2], dtype=np.float32) # little door outside L6 Exp room

    # start_pos = np.array([2.46, -0.9, 0.5], dtype=np.float32)
    # end_pos = np.array([20.6, 8.43, 4.7], dtype=np.float32)

    start_pos = np.array([-0.68, -1.0, 0.1], dtype=np.float32) # from L6-614
    # end_pos = np.array([-0.8, -4.82, 0.8], dtype=np.float32) # Slop 中间平坡
    # start_pos = np.array([-0.8, -4.82, 0.8], dtype=np.float32) # Slop 中间平坡
    # end_pos = np.array([-1, -7.3, 0.2], dtype=np.float32) # Slop 下边
    end_pos = np.array([21.6, -29.3, -4.0], dtype=np.float32) # 5L 公共区域
    # end_pos = np.array([24.1, -22.4, 0.3], dtype=np.float32) # 6L 楼梯交接处
    # end_pos = np.array([28.3, -27.4, -2], dtype=np.float32) # 5L6L 楼梯平台处
    # end_pos = np.array([27.0, -29.4, -2], dtype=np.float32) # 5L6L 楼梯平台处

    # end_pos = np.array([-2.5, -3.84, 0.1], dtype=np.float32) # 
elif args.scene == 'SiiL6':
    tomo_file = '2026-01-02_22-26-52_colorized_L567_FT_AddLift_AddPoints'
    start_pos = np.array([-5.9, -0.9, -0.1], dtype=np.float32)
    # end_pos = np.array([2.4, -21.7, 4.6], dtype=np.float32) # Maker Club
    end_pos = np.array([7.88, -30.4, -4.0], dtype=np.float32) # 5L public area

else:
    tomo_file = 'plaza3_10'
    start_pos = np.array([0.0, 0.0], dtype=np.float32)
    end_pos = np.array([23.0, 10.0], dtype=np.float32)


path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)
raw_path_pub = rospy.Publisher("/pct_raw_path", Path, latch=True, queue_size=1)
planner = TomogramPlanner(cfg)

def pct_plan():
    planner.loadTomogram(tomo_file)

    # 获取优化后的轨迹
    traj_3d = planner.plan(start_pos, end_pos)
    if traj_3d is not None:
        # 首先发布原始路径
        raw_traj_3d = planner.get_raw_path()
        if raw_traj_3d is not None:
            print(f"Raw trajectory shape: {raw_traj_3d.shape}")
            print(f"Raw trajectory sample points: {raw_traj_3d[:5]}")
            raw_path_pub.publish(traj2ros(raw_traj_3d))
            print("Raw trajectory published")
        else:
            print("Raw trajectory is None")

        # 然后发布优化后的轨迹
        print(f"Optimized trajectory shape: {traj_3d.shape}")
        print(f"Optimized trajectory sample points: {traj_3d[:5]}")
        path_pub.publish(traj2ros(traj_3d))
        print("Optimized trajectory published")


if __name__ == '__main__':
    import time
    try:
        print("Initializing ROS node...")
        rospy.init_node("pct_planner", anonymous=True)
        print("ROS node initialized")

        print("Starting planning...")
        pct_plan()
        print("Planning and publishing completed successfully!")

        # 给一点时间让消息发布出去，然后退出
        time.sleep(0.1)
        print("Exiting...")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
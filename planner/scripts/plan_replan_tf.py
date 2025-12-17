#!/usr/bin/env python
# conda env: unitreerl

import sys
import argparse
import numpy as np

import rospy
import tf
from nav_msgs.msg import Path

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config


class TFReplanNode(object):
    """
    基于 tomogram 的路径规划节点：
    - 终点固定；
    - 起点由 TF 中 map->base_link 的实时位姿给出；
    - 每 2s 重新规划一次路径，并发布到与原节点相同的话题 /pct_path。
    """

    def __init__(self):
        # 解析场景参数（与原 plan.py 保持一致）
        parser = argparse.ArgumentParser()
        parser.add_argument(
            '--scene',
            type=str,
            default='sii_l6',
            help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\', \'sii_l6\']'
        )
        args, _ = parser.parse_known_args()

        cfg = Config()

        if args.scene == 'sii_l6':
            tomo_file = '6L7L_Add613_-ready-to-pctplanner'
            # 终点与 planner/scripts/plan.py 保持一致
            # end_pos = np.array([6.32, -20.5, 4.65], dtype=np.float32) # Maker in L7
            end_pos = np.array([-1.0, -4.82, 0.8], dtype=np.float32) # center of slop in L6 Experiment Room

        self.end_pos = end_pos

        # 路径话题与原节点保持一致
        self.path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)

        # 规划器：只加载一次 tomogram，后续重复调用 plan
        self.planner = TomogramPlanner(cfg)
        rospy.loginfo("Loading tomogram file: %s", tomo_file)
        self.planner.loadTomogram(tomo_file)

        # TF 监听器，获取 map->base_link
        self.tf_listener = tf.TransformListener()
        self.map_frame = "map"
        self.base_frame = "base_link"

        # 等待第一次 TF 可用（避免刚启动时立即报错）
        try:
            self.tf_listener.waitForTransform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(5.0)
            )
        except Exception as e:
            rospy.logwarn("Wait for TF %s->%s failed: %s",
                          self.map_frame, self.base_frame, str(e))

        # 定时器：每 2s 触发一次重新规划
        self.timer = rospy.Timer(rospy.Duration(2.0), self.timer_callback)

    def timer_callback(self, event):
        # 从 TF 获取当前起点
        try:
            t = self.tf_listener.getLatestCommonTime(self.map_frame, self.base_frame)
            (trans, rot) = self.tf_listener.lookupTransform(
                self.map_frame, self.base_frame, t
            )
            # trans: (x, y, z)
            start_pos = np.array(
                [trans[0], trans[1], trans[2]], dtype=np.float32
            )
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "TF lookup %s->%s failed: %s",
                                   self.map_frame, self.base_frame, str(e))
            return

        # 调用规划器
        traj_3d = self.planner.plan(start_pos, self.end_pos)
        if traj_3d is None:
            rospy.logwarn_throttle(2.0, "Planner failed to find a path.")
            return

        # 发布路径到 /pct_path
        path_msg = traj2ros(traj_3d)
        self.path_pub.publish(path_msg)
        rospy.loginfo_throttle(2.0, "Published new trajectory from TF start pose.")


if __name__ == '__main__':
    rospy.init_node("pct_planner_tf_replan", anonymous=True)

    node = TFReplanNode()

    rospy.spin()



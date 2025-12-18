#!/usr/bin/env python
# conda env: unitreerl
# 需要chmod+x的权限，才能发布话题？印象中好像是
import sys
import argparse
import numpy as np

import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config


class TFReplanNode(object):
    """
    基于 tomogram 的路径规划节点：
    - 监听 /move_base_simple/goal 话题，接收导航目标点；
    - 起点由 TF 中 map->base_link 的实时位姿给出；
    - 每 1s 重新规划一次路径，并发布到与原节点相同的话题 /pct_path；
    - 当导航成功到达目标点后，停止发布路径。
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

        # 导航目标点（从 /move_base_simple/goal 接收）
        self.end_pos = None
        self.goal_received = False
        self.goal_timestamp = None  # 记录目标接收时间，用于过滤旧的result消息

        # 导航状态标志
        self.navigation_reached = False
        self.result_received = False
        
        # 目标点距离阈值：当距离目标点小于此值时停止规划（避免路径点过少导致崩溃）
        self.goal_distance_threshold = 0.1  # 单位：米

        # 路径话题与原节点保持一致
        self.path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)
        
        # 订阅导航目标点
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal",
            PoseStamped,
            self.goal_callback,
            queue_size=1
        )
        
        # 订阅导航结果
        self.result_sub = rospy.Subscriber(
            "/move_base/result",
            MoveBaseActionResult,
            self.result_callback,
            queue_size=1
        )

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
        rospy.loginfo("等待接收导航目标点 /move_base_simple/goal ...")
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

    def timer_callback(self, event):
        # 如果还未接收到目标点，等待
        if not self.goal_received or self.end_pos is None:
            rospy.loginfo_throttle(5.0, "等待接收导航目标点...")
            return
        
        # 如果导航已成功到达，停止发布路径
        if self.navigation_reached:
            rospy.loginfo_throttle(5.0, "导航已成功，停止发布路径")
            return
        
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

        # 检查当前位置与目标点的距离
        distance = np.linalg.norm(start_pos - self.end_pos)
        
        # 如果距离太近，认为已接近目标，停止规划
        if distance < self.goal_distance_threshold:
            rospy.loginfo_throttle(2.0, "🎯 已接近目标点 (距离: %.3f m < %.3f m)，停止路径规划", 
                                  distance, self.goal_distance_threshold)
            self.navigation_reached = True  # 标记为已到达
            return
        
        # 调用规划器（使用 try-except 捕获可能的异常）
        rospy.logdebug("Planning from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
                      start_pos[0], start_pos[1], start_pos[2],
                      self.end_pos[0], self.end_pos[1], self.end_pos[2])
        
        try:
            traj_3d = self.planner.plan(start_pos, self.end_pos)
        except Exception as e:
            # 捕获规划器异常（包括断言失败等）
            rospy.logwarn_throttle(2.0, "❌ 规划器异常 (距离: %.3f m): %s", distance, str(e))
            # 如果是因为距离太近导致的异常，标记为已到达
            if distance < 0.5:  # 0.5米内的异常认为是接近目标导致的
                rospy.loginfo("已非常接近目标点，标记为已到达")
                self.navigation_reached = True
            return
        
        if traj_3d is None:
            rospy.logwarn_throttle(2.0, "❌ Planner failed to find a path (returned None).")
            return
        
        # 检查路径是否为空
        if len(traj_3d) == 0:
            rospy.logwarn_throttle(2.0, "❌ Path is empty, skipping publish.")
            return

        # 发布路径到 /pct_path
        path_msg = traj2ros(traj_3d)
        if len(path_msg.poses) == 0:
            rospy.logwarn_throttle(2.0, "Converted path is empty, skipping publish.")
            return
            
        self.path_pub.publish(path_msg)
        rospy.loginfo_throttle(2.0, "Published new trajectory with %d points from TF start pose.", len(traj_3d))

    def goal_callback(self, msg):
        """接收导航目标点回调函数"""
        self.end_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=np.float32)
        
        self.goal_received = True
        self.navigation_reached = False  # 重置导航状态
        self.goal_timestamp = rospy.Time.now()  # 记录新目标接收时间
        
        rospy.loginfo("📍 接收到新的导航目标: (%.3f, %.3f, %.3f)",
                      self.end_pos[0], self.end_pos[1], self.end_pos[2])
        rospy.loginfo("开始路径规划...")

    def result_callback(self, msg):
        """导航结果回调函数"""
        # 过滤旧的result消息：在接收新目标后0.5秒内忽略result（避免收到旧任务的结果）
        if self.goal_timestamp is not None:
            time_since_goal = (rospy.Time.now() - self.goal_timestamp).to_sec()
            if time_since_goal < 0.5:
                rospy.logdebug("忽略新目标设置后0.5秒内的result消息（可能是旧任务）")
                return
        
        status = msg.status.status
        if status == 3:
            rospy.loginfo("✅ 导航成功到达目标点！")
            self.navigation_reached = True
        else:
            rospy.logwarn(f"❌ 导航失败，状态码: {status}")
            self.navigation_reached = False
        self.result_received = True


if __name__ == '__main__':
    rospy.init_node("pct_planner_tf_replan", anonymous=True)

    node = TFReplanNode()

    rospy.spin()



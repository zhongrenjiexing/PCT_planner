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
from actionlib_msgs.msg import GoalStatusArray

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
            # tomo_file = '6L7L_Add613_-ready-to-pctplanner'
            tomo_file = '2026-01-02_22-26-52_colorized_L567_FT_AddLift_AddPoints_XYZ'

        # 导航目标点（从 /move_base_simple/goal 接收）
        self.end_pos = None
        self.end_orientation = None  # 存储目标点的朝向
        self.goal_received = False
        self.goal_timestamp = None  # 记录目标接收时间，用于过滤旧的result消息

        # 导航状态标志
        self.navigation_reached = False
        self.result_received = False
        self.navigation_cancelled = False  # 新增：检测是否被取消
        
        # 目标点距离阈值：当距离目标点小于此值时停止规划（避免路径点过少导致崩溃）
        self.goal_distance_threshold = 0.1  # 单位：米

        # 发布原始未优化路径
        self.path_pub = rospy.Publisher("/pct_raw_path", Path, latch=False, queue_size=1)
        
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
        
        # 订阅导航状态（用于检测取消）
        self.status_sub = rospy.Subscriber(
            "/move_base/status",
            GoalStatusArray,
            self.status_callback,
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
        
        # 如果导航已被取消，停止发布路径
        if self.navigation_cancelled:
            rospy.loginfo_throttle(5.0, "导航已取消，停止发布路径")
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
        # 使用3D距离
        distance_3d = np.linalg.norm(start_pos - self.end_pos)
        # 使用2D距离（XY平面），更准确反映导航距离
        distance_2d = np.linalg.norm(start_pos[:2] - self.end_pos[:2])
        
        # 使用2D距离进行安全检查（Z轴偏差不应影响停止判断）
        distance = distance_2d
        
        # 安全距离检查：C++断言要求路径点>1，距离太近会导致路径点不足
        # 必须提前阻止规划，因为C++的assert无法被Python捕获
        min_safe_distance = max(self.goal_distance_threshold, 0.3)  # 至少0.3米
        
        if distance < min_safe_distance:
            rospy.loginfo_throttle(2.0, "🎯 已接近目标点 (2D距离: %.3fm, 3D距离: %.3fm < 阈值: %.3fm)，停止路径规划", 
                                  distance_2d, distance_3d, min_safe_distance)
            self.navigation_reached = True  # 标记为已到达
            return
        
        # 调用规划器（使用 try-except 捕获可能的异常）
        rospy.loginfo("🛣️  Planning: 2D=%.3fm, 3D=%.3fm, from=(%.2f,%.2f,%.2f) to=(%.2f,%.2f,%.2f)",
                      distance_2d, distance_3d,
                      start_pos[0], start_pos[1], start_pos[2],
                      self.end_pos[0], self.end_pos[1], self.end_pos[2])
        
        try:
            traj_3d = self.planner.plan(start_pos, self.end_pos)
            if traj_3d is not None:
                raw_traj_3d = self.planner.get_raw_path()
                rospy.loginfo("✅ 规划成功，原始路径 %d 个点，优化后 %d 个点", len(raw_traj_3d), len(traj_3d))
        except Exception as e:
            # 捕获规划器异常（包括断言失败等）
            rospy.logwarn("❌ 规划器异常 (距离: %.3f m): %s", distance, str(e))
            # 如果是因为距离太近导致的异常，标记为已到达
            if distance < 0.5:  # 0.5米内的异常认为是接近目标导致的
                rospy.loginfo("已非常接近目标点，标记为已到达")
                self.navigation_reached = True
            return

        if traj_3d is None:
            rospy.logwarn_throttle(2.0, "❌ Planner failed to find a path (returned None).")
            return

        # 检查原始路径是否为空
        if len(raw_traj_3d) == 0:
            rospy.logwarn_throttle(2.0, "❌ Raw path is empty, skipping publish.")
            return

        # 检查是否需要进行路径插值
        need_interpolation = False
        if len(raw_traj_3d) <= 2:
            need_interpolation = True
            rospy.loginfo("路径点数量少于等于2个 (%d)，将进行线性插值", len(raw_traj_3d))
        elif distance < 0.3:
            need_interpolation = True
            rospy.loginfo("当前位置到目标距离小于0.3m (%.3fm)，将进行线性插值", distance)

        # 如果需要插值，进行线性插值添加10个密集点
        if need_interpolation:
            raw_traj_3d = self.interpolate_path(raw_traj_3d, num_points=10)
            rospy.loginfo("插值后路径点数量: %d", len(raw_traj_3d))

        # 为路径点添加朝向信息（后三个点，最后一个点使用目标朝向）
        path_msg = self.add_orientation_to_path(raw_traj_3d, self.end_orientation)
        if len(path_msg.poses) == 0:
            rospy.logwarn_throttle(2.0, "Converted raw path is empty, skipping publish.")
            return

        self.path_pub.publish(path_msg)
        rospy.loginfo_throttle(2.0, "Published raw trajectory with %d points from TF start pose.", len(raw_traj_3d))

    def add_orientation_to_path(self, traj, goal_orientation=None):
        """为路径点添加朝向信息，后三个点基于路径方向计算朝向，最后一个点使用目标朝向"""
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        import math

        path_msg = Path()
        path_msg.header.frame_id = "map"

        if len(traj) == 0:
            return path_msg

        # 为所有点创建PoseStamped，默认朝向为向前
        poses = []
        for i, waypoint in enumerate(traj):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]

            # 默认朝向 (朝向正X方向)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            poses.append(pose)

        # 为后三个点（如果存在）计算朝向
        num_points = len(poses)
        if num_points >= 2:  # 至少需要2个点才能计算方向
            # 计算需要添加朝向的点的索引（最后三个点）
            start_idx = max(0, num_points - 3)

            for i in range(start_idx, num_points):
                if i == num_points - 1 and goal_orientation is not None:
                    # 最后一个点：使用目标点的朝向
                    poses[i].pose.orientation.x = goal_orientation[0]
                    poses[i].pose.orientation.y = goal_orientation[1]
                    poses[i].pose.orientation.z = goal_orientation[2]
                    poses[i].pose.orientation.w = goal_orientation[3]
                else:
                    # 其他点：使用当前点到下一个点的方向
                    if i < num_points - 1:
                        # 使用当前点到下一个点的方向
                        current = np.array([poses[i].pose.position.x, poses[i].pose.position.y])
                        next_point = np.array([poses[i+1].pose.position.x, poses[i+1].pose.position.y])
                        direction = next_point - current
                    else:
                        # 最后一个点（如果没有目标朝向）：使用前一个点到当前点的方向
                        current = np.array([poses[i].pose.position.x, poses[i].pose.position.y])
                        prev_point = np.array([poses[i-1].pose.position.x, poses[i-1].pose.position.y])
                        direction = current - prev_point

                    # 归一化方向向量
                    norm = np.linalg.norm(direction)
                    if norm > 1e-6:  # 避免除零
                        direction = direction / norm

                        # 计算朝向角度（相对于X轴）
                        yaw = math.atan2(direction[1], direction[0])

                        # 转换为四元数
                        poses[i].pose.orientation.x = 0.0
                        poses[i].pose.orientation.y = 0.0
                        poses[i].pose.orientation.z = math.sin(yaw / 2.0)
                        poses[i].pose.orientation.w = math.cos(yaw / 2.0)

        path_msg.poses = poses
        return path_msg

    def interpolate_path(self, path_points, num_points=10):
        """
        对路径进行线性插值，添加密集点

        Args:
            path_points: 原始路径点列表，shape为 (N, 3)
            num_points: 插值后期望的总点数

        Returns:
            插值后的路径点列表
        """
        if len(path_points) < 2:
            return path_points

        import numpy as np

        # 将路径点转换为numpy数组
        path_array = np.array(path_points)

        # 计算原始路径的总长度
        total_length = 0
        for i in range(len(path_array) - 1):
            segment_length = np.linalg.norm(path_array[i+1] - path_array[i])
            total_length += segment_length

        if total_length == 0:
            return path_points

        # 计算每个插值点的累计距离间隔
        desired_spacing = total_length / (num_points - 1)

        interpolated_points = []
        current_distance = 0
        segment_start_idx = 0

        # 第一个点
        interpolated_points.append(path_array[0].tolist())

        for i in range(1, num_points - 1):
            target_distance = i * desired_spacing

            # 找到目标距离所在的段
            while segment_start_idx < len(path_array) - 1:
                segment_end_idx = segment_start_idx + 1
                segment_start = path_array[segment_start_idx]
                segment_end = path_array[segment_end_idx]
                segment_length = np.linalg.norm(segment_end - segment_start)

                if current_distance + segment_length >= target_distance:
                    # 在当前段内插值
                    remaining_distance = target_distance - current_distance
                    ratio = remaining_distance / segment_length

                    interpolated_point = segment_start + ratio * (segment_end - segment_start)
                    interpolated_points.append(interpolated_point.tolist())
                    break

                current_distance += segment_length
                segment_start_idx += 1

        # 最后一个点
        interpolated_points.append(path_array[-1].tolist())

        return interpolated_points

    def goal_callback(self, msg):
        """接收导航目标点回调函数"""
        self.end_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z + 0.2 # 高一些
        ], dtype=np.float32)

        # 存储目标点的朝向
        self.end_orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ], dtype=np.float32)

        self.goal_received = True
        self.navigation_reached = False  # 重置导航状态
        self.navigation_cancelled = False  # 重置取消状态
        self.goal_timestamp = rospy.Time.now()  # 记录新目标接收时间

        rospy.loginfo("📍 接收到新的导航目标: (%.3f, %.3f, %.3f)",
                      self.end_pos[0], self.end_pos[1], self.end_pos[2])
        rospy.loginfo("   目标朝向: (%.3f, %.3f, %.3f, %.3f)",
                      self.end_orientation[0], self.end_orientation[1],
                      self.end_orientation[2], self.end_orientation[3])
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
            rospy.logwarn("❌ 导航失败，状态码: %d", status)
            self.navigation_reached = False
        self.result_received = True
    
    def status_callback(self, msg):
        """监听 move_base 状态，检测取消事件"""
        if not msg.status_list:
            return
        
        # 检查最新的目标状态
        latest_status = msg.status_list[-1]
        
        # status = 2: PREEMPTED (被取消)
        # status = 4: ABORTED (失败)
        # status = 5: REJECTED (被拒绝)
        if latest_status.status in [2, 4, 5]:
            if not self.navigation_cancelled:
                rospy.logwarn("🛑 检测到导航被取消/中止 (status=%d)，停止发布路径", latest_status.status)
                self.navigation_cancelled = True
                self.goal_received = False  # 停止接受新目标直到下一次明确的目标


if __name__ == '__main__':
    rospy.init_node("pct_planner_tf_replan", anonymous=True)

    node = TFReplanNode()

    rospy.spin()



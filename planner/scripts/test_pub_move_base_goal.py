#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
测试脚本：发布导航目标点到 /move_base_simple/goal
用于测试 plan_replan_tf.py 的目标点接收功能
"""

import rospy
from geometry_msgs.msg import PoseStamped


def publish_goal():
    """发布一次导航目标点"""
    rospy.init_node('test_move_base_goal_publisher', anonymous=True)
    
    # 创建Publisher - 使用 latch=True 确保晚到的订阅者也能收到
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10, latch=True)
    
    # 给发布者足够的时间注册
    rospy.loginfo("初始化发布者...")
    rospy.sleep(1.0)
    
    # 检查订阅者连接
    rospy.loginfo("等待 /move_base_simple/goal 订阅者连接...")
    wait_count = 0
    while pub.get_num_connections() == 0 and not rospy.is_shutdown() and wait_count < 50:
        rospy.sleep(0.1)
        wait_count += 1
    
    if pub.get_num_connections() == 0:
        rospy.logwarn("⚠️  没有检测到订阅者，但仍会尝试发送...")
    else:
        rospy.loginfo("✓ 检测到 %d 个订阅者", pub.get_num_connections())
    
    # 构造目标消息
    # 默认值：little door outside L6 Exp room
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    
    # 位置（使用原来的默认终点）
    goal.pose.position.x = 5.56
    goal.pose.position.y = 2.01
    goal.pose.position.z = 0.2
    
    # 朝向（默认朝向，由DWA负责实际朝向控制）
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0
    
    # 发布
    rospy.loginfo("=" * 50)
    rospy.loginfo("📍 发布导航目标点:")
    rospy.loginfo("   位置: (%.3f, %.3f, %.3f)", 
                  goal.pose.position.x, 
                  goal.pose.position.y, 
                  goal.pose.position.z)
    rospy.loginfo("   朝向: (%.3f, %.3f, %.3f, %.3f)",
                  goal.pose.orientation.x,
                  goal.pose.orientation.y,
                  goal.pose.orientation.z,
                  goal.pose.orientation.w)
    rospy.loginfo("=" * 50)
    
    # 多次发布确保消息被接收
    for i in range(3):
        pub.publish(goal)
        rospy.loginfo("发送第 %d 次...", i + 1)
        rospy.sleep(0.3)
    
    rospy.loginfo("✅ 导航目标已发送！")
    rospy.loginfo("提示：plan_replan_tf.py 应该会开始规划路径")
    rospy.loginfo("提示：可以使用 'rostopic echo /move_base_simple/goal' 验证消息")


if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass


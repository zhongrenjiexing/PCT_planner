#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
测试脚本：发布导航目标点到 /move_base_simple/goal
用于测试 plan_replan_tf.py 的目标点接收功能
交互式版本：输入序号选择目标位置
"""

import rospy
from geometry_msgs.msg import PoseStamped


# 预定义的目标位置字典
GOAL_POSITIONS = {
    11: {
        "name": "test",
        "position": {"x": -1.5, "y": -4.5, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    1: {
        "name": "Button_AB_6L",
        "position": {"x": 20.56, "y": -9.88, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    2: {
        "name": "Button_BC_6L",
        "position": {"x": 20.58, "y": -12.78, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    3: {
        "name": "Button_DE_6L",
        "position": {"x": 18.18, "y": -12.79, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    4: {
        "name": "Button_EF_6L",
        "position": {"x": 18.17, "y": -9.91, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    5: {
        "name": "Button_AB_5L",
        "position": {"x": 20.66, "y": -9.88, "z": -3.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    6: {
        "name": "Button_BC_5L",
        "position": {"x": 20.68, "y": -12.78, "z": -3.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    7: {
        "name": "Button_DE_5L",
        "position": {"x": 18.08, "y": -12.79, "z": -3.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    8: {
        "name": "Button_EF_5L",
        "position": {"x": 18.07, "y": -9.91, "z": -3.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    }
}


def print_menu():
    """打印位置选择菜单"""
    print("\n" + "=" * 60)
    print("📍 可用的导航目标位置：")
    print("=" * 60)
    for key, value in sorted(GOAL_POSITIONS.items()):
        pos = value["position"]
        print(f"{key}. {value['name']}")
        print(f"   位置: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})")
    print("0. 退出程序")
    print("=" * 60)


def get_user_choice():
    """获取用户输入的序号"""
    while True:
        try:
            choice = input("\n请输入目标位置序号 (0-{}): ".format(len(GOAL_POSITIONS)))
            choice = int(choice)
            if choice == 0:
                return None
            if choice in GOAL_POSITIONS:
                return choice
            else:
                print(f"❌ 无效输入！请输入 0 到 {len(GOAL_POSITIONS)} 之间的数字。")
        except ValueError:
            print("❌ 请输入有效的数字！")
        except KeyboardInterrupt:
            print("\n👋 程序被中断")
            return None


def publish_goal(pub, goal_data):
    """发布导航目标点"""
    # 构造目标消息
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    
    # 设置位置
    goal.pose.position.x = goal_data["position"]["x"]
    goal.pose.position.y = goal_data["position"]["y"]
    goal.pose.position.z = goal_data["position"]["z"]
    
    # 设置朝向
    goal.pose.orientation.x = goal_data["orientation"]["x"]
    goal.pose.orientation.y = goal_data["orientation"]["y"]
    goal.pose.orientation.z = goal_data["orientation"]["z"]
    goal.pose.orientation.w = goal_data["orientation"]["w"]
    
    # 发布
    rospy.loginfo("=" * 50)
    rospy.loginfo("📍 发布导航目标点: %s", goal_data["name"])
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


def main():
    """主函数"""
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
        rospy.logwarn("⚠️  没有检测到订阅者，但仍可以发送消息...")
    else:
        rospy.loginfo("✓ 检测到 %d 个订阅者", pub.get_num_connections())
    
    # 交互式循环
    print("\n🤖 交互式导航目标发布器")
    print("提示：输入 0 或按 Ctrl+C 退出程序\n")
    
    while not rospy.is_shutdown():
        print_menu()
        choice = get_user_choice()
        
        if choice is None:
            print("\n👋 退出程序")
            break
        
        # 发布选定的目标
        goal_data = GOAL_POSITIONS[choice]
        publish_goal(pub, goal_data)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n👋 程序被用户中断")


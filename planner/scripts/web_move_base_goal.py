#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Web界面版本：通过手机浏览器发布导航目标点
使用Flask提供Web服务，可在手机上访问
"""

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from flask import Flask, render_template_string, request, jsonify
import threading
import socket

app = Flask(__name__)

# 预定义的目标位置字典
GOAL_POSITIONS = {
    0: {
        "name": "charging dock",
        "position": {"x": 1.0, "y": -1.7, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0, "w": 1}
    },
    1: {
        "name": "test1",
        "position": {"x": -1.5, "y": -4.5, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    2: {
        "name": "test2",
        "position": {"x": -1.5, "y": -4.5, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    3: {
        "name": "middle_5",
        "position": {"x": 19.5, "y": -10.99, "z": -3.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    4: {
        "name": "middle_6",
        "position": {"x": 19.5, "y": -10.99, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    5: {
        "name": "middle_7",
        "position": {"x": 19.5, "y": -10.99, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    6: {
        "name": "Button_AB_5L",
        "position": {"x": 20.66, "y": -9.88, "z": -3.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    7: {
        "name": "Button_BC_5L",
        "position": {"x": 20.68, "y": -12.78, "z": -3.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    8: {
        "name": "Button_DE_5L",
        "position": {"x": 18.08, "y": -12.79, "z": -3.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    9: {
        "name": "Button_EF_5L",
        "position": {"x": 18.07, "y": -9.91, "z": -3.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    10: {
        "name": "Button_AB_6L",
        "position": {"x": 20.56, "y": -9.88, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    11: {
        "name": "Button_BC_6L",
        "position": {"x": 20.58, "y": -12.78, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    12: {
        "name": "Button_DE_6L",
        "position": {"x": 18.18, "y": -12.79, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    13: {
        "name": "Button_EF_6L",
        "position": {"x": 18.17, "y": -9.91, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    14: {
        "name": "Button_AB_7L",
        "position": {"x": 20.56, "y": -9.88, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    15: {
        "name": "Button_BC_7L",
        "position": {"x": 20.58, "y": -12.78, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    16: {
        "name": "Button_DE_7L",
        "position": {"x": 18.08, "y": -12.79, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    17: {
        "name": "Button_EF_7L",
        "position": {"x": 18.07, "y": -9.91, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}
    },
    18: {
        "name": "Elevator_A_5L",
        "position": {"x": 22.35, "y": -8.14, "z": -4.2},
        "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}
    },
    19: {
        "name": "Elevator_B_5L",
        "position": {"x": 22.35, "y": -10.9, "z": -4.2},
        "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}
    },
    20: {
        "name": "Elevator_C_5L",
        "position": {"x": 22.35, "y": -13.8, "z": -4.2},
        "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}
    },
    21: {
        "name": "Elevator_D_5L",
        "position": {"x": 16.4, "y": -14.67, "z": -4.2},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    22: {
        "name": "Elevator_E_5L",
        "position": {"x": 16.4, "y": -11.74, "z": -4.2},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    23: {
        "name": "Elevator_F_5L",
        "position": {"x": 16.4, "y": -8.86, "z": -4.2},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    24: {
        "name": "Elevator_A_6L",
        "position": {"x": 22.35, "y": -8.14, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}
    },
    25: {
        "name": "Elevator_B_6L",
        "position": {"x": 22.35, "y": -10.9, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}
    },
    26: {
        "name": "Elevator_C_6L",
        "position": {"x": 22.35, "y": -13.8, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}
    },
    27: {
        "name": "Elevator_D_6L",
        "position": {"x": 16.4, "y": -14.67, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    28: {
        "name": "Elevator_E_6L",
        "position": {"x": 16.4, "y": -11.74, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    29: {
        "name": "Elevator_F_6L",
        "position": {"x": 16.4, "y": -8.86, "z": 1.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    30: {
        "name": "Elevator_A_7L",
        "position": {"x": 22.35, "y": -8.14, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}
    },
    31: {
        "name": "Elevator_B_7L",
        "position": {"x": 22.35, "y": -10.9, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}
    },
    32: {
        "name": "Elevator_C_7L",
        "position": {"x": 22.35, "y": -13.8, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}
    },
    33: {
        "name": "Elevator_D_7L",
        "position": {"x": 16.4, "y": -14.67, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    34: {
        "name": "Elevator_E_7L",
        "position": {"x": 16.4, "y": -11.74, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    35: {
        "name": "Elevator_F_7L",
        "position": {"x": 16.4, "y": -8.86, "z": 4.7},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    36: {
        "name": "platform_5L-6L",
        "position": {"x": 28.07, "y": -30.67, "z": -2.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
}


# ROS发布者
pub = None
cancel_pub = None

# HTML模板 - 移动端友好界面
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>🤖 导航控制器</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
        }
        h1 {
            color: white;
            text-align: center;
            margin-bottom: 30px;
            font-size: 28px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.2);
        }
        .input-section {
            background: white;
            border-radius: 20px;
            padding: 25px;
            margin-bottom: 20px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }
        .input-group {
            display: flex;
            gap: 10px;
            margin-bottom: 15px;
        }
        input[type="number"] {
            flex: 1;
            padding: 15px;
            font-size: 18px;
            border: 2px solid #e0e0e0;
            border-radius: 12px;
            outline: none;
            transition: border-color 0.3s;
        }
        input[type="number"]:focus {
            border-color: #667eea;
        }
        .btn {
            padding: 15px 30px;
            font-size: 18px;
            font-weight: bold;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            transition: all 0.3s;
            text-align: center;
            display: inline-block;
            width: 100%;
        }
        .btn-primary {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            box-shadow: 0 4px 15px rgba(102, 126, 234, 0.4);
        }
        .btn-primary:active {
            transform: translateY(2px);
            box-shadow: 0 2px 10px rgba(102, 126, 234, 0.4);
        }
        .btn-stop {
            background: linear-gradient(135deg, #ff416c 0%, #ff4b2b 100%);
            color: white;
            box-shadow: 0 4px 15px rgba(255, 65, 108, 0.4);
            margin-top: 10px;
        }
        .btn-stop:active {
            transform: translateY(2px);
            box-shadow: 0 2px 10px rgba(255, 65, 108, 0.4);
        }
        .btn-goal {
            background: white;
            border: 2px solid #667eea;
            color: #667eea;
            margin-bottom: 10px;
            padding: 12px;
            font-size: 16px;
        }
        .btn-goal:active {
            background: #f0f0f0;
        }
        .message {
            padding: 15px;
            border-radius: 12px;
            margin-top: 15px;
            text-align: center;
            font-weight: bold;
            display: none;
        }
        .success {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        .error {
            background: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
        .goals-section {
            background: white;
            border-radius: 20px;
            padding: 25px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }
        .goals-section h2 {
            margin-bottom: 20px;
            color: #667eea;
            font-size: 20px;
        }
        .goal-item {
            font-size: 14px;
        }
        .goal-name {
            font-weight: bold;
            color: #333;
        }
        .goal-pos {
            color: #666;
            font-size: 12px;
            margin-top: 2px;
        }
        @media (max-width: 480px) {
            h1 {
                font-size: 24px;
            }
            .input-section, .goals-section {
                padding: 20px;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 机器人导航控制</h1>
        
        <div class="input-section">
            <div class="input-group">
                <input type="number" id="goalInput" placeholder="输入目标序号" min="1" max="26">
                <button class="btn btn-primary" onclick="sendGoal()" style="width: auto; padding: 15px 25px;">
                    ✅ 发送
                </button>
            </div>
            <button class="btn btn-stop" onclick="stopNavigation()">
                🛑 停止导航
            </button>
            <div id="message" class="message"></div>
        </div>
        
        <div class="goals-section">
            <h2>📍 可用目标位置</h2>
            {% for key, value in goals.items() %}
            <button class="btn btn-goal goal-item" onclick="sendGoalById({{ key }})">
                <div class="goal-name">{{ key }}. {{ value.name }}</div>
                <div class="goal-pos">
                    ({{ "%.2f"|format(value.position.x) }}, 
                     {{ "%.2f"|format(value.position.y) }}, 
                     {{ "%.2f"|format(value.position.z) }})
                </div>
            </button>
            {% endfor %}
        </div>
    </div>

    <script>
        function showMessage(text, isError = false) {
            const msgDiv = document.getElementById('message');
            msgDiv.textContent = text;
            msgDiv.className = 'message ' + (isError ? 'error' : 'success');
            msgDiv.style.display = 'block';
            setTimeout(() => {
                msgDiv.style.display = 'none';
            }, 3000);
        }

        function sendGoal() {
            const goalId = document.getElementById('goalInput').value;
            if (!goalId) {
                showMessage('请输入目标序号！', true);
                return;
            }
            sendGoalById(parseInt(goalId));
        }

        function sendGoalById(goalId) {
            fetch('/send_goal', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ goal_id: goalId })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showMessage('✅ ' + data.message, false);
                    document.getElementById('goalInput').value = '';
                } else {
                    showMessage('❌ ' + data.message, true);
                }
            })
            .catch(error => {
                showMessage('❌ 发送失败: ' + error, true);
            });
        }

        function stopNavigation() {
            fetch('/stop_navigation', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showMessage('🛑 ' + data.message, false);
                } else {
                    showMessage('❌ ' + data.message, true);
                }
            })
            .catch(error => {
                showMessage('❌ 停止失败: ' + error, true);
            });
        }

        // 允许按回车键发送
        document.getElementById('goalInput').addEventListener('keypress', function(e) {
            if (e.key === 'Enter') {
                sendGoal();
            }
        });
    </script>
</body>
</html>
"""


def publish_goal(goal_data):
    """发布导航目标点"""
    global pub
    
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
    rospy.loginfo("=" * 50)
    
    pub.publish(goal)
    rospy.sleep(0.1)
    
    rospy.loginfo("✅ 导航目标已发送！")


def cancel_navigation():
    """取消当前导航任务"""
    global cancel_pub
    
    # 创建空的GoalID消息来取消所有目标
    cancel_msg = GoalID()
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("🛑 取消导航任务...")
    rospy.loginfo("=" * 50)
    
    cancel_pub.publish(cancel_msg)
    rospy.sleep(0.1)
    
    rospy.loginfo("✅ 取消命令已发送！")


def get_local_ip():
    """获取本机IP地址"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"


@app.route('/')
def index():
    """主页"""
    return render_template_string(HTML_TEMPLATE, goals=GOAL_POSITIONS)


@app.route('/send_goal', methods=['POST'])
def send_goal():
    """处理发送目标请求"""
    try:
        data = request.get_json()
        goal_id = int(data.get('goal_id'))
        
        if goal_id not in GOAL_POSITIONS:
            return jsonify({
                'success': False,
                'message': f'无效的目标序号: {goal_id}'
            })
        
        goal_data = GOAL_POSITIONS[goal_id]
        publish_goal(goal_data)
        
        return jsonify({
            'success': True,
            'message': f'已发送目标: {goal_data["name"]}'
        })
    
    except Exception as e:
        rospy.logerr("发送目标失败: %s", str(e))
        return jsonify({
            'success': False,
            'message': f'发送失败: {str(e)}'
        })


@app.route('/stop_navigation', methods=['POST'])
def stop_navigation():
    """处理停止导航请求"""
    try:
        cancel_navigation()
        
        return jsonify({
            'success': True,
            'message': '导航已停止'
        })
    
    except Exception as e:
        rospy.logerr("停止导航失败: %s", str(e))
        return jsonify({
            'success': False,
            'message': f'停止失败: {str(e)}'
        })


def run_flask():
    """在独立线程中运行Flask"""
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


def main():
    """主函数"""
    global pub, cancel_pub
    
    # 初始化ROS节点
    rospy.init_node('web_move_base_goal_publisher', anonymous=True)
    
    # 创建Publisher
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10, latch=False)
    cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
    
    rospy.loginfo("初始化发布者...")
    rospy.sleep(1.0)
    
    # 获取本机IP
    local_ip = get_local_ip()
    
    # 在独立线程中启动Flask
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    print("\n" + "=" * 60)
    print("🌐 Web服务器已启动！")
    print("=" * 60)
    print(f"📱 在手机浏览器中访问:")
    print(f"   http://{local_ip}:5000")
    print(f"   或 http://localhost:5000 (本机)")
    print("=" * 60)
    print("💡 提示:")
    print("   1. 确保手机和机器人在同一WiFi网络")
    print("   2. 在手机浏览器输入上面的地址")
    print("   3. 输入数字或点击按钮发送目标")
    print("   4. 按 Ctrl+C 退出程序")
    print("=" * 60)
    print()
    
    # 保持ROS节点运行
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n👋 程序被用户中断")


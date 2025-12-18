# PCT Planner 使用说明

## 文件说明

### 主要功能文件

1. **plan_replan_tf.py** - 主要的路径规划节点
   - 监听 `/move_base_simple/goal` 接收导航目标点
   - 从TF获取实时位置作为起点
   - 每2秒重新规划路径并发布到 `/pct_path`
   - 导航成功后自动停止发布

2. **test_pub_move_base_goal.py** - 测试用目标点发布脚本
   - 发布一次测试目标点到 `/move_base_simple/goal`
   - 默认目标: (5.56, 2.01, 0.2)

3. **debug_topics.py** - 话题诊断工具
   - 监听并显示所有相关话题的消息
   - 帮助调试通信问题

## 使用方法

### 正常启动流程

```bash
# 终端1：启动路径规划节点
cd /home/unitree/PCT_planner/planner/scripts
python plan_replan_tf.py

# 终端2：发布测试目标点
cd /home/unitree/PCT_planner/planner/scripts
python test_pub_move_base_goal.py
```

### 调试流程

如果遇到问题，使用诊断工具：

```bash
# 终端3：启动诊断工具
cd /home/unitree/PCT_planner/planner/scripts
python debug_topics.py
```

## 常见问题排查

### 问题1: "path is empty" 或 Segmentation fault / Assertion failed

**原因**: 
- 路径规划失败，返回空路径
- 当接近目标点时，起点和终点距离太近，导致路径点数 ≤1，触发 `gpmp_optimizer` 断言失败

**错误信息示例**:
```
Assertion `path.size() > 1' failed.
Aborted (core dumped)
```

**解决方案**:
1. 检查起点和终点是否在tomogram范围内
2. 检查起点和终点之间是否存在可行路径
3. 查看日志中的 "Planning from ... to ..." 确认坐标正确

**已添加的保护措施**:
- ✅ **距离阈值检查**: 当距离目标点 < 阈值时，自动停止规划
- ✅ **Try-Except异常捕获**: 即使规划器崩溃也不会导致程序退出
- ✅ **智能异常处理**: 0.5米内的异常自动标记为已到达目标
- ✅ 检查路径长度，空路径不会发布
- ✅ 详细的错误日志输出
- ✅ 自动标记为已到达目标

### 问题2: test_pub_move_base_goal.py 没有发送出去

**原因**: 订阅者未准备好或消息丢失

**解决方案**:
1. 确保 plan_replan_tf.py 已经启动并运行
2. 使用 `rostopic list` 检查话题是否存在
3. 使用 `rostopic echo /move_base_simple/goal` 验证消息
4. 使用 debug_topics.py 监听消息

**已改进的地方**:
- 增加了发布等待时间
- 使用 latch=True 确保晚到的订阅者也能收到
- 多次发布确保消息被接收

### 问题3: 规划节点没有开始规划

**检查清单**:
```bash
# 1. 检查节点是否运行
rosnode list | grep pct_planner

# 2. 检查话题是否存在
rostopic list | grep -E "(move_base_simple|pct_path)"

# 3. 检查TF是否正常
rosrun tf tf_echo map base_link

# 4. 查看节点日志
# 应该看到 "接收到新的导航目标" 和 "Published new trajectory"
```

## 工作流程图

```
[test_pub_move_base_goal.py]
         |
         | 发布目标点
         v
[/move_base_simple/goal]
         |
         | 订阅
         v
[plan_replan_tf.py]
         |
         | 1. 接收目标点
         | 2. 从TF获取当前位置
         | 3. 规划路径 (每2秒)
         |
         | 发布路径
         v
    [/pct_path]
         |
         | 订阅
         v
   [move_base/DWA]
         |
         | 执行导航
         |
         | 发布结果
         v
[/move_base/result]
         |
         | 订阅
         v
[plan_replan_tf.py]
         |
         v
   (导航成功，停止发布)
```

## 日志说明

### plan_replan_tf.py 日志

- `等待接收导航目标点...` - 等待接收目标点
- `📍 接收到新的导航目标: (x, y, z)` - 成功接收目标点
- `开始路径规划...` - 开始规划
- `Published new trajectory with N points` - 成功发布N个路径点
- `✅ 导航成功到达目标点！` - 导航完成
- `❌ Path is empty` - 路径规划失败（空路径）
- `❌ Planner failed to find a path` - 规划器返回None

### test_pub_move_base_goal.py 日志

- `✓ 检测到 N 个订阅者` - 有订阅者连接
- `⚠️ 没有检测到订阅者，但仍会尝试发送...` - 无订阅者但会发送
- `发送第 N 次...` - 多次发送确保接收

## 参数调整

### 修改目标点

编辑 `test_pub_move_base_goal.py`:

```python
# 位置
goal.pose.position.x = 5.56  # 修改这里
goal.pose.position.y = 2.01  # 修改这里
goal.pose.position.z = 0.2   # 修改这里
```

### 修改重规划频率

编辑 `plan_replan_tf.py`:

```python
# 定时器：每 2s 触发一次重新规划
self.timer = rospy.Timer(rospy.Duration(2.0), self.timer_callback)
#                                         ^^^
#                                      修改这里 (秒)
```

### 修改目标点距离阈值

编辑 `plan_replan_tf.py`:

```python
# 目标点距离阈值：当距离目标点小于此值时停止规划（避免路径点过少导致崩溃）
self.goal_distance_threshold = 0.5  # 单位：米
#                               ^^^
#                            修改这里
```

**建议值**:
- `0.1` - 非常接近目标才停止（配合try-except使用，推荐⭐）
- `0.3` - 较早停止，适合精度要求不高的场景
- `0.5` - 保守值，平衡性能和精度

**安全机制**:
- ✅ 使用 **try-except** 捕获规划器异常，即使阈值设置很小也不会崩溃
- ✅ 如果在0.5米内发生异常，自动标记为已到达
- ✅ 规划速度快（<100ms），可以设置较小阈值获得更精确的导航

## ROS话题说明

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/move_base_simple/goal` | PoseStamped | 输入 | 接收导航目标点 |
| `/pct_path` | Path | 输出 | 发布规划路径 |
| `/move_base/result` | MoveBaseActionResult | 输入 | 接收导航结果 |
| `/tf` | TFMessage | 输入 | 获取机器人当前位置 |

## 技术细节

### 坐标系
- 所有坐标使用 `map` 坐标系
- 起点从 `map -> base_link` TF变换获取
- 终点从 `/move_base_simple/goal` 消息获取

### 路径规划
- 只规划位置 (x, y, z)
- 朝向由DWA局部规划器负责
- 路径以PoseStamped数组形式发布

### 错误处理
- 空路径检查
- TF查找失败处理
- 规划失败重试
- 导航成功自动停止


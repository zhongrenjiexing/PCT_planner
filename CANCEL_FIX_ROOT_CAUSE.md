# 取消功能真正的根本原因和修复

## 🎯 发现真正的根本原因！

### 问题诊断

之前我们一直在修复 DWA 的自动目标发布逻辑，但真正的根本原因是：

**`plan_replan_tf.py` 持续发布路径，即使用户已经发送了 cancel！**

### 完整的问题循环

```
1. 用户发送目标到 /move_base_simple/goal
   ↓
2. plan_replan_tf.py 收到目标
   → goal_callback 触发
   → navigation_reached = False
   → 开始每 1 秒规划并发布路径到 /pct_raw_path
   ↓
3. DWA 收到 /pct_raw_path
   → 自动发布目标回 /move_base_simple/goal
   ↓
4. ❌ plan_replan_tf.py 又收到这个自动发布的目标！
   → 又触发 goal_callback
   → 又重置 navigation_reached = False
   → 继续发布路径
   ↓
5. 用户发送 cancel 到 /move_base/cancel
   → DWA 检测到并停止自动发布
   → move_base 状态变为 PREEMPTED
   ↓
6. ❌ 但是 plan_replan_tf.py 完全不知道！
   → 继续每秒发布路径到 /pct_raw_path
   → DWA 收到路径（虽然不再自动发布目标）
   → 但路径一直在更新
   ↓
7. 无限循环！机器人无法真正停止
```

### 为什么会这样？

**`plan_replan_tf.py` 的停止条件不完整**：

```python
# 之前只检查这两个：
if not self.goal_received or self.end_pos is None:
    return
    
if self.navigation_reached:  # 只在距离太近或收到 success 时停止
    return

# ❌ 没有检查是否被取消！
```

## ✅ 修复方案

### 1. 添加 cancel 检测

监听 `/move_base/status` 话题，检测取消事件：

```python
# 添加导入
from actionlib_msgs.msg import GoalStatusArray

# 添加状态标志
self.navigation_cancelled = False

# 订阅状态话题
self.status_sub = rospy.Subscriber(
    "/move_base/status",
    GoalStatusArray,
    self.status_callback,
    queue_size=1
)
```

### 2. 实现状态回调

```python
def status_callback(self, msg):
    """监听 move_base 状态，检测取消事件"""
    if not msg.status_list:
        return
    
    latest_status = msg.status_list[-1]
    
    # status = 2: PREEMPTED (被取消)
    # status = 4: ABORTED (失败)
    # status = 5: REJECTED (被拒绝)
    if latest_status.status in [2, 4, 5]:
        if not self.navigation_cancelled:
            rospy.logwarn("🛑 检测到导航被取消/中止 (status=%d)，停止发布路径", 
                         latest_status.status)
            self.navigation_cancelled = True
            self.goal_received = False  # 停止直到下一次明确的目标
```

### 3. 在定时器中检查取消状态

```python
def timer_callback(self, event):
    # ... 其他检查 ...
    
    # 如果导航已被取消，停止发布路径
    if self.navigation_cancelled:
        rospy.loginfo_throttle(5.0, "导航已取消，停止发布路径")
        return
    
    # ... 继续规划和发布 ...
```

### 4. 收到新目标时重置取消状态

```python
def goal_callback(self, msg):
    # ... 设置目标 ...
    
    self.goal_received = True
    self.navigation_reached = False
    self.navigation_cancelled = False  # 重置取消状态
    
    # ... 开始规划 ...
```

## 修复后的流程

```
1. 用户发送目标
   → plan_replan_tf.py 开始发布路径
   → DWA 自动发布目标（带防护标志）
   → 机器人移动
   ↓
2. 用户发送 cancel
   → move_base 状态变为 PREEMPTED (2)
   ↓
3. ✅ plan_replan_tf.py 检测到 status=2
   → navigation_cancelled = True
   → 停止发布路径到 /pct_raw_path
   ↓
4. ✅ DWA 也检测到 status=2
   → enable_auto_goal_publish = false
   → 停止自动发布目标
   ↓
5. ✅ 机器人真正停止！
```

## 三层防护

现在我们有了三层防护机制：

### 第1层：plan_replan_tf.py
```python
if self.navigation_cancelled:  # 检测到 cancel
    return  # 停止发布路径
```

### 第2层：DWA 状态回调
```python
if (current_status == 2):  # PREEMPTED
    enable_auto_goal_publish_ = false;  # 停止自动发布目标
```

### 第3层：DWA 防自循环
```python
if (currently_auto_publishing_) {  // 忽略自己发布的目标
    return;
}
```

## 测试步骤

### 1. 重启 plan_replan_tf.py

```bash
# 停止旧的 plan_replan_tf.py（Ctrl+C）
# 重新启动
rosrun planner plan_replan_tf.py --scene sii_l6
```

### 2. 重启 move_base（如果还没重启）

```bash
# 停止 move_base（Ctrl+C）
# 重新启动导航系统
```

### 3. 测试

```bash
# 发送目标
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {position: {x: -1.5, y: 4.5, z: 1.0}, orientation: {w: 1.0}}
}' --once
```

**预期日志（plan_replan_tf.py）**：
```
📍 接收到新的导航目标: (-1.500, 4.500, 1.200)
开始路径规划...
🛣️  Planning: 2D=5.123m, 3D=5.234m, ...
✅ 规划成功，原始路径 46 个点，优化后 23 个点
```

```bash
# 发送取消
rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {} --once
```

**预期日志（plan_replan_tf.py）**：
```
🛑 检测到导航被取消/中止 (status=2)，停止发布路径
导航已取消，停止发布路径
```

**预期日志（move_base/DWA）**：
```
[MOVE_BASE_DEBUG] *** PREEMPT REQUESTED *** NewGoal=0  ← 应该是 0！
[DWA_DEBUG] *** Auto-goal publishing DISABLED *** (status=2)
[MOVE_BASE_DEBUG] *** PREEMPTED - RETURNING FROM executeCb ***
```

### 4. 验证停止

等待几秒，确认：
- ✅ plan_replan_tf.py 不再输出规划日志
- ✅ 不再发布 /pct_raw_path
- ✅ move_base 显示 PREEMPTED
- ✅ 机器人完全停止

### 5. 重新启动测试

发送新目标，确认可以重新开始导航。

## 状态码参考

| 状态码 | 状态名 | 含义 | plan_replan_tf.py 行为 |
|--------|--------|------|----------------------|
| 0 | PENDING | 等待处理 | 继续发布路径 |
| 1 | ACTIVE | 正在执行 | 继续发布路径 |
| 2 | PREEMPTED | 被取消 | **停止发布路径** ✅ |
| 3 | SUCCEEDED | 成功完成 | 停止发布路径 |
| 4 | ABORTED | 失败中止 | **停止发布路径** ✅ |
| 5 | REJECTED | 被拒绝 | **停止发布路径** ✅ |

## 为什么之前的修复不够？

### 只修复 DWA（不够）
- DWA 停止自动发布目标 ✅
- 但 plan_replan_tf.py 继续发布路径 ❌
- 路径持续更新，系统仍然活跃

### 只修复 plan_replan_tf.py（也不够）
- plan_replan_tf.py 停止发布路径 ✅
- 但如果 DWA 还在自动发布目标 ❌
- 会形成另一种循环

### 两者都修复（完美！）
- plan_replan_tf.py 停止发布路径 ✅
- DWA 停止自动发布目标 ✅
- DWA 防止自己的目标触发循环 ✅
- **真正停止！** ✅

## 架构图

### 修复前（无法停止）

```
User Goal → plan_replan_tf.py → /pct_raw_path → DWA → Auto Goal
    ↑                                                      ↓
    └──────────────────────────────────────────────────────┘
           (plan_replan_tf.py 收到自动目标，继续发布)

User Cancel → ??? (没人监听)
```

### 修复后（可以停止）

```
User Goal → plan_replan_tf.py → /pct_raw_path → DWA → Auto Goal (有防护)
                ↓                                   ↓
         status_callback                    statusCallback
                ↓                                   ↓
User Cancel → /move_base/status → status=2 (PREEMPTED)
                ↓                                   ↓
         navigation_cancelled=true     enable_auto_goal_publish=false
                ↓                                   ↓
         停止发布路径                       停止自动发布目标
                           ↓
                    ✅ 机器人停止！
```

## 总结

这次修复解决了**真正的根本原因**：

1. **之前的问题**：
   - ❌ plan_replan_tf.py 不监听 cancel
   - ❌ 持续发布路径
   - ❌ DWA 自动发布目标形成循环

2. **现在的解决方案**：
   - ✅ plan_replan_tf.py 监听 `/move_base/status`
   - ✅ 检测到 cancel 时停止发布路径
   - ✅ DWA 也停止自动发布目标
   - ✅ DWA 防止自己的目标触发循环
   - ✅ **三层防护，确保真正停止！**

**现在应该可以正常工作了！请重启 plan_replan_tf.py 和 move_base，然后测试！**


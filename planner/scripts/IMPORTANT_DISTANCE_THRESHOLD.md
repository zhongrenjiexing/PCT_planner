# ⚠️ 重要：距离阈值配置说明

## 🐛 核心问题

C++ 优化器 (`gpmp_optimizer.cc`) 在 `SubSamplePath` 函数中有断言：
```cpp
assert(path.size() > 1)  // 第204行
```

当机器人距离目标点**太近**时：
- 规划器生成的路径点数量 ≤ 1
- C++ 断言失败 → 发送 SIGABRT 信号
- 进程崩溃：`Aborted (core dumped)`

## 🛡️ 安全机制

代码中已实现**多层保护**：

### 第1层：距离预检查（最重要）
```python
min_safe_distance = max(self.goal_distance_threshold, 0.3)  # 至少0.3米
if distance < min_safe_distance:
    # 停止规划，标记为已到达
```

### 第2层：try-except（对C++断言无效）
```python
try:
    traj_3d = self.planner.plan(start_pos, self.end_pos)
except Exception as e:
    # 注意：C++的assert无法被捕获！
```

### 第3层：路径点数量检查
```python
if len(traj_3d) <= 1:
    # 阻止发布，标记为已到达
```

## ⚙️ 配置建议

### `goal_distance_threshold` 参数（第55行）

| 值 | 效果 | 风险 | 推荐场景 |
|----|------|------|----------|
| **0.1米** | 非常接近才停止 | ⚠️ **高风险崩溃** | ❌ 不推荐 |
| **0.3米** | 较接近时停止 | ✅ 安全 | ✅ 平衡推荐 |
| **0.5米** | 保守停止 | ✅ 非常安全 | ✅ 生产环境 |
| **0.8米** | 很早停止 | 可能过早 | 大型场景 |

### 当前强制最小值：**0.3米**

即使设置 `0.1米`，代码也会强制使用 `max(0.1, 0.3) = 0.3米`

## 📊 实际测试结果

### ✅ 安全配置
```python
self.goal_distance_threshold = 0.3  # 或更大
```
- 0.35米处停止规划 ✅
- 不会崩溃 ✅
- DWA 继续处理最后0.3米 ✅

### ❌ 危险配置（已被强制修正）
```python
self.goal_distance_threshold = 0.1  # 用户设置
# 实际使用：max(0.1, 0.3) = 0.3  ← 代码自动修正
```

## 🔧 如果仍然崩溃

### 1. 检查日志中的实际距离
查找日志：
```
🛣️  Planning: distance=0.XXXm
```

如果距离 < 0.3米 仍在规划，说明检查没生效。

### 2. 增大最小安全距离

编辑第139行：
```python
min_safe_distance = max(self.goal_distance_threshold, 0.5)  # 改为0.5或更大
```

### 3. 查看路径点数量
查找日志：
```
✅ 规划成功，生成 N 个路径点
```

如果 N ≤ 1，说明距离太近。

### 4. 降低重规划频率

编辑第103行：
```python
self.timer = rospy.Timer(rospy.Duration(2.0), self.timer_callback)  # 改回2秒
```

更慢的频率给DWA更多时间完成最后阶段。

## 💡 原理解释

### 为什么 try-except 捕获不了？

```
Python                          C++
  ↓                              ↓
try:                         assert(path.size() > 1)
  planner.plan()  --------→     ↓ (失败)
except Exception:              abort()  ← 直接终止进程
  # 永远不会执行               ↓
                            SIGABRT
                               ↓
                          Process Killed
```

**C++ 的 assert 失败会调用 `abort()`，直接发送信号终止进程，不会抛出 Python 异常！**

### 唯一有效的方法

**在调用 C++ 代码前，用 Python 检查距离！**

```
距离检查 → 太近就不调用 → 避免 C++ 断言失败
```

## 📝 总结

- ✅ **必须**使用足够大的距离阈值（≥ 0.3米）
- ✅ 代码已强制最小值为 0.3米
- ✅ 如仍崩溃，增大到 0.5米或更大
- ⚠️ try-except **无法**捕获 C++ 断言
- ⚠️ 唯一方法是**提前检查距离**

## 🔗 相关文件

- `plan_replan_tf.py` 第55行：`goal_distance_threshold`
- `plan_replan_tf.py` 第139行：`min_safe_distance` 计算
- `gpmp_optimizer.cc` 第204行：C++ 断言位置


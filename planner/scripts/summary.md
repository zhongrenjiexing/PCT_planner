# A*轨迹浮空问题排查总结

## 问题描述
使用567_-ready-to-pctplanner.ply点云规划的轨迹有一段浮空，不紧贴tomogram。

## 根本原因发现
通过诊断脚本check_astar_raw.py发现：
- **A*原始路径中有80%的点（前10个中的8个）使用了无效的地面高度数据（ground_h=-100.00）**
- 这些点位于layer=3，但在layer=3的这些位置，elev_g是NaN（被转换为-100）
- A*在没有有效地面数据的区域规划路径，导致严重浮空（平均偏差82.473m，最大103.100m）

## tomogram数据特征
1. 共9个slice（layer 0-8）
2. 相邻slice之间高度不连续，很多位置高度差>0.3m，最大达12.77m
3. 许多位置在某些slice没有有效地面数据（NaN）

## 已完成的修改

### 1. 放宽高度约束（a_star_search.cc）
- 将邻居节点高度差约束从0.3m提高到2.0m（第148-156行）
- 将DecideLayer高度差约束从0.2m提高到2.0m（第215-277行）

### 2. 改进DecideLayer逻辑（a_star_search.cc）
- 添加了对无效地面数据的检测（height < -50.0）
- 当当前层无效时，强制切换到有有效数据的层
- 优先选择高度最接近且有效的层

### 3. 邻居节点过滤（a_star_search.cc）
- 在扩展邻居时，跳过没有有效地面数据的节点（第143行）

### 4. Python层面的findValidSlice改进（planner_wrapper.py）
- 添加了基于elev_g和elev_c范围的slice选择
- 添加了详细的调试输出

## 当前状态
修改已编译成功，但测试显示问题仍然存在。A*仍在使用无效数据的节点。

## 进一步排查需要
1. **检查DecideLayer是否在每次邻居扩展时都被调用** ✓（第134行确认每次都调用）
2. **验证DecideLayer的层切换是否真的生效**（需要添加调试输出）
3. **检查起始节点的层选择**（可能需要在Search开始时就调整）

## 可能的后续解决方案
1. **在A*搜索开始时强制验证起始节点**：如果起始层无效，立即切换到有效层
2. **更激进的层切换策略**：在DecideLayer中，如果当前层无效，直接切换到最近的有效层，不管高度差多大
3. **修改tomogram生成**：改进点云处理，确保相邻slice之间有更好的连续性
4. **后处理优化**：在轨迹优化阶段，强制轨迹贴合最近的有效地面

## 关键代码位置
- A*搜索主循环：`a_star_search.cc` 第105-185行
- DecideLayer函数：`a_star_search.cc` 第197-277行  
- 邻居扩展：`a_star_search.cc` 第134-184行
- tomogram加载：`planner_wrapper.py` 第37-63行
- slice选择：`planner_wrapper.py` 第142-229行

## 诊断脚本
- `diagnose_floating.py`: 检查tomogram数据质量和统计
- `test_plan.py`: 测试规划并分析浮空程度
- `check_astar_raw.py`: 对比A*原始路径和优化后轨迹


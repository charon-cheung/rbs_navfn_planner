# Navfn Planner

The NavfnPlanner is a global planner plugin for the Nav2 Planner server. It implements the Navigation Function planner with either A\* or Dij. expansions. It is largely equivalent to its counterpart in ROS 1 Navigation. The Navfn planner assumes a circular robot (or a robot that can be approximated as circular for the purposes of global path planning) and operates on a weighted costmap.

The `global_planner` package from ROS (1) is a refactor on NavFn to make it more easily understandable, but it lacks in run-time performance and introduces suboptimal behaviors. As NavFn has been extremely stable for about 10 years at the time of porting, the maintainers felt no compelling reason to port over another, largely equivalent (but poorer functioning) planner. 

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-navfn.html) for additional parameter descriptions.

## 本地生产代码修改

以下修改位于 `src/navfn_planner.cpp`，作用于路径提取后的输出处理，同时适用于 A* 和 Dijkstra，不改变搜索算法，也不增加配置参数。

### 路径点朝向与行进方向对齐

在 `makePlan()` 中，根据路径几何方向填写非终点 pose 的朝向，减少路径位置与朝向不一致引起的 controller 前向兼容性校验失败：

- 首个有效平移段的起点朝向取该段方向。
- 中间点朝向取入段与出段方向的角平分方向，按最短角差处理跨越 ±π 的情况。
- 计算方向时跳过重合点，并为同组重合点填写一致的朝向。
- 保留 `use_final_approach_orientation` 的终点朝向策略：关闭时保留目标朝向，开启时使用末段接近方向。

该处理不修改中间路径点的位置，也不保证任意急转弯都满足 controller 的运动约束。

### 修复栅格路径连接精确终点时的短折角

在 `smoothApproachToGoal()` 中，先替换或追加终点，再检查最后两段之间的转角。若转角超过约 45°，尝试在很短的末段范围内重新连接终点：

- 按原路径累计弧长向前寻找连接点，最多回溯 `4 × costmap resolution`，且不使用路径首点作为连接点。
- 新连接方向与连接点前一段的转角必须不超过约 45°，并跳过退化的短段或重合点。
- 检查新连接线的栅格包围矩形及周围一格；范围内不得存在内切膨胀障碍、致命障碍或未知栅格，代价不得超过已检查原末段采样点及终点的最大代价。
- 检查通过后，以不大于半个地图分辨率的间距直线插值连接终点，保留连接点及其之前的路径。
- 找不到符合条件的连接点时，保留替换或追加终点后的路径，不强行修直。

这里的终点是规划器在目标容差内选定的 `best_pose`。安全检查沿用 Navfn 基于膨胀代价地图的圆形机器人模型；这是局部末段修复，不是全路径平滑，也不负责修复 controller 的路径进度落后或终点停车控制问题。

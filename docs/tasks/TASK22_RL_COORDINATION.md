# Task22：Learning-based Dual-arm Coordination / 深度强化学习研究线

状态：⚪ 研究增强线，Task16--21 主线稳定后实施。

## 目标

探索深度强化学习在松/紧协调码垛中的作用，但不直接删除 MoveIt / Task08 / FCL 安全链。

最终输入可来自：

```text
Camera / perception
  -> cube initial pose + size
Placement Planner
  -> target placement pose
```

学习策略根据当前双臂状态与任务目标生成协调动作、轨迹参数或航点。

## 三种动作层级

### Level 1 — 高层协调策略（最推荐先做）

Policy 输出：

```text
arm assignment
LOOSE / TIGHT mode
priority
local wait / time scaling
candidate grasp index
candidate placement index
```

底层仍由 MoveIt + Task08/FCL 执行。

优点：动作空间小、训练稳定、最容易与现有系统结合。

### Level 2 — 轨迹 / 航点生成（推荐第二阶段）

Policy 不直接输出电机控制，而输出：

```text
intermediate Cartesian waypoint(s)
preferred redundant posture
trajectory timing / progress rate
```

随后：

```text
RL proposal
  -> IK / trajectory construction
  -> Task08/FCL safety verification
  -> execute if SAFE
```

这是“学习生成轨迹”与现有工程最自然的结合点。

### Level 3 — 直接双臂运动控制（研究价值最高、难度最高）

Policy 直接输出：

```text
14-D joint delta / joint velocity
```

或：

```text
12-D left/right Cartesian delta
```

优点：理论上可学习非常灵活的在线协调。

缺点：

- 动作维度高；
- reward shaping 难；
- 双臂互碰、关节极限、奇异性、吸附事件都要处理；
- 训练可能出现 reward hacking；
- sim-to-real 风险最高；
- 很难直接给出强安全保证。

因此不建议作为第一版。

## 推荐研究路线

```text
22-A  RL scheduler / coordinator
  ↓
22-B  RL waypoint + timing generator
  ↓
22-C  optional low-level dual-arm policy
```

## Observation 建议

至少包含：

```text
left/right joint position
left/right joint velocity
left/right TCP pose
cube current pose
cube target pose
cube size / mass
current task stage
attached state
other placed boxes / compact environment representation
relative arm-arm geometry / distance features
```

第一版可以只训练单个 Box 的随机 pick-to-place，再扩展多 Box。

## Action 方案建议

第一版优先：

```text
[coordination_mode,
 yielding_arm,
 wait_or_timescale,
 waypoint_delta_left,
 waypoint_delta_right]
```

而不是直接 14-D 电机控制。

## Reward 建议

正奖励：

```text
task success
cube-to-goal progress
short makespan
small coordination delay
smooth motion
large collision clearance
stable shared-object geometry (tight mode)
```

惩罚：

```text
collision
near collision
joint limit
singularity / low manipulability
excessive path length
large acceleration / jerk
dropped object
invalid grasp
planner/FCL rejection
```

## Safety Shield

学习策略永远不作为最终安全判据。

```text
RL candidate
   ↓
kinematic / joint-limit validation
   ↓
Task08 / FCL or tight-mode full dual-arm geometry check
   ↓
SAFE -> execute
UNSAFE -> reject / re-sample / fallback classical planner
```

后续若研究在线低层控制，可再考虑 CBF-QP safety filter。

## Curriculum

建议训练顺序：

```text
1. 单臂随机 reach
2. 单臂随机 cube pick-to-place
3. 双臂各自独立 reach，无交叉
4. 双臂交叉 reach + collision penalty
5. 双臂 loose pick-to-place
6. 动态任务分配 / 时间协调
7. shared-object tight coordination
8. mixed-size mixed-mode palletizing
```

不要从最终完整混合码垛直接开始训练。

## 感知接入顺序

```text
第一阶段：Isaac Ground Truth
第二阶段：GT + noise / domain randomization
第三阶段：相机估计 pose
```

保持 policy 输入接口不变，使 perception 与 control 可独立替换。

## 需要比较的 baseline

学习方法必须与现有经典方法比较：

```text
Task16 RobustPlanner
Task09 minimum local wait
Task21 rule-based coordination router
```

指标：

```text
success rate
collision rate
makespan
planning/inference time
path length
smoothness
wait time
robustness to randomized initial poses
robustness to size classes
```

## 验收标准

```text
T22-01  建立可批量并行训练的 randomized environment
T22-02  Policy 输入来自通用 BoxSpec / PlacementSpec，而非硬编码坐标
T22-03  至少完成高层协调或 waypoint policy 的训练与 play
T22-04  未经 FCL / safety shield 通过的动作不得进入正式执行链
T22-05  与 classical baseline 做统一随机种子测试
T22-06  仅在 Level 1/2 已有明确收益后，再决定是否做直接低层 14-D 控制
```

## 当前建议

Task22 是重要的论文增强方向，但不应阻塞 Task16--21。优先让经典系统先具备：鲁棒规划、随机初始位置、自动 placement、多尺寸和自动模式路由；这样才能给 RL 提供稳定环境、清晰 baseline 与大量可自动生成的训练 episode。

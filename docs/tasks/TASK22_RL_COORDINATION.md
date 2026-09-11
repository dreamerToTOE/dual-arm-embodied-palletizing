# Task22：Learning-based Dual-arm Coordination / 深度强化学习研究线

> **状态：⚪ 仅讨论 / 远期研究备忘录。当前不实施、不编码、不训练。**
>
> **启动条件：只有在经典主线已经完整稳定后，且项目时间允许时，才重新评估是否开展 Task22。至少应先完成并稳定 Task08/FCL 安全链、Task16 规划鲁棒性、Task17--21 的随机多尺寸自主码垛主线。**
>
> 本文件当前只用于保留研究想法，不能被解释为近期开发任务，也不应阻塞或改变 Task16--21 的实施顺序。

## 1. 讨论目标

未来如有余力，可探索深度强化学习在松/紧协调码垛中的作用，但不直接删除 MoveIt / Task08 / FCL 安全链。

最终输入可来自：

```text
Camera / perception
  -> cube initial pose + size
Placement Planner
  -> target placement pose
```

学习策略根据当前双臂状态与任务目标生成协调动作、轨迹参数或航点。

当前阶段只讨论方案，不进行训练环境、网络、reward 或部署代码的实际开发。

---

## 2. DRL 与 Task16 RobustPlanner 的区别

Task16 的经典鲁棒规划器负责：

```text
RRTConnect 生成多个候选完整路径
        ↓
经典指标评分
        ↓
剔除不良路径
        ↓
选择最佳可行路径
```

它是一个**候选路径择优器**。

Task22 中更推荐讨论的 Level 2 DRL 不是简单替代这个评分器，而是更进一步：

```text
当前双臂状态 + pick/place + box/environment
        ↓
DRL Policy
        ↓
主动提出中间 Cartesian waypoint(s)
preferred redundant posture
trajectory timing / progress rate
        ↓
IK / trajectory construction
        ↓
Task08/FCL safety verification
```

因此二者职责不同：

```text
Task16 RobustPlanner：
“已有多条完整路径，哪一条最好？”

Task22 Level 2 DRL：
“我主动提出应该从哪里绕、何时通过，构造一条候选轨迹骨架。”
```

未来如果 Task22 真正实施，可以把 Task16 作为 classical baseline，与 DRL waypoint/timing proposal 做统一随机任务对比。

---

## 3. 三种动作层级（仅研究备忘）

### Level 1 — 高层协调策略

Policy 可输出：

```text
arm assignment
LOOSE / TIGHT mode
priority
local wait / time scaling
candidate grasp index
candidate placement index
```

底层仍由 MoveIt + Task08/FCL 执行。

优点：动作空间小、训练相对稳定、最容易与现有系统结合。

### Level 2 — 轨迹骨架 / 航点生成（当前认为最值得未来研究）

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

这是“学习生成轨迹骨架”与现有工程最自然的结合点。

### Level 3 — 直接双臂运动控制

Policy 直接输出：

```text
14-D joint delta / joint velocity
```

或：

```text
12-D left/right Cartesian delta
```

该方案研究价值高，但训练难度、安全验证和 sim-to-real 风险都最高，因此即使未来启动 Task22，也不作为第一阶段目标。

---

## 4. 推荐研究顺序（仅未来参考）

如果未来决定真正启动 Task22，建议顺序为：

```text
22-A  RL scheduler / coordinator
  ↓
22-B  RL waypoint + timing generator
  ↓
22-C  optional low-level dual-arm policy
```

在 Task16--21 未完成前，不进入上述任何实现阶段。

---

## 5. Observation 建议

未来至少包含：

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

如果未来训练闭环低层策略，不能仅使用 Cube 起点和码垛终点，因为 policy 还必须知道机器人当前状态。

---

## 6. Action 方案建议

如未来从较低风险版本开始，可优先考虑：

```text
[coordination_mode,
 yielding_arm,
 wait_or_timescale,
 waypoint_delta_left,
 waypoint_delta_right]
```

而不是直接 14-D 电机控制。

---

## 7. Reward 讨论

未来可能的正奖励：

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

Task14 的连续 `relative_tcp_error`、箱体相对双 TCP 中点误差和姿态误差，未来可直接成为紧协调 RL 的 reward / constraint 候选指标。

---

## 8. Safety Shield 原则

即使未来实施 Task22，学习策略也永远不作为最终安全判据。

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

只有在后续研究在线低层控制时，才再讨论 CBF-QP 等 safety filter。

---

## 9. 未来可能的 Curriculum

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

不从最终完整混合码垛直接开始训练。

---

## 10. 感知接入顺序

如果未来启动：

```text
第一阶段：Isaac Ground Truth
第二阶段：GT + noise / domain randomization
第三阶段：相机估计 pose
```

保持 policy 输入接口不变，使 perception 与 control 可独立替换。

---

## 11. 未来需要比较的 classical baseline

学习方法若实施，必须与现有经典方法比较：

```text
Task16 RobustPlanner
Task09 minimum local wait
Task21 rule-based coordination router
```

指标可包括：

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

---

## 12. 当前项目决策

**截至当前版本，Task22 不进入实施。**

近期唯一主线仍然是：

```text
Task16  Planning Robustness
  ↓
Task17  Box / Task Parameterization
  ↓
Task18  Randomized Scene
  ↓
Task19  Placement Planner
  ↓
Task20  Multi-size Palletizing
  ↓
Task21  Loose/Tight Coordination Router
```

当上述经典系统稳定、Task08/FCL 安全验证完整、随机多尺寸码垛可以批量运行后，如果项目仍有充足时间和算力，再回到本文件重新决定是否启动 DRL。

在此之前：

```text
DO NOT IMPLEMENT
DO NOT TRAIN
DO NOT BLOCK CLASSICAL PIPELINE
```

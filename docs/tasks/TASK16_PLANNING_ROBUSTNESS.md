# Task16：Planning Robustness / 鲁棒运动规划

状态：⚪ 待实现。

## 目标

降低当前 FR3 + MoveIt2 + RRTConnect 因随机采样、冗余 IK、路径质量差异带来的完整任务失败率。

Task16 不要求更换主 planner。第一版仍以 `RRTConnectkConfigDefault` 为主，通过“多候选生成 + 候选质量评分 + 冗余姿态管理 + 明确 benchmark”提高稳定性。

## 当前基线

当前 `PalletizePrimitive`：

```text
planner = RRTConnectkConfigDefault
planning_time = 5 s
MoveIt internal attempts = 1
outer retries = 3
```

问题：

```text
同一 start / goal / scene
→ 不同随机运行得到不同 IK / path
→ 有时路径过近、接近关节极限、后续 Cartesian 段失败
→ 双臂组合后还可能增加 Task08 冲突或 Task09 等待代价
```

## 16-A：多候选规划器

对需要自由空间规划的阶段，不再接受第一条成功路径。

```text
start / goal
  ↓
生成 N 条可行候选
  ↓
逐条检查
  ↓
计算 cost
  ↓
选择最低代价候选
```

第一版建议 `N=3~5`，作为参数而非硬编码结论。

### 候选评分

建议至少记录：

```text
path_length
joint_limit_cost
minimum_clearance / near-collision proxy
singularity_cost
planning_time
predicted_dual_arm_conflict_cost
predicted_wait_cost
```

候选目标函数可从简单加权开始：

```text
J = w_len * path_length
  + w_limit * joint_limit_cost
  + w_sing * singularity_cost
  + w_clear * clearance_cost
  + w_coord * coordination_cost
```

第一版无需一次实现全部项。建议顺序：

1. 路径长度；
2. 关节极限余量；
3. 双臂 Task08 冲突 / Task09 等待代价；
4. 再增加奇异性和最小距离。

## 16-B：FR3 冗余 IK 管理实验

不预设“固定一个关节一定更好”。比较三组：

### Group A — 7-DOF 自由

当前基线：允许 FR3 使用全部冗余自由度。

### Group B — 硬固定一个冗余关节

目的：验证将有效搜索维数从 7 降到 6 后，是否减少 IK / 路径随机性。

注意：

- 可能提高重复性；
- 也可能缩小工作空间；
- 可能让某些 pick / place pose 无解；
- 可能减少绕障和远离奇异点的能力。

因此仅作为 benchmark 组，不直接设为默认。

### Group C — 软冗余偏好

推荐作为长期方案：不锁死关节，只惩罚偏离期望姿态。

例如：

```text
J_redundancy = (q_redundant - q_preferred)^2
```

或通过 joint constraint / IK seed / IK solution ranking 让肘部优先保持在稳定区域，同时仍允许必要时利用第 7 自由度绕障。

### 验收数据

每种模式至少统计：

```text
IK success rate
planning success rate
full primitive success rate
mean planning attempts
mean planning time
joint-limit margin
Task08 conflict rate
Task09 additional wait
```

## 16-C：确定性段与随机规划段分离

标准结构动作继续优先使用确定性 Cartesian 段：

```text
PRE_PICK -> CONTACT
CONTACT -> LIFT
PRE_PLACE -> PLACE
PLACE -> RETREAT
```

自由空间阶段才由鲁棒候选规划器处理：

```text
HOME/current -> PRE_PICK
LIFT -> PRE_PLACE
RETREAT -> SAFE_EGRESS/HOME
```

避免把所有动作都交给随机采样 planner。

## 16-D：Benchmark Harness

新增独立 benchmark，不执行 Isaac 物理动作。

输入：

```text
scenario
random_seed / run_index
number_of_trials
planner_candidate_count
redundancy_mode
```

输出建议保存 CSV/JSON：

```text
trial_id
stage
plan_success
candidate_count
selected_cost
planning_time
path_length
min_joint_margin
Task08_safe
Task09_strategy
Task09_wait
```

## 验收标准

Task16 不预先写死“必须达到 99%”。验收以可重复实验和相对提升为准：

```text
T16-01  能对同一 stage 生成多条 RRTConnect 候选
T16-02  能按明确 cost 选出一条候选，而不是第一条成功即执行
T16-03  记录失败原因和候选评分
T16-04  完成 7-DOF / hard-lock / soft-preference 三组对照
T16-05  完成固定 Task15 场景的多次 benchmark
T16-06  最终执行候选仍通过现有 Task08/FCL 安全门禁
T16-07  根据数据确定默认冗余策略，不凭单次演示决定
```

## 不在本 Task 范围

- 不要求加入备用 planner；
- 不做固定轨迹缓存；
- 不加入相机；
- 不改变码垛点规划；
- 不做深度强化学习。

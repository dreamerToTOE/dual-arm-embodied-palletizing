# Task16：Planning Robustness / 鲁棒运动规划

状态：🟢 第一版已完成（2026-09-12）。

## 目标

降低当前 FR3 + MoveIt2 + RRTConnect 因随机采样、冗余 IK、路径质量差异带来的完整任务失败率。

Task16 不要求更换主 planner。第一版仍以 `RRTConnectkConfigDefault` 为主，通过“多候选生成 + 候选质量评分 + 冗余姿态管理 + 明确 benchmark”提高稳定性。

## 当前基线

Task16 实现前 `PalletizePrimitive`：

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

## 实现

新增 `RobustPlanner`，并接入 `PalletizePrimitive` 的自由空间阶段：

```text
HOME/current -> PRE_PICK
LIFT -> PRE_PLACE
RETREAT -> SAFE_EGRESS/HOME
```

每一个阶段显式调用 `RRTConnectkConfigDefault` 共 `N` 次（每次 MoveIt
内部 attempts 固定为 1），保留每一候选的结果，再按下式选择最低 `J`，而
非“第一条成功即采用”：

```text
J = 1.0 * normalized_joint_path_length
  + 0.2 * joint_limit_cost
  + 0.2 * soft_redundancy_cost

joint_limit_cost = 1 / (minimum_normalized_joint_margin + 0.02)
```

其中软冗余项仅在 `soft_preference` 模式启用，使用 FR3 arm group 的
`joint7`（或显式参数指定的冗余变量）相对 stage 起始值的归一化平方偏差。
所有候选仍由 MoveIt 完整 Planning Scene 做碰撞检查；该评分不会扩大 ACM，
也不会取代 Task08/FCL 门禁。

标准结构段保持确定性 Cartesian：

```text
PRE_PICK -> CONTACT
CONTACT -> LIFT
PRE_PLACE -> PLACE
PLACE -> RETREAT
```

新增只读程序 `task16_planning_benchmark`。它构造 Task15 下层参考场景，
生成左右完整 `TaskTrajectoryCandidate`，再复用 Task08
`SpatioTemporalConflictDetector` 与 Task09 `LocalWaitCoordinator`；它只发布
合成的初始 `joint_states`，不发布关节命令、吸盘命令或 Isaac 物理动作。

输出路径由用户参数指定，程序不会创建目录。CSV 逐候选记录：

```text
trial_id, redundancy_mode, arm, stage, candidate_index, selected,
plan_success, candidate_count, planning_time_sec, path_length,
min_joint_limit_margin, joint_limit_cost, redundancy_cost, total_cost,
full_primitive_success, task08_safe, task09_strategy, task09_wait_sec,
failure_reason
```

JSON 则汇总每一次 trial 的双臂 primitive、Task08 和 Task09 结果。

## 固定场景正式对照（2026-09-12）

共同条件：Task15 lower-layer reference、`random_seed=20260912`、3 trials、
每个随机自由空间阶段 `N=5`、Release build、同一 headless MoveIt 场景。
原始 CSV/JSON 位于本机 `artifacts/`，作为可再生实验产物，不提交版本库。

| 模式 | 双臂完整 primitive | 候选成功率 | 选中路径均长 | 选中最小 joint margin 均值 | Task08 初始安全 | Task09 可协调 | 平均额外等待 |
| --- | --- | --- | ---: | ---: | --- | --- | ---: |
| `free_7dof` | 3/3 | 88/90 (97.8%) | 0.820912 | 0.035307 | 1/3 | 3/3 | 0.400 s |
| `hard_lock_joint` | 0/3 | 27/60* | 1.230780 | 0.011790 | 未进入 | 未进入 | — |
| `soft_preference` | 3/3 | 90/90 (100%) | 0.783359 | 0.036217 | 1/3 | 3/3 | 0.200 s |

\* 硬锁定能生成部分 HOME -> PRE_PICK 候选，但三个 trial 的左右臂均在
`LIFT -> PRE_PLACE` 的全部 5 个候选失败，故并非可执行策略。

结论：在此固定 Task15 场景中，`soft_preference` 保持了完整可达性，
候选成功率和路径/关节余量指标略优于自由 7-DOF，并将 Task09 平均等待从
0.400 s 降为 0.200 s。因此当前 Task15 Phase B 的默认
`redundancy_mode` 设为 `soft_preference`，`planner_candidate_count` 默认为 3。
这不是对所有未来工作空间的永久结论：新增场景、障碍物或末端工具后必须重跑
本 benchmark，再决定是否调整默认。

## 验证命令

先在一个终端启动 headless MoveIt：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=15
export ROS_LOCALHOST_ONLY=1
ros2 launch fr3_dual_compact_suction_description \
  moveit_dual_compact_suction.launch.py use_rviz:=false
```

在第二个终端构建并运行三组 benchmark（以下仅展示 soft-preference；将模式和
输出文件名替换为 `free_7dof`、`hard_lock_joint` 即可复现完整对照）：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_DOMAIN_ID=15
export ROS_LOCALHOST_ONLY=1

ros2 run fr3_dual_palletize task16_planning_benchmark --ros-args \
  -p number_of_trials:=3 \
  -p planner_candidate_count:=5 \
  -p redundancy_mode:=soft_preference \
  -p random_seed:=20260912 \
  -p output_csv:=/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/artifacts/task16_soft_20260912.csv \
  -p output_json:=/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/artifacts/task16_soft_20260912.json
```

`task15_hybrid_palletizing.launch.py` 也暴露了同名参数，可在真实 Task15
一键紧/松协调运行时显式覆盖，例如：

```bash
ros2 launch fr3_dual_palletize task15_hybrid_palletizing.launch.py \
  execute:=true planner_candidate_count:=5 redundancy_mode:=soft_preference
```

## 验收结论

- T16-01 至 T16-03：通过。每个自由空间 stage 都有 5 条独立候选、失败原因和
  完整评分日志。
- T16-04：通过。三种冗余模式均完成同条件比较；硬锁定的负结果已记录。
- T16-05：通过。Task15 固定场景完成每组 3 trials，并输出 CSV/JSON。
- T16-06：通过。成功 candidate 必须再经 Task08；初始冲突时 Task09 仅在
  找到安全 LocalWait 后才报告可协调。
- T16-07：通过。默认选择来自上述数据，当前为 `soft_preference`，不是单次
  可视化演示的预设。

## 不在本 Task 范围

- 不要求加入备用 planner；
- 不做固定轨迹缓存；
- 不加入相机；
- 不改变码垛点规划；
- 不做深度强化学习。

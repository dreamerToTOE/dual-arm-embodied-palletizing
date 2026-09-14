# Task21：Automatic Loose / Tight Coordination Router

状态：🟡 可解释 Router 已接入 loose 与 generic tight runtime preflight；物理执行
验收仍待后续阶段（2026-09-14）。

## 目标

让系统根据 Box 和任务属性自动决定使用松协调还是紧协调，而不是由场景脚本预先指定“大件=tight，小件=loose”。

输入：

```text
BoxSpec
pick pose / grasp candidates
PlacementSpec
robot reachability
payload limits
```

输出：

```text
LOOSE_LEFT
LOOSE_RIGHT
LOOSE_DUAL_PARALLEL
TIGHT_SHARED_OBJECT
NO_FEASIBLE_MODE
```

## 第一版路由规则

先使用可解释规则，不先使用神经网络：

```text
1. 单臂负载是否允许
2. 单臂是否可达 pick + place
3. 顶部单吸盘稳定性是否允许
4. 两臂共同抓取点是否可行
5. 与其他并行任务的冲突 / makespan 代价
```

示意：

```text
single-arm feasible && payload safe
    -> LOOSE candidate

single-arm infeasible && dual grasp feasible
    -> TIGHT candidate

两者都可行
    -> 比较预计 makespan / planning cost / robustness
```

## 与 Task16/19 联动

模式选择不能只看箱体尺寸。

例如同一个中等箱体：

- 左臂当前不可达但右臂可达 -> 单臂 loose；
- 单臂抓取稳定性不足 -> tight；
- 两臂 tight 会严重阻塞另外任务 -> loose 可能更优。

因此 Router 应允许调用快速 reachability / candidate score，而不是简单阈值 `size > X`。

## 已实现接口

`coordination_router.hpp/.cpp` 是不依赖 Isaac 或 MoveIt action 的纯决策层。调用方提供
已经评估好的事实，而非让 Router 根据 `size > X` 猜测：

```text
LooseModeEstimate (left / right)
  allowed_by_task
  payload_safe
  pick_reachable + place_reachable
  top_suction_stable
  fcl_safe
  estimated_duration_sec + planning_cost

TightModeEstimate
  dual_grasp_feasible
  payload_safe
  shared_transport_planner_ready
  geometry_safe
  estimated_duration_sec + planning_cost
```

所有 boolean 都是硬门禁；只有通过门禁的 `LOOSE_LEFT`、`LOOSE_RIGHT` 和
`TIGHT_SHARED_OBJECT` 才按

```text
duration_weight * estimated_duration_sec
+ planning_cost_weight * planning_cost
```

比较。因此路由理由可以追溯到“负载、完整可达性、吸盘稳定性、FCL”中的任一失败项。
`decideLooseParallel()` 还要求两个单臂 route 不同、Task08/FCL pair safe 以及 Task09
安全时间调度同时通过，才输出 `LOOSE_DUAL_PARALLEL`。

## 与 Task20-B 的实际接入

`task20_motion_preflight` 现在对每个 runtime loose Box：

1. 为每一条允许的单臂模式运行完整 Task16 candidate；
2. 以被动另一臂为基准执行 Task08/FCL；
3. 将真实的 candidate 成败、时长和 FCL 结果填入 Router；
4. 只接受 Router 最终选择的 `LOOSE_LEFT` 或 `LOOSE_RIGHT`。

这不是把最短轨迹绕开 Router 的旁路：如果左臂的 candidate 失败，`left.pick_reachable /
place_reachable` 就不成立；如果 FCL 拒绝，`left.fcl_safe` 就不成立。紧协调 runtime
输入由 `SharedObjectPlanner` 写入真实的 `geometry_safe`、时长、planning cost 与诊断；
该候选必须通过 private-scene Shared Box FCL、两个 TCP grasp 约束与 Box-path 约束，
Router 才能输出 `TIGHT_SHARED_OBJECT`。

`loose_payload_limit_kg` 是显式运行参数（默认 `0.50` kg），不是尺寸类别 if/else。
Task18 当前只发布已验证的顶部吸盘抓取 candidate；未来稳定性模型可替换上游
`top_suction_stable` 事实，Router 接口无需变更。

## 回归验证

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run fr3_dual_palletize task21_coordination_router_demo
```

该 demo 已覆盖：

```text
完整 candidate cost 选择 LOOSE_LEFT
左臂 pick IK 不可达时选择 LOOSE_RIGHT
tight-only 且 shared planner / geometry 均可行时选择 TIGHT_SHARED_OBJECT
tight-only 且 generic shared planner 不可用时选择 NO_FEASIBLE_MODE
两条不同单臂任务经 Task08 + Task09 后选择 LOOSE_DUAL_PARALLEL
```

另在双 FR3 MoveIt 隔离回归中，将一个运行时大件和一个运行时小件送入 Task20-B；
实际日志为：

```text
object=task20_large route=TIGHT_SHARED_OBJECT ... shared-object FCL + dual TCP constraint PASS
object=task20_small route=LOOSE_LEFT ... Task16 robust + Task08/FCL PASS
seed=20260922 tasks=2 loose_safe=1 tight_safe=1 tight_deferred=0 executed=0
Task20-B PASS
```

另外在 3 次同一 runtime `large_shared_box` 采样回归中，tight route 均通过；时长范围
19.816--21.720 s，private-scene FCL 样本范围 615--814。未向 Isaac 发布任何机器人或
吸盘执行命令。

## 验收标准

```text
T21-01  模式不再由 Task15 Phase A/B 硬编码
T21-02  至少基于尺寸/质量/单臂可达性给出模式
T21-03  两种模式均不可行时明确拒绝
T21-04  Router 输出理由与评分，便于调试
T21-05  ≥6 类尺寸随机任务中自动路由
T21-06  最终候选仍分别走现有 loose / tight 安全门禁
```

## 当前验收状态

- ✅ T21-01：无 Task15 Phase A/B 硬编码分流；模式来自 `allowed_modes` 与评估事实。
- ✅ T21-02：负载、完整路径可达性、吸盘稳定性与 FCL 都是 route 输入与硬门禁。
- ✅ T21-03：没有可行模式时输出 `NO_FEASIBLE_MODE`，Task20-B 拒绝而不执行。
- ✅ T21-04：每个 mode 保留 score 与具体拒绝理由。
- ✅ T21-05（preflight）：混合 runtime episode 自动选择 tight 大件与 loose 小件，
  并已通过组合回归。
- ✅ T21-06（preflight）：loose 最终候选经过 Task08/FCL；tight 最终候选经过
  Shared Box private FCL、双 TCP grasp 与 Box-path 门禁。物理执行验收仍待后续阶段。

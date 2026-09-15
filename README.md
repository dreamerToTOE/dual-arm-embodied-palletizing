# 双机械臂具身智能码垛

面向混合尺寸箱体码垛的双机械臂协调规划与控制项目。

> 🧊 **路线冻结（2026-09-15）**：Task04–Task22-D 的顶部吸盘实现、场景和验收记录已作为可复现实验基线冻结并保留。根据教师要求，后续末端执行器主线切换为 FR3 二指夹爪；在新的夹爪抓取与规划策略确定前，不再扩展吸盘功能。

当前工程采用 **Isaac Sim 4.5 + ROS 2 Humble + MoveIt 2 / OMPL + Franka FR3**，主线目标是完成：

```text
小箱体：双臂分别抓取、并行码垛（松协调）
大箱体：双臂顶部双吸盘共同搬运与放置（紧协调）
```

## 项目文档

- [总体项目规划](docs/PROJECT_PLAN.md)
- [启动与运行指南](docs/STARTUP_GUIDE.md)
- [关键问题与排错记录](docs/KEY_ISSUES.md)
- [Task 00 — FR3 / Isaac / ROS 2 / MoveIt 基线](docs/tasks/TASK00_BASELINE.md)
- [Task 01 — 单机械臂 Pick & Place](docs/tasks/TASK01_PICK_PLACE.md)
- [Task 02 — Placement Skill](docs/tasks/TASK02_PLACEMENT_SKILL.md)
- [Task 03 — B-A-C 二指夹爪可执行性](docs/tasks/TASK03_BAC_EXECUTABILITY.md)
- [Task 04 — 顶部吸盘单臂基线](docs/tasks/TASK04_TOP_SUCTION_BASELINE.md)
- [Task 04-B — 九 Cube 三维码垛](docs/tasks/TASK04B_NINE_CUBE_3D_PALLETIZING.md)
- [Task 05 — B-A-C 顶部吸盘验证](docs/tasks/TASK05_BAC_SUCTION.md)
- [Task 05-C — MoveIt / Isaac 紧凑吸盘模型对齐](docs/tasks/TASK05C_COMPACT_SUCTION_MOVEIT.md)
- [Task 06 — 双 FR3 + 双紧凑吸盘基线](docs/tasks/TASK06_DUAL_FR3_BASELINE.md)
- [Task 07 — 双臂松协调并行码垛](docs/tasks/TASK07_LOOSE_PARALLEL.md)
- [Task 08 — 时空冲突检测设计](docs/tasks/TASK08_SPATIOTEMPORAL_CONFLICT.md)
- [Task 08-A — 完整任务轨迹候选](docs/tasks/TASK08_FULL_TASK_TRAJECTORY.md)
- [Task 09-A — 松协调延迟启动](docs/tasks/TASK09_TEMPORAL_COORDINATION.md)
- [Task 09-B — 冲突窗口局部等待](docs/tasks/TASK09_LOCAL_WAIT.md)
- [Task 09-B — 安全退出 + 局部等待](docs/tasks/TASK09_SAFE_EGRESS.md)
- [Task 10 — 松协调连续多箱码垛](docs/tasks/TASK10_CONTINUOUS_PALLETIZING.md)
- [Task 11 — 双吸盘共同物体基线](docs/tasks/TASK11_SHARED_OBJECT_BASELINE.md)
- [Task 12 — 双吸盘共同抬升与运输](docs/tasks/TASK12_SHARED_BOX_LIFT.md)
- [Task 13 — 双吸盘共同放置、释放与安全退出](docs/tasks/TASK13_SHARED_BOX_PLACE.md)
- [Task 14-A — 双吸盘共同搬运连续几何观测](docs/tasks/TASK14_CONTINUOUS_GEOMETRY.md)
- [Task 15 — 紧/松协调混合码垛场景与复用策略](docs/tasks/TASK15_HYBRID_PALLETIZING.md)
- [Task 16 — Planning Robustness / 鲁棒规划基准](docs/tasks/TASK16_PLANNING_ROBUSTNESS.md)
- [Task 16+ — 通用化主线与路线图](docs/tasks/TASK16_PLUS_ROADMAP.md)
- [Task 17 — Box 参数化作业模型](docs/tasks/TASK17_BOX_ABSTRACTION.md)
- [Task 18 — 随机场景与 Ground Truth 运行时输入](docs/tasks/TASK18_RANDOMIZED_SCENE.md)
- [Task 19 — 自动码垛点规划](docs/tasks/TASK19_PLACEMENT_PLANNER.md)
- [Task 20 — 多尺寸连续码垛](docs/tasks/TASK20_MULTI_SIZE_PALLETIZING.md)
- [Task 20-C — Runtime Isaac 单任务物理执行验收](docs/tasks/TASK20_RUNTIME_ISAAC_EXECUTION.md)
- [Task 20-D — Runtime 多对象连续调度物理验收](docs/tasks/TASK20_CONTINUOUS_SCHEDULING.md)
- [Task 20-E — Runtime 反馈闭环与效率基准](docs/tasks/TASK20E_RUNTIME_FEEDBACK_EFFICIENCY.md)
- [Task 21 — 松/紧协调自动路由](docs/tasks/TASK21_COORDINATION_ROUTER.md)
- [Task 22 — Ground Truth 在线任务分派与预规划](docs/tasks/TASK22_GT_RUNTIME_DISPATCH.md)
- [Task 23 — 二指夹爪暂存后平推二层验证](docs/tasks/TASK23_GRIPPER_STAGE_PUSH.md)
- [吸盘路线冻结记录（Task04–Task22-D）](docs/tasks/SUCTION_BASELINE_FREEZE.md)
- [项目 Skill — 松协调独立物体码垛](ros_ws/src/fr3_dual_palletize/skills/loose_coordination_palletizing.yaml)
- [项目 Skill — 紧协调共同物体搬运](ros_ws/src/fr3_dual_palletize/skills/tight_coordination_shared_object.yaml)

## 当前进度

| Task | 内容 | 状态 |
|---|---|---|
| 00 | FR3 + Isaac + ROS 2 + MoveIt 基线 | ✅ |
| 01 | 二指夹爪单臂 Pick & Place | ✅ |
| 02 | Placement Skill | ✅ |
| 03 | B-A-C 二指夹爪高密度可执行性 | ✅ |
| 04 | 顶部紧凑吸盘单臂码垛基线 | ✅ |
| 05 | 同一 B-A-C 场景顶部吸盘插入 + MoveIt/Isaac 模型对齐 | ✅ |
| 06 | 双 FR3 + 双紧凑吸盘基础设施 | ✅ |
| 07 | 双臂松协调并行码垛 | ✅ |
| 08 | 双臂完整任务轨迹与时空冲突检测 | ✅ Task08-A/B 完成：SAFE 与 CONFLICT 均已验收 |
| 09 | 松协调时间协调 | ✅ Task09-C 已在 Task08 交叉场景完成一次 Isaac 双臂抓取、放置和安全退出运行时验收 |
| 10 | 松协调完整码垛 Demo | ✅ 四箱、两批次连续 Isaac 码垛完成；每批均经 FCL 验收，第二批保留 A/B 为碰撞物 |
| 11 | 双吸盘共同物体基线 | ✅ 双 Surface Gripper 已对同一 SharedBox 同时 CLOSED；未抬升阶段位移 0.115 mm |
| 12 | 双吸盘共同抬升与运输 | ✅ +50 mm 抬升与 +100 mm X 向共同运输均通过；运输误差 0.311 mm、相对 `link8` 误差 0.123 mm |
| 13 | 双吸盘共同放置、释放与安全退出 | ✅ 完整闭环通过；最终放置误差 0.947 mm、姿态误差 0.026° |
| 14 | 紧协调连续几何观测 | 🟡 已实现双 TCP / SharedBox 连续 max-RMS 误差观测，待 Isaac 验收 |
| 15 | 紧/松协调混合码垛 | 🟡 已形成紧/松复用契约与自包含场景；保留为后续通用执行器的回归基线 |
| 16 | 规划鲁棒性 | ✅ RobustPlanner 多候选生成、评分与 benchmark 已完成 |
| 17 | Box / Placement 参数化 | ✅ YAML BoxSpec / PlacementSpec、任意数量和尺寸的 CollisionObject 已验证 |
| 18 | 随机场景 + Ground Truth 输入 | ✅ 固定 seed `20260912` Isaac Ground Truth → BoxStateArray → runtime pick pose 端到端通过 |
| 19 | 自动 Placement Planner | ✅ 几何 height-map、支撑/碰撞门禁、候选排序与可达性 fallback 已通过离线回归；选择模型接口已开放 |
| 20 | 多尺寸连续码垛 | 🟠 Task20-C 单大件紧协调闭环已通过；Task20-D 三件连续运行在第 3 件出现 92.411 mm 横向放置误差，未通过。Task20-E 已加入真实 TCP release gate、非规则中等共享箱体和效率 profile，待 Isaac 验收 |
| 21 | 松/紧协调自动路由 | 🟡 Router 已实际选择并执行一次 `TIGHT_SHARED_OBJECT` runtime route；loose runtime 与多对象调度的物理验收仍待完成 |
| 22 | Ground Truth 在线任务分派与预规划 | 🧊 吸盘实现冻结：已完成 `GT → Python selector → MoveIt/FCL → 物理执行 → settle/World Commit → 下一件` 串行闭环与三件 Isaac 连续验收；不再扩展吸盘路线，后续以夹爪重新建立执行基线 |
| 23 | 二指夹爪暂存后平推二层验证 | 🟡 已建立独立双 FR3 + 官方手指场景、16 件 Ground Truth 选择器及 `抓取 → 暂存 → 单臂 +X 推送` 执行器；待 Isaac 物理验收 |

## 已冻结的吸盘基线结论

Task03 / Task05 形成直接对照：

```text
同一 B-A-C 几何

二指夹爪：
C_insert = FAIL

紧凑顶部吸盘：
C_insert = PASS
完整放置 / 释放 / RETREAT = PASS
```

这项对照是顶部紧凑吸盘路线的已验证实验结论。该路线现已冻结；后续会以二指夹爪重新建立抓取、放置和协调规划策略，不能直接沿用吸盘的接触、附着或释放结论。

另外，Franka 官方 `cobot_pump` MoveIt 环境碰撞包络与本项目实际紧凑吸盘不一致，会在 B-A-C 插入中产生假碰撞。项目已新增自定义：

```text
ros_ws/src/fr3_compact_suction_description
```

几何与 Isaac 保持一致：

```text
stem radius = 6 mm
stem length = 99 mm
cup radius  = 10 mm
cup length  = 6 mm
TCP offset  = 105 mm
```

## 冻结的吸盘码垛事件单元

Task04 的最终成功时序作为后续 Task 的项目级基线：

```text
PRE_PICK
→ CONTACT
→ SUCTION ON
→ ATTACH
→ LIFT
→ TRANSFER / PRE_PLACE
→ PLACE
→ 释放前先规划 RETREAT
→ SUCTION OFF
→ settle
→ Isaac Ground Truth 同步
→ DETACH / addWorld
→ RETREAT
```

该事件单元仅作为吸盘路线的可复现实验参考。夹爪主线需建立独立的抓取、物体附着、释放、碰撞检查和验收定义。

## 当前稳定平台

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Isaac Sim 4.5.0
- NVIDIA GeForce RTX 3070 8 GB
- NVIDIA 580 系列驱动
- Python 3.10.12
- Franka FR3
- `franka_description` 2.8.1
- `libfranka` 0.20.4
- MoveIt 2 / OMPL

## 当前阶段：Task16–Task19 通用化主线

Task06 已完成双臂基础设施，Task07 已完成两条独立 Task04 primitive 的并行执行。

Task08 已生成完整任务候选：

```text
HOME -> PRE_PICK -> CONTACT -> ATTACH -> LIFT
-> PRE_PLACE -> PLACE -> DETACH -> RETREAT
```

每个候选同时包含完整关节轨迹和 ATTACH / DETACH 的统一时间事件。Task08-B 已在 10 ms 联合采样下通过两项实际验收：Task07 分离通道为 `SAFE`，Task08 交叉通道报告 `ARM_ARM` 冲突。

Task09-A 保留为 Full-task Start Delay baseline。Task09-B 已将 MoveIt 规划的 `RETREAT -> HOME SAFE_EGRESS` 与冲突窗口局部等待结合：若安全退出路径已经分离，则保持并行；否则只在 LIFT 后插入最小 HOLD。Task09-C 只消费 Task08-B/FCL 复检为 SAFE 的联合候选，以共享时钟同步驱动 Isaac 双臂，并在吸附、释放事件处全局 HOLD。2026-09-09 已在 Task08 交叉场景实际完成双臂抓取、放置与安全退出；详见 [Task09-C 记录](docs/tasks/TASK09_COORDINATED_EXECUTION.md)。

Task11--Task13 已完成同一 `SharedBox` 的双吸盘共同吸附、共同抬升、共同运输、共同放置、同步释放和安全退出。Task15 将其封装为仓库内的紧协调复用契约；独立小件则复用松协调契约。它们不再是继续复制状态机的目标，而是后续通用任务的执行基础。

Task16--Task19 已将固定 Demo 升级为配置与运行时输入驱动的链路：

```text
Task17 YAML BoxSpec / PlacementSpec
        +
Task18 Isaac Ground Truth BoxStateArray（seed 可复现）
        ↓
Task19 Placement Planner
  几何候选 + 支撑/边界/碰撞门禁
  + 可替换 PlacementSelectionModel
        ↓
Task20 多尺寸连续码垛
        ↓
Task21 松/紧协调自动路由
```

Task18 的 `seed=20260912` 已实际通过 Isaac 端到端输入验收：3 个随机物体的
`id / pose / size / mass / grasp candidate` 从 `/task18/box_states` 进入 ROS，成功
构造对应 CollisionObject 与 runtime pick pose。Task19 随后在 Task15 兼容作业上
自动重现大件 `(0.650, 0.000, 0.090)` 的放置点，并为四个小件生成多个大件顶面
候选；可达性预检拒绝 28 个候选后，成功回退到下一个完整支撑候选。

当前默认的 `geometric_v1` 只负责候选选择。后续可以通过
`PlacementSelectionModel` 注入自定义启发式、优化器或学习模型；模型只能为已通过
几何门禁的候选打分，不能绕过支撑、边界、碰撞、MoveIt 可达性或 Task08/FCL 安全链。

# 双机械臂具身智能码垛

面向混合尺寸箱体码垛的双机械臂协调规划与控制项目。

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
| 09 | 松协调时间协调 | ✅ 安全退出 + 局部等待已生成 Task08 交叉场景 SAFE 候选；实际双臂执行待后续验收 |
| 11+ | 双吸盘共物体与紧协调 | 计划中 |

## 当前核心工程结论

Task03 / Task05 形成直接对照：

```text
同一 B-A-C 几何

二指夹爪：
C_insert = FAIL

紧凑顶部吸盘：
C_insert = PASS
完整放置 / 释放 / RETREAT = PASS
```

因此项目固定采用顶部紧凑吸盘，不再围绕二指夹爪设计高密度放置补救策略。

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

## 基础码垛事件单元

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

后续 Task 默认不重新设计这套基础事件，只在其外部增加目标选择、双臂调度、碰撞协调或共物体约束。

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

## 当前阶段：Task09

Task06 已完成双臂基础设施，Task07 已完成两条独立 Task04 primitive 的并行执行。

Task08 已生成完整任务候选：

```text
HOME -> PRE_PICK -> CONTACT -> ATTACH -> LIFT
-> PRE_PLACE -> PLACE -> DETACH -> RETREAT
```

每个候选同时包含完整关节轨迹和 ATTACH / DETACH 的统一时间事件。Task08-B 已在 10 ms 联合采样下通过两项实际验收：Task07 分离通道为 `SAFE`，Task08 交叉通道报告 `ARM_ARM` 冲突。

Task09-A 保留为 Full-task Start Delay baseline。Task09-B 已将 MoveIt 规划的 `RETREAT -> HOME SAFE_EGRESS` 与冲突窗口局部等待结合：若安全退出路径已经分离，则保持并行；否则只在 LIFT 后插入最小 HOLD。当前 Task08 交叉场景已得到 Task08-B/FCL 的 SAFE 联合候选；实际执行验收仍由后续任务完成。

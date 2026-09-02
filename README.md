# 双机械臂具身智能码垛

面向混合码垛任务的双机械臂具身智能协调规划与控制项目。

当前工程采用 **Isaac Sim 4.5 + ROS 2 Humble + MoveIt 2 / OMPL + Franka FR3** 作为主要仿真、运动规划与控制验证平台。

## 项目文档

- [总体项目规划](docs/PROJECT_PLAN.md)
- [启动与运行指南](docs/STARTUP_GUIDE.md)
- [关键问题与排错记录](docs/KEY_ISSUES.md)
- [Task 00 — FR3 / Isaac / ROS 2 / MoveIt 基线](docs/tasks/TASK00_BASELINE.md)
- [Task 01 — 单机械臂 Pick & Place](docs/tasks/TASK01_PICK_PLACE.md)
- [Task 02 — 机器人可执行 Placement Skill](docs/tasks/TASK02_PLACEMENT_SKILL.md)
- [Task 03 — B-A-C 放置可执行性](docs/tasks/TASK03_BAC_EXECUTABILITY.md)

## 当前进度

| 任务 | 内容 | 状态 |
|---|---|---|
| Task 00 | FR3 + Isaac Sim + ROS 2 + MoveIt 2 基线 | ✅ 已完成 |
| Task 01 | 单机械臂 Pick & Place | ✅ 已完成 |
| Task 02 | Placement Skill / 机器人可执行放置 | ✅ 已完成：参数化输入、阶段判定、结构化结果、正常目标全链路验证通过 |
| Task 03 | B-A-C 放置可执行性 | 🟡 进行中：构造“箱体可放但夹爪不可插”的邻箱场景 |
| Task 04 | 放置顺序调整 | 计划中 |
| Task 05+ | 双臂松协调、紧协调与具身技能路由 | 计划中 |

## 已验证基础链路

```text
MoveIt 2 / OMPL
        ↓
RRTConnect / Cartesian Path
        ↓
RobotTrajectory / JointTrajectory
        ↓
ROS 2 执行节点
        ↓
100 Hz 插值
        ↓
/joint_command
        ↓
Isaac Sim ROS 2 Bridge
        ↓
Articulation Controller
        ↓
FR3 仿真执行
```

Isaac Sim 发布：

```text
/clock
/joint_states
/tf
```

Isaac Sim 接收：

```text
/joint_command
```

## 当前稳定环境

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Isaac Sim 4.5.0
- NVIDIA GeForce RTX 3070 8 GB
- NVIDIA 580 系列驱动
- Python 3.10.12（`/usr/bin/python3`）
- Franka ROS 2 `humble`
- `franka_description` 2.8.1
- `libfranka` 0.20.4
- MoveIt 2 / OMPL

## Task 01 已验证基线

当前 PickCube：

```text
Center = (0.45, 0.15, 0.065) m
Size   = 0.03 × 0.03 × 0.03 m
Yaw    = 0 rad
```

完整链路：

```text
HOME
→ ALIGNED_APPROACH
→ OPEN_GRIPPER
→ Cartesian GRASP
→ CLOSE_GRIPPER
→ MoveIt ATTACH
→ Cartesian LIFT
→ TRANSFER
→ PLACE
→ MoveIt DETACH
→ removeWorldCube
→ OPEN / RELEASE
→ RETREAT
→ addPlacedCubeToWorld
→ HOME
```

关键修复详见 [关键问题与排错记录](docs/KEY_ISSUES.md)。

## Task 02 已验证能力

Task 02 将固定放置流程封装为：

```text
PlacementTarget
→ C_reach
→ C_insert
→ C_release
→ C_retreat
→ PlacementSkillResult
```

正常目标已验证：

```text
C_reach   = PASS
C_insert  = PASS
C_release = PASS
C_retreat = PASS
RESULT    = SUCCESS
```

## 当前 Task 03

目标是验证：

```text
A 箱体本体几何上可以放入 B、C 之间
但携带 A 的真实夹爪扫掠体无法插入
```

第一版场景：

```text
A target = (0.65, -0.15, 0.065) m
A size   = 30 mm

B center = (0.65, -0.115, 0.065) m
C center = (0.65, -0.185, 0.065) m
B/C size = 30 mm

B-C 净间距 = 40 mm
```

因此 A 本体 30 mm 有约 5 mm/侧几何余量，但预期 FR3 finger / hand 无法随 A 一起完成垂直插入。

这将直接验证：

```text
几何可放置 ≠ 机器人可执行放置
```

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

其中：

```text
docs/PROJECT_PLAN.md
→ 项目总体路线和论文映射

docs/STARTUP_GUIDE.md
→ 长期保存已经验证过的启动顺序、编译方式、运行命令和场景参数

docs/KEY_ISSUES.md
→ 长期保存关键故障的现象、原因、错误理解、解决方案、验证结果和工程意义
```

## 当前进度

| 任务 | 内容 | 状态 |
|---|---|---|
| Task 00 | FR3 + Isaac Sim + ROS 2 + MoveIt 2 基线 | ✅ 已完成 |
| Task 01 | 单机械臂 Pick & Place | ✅ 已完成 |
| Task 02 | Placement Skill / 机器人可执行放置 | 🟡 进行中 |
| Task 03 | B-A-C 放置可执行性 | 计划中 |
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
Pick & Place 执行节点
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

Isaac Sim 通过 ROS 2 发布：

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

## 快速启动

### 终端 1：启动 MoveIt

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

### Isaac Sim

打开当前 FR3 + Table + PickCube 场景，确认 ROS 2 ActionGraph 正常，然后点击 **Play**。

### 终端 2：编译并运行

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --packages-select fr3_moveit_test \
  --symlink-install

source install/setup.bash

ros2 run fr3_moveit_test single_arm_pick_place
```

完整启动与排错记录见：[启动与运行指南](docs/STARTUP_GUIDE.md)。

## Task 01 最终验证链

当前 PickCube：

```text
Center = (0.45, 0.15, 0.065) m
Size   = 0.03 × 0.03 × 0.03 m
Yaw    = 0 rad
```

已验证完整链路：

```text
HOME
↓
ALIGNED_APPROACH
↓
OPEN_GRIPPER
↓
Cartesian Z-only GRASP
↓
CLOSE_GRIPPER
↓
MoveIt ATTACH
↓
Cartesian Z-only LIFT
↓
Cartesian TRANSFER
↓
Cartesian PLACE
↓
MoveIt DETACH
↓
removeWorldCube
↓
OPEN / RELEASE
↓
Cartesian RETREAT
↓
addPlacedCubeToWorld
↓
RRTConnect HOME
```

其中两个关键修复已验证：

```text
1. Cartesian Path 统一使用 base 参考坐标系；
2. DETACH 后在释放过渡阶段临时移除 World Cube，再退出并重新加入 Planning Scene。
```

详细原因见 [关键问题与排错记录](docs/KEY_ISSUES.md)。

## 当前 Task 02

Task 02 将 Task 01 中固定的放置流程升级成可复用、可判定的 Placement Skill：

```text
PlacementTarget
      ↓
C_reach
      ↓
PRE_PLACE
      ↓
C_insert
      ↓
PLACE
      ↓
C_release
      ↓
RELEASE
      ↓
C_retreat
      ↓
SUCCESS / FAILURE + 失败原因
```

第一阶段保持当前空桌面场景不变，先完成放置目标参数化、技能封装和结构化结果；Task 03 再加入 B-A-C 邻近箱体干涉。

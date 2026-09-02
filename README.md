# 双机械臂具身智能码垛

面向混合码垛任务的双机械臂具身智能协调规划与控制项目。

当前工程采用 **Isaac Sim 4.5 + ROS 2 Humble + MoveIt 2 / OMPL + Franka FR3** 作为主要仿真、运动规划与控制验证平台。

## 项目文档

- [总体项目规划](docs/PROJECT_PLAN.md)
- [启动与运行指南](docs/STARTUP_GUIDE.md)
- [Task 00 — FR3 / Isaac / ROS 2 / MoveIt 基线](docs/tasks/TASK00_BASELINE.md)
- [Task 01 — 单机械臂 Pick & Place](docs/tasks/TASK01_PICK_PLACE.md)

其中 `docs/PROJECT_PLAN.md` 是项目总体路线的主要记录文件；`docs/STARTUP_GUIDE.md` 用于长期保存已经验证过的启动顺序、编译方式、运行命令和关键场景参数。

## 当前进度

| 任务 | 内容 | 状态 |
|---|---|---|
| Task 00 | FR3 + Isaac Sim + ROS 2 + MoveIt 2 基线 | ✅ 已完成 |
| Task 01 | 单机械臂 Pick & Place | 🟡 进行中：稳定抓取与提升已验证，正在完成搬运与放置 |
| Task 02 | Placement Skill / 机器人可执行放置 | 计划中 |
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

打开当前 Task 01 场景，确认 FR3、Table、PickCube 和 ROS 2 ActionGraph 正常，然后点击 **Play**。

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

## 当前 Task 01 已验证状态

当前 PickCube：

```text
Center = (0.45, 0.15, 0.065) m
Size   = 0.03 × 0.03 × 0.03 m
Yaw    = 0 rad
```

已经多次重复验证稳定成功：

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
```

抓取采用完整 6D 末端位姿约束，而不是只要求 TCP 到达目标位置：

```text
TCP Z 轴竖直向下
夹爪两指方向与 Cube 侧面平齐
最终抓取阶段保持姿态，只沿 Z 轴下降
```

当前统一使用 MoveIt 模型参考坐标系：

```text
base
```

此前 `fr3_link0 -> base` 的 Cartesian Path 变换失败问题已经定位并解决。

## 下一步

继续完成 Task 01：

```text
LIFT
↓
TRANSFER
↓
PRE_PLACE
↓
PLACE
↓
DETACH
↓
OPEN / RELEASE
↓
RETREAT
↓
HOME
```

Task 01 完成后进入 **Task 02：机器人可执行 Placement Skill**，进一步研究抓取器空间、插入方向、周围箱体干涉、释放与退出可执行性。

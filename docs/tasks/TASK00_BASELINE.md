# Task 00 — FR3 + Isaac Sim + ROS 2 + MoveIt 2 基线

## 状态

✅ **已完成**

本任务用于建立论文工程开始码垛技能开发前最小且可重复验证的仿真与运动规划链路。

---

## 1. 已验证环境

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Python 3.10.12（`/usr/bin/python3`）
- Isaac Sim 4.5.0
- NVIDIA GeForce RTX 3070 8 GB
- NVIDIA 580 系列驱动
- MoveIt 2 / OMPL
- Franka ROS 2：`humble` 分支
- `franka_description`：2.8.1
- `libfranka`：0.20.4

### Python 环境隔离

ROS 2 在 Conda 环境之外运行。已经验证的检查命令：

```bash
source /opt/ros/humble/setup.bash
which python3
python3 --version
which ros2
```

预期：

```text
/usr/bin/python3
Python 3.10.12
/opt/ros/humble/bin/ros2
```

这样可以避免 Miniconda Python 与 ROS 2 Humble 系统包混用。

---

## 2. Isaac Sim 稳定基线

Isaac Sim 5.1 曾在 RTX 渲染初始化阶段崩溃。切换 NVIDIA 驱动并使用 Isaac Sim 4.5.0 后，仿真器可以稳定启动与运行。

因此当前工程固定采用：

```text
Ubuntu 22.04.5
+ ROS 2 Humble
+ Isaac Sim 4.5.0
+ NVIDIA 580 系列驱动
```

除非后续任务确实需要，否则不再主动升级 Isaac Sim 或 NVIDIA 驱动。

---

## 3. Isaac Sim 中的 FR3 模型

直接使用 Isaac Sim 内置 FR3 USD，而不是重新导入 URDF：

```text
Robots/Franka/FR3/fr3.usd
```

已经验证：

- Articulation Root 正常；
- `fr3_joint1` 到 `fr3_joint7` 存在；
- Franka hand 与两根 finger joint 存在；
- `fr3_hand_tcp` 存在；
- 固定底座正常；
- Play / Stop 稳定；
- 七个机械臂关节可独立位置控制；
- 夹爪开合正常。

工程场景以引用方式使用该 Isaac 资产，不修改原始 FR3 USD。

---

## 4. ROS 2 Bridge

Isaac ROS 2 Action Graph 已配置并验证。

已验证 topic：

```text
/clock
/joint_states
/tf
/joint_command
```

### `/joint_states`

Isaac Sim 实际发布的关节顺序：

```text
fr3_joint1
fr3_joint2
fr3_joint3
fr3_joint4
fr3_joint5
fr3_joint6
fr3_joint7
fr3_finger_joint1
fr3_finger_joint2
```

该命名作为 MoveIt 2 与 Isaac Sim 对接时的统一参考。

### 关节命令链路

Isaac 订阅：

```text
/joint_command
```

消息类型：

```text
sensor_msgs/msg/JointState
```

控制链：

```text
ROS 2 /joint_command
        ↓
ROS2 Subscribe Joint State
        ↓
Isaac Articulation Controller
        ↓
FR3 articulation
```

---

## 5. Franka ROS 2 与 MoveIt 安装记录

使用 Franka ROS 2 官方 Humble 仓库：

```bash
git clone -b humble https://github.com/frankarobotics/franka_ros2.git
```

使用官方依赖文件保持版本兼容：

```bash
vcs import src < src/franka_ros2/dependency.repos --recursive --skip-existing
```

关键版本：

```text
franka_description 2.8.1
libfranka          0.20.4
```

如果编译提示 `libfranka/common` 缺少 `CMakeLists.txt`，初始化 Git submodule：

```bash
cd src/libfranka
git submodule sync --recursive
git submodule update --init --recursive
```

安装依赖：

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y
```

完整工作空间构建方式：

```bash
colcon build \
  --symlink-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTS=OFF
```

该工作空间已经完整编译成功。

---

## 6. 官方 FR3 MoveIt 配置

不需要使用 MoveIt Setup Assistant 重新制作配置，直接使用：

```text
franka_fr3_moveit_config
```

### 已验证启动命令

```bash
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

已经验证：

- RViz 正常启动；
- `/move_group` 正常；
- `fr3_arm` 规划组正常；
- FR3 RobotModel 正常加载；
- OMPL 可以规划；
- `RRTConnectkConfigDefault` 存在；
- RViz 中 `Plan` 成功。

执行：

```bash
ros2 param get /move_group planning_pipelines
```

可能返回：

```text
Parameter not set
```

当前不视为错误，因为 `/move_group` 中实际存在 OMPL 的 RRTConnect 配置：

```text
RRTConnectkConfigDefault:
  type: geometric::RRTConnect
```

并且真实规划已经成功。

---

## 7. MoveIt → Isaac 轨迹执行链

工程沿用已经验证的 `moveit_to_isaac` 思路：

1. 等待 `/move_group` 参数服务；
2. 从 `/move_group` 复制 `robot_description` 和 `robot_description_semantic`；
3. 创建 `MoveGroupInterface("fr3_arm")`；
4. 选择 `RRTConnectkConfigDefault`；
5. 生成带碰撞约束的 `RobotTrajectory / JointTrajectory`；
6. 按 `time_from_start` 以 100 Hz 插值；
7. 将关节位置发布到 `/joint_command`；
8. Isaac Sim 通过 Articulation Controller 执行。

已验证链路：

```text
MoveIt 2 / OMPL
        ↓
RRTConnect
        ↓
RobotTrajectory / JointTrajectory
        ↓
100 Hz 插值执行节点
        ↓
/joint_command
        ↓
Isaac ROS 2 Bridge
        ↓
Articulation Controller
        ↓
FR3 物理仿真
```

### 基线测试关节配置

起点：

```text
HOME = [0.0, -0.7, 0.0, -2.2, 0.0, 2.0, 0.8]
```

目标：

```text
Pose A = [0.3, -0.8, 0.2, -2.3, 0.2, 2.1, 0.6]
```

### 结果

✅ MoveIt 规划成功。

✅ Isaac Sim 成功接收 `/joint_command`。

✅ FR3 从 HOME 平滑运动到 Pose A。

✅ 与此前 FR3 + Isaac + MoveIt 原型结果一致。

因此，新电脑上的“规划 → ROS 2 → Isaac 执行”链路已经恢复并验证。

---

## 8. Task 00 验收结果

| 项目 | 结果 |
|---|---|
| Isaac Sim 4.5 稳定启动 | ✅ |
| 内置 FR3 资产正常 | ✅ |
| FR3 Articulation Root | ✅ |
| 七个机械臂关节可控 | ✅ |
| 夹爪可控 | ✅ |
| ROS 2 `/clock` | ✅ |
| ROS 2 `/joint_states` | ✅ |
| ROS 2 `/tf` | ✅ |
| ROS 2 `/joint_command` | ✅ |
| Franka ROS 2 Humble 编译 | ✅ |
| 官方 `franka_fr3_moveit_config` 启动 | ✅ |
| `fr3_arm` 规划组 | ✅ |
| OMPL / RRTConnect 规划 | ✅ |
| MoveIt 轨迹转换为 `/joint_command` | ✅ |
| Isaac FR3 执行规划轨迹 | ✅ |

## 结论

**Task 00 已完成。**

项目已经具备后续所有技能开发所需的基础运动规划与仿真执行链路。

当前进入：

```text
Task 01 — 单机械臂 Pick & Place
```

完整启动与运行方式统一维护在：

```text
docs/STARTUP_GUIDE.md
```

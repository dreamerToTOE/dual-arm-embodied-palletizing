# 启动与运行指南

本文件用于长期保存当前项目已经验证过的启动顺序、编译命令、运行命令和常见检查项。后续如果更换对话、重启电脑或重新恢复工程，优先按照本文件操作。

## 1. 当前工程路径

项目根目录：

```text
~/lmy/dual-arm-embodied-palletizing
```

ROS 2 工作空间：

```text
~/lmy/dual-arm-embodied-palletizing/ros_ws
```

当前单臂抓放节点源码：

```text
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp
```

## 2. 当前稳定环境

```text
Ubuntu 22.04.5 LTS
ROS 2 Humble
Isaac Sim 4.5.0
MoveIt 2 / OMPL
Franka FR3
NVIDIA RTX 3070 8 GB
NVIDIA 580 系列驱动
Python 3.10.12 (/usr/bin/python3)
```

ROS 2 终端不要进入 Conda 环境。

可检查：

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

## 3. 终端 1：启动 MoveIt 2

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

正常情况下应看到 RViz 和 `/move_group` 启动成功。

规划组：

```text
fr3_arm
```

当前使用规划器：

```text
RRTConnectkConfigDefault
```

## 4. Isaac Sim 启动要求

打开当前 Task 01 场景，场景中至少需要包含：

```text
/World/fr3
/World/Table
/World/PickCube
ActionGraph
```

ActionGraph 已验证的 ROS 2 通信包括：

```text
/clock
/joint_states
/tf
/joint_command
```

`/joint_command` 消息类型：

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
/World/fr3
```

运行 ROS 控制时，Isaac 中本地 `Position_Controller` 保持关闭，避免与 ROS 指令冲突。

启动仿真后点击 **Play**。

## 5. 当前 Task 01 场景参数

### FR3

```text
Translate = (0, 0, 0)
Rotate    = (0, 0, 0)
```

### Table

```text
Center = (0.55, 0.00, 0.025) m
Size   = (1.20, 0.80, 0.05) m
Top Z  = 0.05 m
```

### PickCube

```text
Center = (0.45, 0.15, 0.065) m
Size   = (0.03, 0.03, 0.03) m
Yaw    = 0 rad
Mass   = 0.2 kg
```

当前 MoveIt 的统一规划参考坐标系使用：

```text
base
```

不要再把 Cartesian Path 的参考系单独设为 `fr3_link0`。此前已经确认 `fr3_link0 -> base` 的转换会导致：

```text
MoveIt error_code = -21
FRAME_TRANSFORM_FAILURE
```

## 6. 终端 2：编译当前节点

修改 `single_arm_pick_place.cpp` 后执行：

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --packages-select fr3_moveit_test \
  --symlink-install

source install/setup.bash
```

## 7. 终端 2：运行单臂 Pick & Place

```bash
ros2 run fr3_moveit_test single_arm_pick_place
```

当前已经重复验证稳定的前半段为：

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

30 mm Cube 已多次重试稳定夹起并提升。

## 8. 运行前检查

每次重新测试前建议确认：

```text
1. MoveIt /move_group 正常运行
2. Isaac Sim 已点击 Play
3. /joint_command 有 Isaac subscriber
4. FR3 恢复初始 HOME 状态
5. PickCube 恢复到 (0.45, 0.15, 0.065)
6. Cube 尺寸仍为 30 mm
```

可检查 ROS topic：

```bash
ros2 topic list | grep -E 'clock|joint_states|tf|joint_command'
```

## 9. 手动测试夹爪

打开夹爪：

```bash
ros2 topic pub --once \
  /joint_command sensor_msgs/msg/JointState \
  "{name: ['fr3_finger_joint1','fr3_finger_joint2'], position: [0.04,0.04]}"
```

Task 01 当前闭合目标：

```text
fr3_finger_joint1 = 0.014
fr3_finger_joint2 = 0.014
```

## 10. 已知但暂不需要处理的提示

本地 `MoveGroupInterface` 初始化时可能出现：

```text
No kinematics plugins defined. Fill and load kinematics.yaml!
```

目前实际 `/move_group` 的 OMPL / RRTConnect 规划和 Cartesian Path 均已验证可以正常工作，因此该提示暂不作为 Task 01 的阻塞问题。

## 11. 文档维护规则

每完成一个可重复验证的阶段：

1. 更新对应的 `docs/tasks/TASKxx_*.md`；
2. 更新 `docs/PROJECT_PLAN.md` 中的状态；
3. 重大进展同步更新 `README.md`；
4. 启动方式、关键场景参数、必要命令的变化同步更新本文件；
5. 对后续有复现价值的失败原因保留记录，不只记录成功结果。

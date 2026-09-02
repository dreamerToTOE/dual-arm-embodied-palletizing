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

当前主要源码：

```text
Task 01 稳定基线：
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp

Task 02 Placement Skill：
ros_ws/src/fr3_moveit_test/src/placement_skill_demo.cpp
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

当前 Task 01 / Task 02 共用同一基础场景，至少包含：

```text
/World/fr3
/World/Table
/World/PickCube
ActionGraph
```

ActionGraph 已验证 ROS 2 通信：

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

运行 ROS 控制时，Isaac 本地 `Position_Controller` 保持关闭，避免与 ROS 指令冲突。

启动仿真后点击 **Play**。

## 5. 当前场景参数

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

当前 MoveIt 的统一规划参考坐标系：

```text
base
```

不要再把 Cartesian Path 的参考系单独设为 `fr3_link0`。此前已经确认 `fr3_link0 -> base` 会导致：

```text
MoveIt error_code = -21
FRAME_TRANSFORM_FAILURE
```

## 6. 编译 fr3_moveit_test

修改 Task 01 或 Task 02 源码后统一执行：

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --packages-select fr3_moveit_test \
  --symlink-install

source install/setup.bash
```

## 7. Task 01：运行稳定 Pick & Place 基线

```bash
ros2 run fr3_moveit_test single_arm_pick_place
```

Task 01 已完整验证：

```text
HOME
→ APPROACH
→ GRASP
→ LIFT
→ TRANSFER
→ PLACE
→ RELEASE
→ RETREAT
→ HOME
```

关键释放状态切换：

```text
DETACH
→ removeWorldCube
→ OPEN
→ RETREAT
→ addPlacedCubeToWorld
```

详细原因见 `docs/KEY_ISSUES.md`。

## 8. Task 02：运行 Placement Skill

Task 02 新节点：

```bash
ros2 run fr3_moveit_test placement_skill_demo
```

默认 PlacementTarget：

```text
place_x          = 0.65
place_y          = -0.15
place_cube_z     = 0.065
place_yaw        = 0.0
pre_place_tcp_z  = 0.175
place_tcp_z      = 0.076
retreat_tcp_z    = 0.175
```

可以通过 ROS 2 参数覆盖，而不修改源码。例如：

```bash
ros2 run fr3_moveit_test placement_skill_demo --ros-args \
  -p place_x:=0.70 \
  -p place_y:=-0.10
```

Task 02 第一版最终应打印：

```text
[PlacementSkill] C_reach   = PASS / FAIL
[PlacementSkill] C_insert  = PASS / FAIL
[PlacementSkill] C_release = PASS / FAIL
[PlacementSkill] C_retreat = PASS / FAIL
[PlacementSkill] RESULT    = ...
```

## 9. 运行前检查

每次重新测试前确认：

```text
1. MoveIt /move_group 正常运行
2. Isaac Sim 已点击 Play
3. /joint_command 有 Isaac subscriber
4. FR3 恢复 HOME
5. PickCube 恢复到 (0.45, 0.15, 0.065)
6. Cube 尺寸为 30 mm
```

可检查 ROS topic：

```bash
ros2 topic list | grep -E 'clock|joint_states|tf|joint_command'
```

## 10. 手动测试夹爪

打开夹爪：

```bash
ros2 topic pub --once \
  /joint_command sensor_msgs/msg/JointState \
  "{name: ['fr3_finger_joint1','fr3_finger_joint2'], position: [0.04,0.04]}"
```

当前闭合目标：

```text
fr3_finger_joint1 = 0.014
fr3_finger_joint2 = 0.014
```

## 11. 已知但暂不阻塞的提示

本地 `MoveGroupInterface` 初始化时可能出现：

```text
No kinematics plugins defined. Fill and load kinematics.yaml!
```

目前实际 `/move_group` 的 OMPL / RRTConnect 和 Cartesian Path 已验证可以正常工作，因此暂不作为阻塞问题。

## 12. GitHub 包配置同步提醒

当前 GitHub 中 `ros_ws/src/fr3_moveit_test/` 只同步了 `src/`，本地真实 `CMakeLists.txt` 和 `package.xml` 尚未进入仓库。

因此：

```text
不要用 GitHub 中不存在的包配置覆盖本地文件。
```

Task 02 本地需要在现有 CMake 中增加 `placement_skill_demo` executable。对应片段已记录在：

```text
docs/tasks/TASK02_PLACEMENT_SKILL.md
```

后续应把本地真实 CMakeLists.txt/package.xml 原样同步到 GitHub，确保仓库可以独立复现。

## 13. 文档维护规则

每完成一个可重复验证的阶段：

1. 更新对应 `docs/tasks/TASKxx_*.md`；
2. 更新 `docs/PROJECT_PLAN.md` 中的状态；
3. 重大进展同步更新 `README.md`；
4. 启动方式、关键场景参数、必要命令的变化同步更新本文件；
5. 对后续有复现价值的失败原因保留记录，不只记录成功结果。

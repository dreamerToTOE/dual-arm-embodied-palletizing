# 启动与运行指南

本文件用于长期保存当前项目各平台的启动顺序、编译命令、运行命令和常见检查项。以后重启电脑、重新打开工程或更换对话时，优先按照本文件恢复环境。

> 当前工程已经从 Task01~03 的二指夹爪基线进入 Task04 顶部吸盘三 Cube 码垛。两套启动方式都保留，避免旧实验无法复现。

---

## 1. 工程路径

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
Task01：
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp

Task02：
ros_ws/src/fr3_moveit_test/src/placement_skill_demo.cpp

Task03：
ros_ws/src/fr3_moveit_test/src/bac_placement_test.cpp

Task04：
ros_ws/src/fr3_moveit_test/src/suction_three_cube_palletize.cpp
```

当前 Isaac 脚本：

```text
isaac/scripts/task04_three_cube_scene.py
isaac/scripts/task04_suction_ros_bridge.py
```

---

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

检查：

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

---

# 3. 电脑重启后的推荐启动顺序

当前 Task04 推荐按下面顺序启动：

```text
终端 1：启动 MoveIt 2（cobot_pump）
        ↓
启动 Isaac Sim 4.5.0
        ↓
打开 Task04 场景 / 基础 FR3 场景
        ↓
Script Editor：运行三 Cube 随机场景脚本
        ↓
保存 Stage
        ↓
Isaac 点击 Play
        ↓
Script Editor：运行 Surface Gripper ROS Bridge
        ↓
终端 2：检查 ROS topics
        ↓
终端 3：编译 / 运行 Task04 控制节点
```

如果只是复现 Task01~03，则使用本文后面的“二指夹爪 MoveIt 启动方式”。

---

# 4. 终端 1：启动 MoveIt 2

## 4.1 当前 Task04：顶部吸盘 / cobot_pump

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true \
  load_gripper:=true \
  ee_id:=cobot_pump
```

正常应启动：

```text
/move_group
/controller_manager
/robot_state_publisher
/fr3_arm_controller
/joint_state_broadcaster
RViz2
```

检查：

```bash
ros2 node list | grep -E 'move_group|controller_manager|robot_state_publisher'
```

当前规划组：

```text
fr3_arm
```

当前规划器：

```text
RRTConnectkConfigDefault
```

### cobot_pump 末端的重要说明

Franka 官方 `cobot_pump` 模型中存在：

```text
fr3_cobot_pump
fr3_cobot_pump_tcp
```

官方吸盘 TCP 相对末端约沿局部 +Z 偏移：

```text
0.105 m
```

但当前 Franka SRDF 中，`fr3_arm` 在非 `franka_hand` 末端时的规划 tip 仍为：

```text
fr3_link8
```

因此当前 Task04 控制代码需要明确区分：

```text
MoveIt 规划 tip：fr3_link8
物理吸盘接触 TCP：fr3_cobot_pump_tcp / Isaac suction_tcp
```

不要简单把 `fr3_cobot_pump_tcp` 当作 `fr3_arm` 默认规划 tip。

可确认模型中存在吸盘 TCP：

```bash
ros2 param get /move_group robot_description \
  | grep -o 'fr3_cobot_pump_tcp' \
  | head
```

---

## 4.2 Task01~03：原 Franka 二指夹爪基线

旧实验使用：

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

该模式主要用于复现：

```text
Task01 单臂 Pick & Place
Task02 Placement Skill
Task03 B-A-C 二指夹爪可执行性实验
```

---

# 5. Isaac Sim 4.5.0 启动

使用当前已经稳定安装的 Isaac Sim 4.5.0 启动方式，不要随意更换版本、驱动或 ROS Bridge 配置。

基础 Stage 至少应包含：

```text
/World/fr3
/World/Table
ActionGraph
```

ActionGraph 当前负责机械臂 ROS 2 通信：

```text
/clock
/joint_states
/tf
/joint_command
```

`/joint_command` 类型：

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

使用 ROS 控制时：

```text
Isaac 本地 Position_Controller = OFF
```

避免本地控制器和 ROS 指令同时抢机械臂。

---

# 6. Task04：创建顶部吸盘 + 三个随机 Cube

## 6.1 Timeline 先保持停止

在 Isaac Sim 中打开：

```text
Window → Script Editor
```

将下面脚本内容复制到 Script Editor 执行：

```bash
cat ~/lmy/dual-arm-embodied-palletizing/isaac/scripts/task04_three_cube_scene.py
```

脚本会：

```text
1. 清理旧 PickCube / BoxB / BoxC；
2. 隐藏原 Franka Hand / Finger 外观；
3. 关闭原 Hand / Finger 碰撞；
4. 保留 FR3 articulation / joint 结构；
5. 创建紧凑 suction_tool；
6. 创建 Cube1 / Cube2 / Cube3；
7. 三个 Cube 的取料位置随机生成；
8. 目标码垛位置保持固定。
```

当前三个码垛目标：

```text
Cube1 -> (0.620, -0.150, 0.065)
Cube2 -> (0.652, -0.150, 0.065)
Cube3 -> (0.684, -0.150, 0.065)
```

注意：

```text
随机的是 PICK 初始位置；
PLACE 目标不是随机的。
```

创建完成后保存 Stage：

```text
Ctrl + S
```

---

# 7. Isaac 点击 Play

场景和三个 Cube 创建完成后：

```text
Isaac Sim → Play
```

确认 FR3 没有异常跳动，Cube 在桌面上稳定。

---

# 8. Task04：启动 Isaac Surface Gripper ROS Bridge

Isaac 已经处于 Play 状态后，再在 Script Editor 执行：

```bash
cat ~/lmy/dual-arm-embodied-palletizing/isaac/scripts/task04_suction_ros_bridge.py
```

该 Bridge 负责：

```text
ROS -> Isaac：
/task04/suction_command

Isaac -> ROS：
/task04/suction_state
/task04/cube_poses
```

具体含义：

```text
/task04/suction_command
→ std_msgs/Bool
→ 控制 Surface Gripper close/open

/task04/suction_state
→ std_msgs/Bool
→ 返回真实吸附是否 CLOSED

/task04/cube_poses
→ geometry_msgs/PoseArray
→ 发布 Cube1 / Cube2 / Cube3 的 Isaac Ground Truth 位姿
```

为什么 Task01~03 没有这个 Python Bridge：

```text
二指夹爪 = FR3 articulation 中的普通 finger joints
→ /joint_command 可以直接控制

Surface Gripper = Isaac Python API 物理吸附对象
→ 不是普通 articulation joint
→ 需要 Python Bridge 调用 close()/open()/is_closed()
```

另外 Task04 的 Cube 初始位置是随机的，因此也需要 Bridge 把 Isaac 的真实 Cube 位姿送给 ROS 控制节点。

---

# 9. 终端 2：检查 Isaac / ROS 连接

```bash
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash

ros2 topic list | grep -E 'clock|joint_states|tf|joint_command|task04'
```

Task04 至少应看到：

```text
/joint_command
/task04/suction_command
/task04/suction_state
/task04/cube_poses
```

检查三个随机 Cube 位姿：

```bash
ros2 topic echo /task04/cube_poses --once
```

检查吸盘状态：

```bash
ros2 topic echo /task04/suction_state
```

---

# 10. 编译 fr3_moveit_test

源码修改后统一执行：

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --packages-select fr3_moveit_test \
  --symlink-install

source install/setup.bash
```

检查 executable：

```bash
ros2 pkg executables fr3_moveit_test
```

当前应至少包含：

```text
single_arm_pick_place
placement_skill_demo
bac_placement_test
suction_three_cube_palletize
```

---

# 11. 终端 3：运行 Task04 三 Cube 吸盘码垛

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fr3_moveit_test suction_three_cube_palletize
```

当前设计动作链：

```text
RRTConnect -> PRE_PICK
→ SUCTION ON
→ Cartesian 直线下降
→ suction_state = CLOSED
→ MoveIt ATTACH
→ Cartesian LIFT
→ RRTConnect -> PRE_PLACE
→ Cartesian PLACE
→ DETACH
→ SUCTION OFF
→ Cartesian RETREAT
→ 下一个 Cube
```

注意当前 Task04 尚在调试阶段：

```text
PRE_PICK RRTConnect 仍在排查
```

因此不要把 Task04 写成“已验证完整成功”。

---

# 12. Task01：运行稳定 Pick & Place 基线

使用二指夹爪 MoveIt 启动方式后：

```bash
ros2 run fr3_moveit_test single_arm_pick_place
```

已验证流程：

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

关键释放切换：

```text
DETACH
→ removeWorldCube
→ OPEN
→ RETREAT
→ addPlacedCubeToWorld
```

---

# 13. Task02：运行 Placement Skill

```bash
ros2 run fr3_moveit_test placement_skill_demo
```

默认目标：

```text
place_x          = 0.65
place_y          = -0.15
place_cube_z     = 0.065
place_yaw        = 0.0
pre_place_tcp_z  = 0.175
place_tcp_z      = 0.076
retreat_tcp_z    = 0.175
```

参数覆盖示例：

```bash
ros2 run fr3_moveit_test placement_skill_demo --ros-args \
  -p place_x:=0.70 \
  -p place_y:=-0.10
```

---

# 14. Task03：运行 B-A-C 可执行性实验

```bash
ros2 run fr3_moveit_test bac_placement_test
```

Task03 已用于验证：

```text
箱体几何可放置
≠
二指夹爪一定能执行插入
```

该结果是后续改用紧凑顶部吸盘的重要工程依据。

---

# 15. 当前基础场景参数

FR3：

```text
Translate = (0, 0, 0)
Rotate    = (0, 0, 0)
```

Table：

```text
Center = (0.55, 0.00, 0.025) m
Size   = (1.20, 0.80, 0.05) m
Top Z  = 0.05 m
```

Cube：

```text
Size = 0.03 × 0.03 × 0.03 m
Mass = 0.2 kg
Center Z = 0.065 m
```

MoveIt 统一世界参考坐标系：

```text
base
```

不要把 Cartesian Path 参考系单独设为：

```text
fr3_link0
```

此前会导致：

```text
MoveIt error_code = -21
FRAME_TRANSFORM_FAILURE
```

---

# 16. Task01~03 手动二指夹爪测试

打开：

```bash
ros2 topic pub --once \
  /joint_command sensor_msgs/msg/JointState \
  "{name: ['fr3_finger_joint1','fr3_finger_joint2'], position: [0.04,0.04]}"
```

Task01 基线闭合：

```text
fr3_finger_joint1 = 0.014
fr3_finger_joint2 = 0.014
```

Task04 顶部吸盘模式不再依赖这两个 finger joint 进行抓取。

---

# 17. 常见提示

本地 `MoveGroupInterface` 可能打印：

```text
No kinematics plugins defined. Fill and load kinematics.yaml!
```

Task01~03 中实际 `/move_group` 的 OMPL / RRTConnect / Cartesian Path 已经验证可工作，因此该警告目前不是默认阻塞项。

但如果某个新节点出现规划失败，应优先查看：

```text
MoveIt 启动终端中的 move_group 日志
目标 IK 是否可解
起点 / 终点是否碰撞
EEF / TCP 是否使用了正确几何关系
```

不要一看到 RRTConnect 失败就直接增加 planning_time 或修改随机范围。

---

# 18. GitHub 包配置同步提醒

当前仓库中 `ros_ws/src/fr3_moveit_test/` 主要同步了源码；本地真实 `CMakeLists.txt` 和 `package.xml` 仍需要后续完整同步。

因此：

```text
不要用猜测出来的 CMakeLists.txt / package.xml 覆盖本地现有文件。
```

后续应把本机真实配置原样同步到 GitHub，确保仓库能够独立复现。

---

# 19. 文档维护规则

每完成一个可重复验证的阶段：

1. 更新对应 `docs/tasks/TASKxx_*.md`；
2. 更新 `docs/PROJECT_PLAN.md` 状态；
3. 重大进展同步 `README.md`；
4. 平台启动方式、场景、topic、运行命令变化同步更新本文件；
5. 关键失败原因和解决方案记录到 `docs/KEY_ISSUES.md`；
6. 只有用户实际验证成功后，才把某个阶段标记为“已验证”。

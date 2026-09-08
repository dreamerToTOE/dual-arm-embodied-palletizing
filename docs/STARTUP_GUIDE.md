# 启动与运行指南

本文件用于长期保存当前项目各平台的启动顺序、编译命令和稳定入口。旧 Task 的复现入口保留，但当前主线以 **自定义紧凑吸盘 MoveIt 模型** 为准。

## 1. 工程路径

```text
项目根目录：~/lmy/dual-arm-embodied-palletizing
ROS 2 workspace：~/lmy/dual-arm-embodied-palletizing/ros_ws
```

当前稳定环境：

```text
Ubuntu 22.04.5 LTS
ROS 2 Humble
Isaac Sim 4.5.0
MoveIt 2 / OMPL
Franka FR3
NVIDIA RTX 3070 8 GB
NVIDIA 580 系列驱动
Python 3.10.12
```

ROS 终端不要进入 Conda 环境。

---

## 2. 当前主线 MoveIt：自定义紧凑吸盘

Task05-C 已完成 MoveIt / Isaac 末端模型对齐。后续主线不再加载 Franka 官方 `cobot_pump` 环境碰撞模型。

自定义包：

```text
ros_ws/src/fr3_compact_suction_description
```

启动：

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch \
  fr3_compact_suction_description \
  moveit_compact_suction.launch.py
```

当前模型：

```text
FR3
└─ fr3_link8
   ├─ fr3_compact_suction
   └─ fr3_compact_suction_tcp (z = 0.105 m)
```

当前兼容策略仍为：

```text
MoveIt planning tip = fr3_link8
suction TCP target -> +0.105 m -> fr3_link8 pose target
```

确认模型：

```bash
ros2 param get /move_group robot_description | \
  grep -oE 'fr3_compact_suction|fr3_cobot_pump' | sort -u
```

主线预期只看到：

```text
fr3_compact_suction
```

---

## 3. Isaac Sim 基础 Stage

当前基础 Stage 至少包含：

```text
/World/fr3
/World/Table
/World/ActionGraph
```

ActionGraph 当前负责：

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
ROS /joint_command
→ Isaac ROS2 Subscribe Joint State
→ Articulation Controller
→ /World/fr3
```

使用 ROS 控制时，Isaac 本地 Position Controller 保持 OFF。

---

## 4. Task04 基础码垛事件单元

Task04 最终成功版作为后续单臂抓放的项目级基线。

核心源码：

```text
ros_ws/src/fr3_moveit_test/src/suction_nine_cube_palletize.cpp
isaac/scripts/task04b_nine_cube_scene.py
isaac/scripts/task04b_suction_ros_bridge.py
```

基础事件固定为：

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

重要参数：

```text
gripThreshold = 0.003 m
forceLimit = 1e6
torqueLimit = 1e6
TCP offset = 0.105 m
```

后续 Task 默认复用该时序，不重新设计基础抓放逻辑。

---

## 5. Task05 B-A-C 复现

### 5.1 Isaac Timeline Stop

运行场景：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task05_bac_suction_scene.py").read())
```

### 5.2 Isaac Play

运行 Bridge：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task05_suction_ros_bridge.py").read())
```

### 5.3 ROS 控制

确保自定义紧凑吸盘 MoveIt 已按第 2 节启动，然后：

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fr3_moveit_test task05_bac_suction_place
```

已验证结果：

```text
Task05 SUCCESS
BoxA final ≈ (0.6492, -0.1494, 0.0650) m
e_xy ≈ 0.98 mm
e_z  = 0.00 mm
```

---

## 6. 编译

### 自定义吸盘描述包

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --packages-select fr3_compact_suction_description \
  --symlink-install

source install/setup.bash
```

### 控制节点

```bash
colcon build \
  --packages-select fr3_moveit_test \
  --symlink-install

source install/setup.bash
```

---

## 7. 旧 Task01~03 二指夹爪复现

仅复现旧实验时使用官方 MoveIt 启动：

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

对应：

```text
Task01 single_arm_pick_place
Task02 placement_skill_demo
Task03 bac_placement_test
```

Task03 是二指夹爪高密度放置失败的对照实验，不作为后续主线末端执行器。

---

## 8. 已废弃的主线启动方式

下面方式曾用于 Task04 调试官方 cobot_pump，但 Task05 证明其环境碰撞包络与项目紧凑吸盘不一致：

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true \
  load_gripper:=true \
  ee_id:=cobot_pump
```

后续主线 **不要再使用该 cobot_pump 模型做紧密码垛碰撞规划**。

---

## 9. 当前阶段：Task06

Task06 开始建立：

```text
Left FR3  + left compact suction
Right FR3 + right compact suction
```

当前只做基础设施，不直接并行码垛。

分阶段：

```text
Task06-A  Isaac 双 FR3 场景
Task06-B  左右 ROS / TF / joint 通信拆分
Task06-C  MoveIt 双臂 planning groups + 双 compact suction
Task06-D  左右独立 HOME + suction ON/OFF
```

通过后进入 Task07 松协调并行码垛。

---

## 10. 文档维护规则

每完成一个用户实际验证通过的阶段：

1. 更新对应 `docs/tasks/TASKxx_*.md`；
2. 更新 `docs/PROJECT_PLAN.md`；
3. 重大进展更新 `README.md`；
4. 启动方式变化更新本文件；
5. 关键失败原因记录到 `docs/KEY_ISSUES.md`；
6. 未经实际运行验证，不标记为“已完成”。

# 启动与运行指南

本文件统一记录每个 Task 的 **Isaac Sim 场景启动方式、MoveIt2 启动方式、Bridge 与 ROS 节点运行命令**。以后新增 Task 时必须同步维护这里，并在对应 `docs/tasks/TASKxx_*.md` 中保留本 Task 的启动段。

> 为避免 Bash 复制换行问题，ROS 命令优先写成单行。

## 0. 工程路径

```text
项目根目录：~/lmy/dual-arm-embodied-palletizing
ROS 2 workspace：~/lmy/dual-arm-embodied-palletizing/ros_ws
```

ROS 终端公共前置：

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Isaac Script Editor 中的脚本路径统一使用：

```text
/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/
```

---

## Task00 — FR3 + Isaac + ROS2 + MoveIt2 基线

### Isaac Sim

Task00 属于最早期基础 Stage，历史版本没有独立的 `task00_scene.py`。使用保存的基础 Stage，至少应包含：

```text
/World/fr3
/World/Table
/World/ActionGraph
```

ActionGraph 提供：

```text
/clock
/joint_states
/tf
/joint_command
```

### MoveIt2

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

### 基线执行节点

```bash
ros2 run fr3_moveit_test moveit_to_isaac
```

---

## Task01 — 单机械臂 Pick & Place

### Isaac Sim

Task01 仍使用早期保存的单臂基础 Stage，没有单独场景 bootstrap 脚本。场景至少包含：

```text
/World/fr3
/World/Table
/World/PickCube
/World/ActionGraph
```

关键几何：

```text
PickCube = (0.45, 0.15, 0.065)
目标     = (0.65, -0.15, 0.065)
```

打开 Stage 后点击 **Play**。

### MoveIt2

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

### ROS 节点

```bash
ros2 run fr3_moveit_test single_arm_pick_place
```

---

## Task02 — Placement Skill

### Isaac Sim

复用 Task01 的单臂 Stage 与 PickCube，不重新搭场景。点击 **Play**。

### MoveIt2

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

### ROS 节点

```bash
ros2 run fr3_moveit_test placement_skill_demo
```

需要覆盖放置参数时，例如：

```bash
ros2 run fr3_moveit_test placement_skill_demo --ros-args -p place_x:=0.70 -p place_y:=-0.10
```

---

## Task03 — B-A-C 二指夹爪可执行性

### Isaac Sim

Task03 属于早期二指夹爪对照实验，当前仓库没有单独 `task03_scene.py`。使用保存的单臂 Stage，并确保：

```text
A initial = (0.45, 0.15, 0.065)
A target  = (0.65, -0.15, 0.065)
B center  = (0.65, -0.115, 0.065)
C center  = (0.65, -0.185, 0.065)
```

点击 **Play**。

### MoveIt2

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

### ROS 节点

```bash
ros2 run fr3_moveit_test bac_placement_test
```

---

## Task04 — 顶部吸盘单臂基线

Task04 保留两个阶段；后续工程事件单元以 Task04-B 九 Cube 版本为 canonical baseline。

### Task04-A：三 Cube

Isaac **Stop** 时：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task04_three_cube_scene.py").read())
```

然后点击 **Play**，再运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task04_suction_ros_bridge.py").read())
```

历史 MoveIt2 复现入口：

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true load_gripper:=true ee_id:=cobot_pump
```

控制节点：

```bash
ros2 run fr3_moveit_test suction_three_cube_palletize
```

### Task04-B：九 Cube 三维码垛（canonical）

Isaac **Stop** 时：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task04b_nine_cube_scene.py").read())
```

点击 **Play** 后：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task04b_suction_ros_bridge.py").read())
```

历史 MoveIt2 入口同 Task04-A：

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true load_gripper:=true ee_id:=cobot_pump
```

控制节点：

```bash
ros2 run fr3_moveit_test suction_nine_cube_palletize
```

> `cobot_pump` 只用于 Task04 历史复现。Task05 之后主线使用项目自定义 compact suction MoveIt 模型。

---

## Task05 — B-A-C 顶部吸盘高密度放置

### Isaac Sim

在单臂基础 Stage 下，Timeline **Stop**：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task05_bac_suction_scene.py").read())
```

点击 **Play** 后：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task05_suction_ros_bridge.py").read())
```

### MoveIt2：自定义 compact suction

```bash
ros2 launch fr3_compact_suction_description moveit_compact_suction.launch.py
```

### ROS 节点

```bash
ros2 run fr3_moveit_test task05_bac_suction_place
```

---

## Task06 — 双 FR3 + 双 compact suction 基础设施

### Isaac Sim：从空/旧 Stage 恢复双臂基线

Timeline **Stop** 时依次运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_fr3_scene.py").read())
```

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_ros_graph.py").read())
```

然后点击 **Play**。

### MoveIt2：双臂模型

```bash
ros2 launch fr3_dual_compact_suction_description moveit_dual_compact_suction.launch.py
```

Task06 本身没有额外抓放 demo 节点；完成后作为 Task07~Task16 的双臂基础设施复用。

---

## Task07 — 松协调无冲突并行码垛

### Isaac Sim

如果当前 Stage 没有 Task06 双臂基础设施，先在 **Stop** 状态运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_fr3_scene.py").read())
```

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_ros_graph.py").read())
```

然后仍在 **Stop** 状态运行 Task07 场景：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task07_parallel_scene.py").read())
```

点击 **Play** 后运行双吸盘 Bridge：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task07_dual_suction_bridge.py").read())
```

### MoveIt2

```bash
ros2 launch fr3_dual_compact_suction_description moveit_dual_compact_suction.launch.py
```

### ROS 节点

```bash
ros2 run fr3_dual_palletize task07_parallel_demo
```

已验证成功标志：

```text
========== Task07 SUCCESS ==========
```

---

## Task08 — 双臂交叉场景 / 时空冲突检测

### Isaac Sim

如果当前 Stage 没有 Task06 双臂基础设施，先在 **Stop** 状态运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_fr3_scene.py").read())
```

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_ros_graph.py").read())
```

然后仍在 **Stop** 状态运行交叉场景：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task08_cross_conflict_scene.py").read())
```

点击 **Play** 后继续复用 Task07 双吸盘 Bridge：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task07_dual_suction_bridge.py").read())
```

### MoveIt2

```bash
ros2 launch fr3_dual_compact_suction_description moveit_dual_compact_suction.launch.py
```

### ROS 节点

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fr3_dual_palletize task08_candidate_demo
```

该节点先生成两条完整 `TaskTrajectoryCandidate`，再按 `0.01 s` 联合采样；仅输出 `Task08-B SAFE` 或 `Task08-B CONFLICT`，不会执行机器人或吸盘命令。

> 不要运行 `task07_parallel_demo` 来测试 Task08，因为 Task07 demo 内部仍使用无冲突目标坐标。

---

## 后续 Task09+

从 Task09 开始继续沿用同一规则：每个 Task 文档必须明确写出：

```text
1. Isaac Stop 时运行哪些场景脚本
2. Isaac Play 后运行哪些 Bridge
3. MoveIt2 启动命令
4. ROS 主节点运行命令
5. 是否依赖前一个 Task 的场景/模块
```

没有独立脚本或节点时必须明确写“复用上一 Task”，不假造入口。

---

## 编译常用命令

单臂旧控制节点：

```bash
colcon build --packages-select fr3_moveit_test --symlink-install
```

单臂 compact suction 描述：

```bash
colcon build --packages-select fr3_compact_suction_description --symlink-install
```

双臂 MoveIt 描述：

```bash
colcon build --packages-select fr3_dual_compact_suction_description --symlink-install
```

双臂任务/协调模块：

```bash
colcon build --packages-select fr3_dual_palletize --symlink-install
```

编译后：

```bash
source install/setup.bash
```

---

## 文档维护规则

每完成一个用户实际验证通过的阶段：

1. 更新对应 `docs/tasks/TASKxx_*.md`；
2. 更新 `docs/PROJECT_PLAN.md`；
3. 启动方式变化更新本文件；
4. 对应 Task 文档同步写启动命令；
5. 重大进展更新 `README.md`；
6. 关键失败原因记录到 `docs/KEY_ISSUES.md`；
7. 未经实际运行验证，不标记为“已完成”。

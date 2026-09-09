# Task06：双 FR3 + 双紧凑吸盘基线

## 状态

✅ **已完成**

- Task06-A：✅ Isaac 双 FR3 长边布局已本机验收通过。
- Task06-B：✅ ROS / TF / Joint 通信隔离已本机验收通过。
- Task06-C：✅ 双臂 MoveIt2 描述、14 关节状态、标准 TF、RViz 双臂显示、Planning Group 与环境桌面均已通过。
- 原计划 Task06-D（逐臂 HOME + 吸盘 ON/OFF）不再单独重复验收，合并到 Task07 首次双臂并行抓放集成测试中。

## 1. 目标

Task06 只建立双臂基础设施，不重新验证 Task04 / Task05 已经证明的单臂抓放能力。

目标结构：

```text
Left FR3  + left compact suction
Right FR3 + right compact suction
```

双臂基础设施已具备：

```text
独立 joint state / joint command
唯一 left_/right_ joint / link / TF
独立 MoveIt planning group
统一 MoveIt Planning Scene
双臂共同环境模型
```

## 2. 模块化复用原则

从 Task06 结束后，项目不再按“每个 Task 从头验证一遍单臂能力”的方式推进。

已经验证过的模块直接复用：

```text
Task04 primitive
HOME / safe
→ PRE_PICK
→ remove target from MoveIt world
→ Cartesian CONTACT
→ SUCTION ON
→ attach
→ LIFT
→ TRANSFER / PRE_PLACE
→ PLACE
→ plan RETREAT while attached
→ SUCTION OFF
→ settle
→ Ground Truth
→ detach / addWorld
→ RETREAT
```

Task05 已验证 compact suction 的 Isaac / MoveIt 几何一致性和高密度插入能力。

后续只验证“新增的双臂能力”，不重复验证已经稳定的单臂内部步骤。

## 3. Task06-A：Isaac 双 FR3 场景 ✅

脚本：

```text
isaac/scripts/task06_dual_fr3_scene.py
```

固定布局：

```text
Table:
center = (0.55, 0.00, 0.025)
size   = (1.20, 0.80, 0.05)

Left FR3:
position = (0.55, -0.50, 0.00)
yaw      = 0 deg

Right FR3:
position = (0.55, +0.50, 0.00)
yaw      = 0 deg

base separation = 1.00 m
```

本机验收：两台 FR3 位于桌面两条长边外侧、同朝向、HOME 状态无明显穿模，中央保留明显公共工作区，两端 compact suction 均正确显示。

## 4. Task06-B：ROS / TF / Joint 通信拆分 ✅

脚本：

```text
isaac/scripts/task06_dual_ros_graph.py
```

通信：

```text
Left:
PUB /left/joint_states
SUB /left/joint_command
PUB /left/tf

Right:
PUB /right/joint_states
SUB /right/joint_command
PUB /right/tf

Global:
PUB /clock
```

本机验收：

```text
/left/joint_states：PASS
/right/joint_states：PASS
/left/tf：PASS
/right/tf：PASS
/left/joint_command 只驱动 left_fr3：PASS
/right/joint_command 只驱动 right_fr3：PASS
```

## 5. Task06-C：MoveIt2 双臂描述 ✅

ROS2 包：

```text
ros_ws/src/fr3_dual_compact_suction_description/
```

MoveIt 结构：

```text
world
├── left_fr3_link0 ... left_fr3_link8
│   └── left_fr3_compact_suction
└── right_fr3_link0 ... right_fr3_link8
    └── right_fr3_compact_suction
```

Planning groups：

```text
left_arm
right_arm
dual_arm
```

已通过：

```text
双臂 URDF xacro：PASS
14 个唯一 arm joints：PASS
/joint_states 左右 14 关节：PASS
标准 /tf left_fr3_* / right_fr3_*：PASS
move_group：PASS
RViz 双 FR3：PASS
left_arm / right_arm / dual_arm：PASS
```

MoveIt Planning Scene 已自动加载与 Isaac 一致的桌面：

```text
id     = task06_table
frame  = world
center = (0.55, 0.00, 0.025)
size   = (1.20, 0.80, 0.05)
```

本机已确认 RViz 中桌面正常出现。

不再为“桌子会不会挡住单臂轨迹”单独重复做穿桌回归；MoveIt 环境碰撞能力已在单臂阶段反复使用。真正新的双臂互碰能力在 Task08 的冲突场景中直接验收。

## 6. Task06 收口后的模块结构

```text
[SingleArmPalletizePrimitive]   ← Task04 已验证
          ↑ 参数化复用
   ┌──────┴──────┐
   │             │
[Left Arm]   [Right Arm]
   │             │
   └──────┬──────┘
          ↓
[DualArm Coordinator]
   ├─ 同时启动 / 调度
   ├─ 共享 Planning Scene
   ├─ 臂-臂 / 臂-物冲突检测
   └─ 等待 / 优先级 / 重规划
```

后续工作的重点从“基础功能逐项回归”切换为“双臂新增能力集成”。

## 7. 下一步：直接进入 Task07

Task07 不再先分别验证 HOME、吸盘、单臂抓放。

直接做：

```text
Left FR3 复用 Task04 primitive 搬 BoxA
+
Right FR3 复用 Task04 primitive 搬 BoxB
+
两条任务同时启动
```

首轮刻意选取无冲突目标，先证明：

```text
双臂可以真正同时执行
左右状态 / 命令 / Attach / Suction 不串线
共享 MoveIt 场景保持一致
```

然后 Task08 立即构造两条会发生空间 / 时间冲突的轨迹，开始真正的双臂避障与冲突检测。

---

## 8. 启动 / 复现指令

### Isaac Sim

Timeline **Stop** 时依次运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_fr3_scene.py").read())
```

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_ros_graph.py").read())
```

然后点击 **Play**。

### MoveIt2

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fr3_dual_compact_suction_description moveit_dual_compact_suction.launch.py
```

Task06 本身没有额外抓放 demo 节点；它作为 Task07 及后续双臂任务的基础设施复用。

统一启动总表见：

```text
docs/STARTUP_GUIDE.md
```

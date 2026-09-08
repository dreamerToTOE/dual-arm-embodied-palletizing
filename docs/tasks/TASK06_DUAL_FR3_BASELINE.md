# Task06：双 FR3 + 双紧凑吸盘基线

## 状态

🟡 **进行中**

- Task06-A：✅ Isaac 双 FR3 第二版长边布局已本机验收通过。
- Task06-B：✅ ROS / TF / Joint 通信隔离已本机验收通过。
- Task06-C：🟡 双臂 MoveIt2 描述、14 关节状态、标准 TF、RViz 双臂显示与 Planning Group 均已通过；正在补齐 MoveIt Planning Scene 环境桌面与最终碰撞验收。
- Task06-D：计划中，左右臂独立 HOME + 吸盘 ON/OFF。

## 1. 目标

Task06 只建立双臂基础设施，不直接做松协调并行码垛，也不直接做紧协调共同搬运。

目标结构：

```text
Left FR3  + left compact suction
Right FR3 + right compact suction
```

两套机械臂必须具备独立：

```text
joint state
joint command
TF
MoveIt planning group
suction command
suction state
```

## 2. 继承的项目级基线

后续基础码垛事件统一复用 Task04 已验证时序，不重新设计：

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
→ Ground Truth 同步
→ DETACH / addWorld
→ RETREAT
```

MoveIt 末端执行器继续使用项目自定义 compact suction，不再使用 Franka 官方 `cobot_pump` 环境碰撞模型。

当前兼容策略继续保持：

```text
single arm tip = fr3_link8
left arm tip   = left_fr3_link8
right arm tip  = right_fr3_link8
compact suction TCP offset = 0.105 m
```

这样不破坏 Task04 起已经验证的 0.105 m TCP 换算逻辑。

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

因此 Task06-B 收口。

## 5. Task06-C：MoveIt2 双臂描述 🟡

新增 ROS2 包：

```text
ros_ws/src/fr3_dual_compact_suction_description/
```

关键设计：

```text
world
├── left_fr3_link0 ... left_fr3_link8
│   └── left_fr3_compact_suction
└── right_fr3_link0 ... right_fr3_link8
    └── right_fr3_compact_suction
```

两台机械臂使用唯一 joint / link 前缀：

```text
left_fr3_joint1 ... left_fr3_joint7
right_fr3_joint1 ... right_fr3_joint7

left_fr3_link0 ... left_fr3_link8
right_fr3_link0 ... right_fr3_link8
```

MoveIt planning groups：

```text
left_arm
right_arm
dual_arm
```

其中单臂 IK 分别配置 LMAKinematicsPlugin；`dual_arm` 当前只作为组合 group 保留，不为其配置单一末端 IK。

Isaac Task06-B 仍发布原始：

```text
/left/joint_states
/right/joint_states
```

`dual_joint_state_bridge.py` 将两路状态合并并重命名到标准：

```text
/joint_states
```

随后由 `robot_state_publisher` 基于双臂 URDF 发布标准 `/tf`，因此左右 frame 名不再冲突。

### 当前本机已通过

```text
1. fr3_dual_compact_suction_description 可 colcon build：PASS
2. xacro 可生成双臂 URDF：PASS
3. 生成 URDF 中 left_fr3_joint1..7 与 right_fr3_joint1..7 共 14 个唯一关节：PASS
4. /joint_states 含左右共 14 个唯一 arm joints：PASS
5. 标准 /tf 中 left_fr3_* 与 right_fr3_* frame 均正常：PASS
6. move_group 正常启动：PASS
7. RViz 同时显示两台 FR3：PASS
8. MotionPlanning 中 left_arm / right_arm / dual_arm 均存在：PASS
```

### MoveIt 环境桌面

Isaac 的 `/World/Table` 不属于机器人 URDF，因此不会自动进入 MoveIt Planning Scene。Task06-C 新增：

```text
config/environment.yaml
scripts/task06_environment_publisher.py
```

Launch 启动后，环境节点等待 `/apply_planning_scene`，把已验收桌面作为真正的 `CollisionObject` 加入 MoveIt：

```text
id     = task06_table
frame  = world
center = (0.55, 0.00, 0.025)
size   = (1.20, 0.80, 0.05)
```

该桌面用于碰撞检测，不只是 RViz 装饰显示。

Task06-C 剩余验收：

```text
9. RViz Planning Scene 出现 task06_table，尺寸/位置与 Isaac 一致
10. 桌面参与碰撞检测，规划不会穿桌
11. 两臂之间的碰撞没有被 ACM 全局屏蔽
```

## 6. Task06-D：左右臂独立 HOME + 吸盘 ON/OFF

验收：

```text
Left FR3 独立回 HOME
Right FR3 独立回 HOME
Left suction 独立 ON/OFF
Right suction 独立 ON/OFF
```

Task06 到此收口。

Task07 再开始：

```text
Left FR3 运行一个 Task04 primitive 搬 BoxA
Right FR3 运行一个 Task04 primitive 搬 BoxB
→ 两臂并行
```

## 7. 当前下一步

重新同步并编译 `fr3_dual_compact_suction_description`，启动 MoveIt2，先验收 `task06_table` 是否出现在 RViz Planning Scene 中且与 Isaac 桌面重合。通过后再做桌面碰撞与双臂互碰的最终 Task06-C 验收。

# Task06：双 FR3 + 双紧凑吸盘基线

## 状态

🟡 **进行中**

- Task06-A：✅ Isaac 双 FR3 第二版长边布局已本机验收通过。
- Task06-B：🟡 双臂 ROS / TF / Joint ActionGraph 代码已上传，待本机通信隔离验收。
- Task06-C：计划中，MoveIt 双臂描述。
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

MoveIt 末端执行器使用 Task05-C 已验证的项目自定义：

```text
fr3_compact_suction_description
```

不再使用 Franka 官方 `cobot_pump` 环境碰撞模型。

当前单臂兼容策略：

```text
planning tip = fr3_link8
compact suction TCP offset = 0.105 m
```

双臂版本优先保持同样语义，避免在 Task06 同时修改基础码垛原语。

## 3. Task06-A：Isaac 双 FR3 场景

脚本：

```text
isaac/scripts/task06_dual_fr3_scene.py
```

### 第一版布局（已否决）

```text
Left  = (0.00, -0.25, 0.00)
Right = (0.00, +0.25, 0.00)
yaw   = 0 deg / 0 deg
```

问题：两台机械臂都挤在桌面同一短边附近，HOME 状态本体距离过近，初始碰撞概率过高，不适合作为双臂基线。

### 第二版布局（已验收）

采用桌面两条长边各一台、两臂同朝向：

```text
Table:
center = (0.55, 0.00, 0.025)
size   = (1.20, 0.80, 0.05)
long sides: y = +/-0.40 m

Left FR3:
position = (0.55, -0.50, 0.00)
yaw      = 0 deg

Right FR3:
position = (0.55, +0.50, 0.00)
yaw      = 0 deg

base separation = 1.00 m
```

Task06-A 本机验收结果：

```text
1. 两台 FR3 分别位于桌面两条长边外侧：PASS
2. 两台机械臂 yaw 相同：PASS
3. HOME 状态没有明显本体穿模或过近问题：PASS
4. 桌面中央保留明显的双臂共同操作区域：PASS
5. 两端 compact suction 正确显示：PASS
```

因此第二版布局固定为 Task06 后续基线。

## 4. Task06-B：ROS / TF / Joint 通信拆分

脚本：

```text
isaac/scripts/task06_dual_ros_graph.py
```

该脚本会移除旧单臂 `/World/ActionGraph`，创建：

```text
/World/Task06ROSGraph
```

当前通信：

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

Task06-B 暂时把左右 TF 拆到：

```text
/left/tf
/right/tf
```

原因：Isaac 两个 FR3 引用内部仍使用相同 `fr3_link*` Prim 名。当前阶段只验证通信链是否互相隔离，避免直接在标准 `/tf` 中产生同名 frame 覆盖。

**Task06-C 再统一解决：**

```text
唯一 left_/right_ joint / frame 前缀
+
标准 /tf
+
MoveIt 双臂 URDF / SRDF
```

Task06-B 验收：

```text
1. /left/joint_states 持续发布左臂状态
2. /right/joint_states 持续发布右臂状态
3. /left/tf 有左臂 TF 数据
4. /right/tf 有右臂 TF 数据
5. 向 /left/joint_command 发命令时只有左臂动作
6. 向 /right/joint_command 发命令时只有右臂动作
7. 两套控制不能串线
```

本阶段不运行抓取任务，也不修改 Task04 基础码垛原语。

## 5. Task06-C：MoveIt 双臂描述

验收：

```text
left_arm planning group
right_arm planning group
双臂碰撞模型同时存在
两端均使用 compact suction collision
标准 /tf 中 frame 名唯一
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

先本机验收 **Task06-B ROS 通信隔离**。通过后再开始 Task06-C 双臂 MoveIt 描述。

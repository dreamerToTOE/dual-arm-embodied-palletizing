# Task06：双 FR3 + 双紧凑吸盘基线

## 状态

🟡 **进行中**

- Task06-A：🟡 Isaac 双 FR3 场景代码已完成并上传，待本机布局验收。
- Task06-B：计划中，ROS / TF / Joint 通信拆分。
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

第一版采用**并排、同朝向、大公共工作空间**布局：

```text
Left FR3
position = (0.00, -0.25, 0.00)
yaw      = 0 deg

Right FR3
position = (0.00, +0.25, 0.00)
yaw      = 0 deg

base separation = 0.50 m
```

桌面仍沿用：

```text
center = (0.55, 0.00, 0.025)
size   = (1.20, 0.80, 0.05)
```

设计目的：

1. 两台机械臂保持相同底座朝向，便于后续统一坐标约定和动力学分析；
2. 两台机械臂都面向桌面 +X 方向；
3. 桌面中央形成明显公共工作区，为后续松协调冲突检测和紧协调共同搬运提供基础；
4. 初始只调整左右基座 Y 间距，不同时修改 yaw / X，保持实验变量单一。

Task06-A 只搭建场景，不运行抓取任务，不创建双臂 ROS / MoveIt 控制。

两个 FR3 顶部 Prim：

```text
/World/left_fr3
/World/right_fr3
```

两个末端都复用 Task04 / Task05 已验证紧凑吸盘几何：

```text
stem radius = 6 mm
stem length = 99 mm
cup radius  = 10 mm
cup length  = 6 mm
TCP offset  = 105 mm
```

Task06-A 本机验收：

```text
1. 两台 FR3 HOME 外观不互相穿模；
2. 两台机械臂朝向相同；
3. 桌面中央存在明显公共工作空间；
4. 两端吸盘均正确显示；
5. 若需要调整基座，仅优先修改 ±Y 间距。
```

通过本机验收后再标记 Task06-A ✅。

## 4. Task06-B：ROS / TF / Joint 通信拆分

验收：

```text
Left joint_states / joint_command
Right joint_states / joint_command
Left TF
Right TF
```

两套控制不能串线。

## 5. Task06-C：MoveIt 双臂描述

验收：

```text
left_arm planning group
right_arm planning group
双臂碰撞模型同时存在
两端均使用 compact suction collision
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

现在只验证 **Task06-A：Isaac 双 FR3 + 双紧凑吸盘场景**。

不同时修改 MoveIt、ROS topic、控制器和码垛流程。

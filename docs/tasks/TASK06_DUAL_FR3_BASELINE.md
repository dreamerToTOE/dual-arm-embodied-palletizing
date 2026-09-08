# Task06：双 FR3 + 双紧凑吸盘基线

## 状态

🟡 **进行中**

- Task06-A：🟡 Isaac 双 FR3 场景代码已完成并上传，当前验证“桌面两条长边各一台、同朝向”的第二版布局。
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

### 第一版布局（已否决）

第一版使用：

```text
Left  = (0.00, -0.25, 0.00)
Right = (0.00, +0.25, 0.00)
yaw   = 0 deg / 0 deg
```

问题：两台机械臂都挤在桌面同一短边附近，HOME 状态本体距离过近，初始碰撞概率过高，不适合作为双臂基线。

### 第二版布局（当前）

改为**桌面两条长边各一台机械臂，两臂保持相同朝向**：

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

两个基座都位于对应长边外侧约 0.10 m，到桌面中心的水平距离都是 0.50 m。

设计目的：

1. 两台机械臂不再集中在同一短边，显著降低 HOME / 初始动作阶段的本体碰撞概率；
2. 两台机械臂保持相同底座朝向，便于后续统一坐标约定和动力学表达；
3. 两台 FR3 都距离桌面中心约 0.50 m，中央仍属于双方可达区域，保留真正的公共工作空间；
4. 后续松协调仍可在公共区域产生臂-臂 / 臂-携带物冲突，不退化为两个完全独立的单臂任务；
5. 后续紧协调可让两臂在大箱顶部两个分离吸附点共同操作。

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
1. 两台 FR3 分别位于桌面两条长边外侧；
2. 两台机械臂 yaw 相同；
3. HOME 状态没有明显本体穿模或过近问题；
4. 桌面中央仍存在足够大的双臂共同操作区域；
5. 两端 compact suction 均正确显示。
```

第二版通过本机验收后再标记 Task06-A ✅。

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

现在只验证 **Task06-A 第二版长边布局**。

不同时修改 MoveIt、ROS topic、控制器和码垛流程。

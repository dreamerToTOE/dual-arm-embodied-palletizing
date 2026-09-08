# Task06：双 FR3 + 双紧凑吸盘基线

## 状态

🟡 **进行中**

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

## 3. Task06 分阶段

### Task06-A：Isaac 双 FR3 场景

只完成：

```text
两个 FR3 articulation
两个 compact suction tool
独立且不重名的 Prim 路径
合理基座位姿
```

不运行抓取任务。

### Task06-B：ROS / TF / Joint 通信拆分

验收：

```text
Left joint_states / joint_command
Right joint_states / joint_command
Left TF
Right TF
```

两套控制不能串线。

### Task06-C：MoveIt 双臂描述

验收：

```text
left_arm planning group
right_arm planning group
双臂碰撞模型同时存在
两端均使用 compact suction collision
```

### Task06-D：左右臂独立 HOME + 吸盘 ON/OFF

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

## 4. 当前下一步

下一步只做 **Task06-A：Isaac 双 FR3 + 双紧凑吸盘场景**。

不同时修改 MoveIt、ROS topic、控制器和码垛流程；先把双臂 Stage 的 Prim 命名、基座位置和两个吸盘模型确定下来，再继续下一阶段。

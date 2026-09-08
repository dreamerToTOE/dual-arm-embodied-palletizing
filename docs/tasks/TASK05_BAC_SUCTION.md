# Task 05 — 顶部吸盘 B-A-C 高密度放置验证

## 状态

🟡 **进行中**

- Task05-A：✅ Isaac B-A-C 场景已完成并本机验证可正常创建。
- Task05-B：🟡 ROS / MoveIt 顶部吸盘控制代码已写入 GitHub，待 Isaac + MoveIt 联合验证。

---

## 1. 目标

Task03 已验证：同一 B-A-C 几何场景中，A 本体可以进入 B/C 中间，但 Franka 二指夹爪因为 finger 横向包络与 B/C 干涉，最终：

```text
C_reach = PASS
C_insert = FAIL
```

Task05 使用固定顶部吸盘复用同一场景，验证：

> 顶部吸盘是否能够完成二指夹爪无法完成的垂直高密度放置。

本 Task 不做零间隙、不做自动纠偏、不做掉落恢复、不做双臂。

---

## 2. B-A-C 场景参数

```text
BoxA size   = 30 x 30 x 30 mm
BoxA initial center = (0.45, 0.15, 0.065) m
BoxA target center  = (0.65, -0.15, 0.065) m

BoxB center = (0.65, -0.115, 0.065) m
BoxC center = (0.65, -0.185, 0.065) m
```

B/C 内侧净间距：

```text
40 mm
```

A 宽度：

```text
30 mm
```

因此 A 本体仍有总计 10 mm 的几何余量。

---

## 3. Isaac ROS Bridge

文件：

```text
isaac/scripts/task05_suction_ros_bridge.py
```

接口：

```text
SUB /task05/suction_command   std_msgs/Bool
PUB /task05/suction_state     std_msgs/Bool
PUB /task05/box_poses         geometry_msgs/PoseArray
```

`box_poses` 顺序固定：

```text
poses[0] = BoxA
poses[1] = BoxB
poses[2] = BoxC
```

Surface Gripper 使用 Task04 后期稳定参数：

```text
gripThreshold = 0.003 m
forceLimit = 1e6
torqueLimit = 1e6
```

抓取时序：

```text
PRE_PICK
-> SUCTION 保持 OFF
-> CONTACT
-> SUCTION ON
-> CLOSED
-> LIFT
```

避免早期版本在下降途中提前 attach 造成“浮空抓”。

---

## 4. ROS / MoveIt 控制节点

文件：

```text
ros_ws/src/fr3_moveit_test/src/task05_bac_suction_place.cpp
```

流程：

```text
HOME
-> PRE_PICK
-> SUCTION_CONTACT
-> SUCTION ON
-> ATTACH BoxA
-> LIFT
-> PRE_PLACE
-> PRE_PLACE -> PLACE 垂直插入 B/C
-> 释放前预规划 RETREAT
-> SUCTION OFF
-> 等待 BoxA settle
-> Isaac Ground Truth 同步实际 pose
-> MoveIt detach + addWorldBox
-> 执行已规划 RETREAT
-> HOME
```

MoveIt planning tip 继续使用：

```text
fr3_link8
```

cobot_pump TCP offset：

```text
0.105 m
```

---

## 5. Task03 / Task05 对照验收

Task03：

```text
二指夹爪
C_insert = FAIL
```

Task05 预期验证：

```text
顶部吸盘
PRE_PLACE -> PLACE fraction = 1.0
BoxA 能真实进入 B/C 中间
SUCTION OFF 后稳定留在目标区
RETREAT 成功
```

在本机 Isaac + MoveIt 实际通过之前，不将 Task05 标记为完成。

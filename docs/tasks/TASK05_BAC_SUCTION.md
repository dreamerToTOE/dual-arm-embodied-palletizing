# Task 05 — 顶部吸盘 B-A-C 高密度放置验证

## 状态

✅ **已完成并本机联合验证通过**

- Task05-A：✅ Isaac B-A-C 场景完成。
- Task05-B：✅ ROS / MoveIt 顶部吸盘控制完成并实际运行成功。
- Task05-C：✅ MoveIt / Isaac 紧凑吸盘模型对齐完成并实际运行成功。

---

## 1. 目标

Task03 已验证：同一 B-A-C 几何场景中，A 本体可以进入 B/C 中间，但 Franka 二指夹爪因为 finger 横向包络与 B/C 干涉：

```text
C_reach = PASS
C_insert = FAIL
```

Task05 使用固定顶部紧凑吸盘复用同一场景，验证顶部吸盘能否完成二指夹爪无法完成的垂直高密度放置。

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

B/C 内侧净间距：40 mm；A 宽度：30 mm，因此 A 本体总计有 10 mm 几何余量。

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

`box_poses` 顺序固定：BoxA、BoxB、BoxC。

Surface Gripper 沿用 Task04 最终稳定参数：

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

---

## 4. ROS / MoveIt 控制节点

文件：

```text
ros_ws/src/fr3_moveit_test/src/task05_bac_suction_place.cpp
```

基础事件时序严格复用 Task04：

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

当前 planning tip 继续保持 `fr3_link8`，吸盘 TCP offset 为 0.105 m。

---

## 5. 关键问题：官方 cobot_pump 假碰撞

使用 Franka 官方 `cobot_pump` MoveIt 模型时：

```text
PRE_PLACE -> PLACE (B-A-C INSERT)
Cartesian fraction ≈ 0.6531
```

同一运动逻辑在不加载官方 cobot_pump 后完整成功，说明失败不是 Task04 基础码垛时序造成，而是官方末端碰撞包络与项目实际紧凑吸盘不一致。

因此项目建立自定义：

```text
fr3_compact_suction_description
```

并让 MoveIt 与 Isaac 使用一致的紧凑吸盘几何。

---

## 6. 最终验收结果

自定义紧凑吸盘模型启动后，Task05 实际运行完成：

```text
Task05 SUCCESS：顶部吸盘完成 B-A-C 放置
```

最终 BoxA Ground Truth：

```text
(0.6492, -0.1494, 0.0650) m
```

相对目标：

```text
e_xy ≈ 0.98 mm
e_z  = 0.00 mm
```

本 Task 只记录误差，不做自动纠偏。

---

## 7. Task03 / Task05 对照结论

```text
同一 B-A-C 几何

Task03 二指夹爪：
C_insert = FAIL

Task05 紧凑顶部吸盘：
C_insert = PASS
完整放置 / 释放 / RETREAT = PASS
```

工程结论：顶部紧凑吸盘解决了二指夹爪在该高密度放置场景中的侧向空间干涉问题。

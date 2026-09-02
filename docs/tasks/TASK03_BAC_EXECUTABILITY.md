# Task 03 — B-A-C 放置可执行性

## 状态

🟡 **进行中 — 首轮已复现“Isaac 真实碰撞但 MoveIt 未阻止”的状态不同步问题，下一步修正 MoveIt 夹爪关节状态**

Task 02 已完成第一版 Placement Skill。Task 03 开始正式验证论文中的核心问题：

> **目标箱 A 的几何体能够放入目标空位，并不代表携带 A 的机械臂夹爪也能完成插入、释放和退出。**

---

## 1. 核心实验思想

保持 Task 02 的目标箱 A：

```text
A size   = 30 × 30 × 30 mm
A target = (0.65, -0.15, 0.065) m
Yaw      = 0
```

在 A 目标位置两侧沿世界 Y 方向放置邻箱 B、C。

第一版使用：

```text
B size   = 30 × 30 × 30 mm
B center = (0.65, -0.115, 0.065) m

C size   = 30 × 30 × 30 mm
C center = (0.65, -0.185, 0.065) m
```

B、C 内侧表面的净间距：

```text
d_BC = 40 mm
```

A 沿 Y 方向宽度：

```text
w_A = 30 mm
```

因此单看箱体几何：

```text
d_BC > w_A
40 mm > 30 mm
```

A 可以放入，左右各有约 5 mm 几何间隙。

但真实放置时，携带 A 的 Franka 两指夹爪占用空间大于 A 本体。目标是让 MoveIt 在垂直插入过程中检测 finger / hand 与 B/C 的碰撞，使 Placement Skill 返回：

```text
C_reach = PASS
C_insert = FAIL
RESULT = INSERT_FAILED
```

---

## 2. 与论文条件的对应

简化几何条件：

```text
d_BC >= w_A
```

只能说明箱体 A 本体在几何上有空间。

机器人实际可执行条件应进一步考虑夹爪厚度和安全间隙：

```text
d_BC >= w_A + 2 t_g + 2 delta
```

其中：

```text
t_g   = 单侧夹爪 / 手指额外占用厚度
delta = 安全间隙
```

Task 03 第一版先使用 MoveIt 的真实 FR3 碰撞模型直接判定，不急于人为给出 t_g 的精确常数。

后续可以从实验结果或模型包络进一步提取等效夹爪宽度。

---

## 3. 实验结构

```text
抓取 A
↓
LIFT
↓
携带 A 到 B/C 上方 PRE_PLACE
↓
C_reach
↓
保持 x/y/姿态，Cartesian 垂直下降
↓
MoveIt 检测 FR3 finger / hand / attached A 与 B/C 的碰撞
↓
C_insert
↓
PASS 或 FAIL
```

第一轮目标不是强行让动作成功，而是验证 Placement Skill 能否正确识别“几何上有空位、机器人却插不进去”。

---

## 4. Isaac Sim 场景要求

继续使用 Task 02 场景，并新增：

```text
/World/BoxB
/World/BoxC
```

第一版 B、C 作为静态环境障碍物：

```text
Collider    = ON
Rigid Body  = OFF
```

这样即使真实机械臂发生接触，B、C 也不会被撞走，便于稳定复现实验。

A 仍然是当前 `/World/PickCube`，保留：

```text
Rigid Body + Collider
Mass = 0.2 kg
```

2026-09-02 已完成 Isaac 场景创建。

---

## 5. MoveIt Planning Scene 要求

仅在 Isaac 中加入 B/C 不够。

Task 03 节点同时把 B、C 作为 `CollisionObject` 加入 MoveIt Planning Scene：

```text
table
pick_cube / attached A
box_b
box_c
```

B、C 的位置和尺寸与 Isaac 保持一致。

---

## 6. 源码策略

不修改已经验证成功的：

```text
single_arm_pick_place.cpp
```

当前 Task 03 节点：

```text
ros_ws/src/fr3_moveit_test/src/bac_placement_test.cpp
```

它先把 B/C 注入 MoveIt Planning Scene，再复用 Task 02 的 Placement Skill 流程。

---

## 7. 第一轮实际结果：MoveIt 漏检夹爪与 B/C 的碰撞

2026-09-02 首轮运行得到：

```text
B/C 内侧净间距 = 40.0 mm
A 宽度          = 30.0 mm

C_reach   = PASS
C_insert  = PASS
C_release = PASS
C_retreat = PASS
RESULT    = SUCCESS
```

关键日志：

```text
C_insert: PRE_PLACE -> PLACE：Cartesian fraction = 1.0000, error_code = 1
```

但是 Isaac Sim 中可以直接观察到：

```text
真实夹爪在插入过程中与 BoxB / BoxC 发生碰撞，
机械臂仍按照 ROS 下发轨迹继续运动，最终把 A 硬塞入 B/C 之间。
```

因此本轮结果不能解释为“40 mm 间距对夹爪可执行”，而应解释为：

> **MoveIt 本次 Cartesian 碰撞检查所使用的夹爪状态，与 Isaac Sim 中真实执行的夹爪状态不一致。**

---

## 8. 根本原因：规划 RobotState 只显式写入 fr3_arm，未写入 finger joints

Task 02 / Task 03 当前规划函数构造起始状态时采用：

```cpp
moveit::core::RobotState start_state(move_group.getRobotModel());
start_state.setToDefaultValues();
start_state.setJointGroupPositions(joint_model_group, start_q);
start_state.update();
```

其中：

```text
joint_model_group = fr3_arm
```

只包含 7 个机械臂关节，不包含：

```text
fr3_finger_joint1
fr3_finger_joint2
```

而 Isaac 执行抓取后持续保持：

```text
fr3_finger_joint1 = 0.014
fr3_finger_joint2 = 0.014
```

因此出现两套状态：

```text
Isaac Sim 真实执行状态
→ finger = 0.014 / 0.014
→ 真实夹爪外包络较宽
→ 与 B/C 发生物理碰撞

MoveIt 本次规划 RobotState
→ 只更新 fr3_arm
→ finger joints 保持 RobotState 默认值
→ 碰撞模型中的手指位置与真实夹爪不一致
→ Cartesian Path 误判为无碰撞
→ fraction = 1.0000
```

这不是 PhysX 碰撞检测失效，也不是 B/C 没有加入 MoveIt；B/C 已成功加入 Planning Scene。问题在于 **机器人自身碰撞几何的关节状态没有完整同步**。

---

## 9. 下一步修复方案

在进行携物规划时，MoveIt `start_state` 必须显式写入当前夹爪关节位置。

例如 C_reach / C_insert 阶段：

```cpp
start_state.setVariablePosition(
    "fr3_finger_joint1",
    GRIPPER_CLOSE_POS);

start_state.setVariablePosition(
    "fr3_finger_joint2",
    GRIPPER_CLOSE_POS);

start_state.update();
```

其中：

```text
GRIPPER_CLOSE_POS = 0.014 m
```

释放后的 C_retreat 则应使用：

```text
GRIPPER_OPEN_POS = 0.040 m
```

更通用的实现方式是让 `planPoseStage()` 和 `planCartesianStage()` 接受一个 `finger_position` 参数，使规划状态与动作阶段一致。

预期修复后再次运行同一 40 mm B-A-C 场景时：

```text
C_reach  = PASS
C_insert < 0.999
RESULT   = INSERT_FAILED
```

如果修复夹爪状态后 40 mm 仍然无碰撞，再逐步缩小 B/C 间距，但始终保持：

```text
d_BC > 30 mm
```

以寻找“箱体可放、夹爪不可插”的临界范围。

---

## 10. 本轮工程意义

本轮意外得到一个比单纯 `INSERT_FAILED` 更重要的工程结论：

```text
Planning Scene 中存在障碍物
≠
MoveIt 一定能正确判断真实机器人是否碰撞
```

碰撞检测还依赖：

```text
完整 RobotState
= arm joints + finger joints + attached object state
```

因此后续所有抓取、放置、双臂避碰和紧协调规划，都不能只同步 `fr3_arm` 的 7 个关节而忽略夹爪状态。

---

## 11. 与 Task 04 的衔接

Task 03 最终负责发现：

```text
当前 B-A-C 顺序导致 A 无法执行插入
```

Task 04 再处理：

```text
如果先放 B/C 导致 A 后续不可插入
↓
调整码垛顺序
↓
例如优先放 A，再放 B/C
```

因此 Task 03 是 Task 04“放置顺序调整”的直接前置实验。

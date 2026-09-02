# Task 02 — 机器人可执行 Placement Skill

## 状态

🟡 **进行中 — 从 Task 01 的固定放置流程升级为可复用、可判定的放置技能**

Task 01 已经完整验证单机械臂确定性 Pick & Place：

```text
HOME
→ ALIGNED_APPROACH
→ GRASP
→ LIFT
→ TRANSFER
→ PLACE
→ RELEASE
→ RETREAT
→ HOME
```

Task 02 不再重点解决“机械臂能不能把一个方块放下”，而是解决更接近论文问题的：

> **一个给定的目标放置位姿，是否真的可以由当前机器人、夹爪和携带物体完整执行？**

也就是说：

```text
几何上可以放
≠
机器人一定可以执行
```

---

## 1. Task 02 的目标

把 Task 01 中已经验证过的放置动作抽象成一个独立的 Placement Skill：

```text
PlacementTarget
      ↓
PRE_PLACE
      ↓
INSERT
      ↓
PLACE
      ↓
RELEASE
      ↓
RETREAT
      ↓
SUCCESS / FAILURE + 失败原因
```

第一版先使用当前单 FR3、30 mm Cube 和空桌面场景，不增加 B/C 障碍箱体。

Task 03 再正式加入 B-A-C 场景，验证周围箱体导致的插入不可行问题。

---

## 2. 放置目标参数化

Task 01 中放置点是硬编码常量：

```text
Place Cube center = (0.65, -0.15, 0.065)
Yaw               = 0
```

Task 02 改为结构化输入，例如：

```cpp
struct PlacementTarget
{
    double x;
    double y;
    double cube_z;
    double yaw;
    double pre_place_tcp_z;
    double place_tcp_z;
    double retreat_tcp_z;
};
```

这样后面候选码垛位置可以直接传入 Placement Skill，而不是为每个位置修改源码。

---

## 3. 可执行性定义

当前采用：

```text
C_place = C_reach * C_insert * C_release * C_retreat
```

其中：

### C_reach

机器人携带物体能够到达目标上方的 `PRE_PLACE`。

第一版：

```text
LIFT / current carry state
→ RRTConnect / Cartesian
→ PRE_PLACE
```

需要考虑机械臂、夹爪和 Attached Cube 的碰撞。

### C_insert

从 `PRE_PLACE` 到 `PLACE` 的插入路径能够完整执行。

第一版采用：

```text
保持 X / Y / orientation
仅沿 Z 下降
```

判定：

```text
Cartesian fraction >= 0.999
```

### C_release

到达放置位姿以后，可以进入释放状态。

Task 02 基线继续沿用 Task 01 已验证状态切换：

```text
DETACH
→ removeWorldCube
→ OPEN
```

后续 Task 03 才加入邻近箱体对手指张开空间的影响。

### C_retreat

释放以后，夹爪能够安全退出。

第一版采用：

```text
保持 X / Y / orientation
仅沿 Z 上升
```

判定：

```text
Cartesian fraction >= 0.999
```

退出完成后再：

```text
addPlacedCubeToWorld
```

---

## 4. Task 02 与 Task 01 的核心区别

Task 01：

```text
给定一个已经知道能工作的放置点
→ 把完整 Pick & Place 跑通
```

Task 02：

```text
给定任意 PlacementTarget
→ 先检查是否可执行
→ 再执行
→ 返回明确成功 / 失败阶段
```

因此 Task 02 必须具备“失败原因”。

第一版返回状态计划为：

```text
PLACEMENT_SUCCESS
REACH_FAILED
INSERT_FAILED
RELEASE_FAILED
RETREAT_FAILED
```

后续可以继续扩展：

```text
GRIPPER_CLEARANCE_FAILED
TARGET_COLLISION
NEIGHBOR_COLLISION
NO_IK
```

---

## 5. 第一阶段实现方式

保留 Task 01 稳定源码：

```text
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp
```

不再直接在这个文件上做结构性重构。

Task 02 新建：

```text
ros_ws/src/fr3_moveit_test/src/placement_skill_demo.cpp
```

第一版仍然复用 Task 01 已验证的抓取和提升流程；在 `LIFT` 之后，把：

```text
TRANSFER
PLACE
RELEASE
RETREAT
```

封装为：

```cpp
executePlacementSkill(target, ...)
```

并返回结构化结果。

---

## 6. 第一阶段场景不变

暂时不增加任何新障碍物。

继续使用：

```text
FR3       /World/fr3
Table     center=(0.55, 0.00, 0.025)
PickCube  center=(0.45, 0.15, 0.065)
Cube size 0.03 × 0.03 × 0.03 m
```

第一版 PlacementTarget：

```text
x              = 0.65
y              = -0.15
cube_z         = 0.065
yaw            = 0
pre_place_tcp_z= 0.175
place_tcp_z    = 0.076
retreat_tcp_z  = 0.175
```

先证明参数化后的 Placement Skill 与 Task 01 固定流程具有相同稳定性。

---

## 7. 第一阶段验收标准

运行时终端应明确输出：

```text
[PlacementSkill] target = (...)
[PlacementSkill] C_reach   = PASS
[PlacementSkill] C_insert  = PASS
[PlacementSkill] C_release = PASS
[PlacementSkill] C_retreat = PASS
[PlacementSkill] RESULT    = SUCCESS
```

Isaac Sim 中：

```text
1. Cube 被稳定抓起；
2. 到达目标 PRE_PLACE；
3. 垂直插入；
4. 正常释放；
5. 垂直退出；
6. Cube 留在目标位置；
7. 机械臂返回 HOME。
```

第一阶段成功后，Task 02 再加入一个“故意不可达/不可插入”的测试目标，确认 Placement Skill 能在执行前返回失败，而不是盲目执行。

---

## 8. 与 Task 03 的边界

Task 02 解决：

```text
单目标放置动作的参数化
+ Reach
+ Insert
+ Release
+ Retreat
+ 结构化结果
```

Task 03 才加入：

```text
B   A   C
```

重点研究：

```text
- A 箱体本身能放进去，但夹爪放不进去；
- 插入扫掠体与 B/C 碰撞；
- 手指张开空间不足；
- 放下 A 后夹爪无法退出。
```

因此 Task 02 是后续“机器人可执行码垛规划”的基础技能层。

---

## 9. 当前下一步

本轮首先：

```text
1. 保留 Task 01 稳定文件不动；
2. 复制为 placement_skill_demo.cpp；
3. 新增 PlacementTarget；
4. 把放置段封装成 executePlacementSkill()；
5. 使用原来的放置目标做等价验证。
```

启动方式继续沿用：

```text
docs/STARTUP_GUIDE.md
```

# Task 02 — 机器人可执行 Placement Skill

## 状态

✅ **已完成 — 第一版参数化 Placement Skill 已在 Isaac Sim 中完整验证成功**

Task 02 不再继续单独构造人工失败案例。后续 `INSERT_FAILED / RETREAT_FAILED` 等失败行为直接在 Task 03 的真实 B-A-C 邻箱干涉场景中验证。

---

## 1. 目标

Task 01 解决的是：

> 固定场景、固定目标下，单机械臂 Pick & Place 能否完整跑通。

Task 02 将其中的“放置”升级为可复用、可判定的 Placement Skill：

```text
PlacementTarget
      ↓
目标合法性检查
      ↓
C_reach：LIFT → PRE_PLACE
      ↓
C_insert：PRE_PLACE → PLACE
      ↓
C_release：DETACH + OPEN
      ↓
C_retreat：PLACE → RETREAT
      ↓
SUCCESS / FAILURE + 失败原因
```

定义：

```text
C_place = C_reach * C_insert * C_release * C_retreat
```

---

## 2. 源码

```text
ros_ws/src/fr3_moveit_test/src/placement_skill_demo.cpp
```

Task 01 稳定基线继续保留：

```text
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp
```

---

## 3. PlacementTarget

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

默认已验证目标：

```text
place_x          = 0.65
place_y          = -0.15
place_cube_z     = 0.065
place_yaw        = 0.0
pre_place_tcp_z  = 0.175
place_tcp_z      = 0.076
retreat_tcp_z    = 0.175
```

目标可通过 ROS 2 参数覆盖，例如：

```bash
ros2 run fr3_moveit_test placement_skill_demo --ros-args \
  -p place_x:=0.70 \
  -p place_y:=-0.10
```

---

## 4. 结构化结果

第一版定义：

```text
SUCCESS
INVALID_TARGET
REACH_FAILED
INSERT_FAILED
RELEASE_FAILED
RETREAT_FAILED
WORLD_UPDATE_FAILED
```

并输出：

```text
[PlacementSkill] C_reach   = PASS / FAIL
[PlacementSkill] C_insert  = PASS / FAIL
[PlacementSkill] C_release = PASS / FAIL
[PlacementSkill] C_retreat = PASS / FAIL
[PlacementSkill] RESULT    = ...
```

因此 Task 02 的主要提升不是新增运动原语，而是把 Task 01 的固定动作脚本升级为具有明确输入、阶段判定和结构化输出的技能模块。

---

## 5. 四个条件的第一版定义

### C_reach

携带 Cube，从 LIFT 状态使用 RRTConnect 到达 PRE_PLACE 完整 6D 位姿。

### C_insert

从 PRE_PLACE 到 PLACE 保持 x、y、orientation 不变，只沿 Z 做 Cartesian 直线下降，要求：

```text
Cartesian fraction >= 0.999
```

### C_release

沿用 Task 01 已验证的释放状态切换：

```text
DETACH
↓
MoveIt 自动把 Cube 加回 World
↓
removeWorldCube
↓
OPEN / RELEASE
```

当前属于命令层释放确认。

### C_retreat

释放后保持 x/y/姿态，只沿 Z 轴向上退出；退出完成后使用：

```text
addPlacedCubeToWorld(target)
```

按目标位姿把 Cube 恢复为 MoveIt World CollisionObject。

---

## 6. 基础目标合法性检查

运动前检查：

```text
1. 参数必须是有限数值；
2. Cube XY 投影不能超出桌面；
3. Cube 不能穿入桌面；
4. PRE_PLACE 必须高于 PLACE；
5. RETREAT 必须高于 PLACE。
```

---

## 7. 最终验证结果

2026-09-02，默认 PlacementTarget 在 Isaac Sim 中完整执行成功：

```text
HOME
→ PICK
→ LIFT
→ C_reach: PRE_PLACE
→ C_insert: PLACE
→ C_release
→ C_retreat
→ HOME
```

终端确认：

```text
[PlacementSkill] C_reach   = PASS
[PlacementSkill] C_insert  = PASS
[PlacementSkill] C_release = PASS
[PlacementSkill] C_retreat = PASS
[PlacementSkill] RESULT    = SUCCESS
```

Isaac 中 Cube 完成搬运、放置和释放，机械臂安全退出并返回 HOME。

---

## 8. 为什么不再单独做人工失败测试

Task 02 的软件接口、正常目标执行链和阶段化判定已经得到验证。

原计划额外构造：

```text
INVALID_TARGET
REACH_FAILED
INSERT_FAILED
RETREAT_FAILED
```

等人工失败案例，但这些测试对当前论文主线价值有限，而且 `INSERT_FAILED / RETREAT_FAILED` 本来就应该在邻箱真实干涉条件下出现。

因此项目直接进入 Task 03，在 B-A-C 场景中验证：

```text
目标箱 A 几何上能够放入 B、C 之间
但携带 A 的真实夹爪扫掠体无法插入
→ C_reach = PASS
→ C_insert = FAIL（预期）
```

这样既完成失败语义验证，又直接对应论文的“几何可放置 ≠ 机器人可执行放置”。

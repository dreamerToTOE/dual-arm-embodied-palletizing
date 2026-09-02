# Task 02 — 机器人可执行 Placement Skill

## 状态

🟡 **进行中 — 第一版参数化 Placement Skill 正常目标已在 Isaac Sim 中完整验证成功，下一步验证失败分类**

Task 01 已完整验证：

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

Task 02 的重点不再是“机械臂能不能把一个方块放下”，而是：

> 给定一个目标放置位姿，机器人能否完成到达、插入、释放和退出，并在失败时给出明确原因。

因此将放置可执行性写成：

```text
C_place = C_reach * C_insert * C_release * C_retreat
```

只有四项全部通过，Placement Skill 才返回 SUCCESS。

---

## 1. 第一版技能结构

```text
PlacementTarget
      ↓
目标基础几何检查
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

第一版继续使用当前单 FR3、30 mm Cube 和空桌面场景，不加入 B/C 邻近箱体。

Task 03 再加入 B-A-C 场景，研究夹爪插入空间和邻箱干涉。

---

## 2. 新源码

```text
ros_ws/src/fr3_moveit_test/src/placement_skill_demo.cpp
```

Task 01 的：

```text
single_arm_pick_place.cpp
```

继续作为稳定基线，不直接在其上重构。

---

## 3. PlacementTarget

当前放置目标被封装为：

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

默认参数沿用 Task 01 已成功验证的位置：

```text
place_x          = 0.65
place_y          = -0.15
place_cube_z     = 0.065
place_yaw        = 0.0
pre_place_tcp_z  = 0.175
place_tcp_z      = 0.076
retreat_tcp_z    = 0.175
```

这些值现在使用 ROS 2 参数，可以不改源码直接覆盖。

例如：

```bash
ros2 run fr3_moveit_test placement_skill_demo --ros-args \
  -p place_x:=0.70 \
  -p place_y:=-0.10
```

---

## 4. PlacementResult

第一版失败原因明确分类为：

```text
SUCCESS
INVALID_TARGET
REACH_FAILED
INSERT_FAILED
RELEASE_FAILED
RETREAT_FAILED
WORLD_UPDATE_FAILED
```

程序最终会输出：

```text
[PlacementSkill] C_reach   = PASS / FAIL
[PlacementSkill] C_insert  = PASS / FAIL
[PlacementSkill] C_release = PASS / FAIL
[PlacementSkill] C_retreat = PASS / FAIL
[PlacementSkill] RESULT    = ...
```

这使 Placement Skill 从固定动作脚本升级为可以做可执行性判定的工程模块。

---

## 5. 四个可执行性条件的第一版定义

### C_reach

携带 Cube，从 LIFT 状态使用 RRTConnect 到达 PRE_PLACE 完整 6D 位姿。

失败：

```text
RESULT = REACH_FAILED
```

### C_insert

从 PRE_PLACE 到 PLACE 保持 x、y、orientation 不变，只沿 Z 做 Cartesian 直线下降。

要求：

```text
Cartesian fraction >= 0.999
```

失败：

```text
RESULT = INSERT_FAILED
```

### C_release

第一版继续使用 Task 01 已验证的释放状态切换：

```text
DETACH
↓
MoveIt 自动把 Cube 加回 World
↓
removeWorldCube
↓
OPEN / RELEASE
```

当前属于“命令层释放确认”。后续可加入 finger joint / 接触反馈形成更严格的闭环释放判定。

### C_retreat

释放后保持 x/y/姿态，只沿 Z 轴向上退出。

目标 Cube 在释放接触过渡期间临时从 MoveIt World 移除，但其他环境障碍物仍参与碰撞检测。

退出完成后：

```text
addPlacedCubeToWorld(target)
```

将 Cube 按最终目标位姿重新加入 Planning Scene。

---

## 6. 基础目标合法性检查

第一版在机械臂运动前先检查：

```text
1. 参数必须是有限数值；
2. Cube 的 XY 投影不能超出桌面边界；
3. Cube 不能穿入桌面；
4. PRE_PLACE 必须高于 PLACE；
5. RETREAT 必须高于 PLACE。
```

失败时直接：

```text
RESULT = INVALID_TARGET
```

避免机械臂先抓起物体以后才发现最基本的目标定义错误。

---

## 7. 编译与运行

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --packages-select fr3_moveit_test \
  --symlink-install

source install/setup.bash

ros2 run fr3_moveit_test placement_skill_demo
```

Isaac 场景仍使用 Task 01 的 FR3、Table、30 mm PickCube，运行前恢复 Cube 初始位置并点击 Play。

---

## 8. 正常目标验证结果

2026-09-02，默认 PlacementTarget 已在 Isaac Sim 中完整执行成功。

验证链：

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

终端最终确认：

```text
[PlacementSkill] C_reach   = PASS
[PlacementSkill] C_insert  = PASS
[PlacementSkill] C_release = PASS
[PlacementSkill] C_retreat = PASS
[PlacementSkill] RESULT    = SUCCESS
```

Isaac 中 Cube 成功完成搬运、放置和释放，机械臂安全退出并返回 HOME。

---

## 9. Task 02 与 Task 01 的本质区别

两者在当前空桌面正常目标下的物理动作看起来非常接近，这是有意设计的：Task 02 复用 Task 01 已验证动作，不重复发明新的运动原语。

Task 01 的定位是：

```text
固定场景 + 固定目标
→ 按既定流程完整执行
→ 最终返回 true / false
```

它回答：

> 这套单臂 Pick & Place 链能不能跑通？

Task 02 的定位是：

```text
PlacementTarget
→ 目标合法性检查
→ 分阶段可执行性判断
→ SUCCESS / 明确失败原因
```

它回答：

> 对于“这个”放置目标，究竟能不能执行？如果不能，是在哪个阶段失败？

除 ROS 参数化外，Task 02 还新增了：

```text
1. PlacementTarget 作为独立技能输入接口；
2. 运动前目标合法性检查；
3. C_reach / C_insert / C_release / C_retreat 四阶段显式判定；
4. REACH_FAILED / INSERT_FAILED / RETREAT_FAILED 等失败语义；
5. PlacementSkillResult 作为高层决策可读取的技能输出；
6. 放置后按目标参数更新 MoveIt World，而不是依赖固定常量；
7. 为 Task 03 的邻箱干涉和 Task 16 的技能路由提供统一接口。
```

因此，Task 02 当前最重要的进步是“软件架构和可执行性判定能力”，而不是新增一种机械臂运动。

真正明显的算法行为差异会在 Task 03 加入 B/C 邻箱以后出现：某些目标虽然几何上有空位，但 C_insert 或 C_retreat 会因为夹爪扫掠体与邻箱碰撞而失败。

---

## 10. 后续验证

正常目标已经 SUCCESS。下一步依次验证：

```text
A. INVALID_TARGET：目标直接超出桌面；
B. REACH_FAILED：几何目标合法，但机械臂无法到达；
C. INSERT_FAILED：PRE_PLACE 可达，但插入路径被阻挡；
D. RETREAT_FAILED：可放入但夹爪无法安全退出。
```

其中 C/D 需要加入障碍物，最终自然衔接 Task 03 B-A-C 放置可执行性。

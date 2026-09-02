# Task 02 — 机器人可执行 Placement Skill

## 状态

🟡 **进行中 — 第一版参数化 Placement Skill 源码已建立，待本地编译与仿真验证**

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

第一版源码已经提交到 GitHub。

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

这使 Placement Skill 从一个固定动作脚本，升级为可以做“可执行性判定”的工程模块。

---

## 5. 四个可执行性条件的第一版定义

### C_reach

携带 Cube，从 LIFT 状态使用 RRTConnect 到达 PRE_PLACE 完整 6D 位姿。

如果无法规划：

```text
RESULT = REACH_FAILED
```

### C_insert

从 PRE_PLACE 到 PLACE 保持：

```text
x 不变
y 不变
orientation 不变
```

只沿 Z 做 Cartesian 直线下降。

要求：

```text
Cartesian fraction >= 0.999
```

失败返回：

```text
INSERT_FAILED
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

这里暂时属于“命令层释放确认”。后续可加入 finger joint / 接触反馈形成更严格的闭环释放判定。

### C_retreat

释放后保持 x/y/姿态，只沿 Z 轴向上退出。

目标 Cube 在释放接触过渡期间被临时从 MoveIt World 移除，但其他环境障碍物仍参与碰撞检测。

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

## 7. CMakeLists 本地修改

当前 GitHub 的 `fr3_moveit_test` 目录还没有同步本地 `CMakeLists.txt/package.xml`，因此本轮不要用 GitHub 中不存在的配置覆盖本地包。

在本地现有 `CMakeLists.txt` 中，为新节点增加一个 executable。依赖与已经能编译的 `single_arm_pick_place` 保持一致。

推荐新增：

```cmake
add_executable(placement_skill_demo
  src/placement_skill_demo.cpp
)

ament_target_dependencies(placement_skill_demo
  rclcpp
  sensor_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

install(TARGETS
  placement_skill_demo
  DESTINATION lib/${PROJECT_NAME}
)
```

如果本地 CMake 已经使用统一 `${dependencies}` 变量，则直接复用原有依赖写法，不要重复改 `find_package(...)`。

后续应把本地真实 `CMakeLists.txt` 和 `package.xml` 同步到 GitHub，避免仓库只保存 `src/`。

---

## 8. 编译与运行

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

## 9. 第一轮验收标准

默认目标先只验证正常情况。

预期完整链：

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

终端应最终出现：

```text
[PlacementSkill] C_reach   = PASS
[PlacementSkill] C_insert  = PASS
[PlacementSkill] C_release = PASS
[PlacementSkill] C_retreat = PASS
[PlacementSkill] RESULT    = SUCCESS
```

且 Isaac 中 Cube 应稳定落在：

```text
(0.65, -0.15, 0.065)
```

附近，机械臂安全退出并返回 HOME。

---

## 10. 后续验证

第一轮 SUCCESS 后，再做失败案例，而不是同时引入多个变量。

计划依次验证：

```text
A. INVALID_TARGET：目标直接超出桌面；
B. REACH_FAILED：几何目标合法，但机械臂无法到达；
C. INSERT_FAILED：PRE_PLACE 可达，但插入路径被阻挡；
D. RETREAT_FAILED：可放入但夹爪无法安全退出。
```

其中 C/D 需要在后续加入障碍物，最终自然衔接 Task 03 B-A-C 放置可执行性。

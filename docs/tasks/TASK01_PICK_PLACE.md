# Task 01 — 单机械臂 Pick & Place

## 状态

🟡 **进行中 — 抓取、提升、搬运、放置和释放均已运行到位，正在修复释放后的退出规划**

当前 30 mm Cube 已多次稳定完成抓取与提升。本轮完整 Pick & Place 测试进一步验证了：

```text
HOME
→ ALIGNED_APPROACH
→ OPEN_GRIPPER
→ Cartesian Z-only GRASP
→ CLOSE_GRIPPER
→ MOVEIT_ATTACH
→ Cartesian Z-only LIFT
→ Cartesian TRANSFER
→ Cartesian PLACE
→ MOVEIT_DETACH
→ OPEN / RELEASE
```

本轮在释放之后的 `PLACE -> RETREAT` 出现：

```text
Cartesian fraction = 0.0000
MoveIt error_code   = 1
```

这说明 Cartesian 服务本身正常，但退出路径从起点开始就被判定为不可执行。

---

## 1. 当前场景

### FR3

```text
/World/fr3
Translate = (0, 0, 0)
Rotate    = (0, 0, 0)
```

Isaac `World` 与 MoveIt 模型参考坐标系 `base` 按重合处理。

### Table

```text
Center = (0.55, 0.00, 0.025) m
Size   = (1.20, 0.80, 0.05) m
Top Z  = 0.05 m
```

### PickCube

```text
Center = (0.45, 0.15, 0.065) m
Size   = (0.03, 0.03, 0.03) m
Yaw    = 0 rad
Mass   = 0.2 kg
```

### Place 目标

```text
Cube center = (0.65, -0.15, 0.065) m
Yaw         = 0 rad
```

---

## 2. 当前启动方式

长期启动、编译和运行命令统一保存在：

```text
docs/STARTUP_GUIDE.md
```

当前关键命令：

```bash
# 终端 1：MoveIt
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

Isaac Sim 打开当前 FR3 + Table + PickCube 场景并点击 Play。

```bash
# 终端 2：编译与运行
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select fr3_moveit_test --symlink-install
source install/setup.bash
ros2 run fr3_moveit_test single_arm_pick_place
```

---

## 3. 已解决的 `base` 坐标系问题

曾出现：

```text
Robot model frame    = base
Pose reference frame = fr3_link0
Cartesian fraction   = -1.0000
MoveIt error_code    = -21
```

原因为 `FRAME_TRANSFORM_FAILURE`。

当前统一使用：

```text
MoveIt pose reference frame = base
Table CollisionObject frame = base
Cube CollisionObject frame  = base
```

修复后 Cartesian 抓取下降和提升已经多次稳定成功。

---

## 4. 抓取姿态原则

抓取目标使用完整 6D 位姿，而不是只要求 TCP 到达目标点。

当前轴对齐 Cube：

```text
TCP +Z 轴       → 竖直向下
Finger 运动方向 → 与 Cube Y 方向平行
两指夹持面      → 与 Cube 两个侧面平齐
```

抓取姿态：

```text
R_grasp = Rz(cube_yaw) * Rx(pi)
```

当前 `cube_yaw = 0` 时：

```text
Quaternion (x,y,z,w) = (1,0,0,0)
```

---

## 5. 已验证运动参数

```text
APPROACH TCP z = 0.140 m
GRASP    TCP z = 0.075 m
LIFT     TCP z = 0.175 m
PLACE    TCP z = 0.076 m
RETREAT  TCP z = 0.175 m
```

Cartesian 插值步长：

```text
2 mm
```

要求：

```text
fraction >= 0.999
```

夹爪：

```text
OPEN  = 0.040 m / finger
CLOSE = 0.014 m / finger
```

---

## 6. 当前验证结果

已经确认：

```text
ALIGNED_APPROACH          ✅
Cartesian GRASP           ✅ 多次稳定成功
CLOSE_GRIPPER             ✅ 多次稳定成功
MOVEIT_ATTACH             ✅
Cartesian LIFT            ✅ 多次稳定成功
Cartesian TRANSFER        ✅
Cartesian PLACE           ✅ fraction = 1.0000
MOVEIT_DETACH             ✅
OPEN / RELEASE            ✅ Cube 已释放
PLACE -> RETREAT          ❌ fraction = 0.0000
```

因此当前问题已经缩小到“释放后的 MoveIt Planning Scene 状态”。

---

## 7. `DETACH -> RETREAT` 问题原因

MoveIt 的 `AttachedCollisionObject REMOVE` 不只是删除 attached body。

MoveIt `PlanningScene::processAttachedCollisionObjectMsg()` 在执行 DETACH 时，会：

```text
1. 读取 AttachedBody；
2. 从 RobotState 中解除 attached body；
3. 自动把该物体按当前全局姿态重新加入 collision world。
```

因此当前代码注释中“DETACH 后暂时不把 Cube 加回 World”的假设是不正确的。

实际流程变成了：

```text
PLACE
↓
DETACH
↓
MoveIt 自动把 pick_cube 加回 World
↓
夹爪仍处在刚刚接触 Cube 的放置位姿
↓
OPEN
↓
规划 RETREAT
↓
Cartesian 起点被判定与 World Cube 接触 / 碰撞
↓
fraction = 0.0000
```

这与本轮日志完全吻合：`error_code = 1` 表示 Cartesian 请求本身正常，而 `fraction = 0` 表示从起点就无法生成有效无碰撞路径。

---

## 8. 当前修复方案

Task 01 的第一版继续采用简单、确定的释放逻辑：

```text
PLACE
↓
MoveIt DETACH
↓
立即再次从 MoveIt World 删除 pick_cube
↓
OPEN / RELEASE（Isaac 中真实 Cube 不受影响）
↓
Cartesian Z-only RETREAT
↓
机械臂退出完成
↓
按目标位置 (0.65, -0.15, 0.065) 把 pick_cube 加回 MoveIt World
↓
RRTConnect -> HOME
```

即：在退出动作完成前，MoveIt 暂时不让释放后的 Cube 参与 RETREAT 碰撞检查；退出完成以后，再将 Cube 作为新的环境障碍物加入 Planning Scene。

这是 Task 01 的基线实现。Task 02 Placement Skill 将进一步正式处理释放接触、退刀空间和可执行放置等问题。

---

## 9. 当前源码

```text
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp
```

本次需要在 `detachCubeFromTcp()` 成功之后、`OPEN / RELEASE` 之前增加一次：

```cpp
removeWorldCube();
```

验证成功后，将该修复正式保留在 Task 01 源码中。

---

## 10. Task 01 剩余验收

下一轮只需要确认：

```text
1. PLACE -> RETREAT Cartesian fraction = 1.0000；
2. 机械臂竖直退出；
3. Cube 留在目标位置附近；
4. Cube 重新加入 MoveIt World；
5. RETREAT -> HOME 成功；
6. 整个 Pick & Place 完整闭环完成。
```

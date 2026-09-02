# Task 01 — 单机械臂 Pick & Place

## 状态

✅ **已完成**

Task 01 已完成完整闭环验证，并且最终版本可以稳定执行：

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
→ 临时 removeWorldCube
→ OPEN / RELEASE
→ Cartesian Z-only RETREAT
→ addPlacedCubeToWorld
→ RRTConnect HOME
```

用户最终确认：修改 `DETACH -> RETREAT` 释放状态切换后，完整流程可以一次执行到底，Cube 能稳定抓取、提升、搬运、放置、释放，机械臂能够正常退出并返回 HOME。

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

## 2. 启动方式

完整启动、编译和运行方式长期保存在：

```text
docs/STARTUP_GUIDE.md
```

当前关键命令：

```bash
# 终端 1：启动 MoveIt
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

Isaac Sim 打开 FR3 + Table + PickCube 场景并点击 Play。

```bash
# 终端 2：编译并运行
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select fr3_moveit_test --symlink-install
source install/setup.bash
ros2 run fr3_moveit_test single_arm_pick_place
```

---

## 3. 抓取姿态设计

抓取目标必须是完整 6D 位姿，而不是只要求 TCP 到达目标点。

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

当前：

```text
cube_yaw = 0
Quaternion (x,y,z,w) = (1,0,0,0)
```

最终抓取与提升阶段均采用 Cartesian 直线运动，保持 X、Y 和姿态不变，只改变 Z。

---

## 4. 已验证参数

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

Cartesian 路径要求：

```text
fraction >= 0.999
```

夹爪：

```text
OPEN  = 0.040 m / finger
CLOSE = 0.014 m / finger
```

当前 30 mm Cube 在多次重复运行中能够稳定夹持和提升，不需要额外添加 Isaac FixedJoint。

---

## 5. 已解决问题一：Cartesian -21 坐标系错误

曾出现：

```text
Robot model frame    = base
Pose reference frame = fr3_link0
Cartesian fraction   = -1.0000
MoveIt error_code    = -21
```

原因为：

```text
FRAME_TRANSFORM_FAILURE
```

最终统一使用：

```text
MoveIt pose reference frame = base
Table CollisionObject frame = base
Cube CollisionObject frame  = base
```

修复后 Cartesian 抓取下降、提升、搬运、放置和退出均可以正常规划。

---

## 6. 已解决问题二：TCP 到点但抓取不稳定

60 mm Cube 对夹爪 yaw 误差较敏感。仅保证 TCP 到达目标位置无法保证两个 finger 对称接触。

最终采用：

```text
30 mm Cube
+ 完整抓取姿态
+ 抓取前先完成姿态对齐
+ 最终只沿 Z 轴下降
+ 提升只沿 Z 轴上升
```

该方案多次重复测试均能稳定抓取并提升。

---

## 7. 已解决问题三：DETACH 后 RETREAT fraction = 0

### 现象

完整流程第一次运行到释放后出现：

```text
PLACE -> RETREAT
Cartesian fraction = 0.0000
MoveIt error_code   = 1
```

### 根本原因

MoveIt 的 `AttachedCollisionObject::REMOVE` 执行 DETACH 时，并不是只删除附着关系，而是：

```text
AttachedBody
↓
解除附着
↓
自动按当前全局姿态重新加入 collision world
```

因此实际状态是：

```text
PLACE
↓
DETACH
↓
Cube 自动重新成为 MoveIt World CollisionObject
↓
夹爪仍处于释放接触区域
↓
规划 RETREAT
↓
起点附近被判定存在夹爪 / Cube 接触或碰撞
↓
fraction = 0
```

这里不是 Isaac Sim 拒绝运动，而是 MoveIt Planning Scene 在规划阶段判定路径起点无效。

此外，当前 Cartesian 起始 RobotState 只显式设置 `fr3_arm` 7 个关节，Isaac 中 finger 的真实张开状态并不一定完全同步到 MoveIt 当前规划状态，因此不能仅凭 Isaac 画面中“夹爪已经张开”判断 MoveIt 起点一定无碰撞。

### 最终修复

采用确定性的释放状态切换：

```text
PLACE
↓
MoveIt DETACH
↓
MoveIt 自动把 Cube 放回 World
↓
removeWorldCube()
↓
OPEN / RELEASE
↓
Cartesian RETREAT
↓
addPlacedCubeToWorld()
↓
HOME
```

也就是：只在释放接触和退出的过渡阶段临时从 MoveIt Planning Scene 删除 Cube，机械臂退出后马上按最终放置位置重新加入。

该修复已实际验证成功，完整 Pick & Place 可以顺利执行到底。

---

## 8. MoveIt 与 Isaac Sim 的职责边界

```text
MoveIt
→ RobotState
→ Planning Scene
→ CollisionObject / AttachedCollisionObject
→ 路径规划和几何碰撞检查

Isaac Sim
→ 关节真实执行
→ PhysX 刚体
→ 接触
→ 摩擦
→ 重力
```

因此操作任务需要区分：

```text
非法碰撞
和
抓取 / 夹持 / 放置 / 释放等有意接触
```

这也是后续 Placement Skill 的基础。

---

## 9. Task 01 最终验收

```text
ALIGNED_APPROACH          ✅
Cartesian GRASP           ✅ 多次稳定成功
CLOSE_GRIPPER             ✅ 多次稳定成功
MOVEIT ATTACH             ✅
Cartesian LIFT            ✅ 多次稳定成功
Cartesian TRANSFER        ✅
Cartesian PLACE           ✅
MOVEIT DETACH             ✅
OPEN / RELEASE            ✅
Cartesian RETREAT         ✅
Cube 重新加入 MoveIt World ✅
RETREAT -> HOME           ✅
完整 Pick & Place 闭环     ✅
```

## 结论

**Task 01 正式完成。**

下一阶段进入：

```text
Task 02 — Placement Skill / 机器人可执行放置
```

重点从“基础抓放能否执行”转向：放置目标是否对夹爪插入、释放和退出全过程均可执行。

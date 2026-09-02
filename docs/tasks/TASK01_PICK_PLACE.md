# Task 01 — 单机械臂 Pick & Place

## 状态

🟡 **进行中 — 稳定抓取与提升已经验证**

30 mm Cube 已经过多次重复测试，FR3 能够稳定完成：末端姿态对齐 → 垂直下降 → 闭合夹爪 → 抓住 Cube → 垂直提升，并且多次重试后仍然没有掉落。

已经验证的链路：

```text
HOME
  ↓
ALIGNED_APPROACH
  ↓
OPEN_GRIPPER
  ↓
Cartesian Z-only GRASP
  ↓
CLOSE_GRIPPER
  ↓
MOVEIT_ATTACH
  ↓
Cartesian Z-only LIFT
```

当前下一步是一次完成剩余抓放链路：

```text
LIFT
  ↓
TRANSFER
  ↓
PRE_PLACE
  ↓
Cartesian Z-only PLACE
  ↓
MOVEIT_DETACH
  ↓
OPEN / RELEASE
  ↓
Cartesian Z-only RETREAT
  ↓
HOME
```

---

## 1. 当前 Isaac Sim 场景

### FR3

```text
/World/fr3
Translate = (0, 0, 0)
Rotate    = (0, 0, 0)
```

当前 Isaac `World` 与 MoveIt 模型参考坐标系 `base` 按重合处理。

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

30 mm Cube 是当前 Task 01 的基线测试物体。此阶段优先验证完整 Pick & Place 链路，后续 Placement Skill、码垛规划和可执行性实验再重新引入更大的箱体。

---

## 2. 已解决的 Cartesian 坐标系问题

曾出现：

```text
Robot model frame        = base
Pose reference frame     = fr3_link0
Cartesian fraction       = -1.0000
MoveIt error_code        = -21
```

确认原因是：

```text
FRAME_TRANSFORM_FAILURE
```

即 Cartesian Path 服务无法完成 `fr3_link0 -> base` 的坐标变换。

当前统一修复为：

```text
MoveIt pose reference frame = base
Table CollisionObject frame  = base
Cube CollisionObject frame   = base
```

修复后，Cartesian 垂直下降和提升已经多次稳定成功。

---

## 3. 抓取姿态原则

当前已经明确：**抓取目标必须是完整 6D 位姿，不能只要求 TCP 到达 Cube 中心上方。**

对于当前轴对齐 Cube：

```text
TCP +Z 轴        → 竖直向下
Finger 运动方向  → 与 Cube 的 Y 方向平行
两根手指夹持面   → 与 Cube 两个侧面平齐
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

代码由 `CUBE_YAW` 生成四元数，因此后续箱体发生平面旋转时可以直接复用该抓取姿态规则。

---

## 4. 已验证运动方式

当前抓取阶段：

```text
HOME
  ↓
RRTConnect → ALIGNED_APPROACH
  ↓
OPEN_GRIPPER
  ↓
从 MoveIt World 临时删除 pick_cube，允许有意接触
  ↓
Cartesian：保持 X / Y / 姿态不变，只降低 Z
  ↓
CLOSE_GRIPPER
  ↓
MoveIt AttachedCollisionObject
  ↓
Cartesian：保持 X / Y / 姿态不变，只提高 Z
```

当前 TCP 高度：

```text
APPROACH z = 0.140 m
GRASP    z = 0.075 m
LIFT     z = 0.175 m
```

Cartesian 插值步长：

```text
2 mm
```

只有在：

```text
fraction >= 0.999
```

时才执行 Cartesian 轨迹。

---

## 5. 夹爪参数

```text
OPEN  = 0.040 m / 每个 finger joint
CLOSE = 0.014 m / 每个 finger joint
```

对应约：

```text
OPEN  ≈ 80 mm 总开口
CLOSE ≈ 28 mm 总目标开口
```

对 30 mm Cube 形成轻微预紧。

在抓取下降过程中持续保持 OPEN 指令；提升和搬运过程中持续保持 CLOSE 指令。

---

## 6. MoveIt Attached Object

闭合夹爪后，MoveIt 将 `pick_cube` 表示为 AttachedCollisionObject：

```text
attached link = fr3_hand_tcp
object size   = 0.03 × 0.03 × 0.03 m
touch links   = fr3_hand_tcp, fr3_hand, fr3_leftfinger, fr3_rightfinger
```

当前 Isaac Sim 不使用 FixedJoint，而是直接依赖真实 finger / Cube 接触。

多次稳定提升已经证明：Task 01 当前基线不需要添加 Isaac FixedJoint。

---

## 7. 下一阶段放置参数

第一版目标仍然放在同一张桌面上，减少额外变量。

### Pick 位置

```text
Cube center = (0.45, 0.15, 0.065) m
```

### Place 位置

```text
Cube center = (0.65, -0.15, 0.065) m
Yaw         = 0 rad
```

为了避免下降时把物体压进桌面，释放前的 TCP 高度计划采用：

```text
PLACE_TCP_Z = 0.076 m
```

即理论上 Cube 底面距离桌面约 1 mm，打开夹爪后让 Cube 自由下落约 1 mm 到桌面。

预放置 / 退出高度：

```text
PRE_PLACE / RETREAT TCP z = 0.175 m
```

---

## 8. 下一版完整运动链

下一版源码一次完成：

```text
HOME
  ↓
ALIGNED_APPROACH
  ↓
OPEN
  ↓
Cartesian GRASP
  ↓
CLOSE
  ↓
MoveIt ATTACH
  ↓
Cartesian LIFT
  ↓
Cartesian TRANSFER
  ↓
Cartesian PLACE
  ↓
MoveIt DETACH
  ↓
OPEN / RELEASE
  ↓
Cartesian RETREAT
  ↓
将已放置 Cube 重新加入 MoveIt World
  ↓
RRTConnect 回 HOME
```

其中 `LIFT → PRE_PLACE` 第一版直接使用 Cartesian 水平搬运，以保持：

```text
Z 不变
末端姿态不变
Cube 始终保持竖直
```

这样优先保证真实接触夹持的稳定性。后续进入存在障碍物的码垛任务后，再使用 RRT / RRT-Connect 进行更复杂的携物避障规划。

---

## 9. 当前源码

```text
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp
```

下一版源码应同时包含：

```text
30 mm Cube
+ base 统一参考坐标系
+ 完整抓取姿态
+ Cartesian Z-only grasp
+ MoveIt attach
+ Cartesian lift
+ Cartesian transfer
+ Cartesian place
+ MoveIt detach
+ open / release
+ Cartesian retreat
+ return HOME
```

---

## 10. 验证状态

```text
Task 00 MoveIt → Isaac 基线            ✅
60 mm Cube 抓取                       ✅ 对齐时可成功
60 mm 鲁棒性问题定位                  ✅
30 mm Isaac Cube                     ✅
完整抓取姿态设计                      ✅
base 参考坐标系修复                   ✅
ALIGNED_APPROACH                      ✅
Cartesian Z-only GRASP               ✅ 多次稳定成功
CLOSE_GRIPPER                         ✅ 多次稳定成功
MOVEIT ATTACH                         ✅
Cartesian Z-only LIFT                ✅ 多次稳定成功
TRANSFER                              下一步
PLACE                                 下一步
DETACH                                下一步
OPEN / RELEASE                        下一步
RETREAT                               下一步
HOME / DONE                           下一步
```

下一版验收标准：

```text
1. 稳定抓起 Cube；
2. 保持姿态搬运到目标上方；
3. 垂直下降到放置高度；
4. 打开夹爪后 Cube 留在目标位置附近；
5. 机械臂垂直退出；
6. 返回 HOME；
7. 整个过程无明显碰撞、弹飞或掉落。
```

启动、编译和运行方式见：

```text
docs/STARTUP_GUIDE.md
```

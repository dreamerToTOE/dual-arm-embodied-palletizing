# 关键问题与排错记录

本文件专门记录项目开发过程中具有复用价值的关键问题，包括：**现象、日志、根本原因、错误理解、解决方案、验证方法和后续工程意义**。

原则：

```text
1. 只要问题会影响后续复现、规划逻辑、仿真物理或论文方法设计，就记录在这里；
2. 不只记录“怎么修”，还要记录“为什么出错”；
3. 区分 MoveIt 规划层和 Isaac Sim 物理层；
4. 关键修复同时同步到对应 Task 文档和源码注释。
```

---

# 问题 01：Cartesian Path 返回 -1，MoveIt error_code = -21

## 现象

在 Task 01 中，HOME 到抓取上方的 RRTConnect 规划可以成功，但从 APPROACH 向 GRASP 做 Cartesian 直线下降时出现：

```text
Robot model frame = 'base'
Current pose reference frame = 'fr3_link0'

APPROACH -> GRASP [Cartesian Z-only]
Cartesian fraction = -1.0000
Cartesian MoveIt error_code = -21
```

## 根本原因

`-21` 对应：

```text
FRAME_TRANSFORM_FAILURE
```

当前 RobotModel 的模型参考坐标系是：

```text
base
```

但代码曾设置：

```cpp
move_group.setPoseReferenceFrame("fr3_link0");
```

Cartesian Path 服务需要把 waypoint 从 `fr3_link0` 转换到 RobotModel 的 `base` 坐标系。当前 MoveIt/TF 环境没有提供这条服务所需的有效变换，因此整个 Cartesian 请求在轨迹计算前就失败。

这里的：

```text
fraction = -1
```

不是“轨迹完成了 -100%”，而是表示 Cartesian 服务调用本身发生错误。

## 解决方案

整个 Task 01 统一使用 MoveIt RobotModel 的模型坐标系：

```text
MoveIt Pose Reference Frame = base
Table CollisionObject Frame = base
Cube CollisionObject Frame  = base
```

代码：

```cpp
move_group.setPoseReferenceFrame("base");
```

Planning Scene 中：

```cpp
table.header.frame_id = "base";
cube.header.frame_id  = "base";
```

## 验证结果

修复后：

```text
Cartesian fraction = 1.0000
MoveIt error_code = 1
```

抓取垂直下降和垂直提升均可稳定执行。

## 工程意义

后续所有 Cartesian Skill 在定义 waypoint 时，应优先使用统一、明确、稳定的规划参考系。不要仅因为两个坐标系在仿真场景中“看起来重合”，就假设 MoveIt 服务一定能完成二者之间的 TF 转换。

---

# 问题 02：仅保证 TCP 到达抓取点，仍会出现抓取失败

## 现象

最初使用约 60 mm Cube。TCP 虽然可以移动到 Cube 上方并下降，但只要末端 yaw 略有偏差，两个手指就不能同时、对称地接触 Cube 两侧，导致：

```text
- 一侧先接触；
- 另一侧接触不足；
- Cube 偏转；
- 抬升时脱落；
- 多次运行成功率不稳定。
```

## 根本原因

抓取目标不能只定义为三维位置：

```text
(x, y, z)
```

而必须定义完整的 6D 抓取位姿：

```text
位置 + 姿态
```

对于平行两指夹爪，除了 TCP 到达 Cube 中心上方外，还必须满足：

```text
1. TCP 工具轴竖直向下；
2. 两指开合方向与目标物体对应侧面方向对齐；
3. 最终接近阶段保持姿态不变；
4. 只沿抓取方向直线下降。
```

当前轴对齐 Cube 的抓取姿态定义为：

```text
R_grasp = Rz(cube_yaw) * Rx(pi)
```

当：

```text
cube_yaw = 0
```

使用：

```text
Quaternion (x,y,z,w) = (1,0,0,0)
```

## 同时采取的基线简化

Task 01 当前目标是先打通完整 Pick & Place 链，因此 Cube 从 60 mm 缩小为：

```text
30 mm × 30 mm × 30 mm
```

Isaac 中当前参数：

```text
Center = (0.45, 0.15, 0.065) m
Size   = (0.03, 0.03, 0.03) m
Yaw    = 0 rad
```

夹爪：

```text
OPEN  = 0.040 m / finger
CLOSE = 0.014 m / finger
```

## 运动策略修正

原来：

```text
RRTConnect 到 Approach
↓
再次自由规划到 Grasp
```

修改为：

```text
RRTConnect 到“完整姿态已经对齐”的 Approach
↓
Cartesian：保持 X / Y / orientation
↓
只改变 Z
↓
垂直下降到 Grasp
```

Lift 同样采用保持姿态不变的 Cartesian Z-only 上升。

## 验证结果

30 mm Cube + 完整末端姿态 + Cartesian Z-only 抓取后，多次重复测试仍可稳定：

```text
抓住 Cube
↓
不弹飞
↓
稳定提升
```

## 工程意义

该问题直接对应后续抓取技能的基本定义：

```text
抓取目标不是“一个点”，而是“一个满足几何约束的末端位姿 + 接近方向”。
```

未来对于有 yaw 的箱体，应由箱体姿态动态生成抓取姿态，而不是永久硬编码一个四元数。

---

# 问题 03：DETACH 后 PLACE -> RETREAT 的 Cartesian fraction = 0

## 现象

完整 Pick & Place 已经成功执行到：

```text
TRANSFER
↓
PLACE
↓
DETACH
↓
OPEN / RELEASE
```

随后规划：

```text
PLACE -> RETREAT
```

出现：

```text
PLACE -> RETREAT：Cartesian fraction = 0.0000
PLACE -> RETREAT：MoveIt error_code = 1
```

## 首先要区分：这不是 Isaac Sim 拒绝了运动

该失败发生在：

```text
MoveIt Cartesian Path 规划阶段
```

而不是 Isaac Sim 的 PhysX 执行阶段。

两层职责：

```text
MoveIt
  → 维护 Planning Scene
  → 判断几何碰撞
  → 生成轨迹

Isaac Sim
  → 真实执行关节运动
  → 计算接触、摩擦、重力和刚体运动
```

因此这里的 `fraction = 0` 是 MoveIt 认为起点附近没有有效的无碰撞 Cartesian 路径，不是 Isaac Sim 直接阻止了机械臂。

## DETACH 的真实含义

### ATTACH

抓取后：

```text
World CollisionObject
↓
AttachedCollisionObject
↓
MoveIt 认为 Cube 随机器人运动
```

它主要是规划模型中的附着关系。

### DETACH

DETACH 不只是“删除 attached object”。

MoveIt 在 `AttachedCollisionObject::REMOVE` 时会：

```text
1. 从 RobotState 获取 AttachedBody；
2. 清除 AttachedBody；
3. 按当前全局姿态把该物体重新加入 collision world。
```

因此：

```text
DETACH = 解除机器人与物体的规划附着关系
       + 让物体重新成为环境中的 World CollisionObject
```

这与 Isaac Sim 中夹爪是否已经物理张开是两个不同层面的状态。

## 根本原因

本轮原代码的错误假设是：

```text
DETACH 后 Cube 暂时不在 MoveIt World 中
```

实际上 MoveIt 会自动把它加回 World。

于是实际状态为：

```text
PLACE
↓
DETACH
↓
MoveIt 自动把 Cube 加回 World
↓
夹爪仍处在刚完成放置的接触区域
↓
OPEN / RELEASE
↓
开始规划 RETREAT
↓
MoveIt Planning Scene 中起点附近夹爪与 World Cube 接触 / 碰撞
↓
Cartesian Path 从第一个插值点就无法前进
↓
fraction = 0.0000
```

这里：

```text
MoveIt error_code = 1
```

说明 Cartesian 服务本身是正常执行的；与此前 `-21 FRAME_TRANSFORM_FAILURE` 不同。本次是路径有效性/碰撞层面的问题。

## 为什么 Isaac 已经张开夹爪，MoveIt 仍可能认为有碰撞

当前程序自己构造 Cartesian 起始 RobotState：

```cpp
start_state.setToDefaultValues();
start_state.setJointGroupPositions(joint_model_group, start_q);
```

其中：

```text
joint_model_group = fr3_arm
```

只包含机械臂 7 个关节。

Isaac 中两个 finger joint 的实际张开状态并没有在这里显式写入当前 MoveIt start_state。

因此：

```text
Isaac 中真实夹爪状态
```

与：

```text
MoveIt 在本次规划中使用的夹爪状态
```

并不一定完全同步。

所以不能用“Isaac 画面中已经张开”来证明 MoveIt Planning Scene 的起点一定无碰撞。

## Task 01 当前解决方案

在基线 Task 01 中采用确定性释放流程：

```text
PLACE
↓
MoveIt DETACH
↓
MoveIt 自动把 Cube 放回 World
↓
立即 removeWorldCube()
↓
暂时从 MoveIt World 删除 Cube
↓
Isaac OPEN / RELEASE
↓
Cube 在真实物理仿真中落到桌面
↓
MoveIt 规划 Cartesian Z-only RETREAT
↓
夹爪安全退出
↓
addPlacedCubeToWorld()
↓
按目标位置把 Cube 重新加入 MoveIt Planning Scene
↓
RRTConnect -> HOME
```

对应代码关键修复：

```cpp
if (!detachCubeFromTcp())
{
    return false;
}

// DETACH 会自动把 Cube 放回 MoveIt World。
// 释放瞬间属于有意接触阶段，为避免 RETREAT 起点碰撞，
// 先暂时从 Planning Scene 删除。
removeWorldCube();

commandGripper(GRIPPER_OPEN_POS, 1.0);

// ... RETREAT ...

addPlacedCubeToWorld();
```

## 为什么这不是“关闭碰撞检测”

这里不是永久忽略 Cube，而是只在**释放接触过渡阶段**暂时不让 Cube 参与 MoveIt 的退出路径碰撞判断。

状态机可理解为：

```text
环境物体
↓
允许抓取接触
↓
Attached Object
↓
允许释放接触
↓
环境物体
```

机械臂退出以后 Cube 会重新加入 MoveIt World，后续规划仍会把它作为障碍物。

## 后续更正式的处理方向

Task 01 只要求建立稳定基线，因此采用上面的确定性状态切换。

进入 Task 02 Placement Skill 后，应进一步研究：

```text
- release contact 的允许碰撞策略；
- 夹爪真实 joint state 与 MoveIt RobotState 同步；
- 放置后的物体真实位姿更新；
- retreat swept volume；
- 释放方向和退出方向；
- 周围箱体对夹爪退出空间的限制。
```

这也是“几何可放置”与“机器人实际可执行放置”之间的重要区别。

---

# 当前关键概念总结

## MoveIt 与 Isaac Sim 的分工

```text
MoveIt：
规划、RobotState、Planning Scene、CollisionObject、AttachedCollisionObject

Isaac Sim：
刚体、碰撞接触、摩擦、重力、关节动力学、真实运动结果
```

## ATTACH / DETACH

```text
ATTACH
= MoveIt 将物体作为机器人携带物体进行规划

DETACH
= MoveIt 解除携带关系，并把物体重新放回环境碰撞世界
```

## 有意接触与非法碰撞

后续操作规划必须区分：

```text
非法碰撞：
机械臂与障碍物、非目标箱体等不应该发生的碰撞

有意接触：
抓取接触、夹持接触、放置接触、释放接触
```

机器人操作任务不能简单采用“任何接触都禁止”的碰撞逻辑。

---

# 文档维护规则

后续遇到以下类型问题时继续追加本文件：

```text
- 坐标系 / TF；
- Planning Scene；
- CollisionObject / AttachedCollisionObject；
- 抓取失败；
- 接触与摩擦；
- Cartesian Path / RRT 失败；
- 双臂碰撞；
- 轨迹时序；
- 力控；
- Isaac / ROS 2 状态不同步；
- 论文方法中具有解释价值的失败案例。
```

# 关键问题与排错记录

本文件专门记录项目开发过程中具有复用价值的关键问题，包括：**现象、日志、根本原因、错误理解、解决方案、验证方法、最终结果和工程意义**。

原则：

```text
1. 只要问题会影响后续复现、规划逻辑、仿真物理或论文方法设计，就记录在这里；
2. 不只记录“怎么修”，还要记录“为什么出错”；
3. 明确区分 MoveIt 规划层和 Isaac Sim 物理层；
4. 关键修复同时同步到对应 Task 文档和源码注释；
5. 只有用户实际验证通过后，才把解决方案标记为“已验证”。
```

---

# 问题 01：Cartesian Path 返回 -1，MoveIt error_code = -21

## 现象

在 Task 01 中，HOME 到抓取上方的 RRTConnect 规划可以成功，但 APPROACH 向 GRASP 做 Cartesian 直线下降时出现：

```text
Robot model frame = 'base'
Current pose reference frame = 'fr3_link0'

APPROACH -> GRASP
Cartesian fraction = -1.0000
Cartesian MoveIt error_code = -21
```

## 根本原因

`-21` 对应：

```text
FRAME_TRANSFORM_FAILURE
```

RobotModel 的模型坐标系是：

```text
base
```

但程序曾设置：

```cpp
move_group.setPoseReferenceFrame("fr3_link0");
```

Cartesian Path 服务需要把 waypoint 从 `fr3_link0` 转换到 `base`。当前规划环境没有提供该服务所需的有效变换，因此请求在真正计算笛卡尔轨迹之前就失败。

这里：

```text
fraction = -1
```

不是“完成比例为 -100%”，而是表示 Cartesian 请求本身发生错误。

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
table.header.frame_id = "base";
cube.header.frame_id  = "base";
```

## 最终验证

修复后：

```text
Cartesian fraction = 1.0000
MoveIt error_code = 1
```

并且后续抓取下降、提升、水平搬运、放置下降和退出均可以正常计算 Cartesian Path。

## 工程意义

后续所有 Cartesian Skill 应统一、明确地定义规划参考坐标系。两个坐标系在 Isaac 场景里“看起来重合”，不代表 MoveIt 服务一定能够完成对应 TF 转换。

---

# 问题 02：仅保证 TCP 到达抓取点，仍然抓取不稳定

## 现象

最初采用约 60 mm Cube。TCP 虽然能够到达 Cube 上方并下降，但只要末端 yaw 有一定偏差，两根手指就不能同时、对称地接触 Cube 两侧，出现：

```text
一侧先接触
另一侧接触不足
Cube 偏转
Cube 被弹开
Lift 时脱落
重复运行稳定性差
```

## 根本原因

抓取目标不能只定义三维位置：

```text
(x, y, z)
```

而应定义完整的 6D 抓取位姿：

```text
位置 + 姿态
```

对于平行两指夹爪，至少需要：

```text
1. TCP 工具轴竖直向下；
2. 两指开合方向与目标物体对应侧面方向对齐；
3. 最终接近阶段保持姿态不变；
4. 只沿抓取方向直线下降。
```

当前轴对齐 Cube：

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

## 基线简化

Task 01 的目标是先稳定打通完整 Pick & Place，因此 Cube 缩小为：

```text
30 mm × 30 mm × 30 mm
```

Isaac 参数：

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

## 运动策略修改

原方式：

```text
RRTConnect -> Approach
↓
再次自由规划 -> Grasp
```

修改为：

```text
RRTConnect -> 已完成姿态对齐的 Approach
↓
Cartesian：保持 X / Y / orientation
↓
只降低 Z
↓
GRASP
```

Lift 同样保持姿态不变，只提高 Z。

## 最终验证

30 mm Cube + 完整末端姿态 + Cartesian Z-only 抓取以后，多次重复测试均能够稳定：

```text
抓住 Cube
↓
不弹飞
↓
稳定提升
↓
继续完成搬运和放置
```

## 工程意义

抓取目标不是一个“点”，而是：

```text
满足几何约束的末端位姿
+
明确的接近方向
```

后续有 yaw 的箱体应由箱体位姿动态生成抓取姿态，而不是永久硬编码一个四元数。

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

## 首先区分：不是 Isaac Sim 拒绝了运动

失败发生在：

```text
MoveIt Cartesian Path 规划阶段
```

而不是 Isaac Sim 的 PhysX 执行阶段。

职责区分：

```text
MoveIt
→ RobotState
→ Planning Scene
→ 几何碰撞判断
→ 路径规划

Isaac Sim
→ 实际执行关节运动
→ 接触
→ 摩擦
→ 重力
→ 刚体动力学
```

所以 `fraction = 0` 表示 MoveIt 从规划起点开始就无法生成有效无碰撞 Cartesian Path，而不是 Isaac Sim 主动阻止了机械臂。

## ATTACH / DETACH 的真实含义

### ATTACH

```text
World CollisionObject
↓
AttachedCollisionObject
↓
MoveIt 把 Cube 当作机器人携带物体参与规划
```

### DETACH

DETACH 不是简单删除 attached object。

MoveIt 在 `AttachedCollisionObject::REMOVE` 时会：

```text
1. 从 RobotState 获取 AttachedBody；
2. 清除 AttachedBody；
3. 按当前全局姿态把该物体重新加入 collision world。
```

因此：

```text
DETACH
= 解除机器人与物体的规划附着关系
+ 让物体重新成为环境中的 World CollisionObject
```

这与 Isaac 中夹爪是否已经张开属于不同层面的状态。

## 根本原因

原代码错误假设：

```text
DETACH 后 Cube 暂时不在 MoveIt World
```

实际：

```text
PLACE
↓
DETACH
↓
MoveIt 自动把 Cube 加回 World
↓
夹爪仍处于刚完成放置的释放接触区域
↓
OPEN / RELEASE
↓
规划 RETREAT
↓
Planning Scene 认为起点附近夹爪与 World Cube 接触 / 碰撞
↓
Cartesian Path 从第一个插值点就不能前进
↓
fraction = 0.0000
```

这里：

```text
MoveIt error_code = 1
```

说明 Cartesian 服务调用本身正常，与问题 01 的 `-21 FRAME_TRANSFORM_FAILURE` 完全不同。本次问题在路径有效性 / 碰撞层面。

## 为什么 Isaac 已经张开夹爪，MoveIt 仍可能判断碰撞

当前 Cartesian 起始状态由程序构造：

```cpp
start_state.setToDefaultValues();
start_state.setJointGroupPositions(joint_model_group, start_q);
```

其中：

```text
joint_model_group = fr3_arm
```

只显式设置机械臂 7 个关节。

Isaac 中：

```text
fr3_finger_joint1
fr3_finger_joint2
```

的真实张开状态并没有在这里显式写入本次 MoveIt `start_state`。

所以：

```text
Isaac 中真实夹爪状态
```

和：

```text
MoveIt 本次规划使用的夹爪状态
```

并不保证完全一致。

不能仅根据 Isaac 画面里“夹爪已经张开”推断 MoveIt Planning Scene 起点必然无碰撞。

## 最终解决方案

Task 01 使用确定性的释放状态切换：

```text
PLACE
↓
MoveIt DETACH
↓
MoveIt 自动把 Cube 放回 World
↓
removeWorldCube()
↓
暂时从 MoveIt World 删除 Cube
↓
Isaac OPEN / RELEASE
↓
Cube 在 PhysX 中落到桌面
↓
MoveIt 规划 Cartesian Z-only RETREAT
↓
机械臂退出
↓
addPlacedCubeToWorld()
↓
按目标位置重新把 Cube 加入 MoveIt Planning Scene
↓
RRTConnect -> HOME
```

关键代码：

```cpp
if (!detachCubeFromTcp())
{
    return false;
}

// DETACH 会自动把 Cube 放回 MoveIt World。
// 释放瞬间属于有意接触阶段，先临时从 Planning Scene 删除。
removeWorldCube();

commandGripper(GRIPPER_OPEN_POS, 1.0);

// Cartesian RETREAT

addPlacedCubeToWorld();
```

## 为什么这不是“关闭碰撞检测”

Cube 只在“释放接触 -> 夹爪退出”的短暂过渡阶段被临时移出 Planning Scene。

状态可以理解为：

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

机械臂退出以后，Cube 马上重新加入 MoveIt World，后续规划仍然会把它当作障碍物。

## 最终验证结果

加入：

```cpp
removeWorldCube();
```

后重新完整运行，用户确认完整执行成功。

最终实际验证：

```text
PLACE -> RETREAT          ✅
Cartesian RETREAT         ✅
Cube 留在目标放置位置      ✅
addPlacedCubeToWorld      ✅
RETREAT -> HOME           ✅
完整 Pick & Place 闭环     ✅
```

因此该问题正式标记为：

```text
✅ 已解决并仿真验证
```

## 工程意义

这是后续 Placement Skill 非常重要的基础问题：

```text
几何上能放下物体
≠
机器人能够完成插入、释放和退出
```

后续应正式考虑：release contact 的允许碰撞策略、夹爪真实 joint state 与 MoveIt RobotState 同步、放置后的物体真实位姿更新、retreat swept volume、释放方向、退出方向以及周围箱体对夹爪退出空间的限制。

---

# 问题 04：Task 03 中 Isaac 已发生夹爪碰撞，但 MoveIt C_insert 仍返回 1.0000

## 现象

Task 03 构造 B-A-C 场景：

```text
A size   = 30 mm
A target = (0.65, -0.15, 0.065)
B center = (0.65, -0.115, 0.065)
C center = (0.65, -0.185, 0.065)
d_BC     = 40 mm
```

满足：

```text
d_BC = 40 mm > w_A = 30 mm
```

也就是 A 本体几何上可以放入 B/C 之间。

Task 03 已确认 `box_b`、`box_c` 被加入 MoveIt Planning Scene，但运行结果却是：

```text
C_reach   = PASS
C_insert  = PASS
C_release = PASS
C_retreat = PASS
RESULT    = SUCCESS
```

其中：

```text
C_insert: PRE_PLACE -> PLACE
Cartesian fraction = 1.0000
MoveIt error_code = 1
```

与此同时，Isaac Sim 画面中能够直接观察到：

```text
夹爪与 B/C 发生物理碰撞，
机械臂仍继续沿 ROS 轨迹运动，最终把 A 硬塞进 B/C 之间。
```

## 为什么这是 MoveIt 与 Isaac 状态不一致，而不是 PhysX 失效

Isaac Sim 负责实际动力学执行。它已经发生接触，说明真实仿真几何确实发生了碰撞。

但 `/joint_command` 当前是位置目标控制。Isaac 的 Articulation Controller 仍会努力跟随位置指令，所以“发生碰撞”并不自动等价于“机械臂立即停止”。

真正应该在运动前阻止这条插入轨迹的是 MoveIt 的碰撞检测。

因此本次异常点是：

```text
MoveIt 判断无碰撞
但 Isaac 执行时发生碰撞
```

## 根本原因

Task 02 / Task 03 当前规划函数构造 `RobotState` 的方式是：

```cpp
moveit::core::RobotState start_state(move_group.getRobotModel());
start_state.setToDefaultValues();
start_state.setJointGroupPositions(joint_model_group, start_q);
start_state.update();
```

而：

```text
joint_model_group = fr3_arm
```

只包含 FR3 的 7 个机械臂关节。

程序执行抓取后，Isaac 中真实夹爪持续保持：

```text
fr3_finger_joint1 = 0.014
fr3_finger_joint2 = 0.014
```

但是 MoveIt 规划状态没有把这两个 finger joint 写入，只保留 `setToDefaultValues()` 给出的模型默认状态。

于是形成：

```text
真实 Isaac RobotState
= 7 arm joints
+ finger1 = 0.014
+ finger2 = 0.014

MoveIt 规划 RobotState
= 7 arm joints
+ finger1 = 默认值
+ finger2 = 默认值
```

如果默认 finger 状态对应的夹爪外包络比真实抓取状态更窄，MoveIt 就会认为 40 mm 间隙可穿过；而 Isaac 中真实夹爪则会撞到 B/C。

这与 Task 01 DETACH 问题中发现的“MoveIt 夹爪状态不一定等于 Isaac 画面状态”属于同一类根因，只是在 Task 03 中第一次直接表现为 **环境障碍物碰撞漏检**。

## 当前修复方案

必须让 MoveIt 用完整 RobotState 进行碰撞检查。

携物阶段 C_reach / C_insert 应显式设置：

```cpp
start_state.setVariablePosition(
    "fr3_finger_joint1",
    GRIPPER_CLOSE_POS);
start_state.setVariablePosition(
    "fr3_finger_joint2",
    GRIPPER_CLOSE_POS);
start_state.update();
```

当前：

```text
GRIPPER_CLOSE_POS = 0.014 m
```

释放后的 C_retreat 则应显式使用：

```text
GRIPPER_OPEN_POS = 0.040 m
```

推荐把 `finger_position` 作为规划函数输入，而不是在函数内部猜测当前动作阶段，例如：

```text
planPoseStage(..., finger_position)
planCartesianStage(..., finger_position)
```

从而保证：

```text
规划状态
=
实际执行状态
```

## 下一轮验收

保持完全相同的 B-A-C 场景：

```text
d_BC = 40 mm
w_A  = 30 mm
```

只修改 MoveIt RobotState 的 finger joints，不改变 B/C 位置、不膨胀障碍物、不降低碰撞阈值。

预期：

```text
C_reach = PASS
C_insert < 0.999
RESULT = INSERT_FAILED
```

如果显式同步 finger=0.014 后仍然 `C_insert = PASS`，再检查：

```text
1. FR3 finger collision geometry 是否实际存在；
2. box_b / box_c 是否仍在 Planning Scene；
3. touch_links / Allowed Collision Matrix 是否误放行 finger ↔ box_b/c；
4. 40 mm 是否确实仍大于真实 MoveIt finger collision envelope 的临界宽度。
```

当前解决方案状态：

```text
🟡 根因已定位，待修改并验证
```

## 工程意义

以后双臂避碰、抓取搬运、放置插入、紧协调共同搬运时，不能只同步机械臂关节。

完整碰撞状态至少需要：

```text
RobotState
=
arm joints
+ gripper joints
+ attached object
+ Planning Scene environment
```

否则即使 Planning Scene 中障碍物建模完全正确，也可能得到错误的“无碰撞”规划结果。

---

# 当前关键概念总结

## MoveIt 与 Isaac Sim 的分工

```text
MoveIt：
规划、RobotState、Planning Scene、CollisionObject、AttachedCollisionObject

Isaac Sim：
刚体、接触、摩擦、重力、关节动力学、真实运动结果
```

## ATTACH / DETACH

```text
ATTACH
= MoveIt 将物体作为机器人携带物体进行规划

DETACH
= MoveIt 解除携带关系，并把物体重新放回环境碰撞世界
```

## 有意接触与非法碰撞

机器人操作规划必须区分：

```text
非法碰撞：
机械臂与障碍物、非目标箱体等不应该发生的碰撞

有意接触：
抓取接触、夹持接触、放置接触、释放接触
```

不能简单使用“所有接触都禁止”的策略处理完整操作任务。

---

# 文档维护规则

后续出现以下类型问题时继续追加到本文件：

```text
TF / 坐标系
Planning Scene
CollisionObject
AttachedCollisionObject
抓取失败
接触 / 摩擦
Cartesian Path
RRT / RRT-Connect
双臂碰撞
轨迹时序
Isaac 与 MoveIt 状态不同步
力控 / wrench / internal force
具有论文方法价值的失败案例
```

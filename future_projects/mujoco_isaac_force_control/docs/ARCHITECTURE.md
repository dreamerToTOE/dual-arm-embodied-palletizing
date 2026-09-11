# 项目架构：Controller Core + Simulator Adapter

## 核心原则

控制算法与仿真器 API 必须分离。

错误结构：

```text
MuJoCo API + 控制公式 + 日志 + 场景逻辑
全部写在一个脚本
```

推荐结构：

```text
Simulator Adapter
      ↓
统一 RobotState / DynamicsState
      ↓
Controller Core
      ↓
JointTorqueCommand
      ↓
Simulator Adapter
```

## 数据结构建议

```text
RobotState
  q
  qdot
  tcp_pose
  tcp_twist

DynamicsState
  M
  Cqdot
  g
  J

ContactState
  tcp_wrench
  contact_active

ControlCommand
  tau
```

## RobotModelInterface

控制器只能依赖抽象接口：

```text
get_joint_state()
get_tcp_state()
get_mass_matrix()
get_coriolis()
get_gravity()
get_jacobian()
get_tcp_wrench()
set_joint_torque()
step()
```

## MuJoCoAdapter

职责：

- MuJoCo joint index -> 项目统一 FR3 joint order；
- 从 MuJoCo 读取 q/qdot；
- 提供 Jacobian、动力学量、传感器 wrench；
- 将 `tau` 写入 actuator/control；
- 不包含 impedance/QP 等控制算法。

## IsaacAdapter

职责与 MuJoCoAdapter 相同，但内部调用 Isaac Sim / Isaac Lab / PhysX 对应接口。

## Controller Core

示例：

```text
JointPDController
ComputedTorqueController
CartesianImpedanceController
ForceController
HybridPositionForceController
DualArmObjectController
WrenchAllocatorQP
InternalForceController
```

输入只接受统一数据结构，输出 `tau`。

## 坐标系约定

正式实现前固定：

```text
world frame
left_base / right_base
left_tcp / right_tcp
object frame
left_grasp / right_grasp
```

任何 wrench 和 Jacobian 必须记录表达坐标系。

推荐：

- 单臂动力学：base frame；
- 接触 wrench：TCP/contact frame 后统一转换；
- 双臂共同物体：object frame；
- 日志同时记录 frame id。

## 单位约定

统一 SI：

```text
m
rad
s
kg
N
N·m
```

## 测试原则

每新增一个 Adapter，先运行与 MuJoCo 相同的小测试：

```text
joint order
zero torque
gravity
Jacobian finite difference
J^T W sign
wrench frame
```

这些底层验证未通过前，不允许直接调高层 QP / impedance 参数。

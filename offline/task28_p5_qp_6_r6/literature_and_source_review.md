# QP-6-R6 技术审查（主动诊断前）

本审查在任何新的 Isaac 运行代码之前完成。R6 只增加一个受限的
`DIAGNOSTIC_ACTUATION` 层级；它不把主动输入等同于完整逆动力学、硬件力矩
传感器或生产安全认证。

## 资料与安装版本

* PhysX 5.7 的 Simulation 文档说明固定步长由 `simulate()`/`fetchResults()`
  组成，fetch 完成后才可读取该步结果；TGS 子步的末步速度因此可以和整步
  位移差分不同。R5 的 `end_step_dq` 解释继续有效，R4 的
  `fd(q)/dt == dq` 作为硬门则过强。
* Isaac Sim 4.5 的 `Articulation.set_joint_efforts()` 实现每次先建立全
  DOF 零 buffer，再把选中的索引写入；只写七个 arm DOF 会清掉两个手指的
  command buffer。R6 发送完整九维 buffer，保留手指输入，并把
  `get_applied_joint_efforts()` 只记为 command readback，而不称为 torque
  sensor。`get_measured_joint_efforts()` 只作 solver-projected 辅助量。
* TSID/Pinocchio 的项目说明把任务空间逆动力学建立在适用的刚体模型上。R6
  因而只有在原场景实际响应、符号/重复性和 residual 检查后才升级到
  `MODEL_BASED_CONTROL_SOURCE_READY`。

## 可保留的旧门

R3/R4/R5 已经覆盖且 R6 原样保留：原始 fixture builder、FR3 hash、左右
articulation/DOF 顺序、link8/TCP FK、60 Hz 实际步、双臂同步、fresh copied
q/dq、原始 TGS/物理场景、FCL 意外碰撞、ROS graph 关闭、native finger/mimic
行为、effort mode、限值和速率门、双臂联合 pre-command gate、durable journal、
failure latch，以及约 0.92 path-fraction 的 direct-interpolation 碰撞负例。

## 必须替换的旧门

1. R4 的 `FROZEN_Q_NONZERO_DQ` 和整步差分相等性替换为 TGS 语义感知的
   `DIAGNOSTIC_ACTUATION_SOURCE_READY`；整步差分只作位置演化 estimator。
2. R5 的“没有独立激励证书就禁止任何 effort”循环门替换为：先通过 Level 1
   身份、fresh state、碰撞、ownership、finger-preserve、effort/rate gates，
   才允许预注册的很小诊断输入。Level 2 仍要求实际响应、模型 residual、
   手指耦合与独立 sign/repeat 检查。
3. R3 `ARM_SUBSYSTEM_SOURCE_VALIDATED` 证书不适合作为诊断输入的前置条件；
   新证书命名为 `DIAGNOSTIC_ACTUATION_SOURCE_READY`，明确不声称完整 ID。

## 实验边界

所有 cell 都是新 Isaac 进程和 anonymous stage。让原生位置 drive 先把两个
臂带到 FREE_HOME；之后只在运行时把七个 arm 的 Kp/Kd 置零
(`save_to_usd=False`)，保留两个 finger 的增益、模式和约束。A 使用注册的
gravity holding effort，B/C 为相反符号的小幅关节扰动，D 重复 B。每次写入
之前联合验证两臂并持久化完整状态与九维 command；任何第一次 rejection、
碰撞、状态失效或 buffer 不一致都会立刻停止该 cell。禁止 GPU 重试、ROS、
吸盘闭合、Cube 接触、D6、硬件和生产控制。

## 依据

* [PhysX Simulation](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/Simulation.html)
* [Isaac Sim 4.5 Core Prims API](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.prims/docs/index.html)
* [TSID project](https://github.com/stack-of-tasks/tsid)


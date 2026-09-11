# MuJoCo + Isaac Sim 双机械臂力控教学研究项目

> 状态：项目骨架 / 教学路线设计。当前先作为独立 scaffold 存放；后续迁移到单独 GitHub 仓库后再开始正式实现。

## 项目目标

本项目用于循序渐进学习并研究机械臂力控，最终目标是：

```text
MuJoCo 中快速验证控制算法
        ↓
控制器与仿真器接口解耦
        ↓
Isaac Sim / Isaac Lab 中复现
        ↓
双 FR3 共同物体搬运
        ↓
力 / 力矩分配、内力控制、QP
        ↓
最终可选：接回双机械臂码垛项目的紧协调模块
```

本项目与 `dual-arm-embodied-palletizing` 的职责不同：

- 码垛项目：任务规划、MoveIt、FCL、松/紧协调、随机多尺寸码垛；
- 本项目：动力学、关节力矩控制、阻抗/力控制、双臂协同受力、负载分配。

## 教学原则

1. 每个 Task 只引入一个主要新概念；
2. 先单关节，再单臂，再接触，再双臂；
3. 先会解释公式，再写控制器；
4. MuJoCo 用于算法开发，Isaac 用于跨仿真器验证；
5. 不把 ROS2 / MoveIt / FCL 提前塞进基础力控 Task；
6. 每个 Task 都必须有可测量验收指标，而不是只看动画。

## 推荐模型

优先使用 MuJoCo Menagerie 的 Franka FR3 7-DoF 模型。第一阶段不做吸盘真空流体，只研究机械臂动力学、末端 wrench 和共同物体约束。

## 总体 Task

| Task | 主题 | 目标 |
|---|---|---|
| 00 | 环境与 FR3 基线 | 能加载 FR3、读取状态、施加 torque |
| 01 | 力/力矩/wrench 基础 | 学懂力、力矩、Jacobian transpose |
| 02 | 单关节 torque control | 从最简单的一维动力学入门 |
| 03 | 7DoF joint-space control | PD+重力补偿 / computed torque |
| 04 | Jacobian 与末端 wrench | 验证 `tau = J^T F` |
| 05 | Cartesian impedance | 末端像弹簧阻尼器一样跟踪目标 |
| 06 | 接触力控制 | 让末端以指定法向力压住平面 |
| 07 | Hybrid position/force | 某些方向控位置，某些方向控力 |
| 08 | 双 FR3 共物体基线 | 两臂共同抓持并搬运同一刚体 |
| 09 | Wrench distribution + QP | 计算左右臂应该各出多少力 |
| 10 | Internal force | 控制不改变物体运动的双臂内力 |
| 11 | 鲁棒性实验 | 质量、摩擦、扰动、模型误差 |
| 12 | Isaac Adapter | 同一个 Controller Core 移植 Isaac |
| 13 | MuJoCo vs Isaac 对照 | 比较同控制器在两引擎中的表现 |
| 14 | 码垛项目集成（可选） | 将成熟力控接回紧协调大箱搬运 |

## 建议代码结构

```text
mujoco_isaac_force_control/
├─ controllers/
│  ├─ joint_pd.py
│  ├─ computed_torque.py
│  ├─ cartesian_impedance.py
│  ├─ force_controller.py
│  ├─ hybrid_controller.py
│  ├─ wrench_allocator.py
│  └─ internal_force_controller.py
├─ robot_model/
│  └─ robot_model_interface.py
├─ sim/
│  ├─ mujoco_adapter.py
│  └─ isaac_adapter.py
├─ models/
├─ experiments/
├─ tests/
└─ docs/tasks/
```

核心要求：控制器只依赖统一接口，不直接写 `mujoco.data` 或 Isaac API。

## 最终控制接口

```text
get_q()
get_qdot()
get_mass_matrix()
get_gravity()
get_coriolis()
get_jacobian()
get_tcp_pose()
get_tcp_velocity()
get_tcp_wrench()
set_joint_torque(tau)
```

因此：

```text
MuJoCoAdapter ─┐
               ├─> Controller Core -> tau
IsaacAdapter ──┘
```

## 当前边界

- 当前只建立教学与研究路线，不开始 DRL；
- 不在基础阶段研究视觉、路径规划和码垛点；
- 不把当前码垛项目整体迁移到 MuJoCo；
- 只有 Task12 以后才正式进入 Isaac 跨仿真器验证；
- Task14 为可选集成，不阻塞本项目独立完成。

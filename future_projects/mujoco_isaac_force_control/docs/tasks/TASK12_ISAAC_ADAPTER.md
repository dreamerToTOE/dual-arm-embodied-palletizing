# Task12：Isaac Sim / Isaac Lab Adapter 移植

## 学习目标

证明控制器核心不依赖 MuJoCo。此阶段不修改控制公式，只更换 simulator adapter。

目标架构：

```text
MuJoCoAdapter ─┐
               ├─ Controller Core -> tau
IsaacAdapter ──┘
```

## 统一接口

Isaac 侧实现与 MuJoCo 一致的：

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

## 移植顺序

1. FR3 joint ordering 对齐；
2. base/TCP frame 对齐；
3. torque sign/unit 对齐；
4. Jacobian 对齐；
5. mass matrix / gravity 对齐；
6. 先跑 Task03 joint controller；
7. 再跑 Task05 impedance；
8. 最后跑 Task08--10 双臂控制。

## 禁止事项

- 不为了 Isaac 单独复制一套控制器；
- 不在 controller 内写 PhysX API；
- 不因为结果不同就直接改公式，先检查模型、坐标系、频率与接触参数。

## 验收

同一个 Controller Core 在两边均可运行，只通过 Adapter 提供数据和输出 torque。

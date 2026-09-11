# Task02：单关节 Torque Control

## 学习目标

先把 7DoF 问题降成最简单的一维控制。固定其余关节或选一个简化测试姿态，仅控制一个关节。

## 核心模型

先从直观模型理解：

```text
I qdd + b qdot + g(q) ≈ tau
```

然后实现最基础 PD：

```text
tau = Kp (q_d - q) + Kd (qdot_d - qdot)
```

并比较加入重力补偿前后差异：

```text
tau = PD + g(q)
```

## 实验

- Step target；
- 小幅正弦 target；
- 多组 `Kp/Kd`；
- 有/无 gravity compensation。

## 观察指标

```text
position error
overshoot
settling time
peak torque
steady-state error
```

## 教学重点

- `Kp` 为什么像弹簧；
- `Kd` 为什么像阻尼；
- 增大 `Kp` 为什么不等于一定更好；
- 重力为什么会造成 steady-state error。

## 验收

- 至少 3 组增益曲线对比；
- 能解释欠阻尼/过阻尼趋势；
- torque 不超过人为设置的安全限制；
- 输出 `q, q_d, tau` 时序曲线。

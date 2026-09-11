# Task03：7DoF Joint-space Dynamics Control

## 学习目标

从单关节扩展到完整 FR3，理解机械臂动力学：

```text
M(q) qdd + C(q,qdot) qdot + g(q) = tau + tau_ext
```

第一版不要求手推 FR3 的 M/C/g，只需要知道每一项代表什么，并能从仿真器接口读取/计算。

## 控制器

依次完成：

1. 7DoF joint PD；
2. PD + gravity compensation；
3. computed-torque / inverse-dynamics tracking。

典型形式：

```text
tau = M(q) [qdd_d + Kd e_dot + Kp e] + C qdot + g
```

## 实验

- HOME -> 一个固定关节目标；
- 多关节正弦轨迹；
- 改变 payload mass 后重复实验。

## 验收

- 7 关节位置误差 RMS；
- 峰值 torque；
- 有/无动力学补偿对比；
- 质量变化后能观察并解释跟踪误差变化。

## 完成后应该能回答

> 为什么位置控制器最终也必须通过力矩作用于机械臂？动力学补偿到底补偿了什么？

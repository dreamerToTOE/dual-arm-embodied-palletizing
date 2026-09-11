# Task04：Jacobian、末端速度与 Wrench 映射

## 学习目标

把此前学过的 Jacobian 真正用于完整 FR3：

```text
xdot = J(q) qdot

tau_ext = J(q)^T W_ext
```

## 实验 A：速度映射验证

给一个小 `qdot`，用 Jacobian 预测末端 twist，再用仿真器实际末端位姿差分验证。

## 实验 B：wrench 映射验证

在多个 FR3 姿态下，对 TCP 施加相同方向的虚拟 wrench，比较 `J^T W` 的关节 torque 分布。

## 实验 C：接近奇异位形

观察 manipulability / Jacobian singular values 与所需关节运动、控制敏感性的变化。

## 验收

- `J` 数值维度与关节顺序确认；
- Jacobian 预测 twist 与数值差分误差可量化；
- `J^T W` 可正确生成 7DoF joint torque；
- 能说明奇异性为什么同时影响运动控制和力控制。

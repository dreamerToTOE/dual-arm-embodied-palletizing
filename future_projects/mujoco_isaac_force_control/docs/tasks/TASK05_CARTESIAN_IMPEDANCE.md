# Task05：Cartesian Impedance Control

## 学习目标

第一次从“控制关节”切换到“控制末端机械行为”。目标不是死死跟踪一个点，而是让 TCP 表现得像虚拟弹簧-阻尼器。

核心形式：

```text
W_cmd = Kx (x_d - x) + Dx (xdot_d - xdot)

tau = J^T W_cmd + gravity/dynamics compensation
```

## 教学重点

- 阻抗控制不是直接“控制力”；
- `Kx` 决定末端有多硬；
- `Dx` 决定阻尼；
- Cartesian stiffness 与 joint stiffness 的区别；
- orientation error 不能直接用欧拉角简单相减作为最终实现。

## 实验

1. TCP 在自由空间跟踪固定 pose；
2. 用低/中/高三组 stiffness；
3. 对 TCP 施加外部扰动，观察恢复；
4. 记录位置误差、速度和 torque。

## 验收

- 能稳定跟踪 TCP 目标；
- 不同 stiffness 下扰动后的偏移明显不同；
- 无持续振荡或数值爆炸；
- 可以解释“柔顺”和“刚性”的区别。

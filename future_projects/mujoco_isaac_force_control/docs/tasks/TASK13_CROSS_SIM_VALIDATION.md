# Task13：MuJoCo vs Isaac 跨仿真器验证

## 学习目标

不要求两个物理引擎逐点完全一致，而是验证：同一控制方法在两套动力学/接触求解器中是否仍保持稳定和相同趋势。

## 统一实验

固定：

```text
robot initial state
target trajectory / target force
controller gains
control rate
payload nominal mass
```

分别在 MuJoCo 与 Isaac 运行。

## 比较指标

```text
joint tracking RMS
TCP tracking RMS
force RMS / peak
joint torque RMS / peak
internal force error
settling time
stability / failure rate
```

## 需要允许的差异

- contact transient 不应期待完全一致；
- damping/friction/contact solver 参数可能不同；
- 控制增益可做少量平台重新整定，但必须记录。

## 教学重点

- 算法可移植 ≠ 数值完全相同；
- simulator-specific parameter 与 controller principle 的区别；
- 跨物理引擎一致趋势本身就是鲁棒性证据。

## 验收

至少完成：

1. joint-space control 对照；
2. Cartesian impedance 对照；
3. contact force 对照；
4. dual-arm shared-object / QP allocation 对照。

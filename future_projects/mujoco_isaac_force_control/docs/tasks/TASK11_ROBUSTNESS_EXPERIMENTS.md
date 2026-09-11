# Task11：力控鲁棒性与扰动实验

## 学习目标

控制器在“标称模型”里稳定并不代表有工程价值。本阶段系统测试模型误差、外力和参数变化。

## 变量

逐项改变：

```text
payload mass
joint damping
contact stiffness
friction
control rate
sensor noise
external disturbance
```

## 对比控制器

至少比较：

```text
joint PD
Cartesian impedance
hybrid position/force
dual-arm QP allocation
```

## 指标

```text
tracking RMS
force RMS / peak
maximum joint torque
settling time
internal force error
success/failure
```

## 教学重点

- nominal model 与 true model；
- 为什么 inverse dynamics 对模型误差敏感；
- 为什么柔顺控制往往比刚性高增益更适合接触任务；
- 为什么只展示一条成功曲线不能说明鲁棒性。

## 验收

形成统一实验脚本，能一次运行多组参数并输出 CSV/图表。结论必须来自批量结果，而不是单次动画。

# Task09：双臂 Wrench Distribution 与 QP 入门

## 学习目标

第一次正式回答：

> 为了让 SharedBox 产生期望的合力/合力矩，左右机械臂各应该承担多少 wrench？

## 核心模型

定义物体期望 wrench：

```text
W_obj_des = [Fx,Fy,Fz,Mx,My,Mz]
```

左右接触 wrench：

```text
f = [W_L; W_R]
```

抓取映射：

```text
G f = W_obj
```

第一版先理解“多组左右力都可能产生同一个物体合力”。

## 最小二乘基线

```text
min ||f||^2
s.t. G f = W_obj_des
```

再升级成 QP：

```text
min  ||Gf-W_des||_Q^2 + lambda ||f||^2
s.t. wrench limits
     joint torque limits
```

其中关节限制通过：

```text
tau_L = J_L^T W_L
tau_R = J_R^T W_R
```

## 教学重点

- 为什么不能简单左右各分 50%；
- grasp geometry 为什么影响力矩分配；
- QP 的变量、目标函数、等式/不等式约束分别是什么；
- “满足物体运动”与“让两个机器人都不过载”之间的关系。

## 验收

- 静止承重情况下左右 wrench 分配可解释；
- 改变 grasp 间距后分配变化合理；
- 加入单臂 torque limit 后 QP 会主动调整负载；
- QP infeasible 时能明确报告而不是输出异常 torque。

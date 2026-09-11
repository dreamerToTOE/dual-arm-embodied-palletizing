# Task07：Hybrid Position / Force Control

## 学习目标

把 Task05 的位置/阻抗控制和 Task06 的接触力控制组合起来。

典型任务：TCP 压住水平面并沿 X 方向移动。

```text
X/Y/姿态 -> position / impedance
Z        -> force control
```

## 核心概念

用选择矩阵理解哪些方向控位置、哪些方向控力：

```text
S_f : force-controlled axes
S_x : motion-controlled axes
```

概念上：

```text
W_cmd = S_x W_motion + S_f W_force
```

## 实验

1. TCP 建立 10 N 接触；
2. 保持 Fz 的同时沿 X 走直线；
3. 改变平面高度/小坡度；
4. 比较纯位置控制与 hybrid control 的接触力峰值。

## 验收

- X 方向轨迹误差；
- Z 向力 RMS/peak error；
- 接触过程中无明显弹跳；
- 能解释为什么这类控制适合装配、打磨和双臂共同物体接触任务。

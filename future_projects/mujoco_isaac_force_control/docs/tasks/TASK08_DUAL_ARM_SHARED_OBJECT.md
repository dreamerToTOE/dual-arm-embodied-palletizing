# Task08：双 FR3 共同物体基线

## 学习目标

从单臂力控进入双臂，但暂时不做 QP 力分配。目标只是建立一个清楚、可测量的共同物体控制基线。

场景：

```text
Left FR3  ---- SharedBox ----  Right FR3
```

两臂从物体两侧/顶部两个固定 grasp frame 共同约束同一个刚体。

## 第一版控制策略

先用两个单臂 Cartesian impedance controller 跟踪由同一个 object reference 推导出的左右 TCP 目标：

```text
T_world_object_des
      ↓
T_object_left_grasp / T_object_right_grasp
      ↓
T_world_left_tcp_des / T_world_right_tcp_des
```

## 必须记录

- SharedBox pose error；
- 左右 TCP 相对位姿误差；
- 左右 wrench；
- 左右 joint torque。

## 教学重点

- “两臂都跟踪自己的目标”不等于真正解决了双臂受力协调；
- 为什么两个控制器很容易互相顶；
- 为什么物体运动和内部受力是两个不同问题。

## 验收

- 两臂能稳定抬起并平移 SharedBox；
- 相对 grasp geometry 保持；
- 左右 wrench 不出现无界增长；
- 为 Task09/10 建立 baseline 数据。

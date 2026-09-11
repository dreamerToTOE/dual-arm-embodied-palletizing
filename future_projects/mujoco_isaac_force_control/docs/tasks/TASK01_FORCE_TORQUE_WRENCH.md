# Task01：力、力矩、Wrench 与 Jacobian 入门

## 学习目标

这一阶段以“看懂量的含义”为主，不追求复杂控制。

需要掌握：

```text
Force F       [N]
Torque M      [N·m]
Wrench W      [Fx,Fy,Fz,Mx,My,Mz]
Joint torque  tau
Jacobian J
```

核心关系：

```text
xdot = J(q) qdot

tau = J(q)^T W
```

第一条描述关节速度如何产生末端速度；第二条描述末端 wrench 如何映射成关节广义力。

## 教学实验

1. 在 FR3 某一姿态读取 Jacobian；
2. 人工定义一个末端 `+Z` 方向小力；
3. 计算 `tau = J^T W`；
4. 将该 torque 短时施加到机器人；
5. 观察末端运动趋势是否与预期一致；
6. 更换机器人姿态，比较同一个末端力对应的 joint torque 如何变化。

## 必须理解

- 同一个末端力，在不同姿态下为什么需要不同关节力矩；
- `J^-1` 和 `J^T` 不是同一个用途；
- wrench 必须说明在哪个坐标系表达。

## 验收

- 能打印并解释 `J ∈ R^(6×7)`；
- 能手算/代码算出 `J^T W`；
- 明确 world/base/TCP frame 的区别；
- 能解释为什么 FR3 7DoF 下 `J` 不是普通可逆方阵。

## 完成后应该能回答

> 末端向下压 10 N，为什么不能简单理解为“每个关节平均分一点力矩”？

# Task00：MuJoCo 环境与 FR3 力矩控制基线

## 学习目标

这一阶段不研究控制算法，只回答四个问题：

1. FR3 模型能否稳定加载；
2. `q`、`qdot` 分别是什么；
3. 如何读取 7 个关节状态；
4. 如何向关节发送 torque / effort。

## 必须先理解

- `q`：关节位置；
- `qdot`：关节速度；
- `tau`：关节广义力/力矩；
- simulation timestep；
- actuator 与 joint 的区别。

## 实现

建立最小 MuJoCo 程序：

```text
load FR3
→ reset
→ read q / qdot
→ apply zero torque
→ simulation step
→ log state
```

随后只对一个关节施加很小的测试 torque，确认符号、关节索引和单位。

## 禁止事项

- 不做双臂；
- 不做 Cartesian 控制；
- 不加入 ROS2 / MoveIt；
- 不靠肉眼判断成功。

## 验收

- 7 个关节名称与索引明确；
- `q/qdot/tau` 单位明确；
- zero torque / gravity 下行为能解释；
- 指定单关节 torque 后状态变化方向正确；
- 输出 CSV 日志和至少一张 `q(t)` 曲线。

## 完成后应该能回答

> 为什么“给关节一个位置目标”和“给关节一个力矩”是两种完全不同的控制接口？

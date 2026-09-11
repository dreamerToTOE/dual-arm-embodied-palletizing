# Task10：Internal Force / 内力控制

## 学习目标

理解双臂共同抓取中最容易混淆的概念：

> 有些左右接触力会很大，但它们互相抵消，不改变 SharedBox 的整体运动。

这部分就是 internal force。

## 核心关系

若：

```text
G f_internal = 0
```

则 `f_internal` 位于抓取映射的零空间，不改变物体净 wrench。

概念分解：

```text
f = f_motion + f_internal
```

其中：

- `f_motion`：负责物体运动/抗重力；
- `f_internal`：负责抓取夹持、接触稳定等，但不能过大。

## 实验

1. SharedBox 保持静止；
2. 保持相同物体净 wrench；
3. 人为改变 internal force reference；
4. 比较左右接触 wrench 与 SharedBox pose 是否变化；
5. 测试过大 internal force 带来的 joint torque / 接触风险。

## 控制目标

第一版只做简单内力反馈或 QP 中的目标项：

```text
min ... + w_int ||f_internal - f_internal_des||^2
```

## 验收

- 改变 internal force 时 SharedBox 整体位姿基本不变；
- 左右接触力按照预期变化；
- 关节 torque 不越界；
- 能解释“为什么双臂互相顶得很用力，但箱子可能完全不动”。

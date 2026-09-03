# Task 04 — 顶部吸盘单臂基线

## 状态

🟡 **进行中 — 将现有单 FR3 二指抓放替换为固定顶部吸盘抓放。**

## 1. 目标

当前阶段只解决一个问题：

> 让单台 FR3 使用紧凑顶部吸盘稳定完成 Pick & Place，为后续双臂统一吸盘系统建立基线。

暂不做双臂、不做紧协调控制、不提前绑定论文结构。

## 2. 设计原则

后续左右 FR3 使用同一种固定顶部吸盘。

```text
小箱体：单臂顶部单吸附
大箱体：双臂在同一箱体上表面两个分离吸附点共同吊运
```

吸盘工具必须是紧凑顶部工具，不能只在原 Franka 二指夹爪中间增加“吸附开关”而继续保留大侧向 finger 碰撞包络。

第一版吸盘模型：

```text
紧凑刚性工具几何
+
吸附 ON/OFF
+
Isaac 物理吸附约束
+
MoveIt AttachedCollisionObject
```

不模拟真空流体细节。

## 3. 第一版执行链

```text
HOME
→ PRE_GRASP
→ 顶部垂直接近
→ SUCTION ON
→ LIFT
→ TRANSFER
→ PLACE
→ SUCTION OFF
→ RETREAT
→ HOME
```

## 4. 第一轮验收标准

```text
1. Cube 能稳定吸起；
2. 搬运过程中不脱落；
3. 放置过程中工具不需要进入 Cube 左右两侧；
4. 释放后 Cube 留在目标位置；
5. MoveIt 的 attached object 与 Isaac 的吸附状态一致；
6. 完整流程可重复执行。
```

## 5. 与 Task 05 的衔接

Task 04 成功后，直接回到 Task 03 的 B-A-C 高密度场景：

```text
Task 05 — 吸盘高密度放置验证
```

目标是验证同样的紧邻箱体布局下，顶部吸盘可以完成：

```text
PRE_PLACE
→ 垂直下降
→ PLACE
→ RELEASE
→ 垂直退出
```

从而确认固定顶部吸盘解决二指夹爪的侧向插入空间问题。

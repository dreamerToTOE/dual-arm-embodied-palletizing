# Task21：Automatic Loose / Tight Coordination Router

状态：⚪ 待实现。

## 目标

让系统根据 Box 和任务属性自动决定使用松协调还是紧协调，而不是由场景脚本预先指定“大件=tight，小件=loose”。

输入：

```text
BoxSpec
pick pose / grasp candidates
PlacementSpec
robot reachability
payload limits
```

输出：

```text
LOOSE_LEFT
LOOSE_RIGHT
LOOSE_DUAL_PARALLEL
TIGHT_SHARED_OBJECT
NO_FEASIBLE_MODE
```

## 第一版路由规则

先使用可解释规则，不先使用神经网络：

```text
1. 单臂负载是否允许
2. 单臂是否可达 pick + place
3. 顶部单吸盘稳定性是否允许
4. 两臂共同抓取点是否可行
5. 与其他并行任务的冲突 / makespan 代价
```

示意：

```text
single-arm feasible && payload safe
    -> LOOSE candidate

single-arm infeasible && dual grasp feasible
    -> TIGHT candidate

两者都可行
    -> 比较预计 makespan / planning cost / robustness
```

## 与 Task16/19 联动

模式选择不能只看箱体尺寸。

例如同一个中等箱体：

- 左臂当前不可达但右臂可达 -> 单臂 loose；
- 单臂抓取稳定性不足 -> tight；
- 两臂 tight 会严重阻塞另外任务 -> loose 可能更优。

因此 Router 应允许调用快速 reachability / candidate score，而不是简单阈值 `size > X`。

## 验收标准

```text
T21-01  模式不再由 Task15 Phase A/B 硬编码
T21-02  至少基于尺寸/质量/单臂可达性给出模式
T21-03  两种模式均不可行时明确拒绝
T21-04  Router 输出理由与评分，便于调试
T21-05  ≥6 类尺寸随机任务中自动路由
T21-06  最终候选仍分别走现有 loose / tight 安全门禁
```

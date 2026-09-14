# Task20-D：运行时多对象连续调度物理验收

状态：🟡 已实现并通过编译；等待 Isaac 实际验收。

## 目标

Task20-C 已真实通过一件大件的紧协调搬运。本 Task20-D 将验收扩展为同一 episode 内
连续完成三件物体：

```text
task18_box_01 large_shared_box  -> 紧协调共同搬运
task18_box_02 tall_box           -> 松协调单臂搬运
task18_box_03 tall_box           -> 松协调单臂搬运
```

固定 replay seed 为 `20260922`，物体按体积从大到小选择。首件大件的目标为窄托盘的
table 基层；后两件不能再合法放到该基层，必须由 Task19 PlacementPlanner 在大件顶面
生成完整支撑候选。此验收不声称五件全完成，也不把松协调任务改成并行；它专门验证
“真实放置后继续调度”的闭环。

## 连续闭环

```text
获取 /task18/box_states
    ↓
初始化全部 runtime CollisionObject 到 MoveIt World（一次）
    ↓
第 i 件：读取该件最新 Isaac source pose
    ↓
PlacementPlanner(placed 的真实 settle pose)
    ↓
每个几何候选最多 3 次：MoveIt candidate + Router + FCL
    ↓
执行 TIGHT 或 LOOSE primitive
    ↓
SUCTION OFF -> PhysX settle -> Ground Truth -> MoveIt World writeback
    ↓
/task20/lock_placed_object（保留 Collider 的 kinematic 稳定支撑）
    ↓
把实际 settle pose 加入 placed，下一件重新规划
```

不共享前一件的轨迹、PlacementSpec 或 FCL 结论。任一件失败时立即停止，且不会对其后
物体继续规划或发布命令。该设计不会扩大 ACM，也不会忽略已放物体碰撞。

## 自动回退范围

每件物体：

- 读取实时 source Ground Truth；规划中位置变化超过 `3 mm` 则拒绝旧轨迹；
- 在最多 8 个已通过边界、无重叠、完整支撑门禁的 placement candidates 中顺序尝试；
- 每个 candidate 最多重新生成完整 route 3 次；每次均重新运行 Router 与 FCL；
- 只有上述全部通过，才执行机器人。

这不是对失败路径的盲目重复：先改变已安全的放置候选，再才对同一候选重采样。

## 运行前置条件

- 已 build `fr3_dual_palletize`；
- MoveIt Task06 双臂环境运行；
- 新 Isaac 会话中运行 Task20 acceptance scene；
- Timeline 处于 Play；
- Task18 Ground Truth bridge 和 Task20 dual-suction bridge 均在运行。

详细的 Isaac 与 ROS 命令将在本 Task 物理验收时使用，且两个 bridge 都是必需项。

## 通过判据

1. 每件打印 `PLAN READY`，并包含 route、support 与目标位姿；
2. 第 1 件为 `TIGHT_SHARED_OBJECT`，第 2/3 件为 `LOOSE_LEFT` 或 `LOOSE_RIGHT`；
3. 每件真实释放后打印 `WORLD COMMITTED`，下一件随后重新打印 `PLAN READY`；
4. 最终同时出现：

```text
Task20-D CONTINUOUS EXECUTION PASS: completed=3/3 tight=1 loose=2
Task20 runtime EXECUTION PASS
```

若输出 `CONTINUOUS EXECUTION STOPPED`，该次为未通过；记录日志，不应通过关闭碰撞、
扩大吸附阈值或手动移动物体来绕过。

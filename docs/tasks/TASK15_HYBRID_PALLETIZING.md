# Task15 — 紧/松协调混合码垛场景与复用策略

状态：🟡 场景与项目级 skill 已就绪；控制器与 Isaac 执行验收尚未开始。

## 目标

在同一张桌面上验证混合尺寸箱体的协调路由：先用两台 FR3 顶部吸盘对同一个大件做紧协调搬运，再用两臂各自搬运独立小箱体，按两层放到大件顶面。

Task15 的新增内容只包括：可重复加载的物理场景、明确的任务分批和两个项目级复用契约。它不复制 Task04、Task10 或 Task11--Task13 的控制逻辑，也不提前增加新的 Bridge 或控制节点。

## 项目级复用 skill

这两个 skill 是仓库内随 `fr3_dual_palletize` 安装的 YAML 契约，不是 Codex 本地 skill：

- [松协调独立物体码垛](../../ros_ws/src/fr3_dual_palletize/skills/loose_coordination_palletizing.yaml)
- [紧协调共同物体搬运](../../ros_ws/src/fr3_dual_palletize/skills/tight_coordination_shared_object.yaml)

它们将已经实际验收的工程路径固定下来：

```text
大件（同一物体）
  -> tight_coordination_shared_object
  -> 复用 Task11--13：双吸附、共同抬升、共同运输、共同放置、同步释放、GT 回写、共同退出

小件（两个独立物体）
  -> loose_coordination_palletizing
  -> 复用 Task04 / Task07--10：独立 primitive、Task08 FCL、Task09 等待策略、Task10 连续 World 回写
```

选择 tight / loose 是由“物体是否共同承载”决定，不以关闭碰撞或临时扩大 ACM 规避协调问题。

## 场景

脚本：[task15_hybrid_palletizing_scene.py](../../isaac/scripts/task15_hybrid_palletizing_scene.py)。它要求已有 Task06 Stage 中的 `/World/left_fr3`、`/World/right_fr3` 和 `/World/Table`；不会创建或复制桌面、机器人、吸盘和 ActionGraph。

每次运行会仅清理旧的 Task15、Task10/11 任务物体及 `/World/Cube1`--`/World/Cube9`，因此可重复重置任务对象。

### 大件：紧协调阶段

大件 Prim 为 `/World/Task15LargeCube`。为严格复用 Task11--Task13 已经通过的共同搬运几何，它使用同一非均匀立方体尺寸；这里的 “LargeCube” 是任务语义名称。

| 属性 | 值 |
|---|---|
| 尺寸 | `0.220 × 0.320 × 0.080 m` |
| 质量 | `1.00 kg` |
| 初始中心 | `(0.550, 0.000, 0.090) m` |
| 紧协调目标中心 | `(0.650, 0.000, 0.090) m` |
| 目标顶面 | `z = 0.130 m` |
| 物理 | Dynamic Rigid Body + Collider |

目标刚好沿 `+X` 共同运输 `0.100 m`，与 Task12-B 已通过的运输位移一致。

### 小件：松协调两层码垛

四个小件均为 `0.030 m` 立方体、`0.20 kg`、Dynamic Rigid Body + Collider。初始中心高度均为 `z=0.065 m`。左臂使用负 `y` 一侧，右臂使用正 `y` 一侧，沿用 Task10 的工作区约定。

| 批次 | Prim | 负责臂 | 初始中心 (m) | 目标中心 (m) |
|---|---|---|---|---|
| 下层 | `/World/Task15SmallCube1` | left | `(0.320, -0.250, 0.065)` | `(0.650, -0.080, 0.145)` |
| 下层 | `/World/Task15SmallCube2` | right | `(0.320, +0.250, 0.065)` | `(0.650, +0.080, 0.145)` |
| 上层 | `/World/Task15SmallCube3` | left | `(0.380, -0.250, 0.065)` | `(0.650, -0.080, 0.175)` |
| 上层 | `/World/Task15SmallCube4` | right | `(0.380, +0.250, 0.065)` | `(0.650, +0.080, 0.175)` |

高度关系：大件顶面为 `0.130 m`；下层小件中心为 `0.130 + 0.015 = 0.145 m`；上层小件中心为 `0.145 + 0.030 = 0.175 m`。这使上层恰好落在对应下层小件顶面。

## Task15 计划执行策略

```text
Phase A — 紧协调（一个共同大件）
  Task11--13 skill
  DUAL PRE_CONTACT -> CONTACT -> BOTH CLOSED
  -> COMMON LIFT -> COMMON +X 0.100 m TRANSPORT -> COMMON PLACE
  -> 释放前预规划共同 RETREAT -> BOTH OPEN -> settle
  -> Isaac Ground Truth 回写 MoveIt World -> common RETREAT

Phase B1 — 松协调（下层两个独立小件）
  left:  SmallCube1 -> (-y 下层目标)
  right: SmallCube2 -> (+y 下层目标)
  -> 分别生成完整 TaskTrajectoryCandidate
  -> Task08 FCL 联合检查
  -> SAFE: 共享时钟并行；CONFLICT: Task09 最小局部等待 / 延迟策略
  -> 两件均 settle，并以 Isaac Ground Truth 加入 MoveIt World

Phase B2 — 松协调（上层两个独立小件）
  left:  SmallCube3 -> SmallCube1 顶面
  right: SmallCube4 -> SmallCube2 顶面
  -> 完整候选 + FCL + 必要的 Task09 调度
  -> 并行执行、释放、settle、Ground Truth 回写
```

因此“大件”与“小件”不会同时调度：紧协调完成并把大件回写为 World 障碍物后，才开始松协调。两层小件也不跨层并行，避免上层规划忽略尚未落稳的支撑物；但同一层左右两个独立小件会优先真正并行。

## 当前验收边界

- T15-01：Task15 场景不修改 Task06 的 FR3、吸盘、Table 或 ActionGraph。
- T15-02：大件为可碰撞、可动力学搬运的 `1.00 kg` 物体，且复用 Task11--Task13 几何与 `+X 0.100 m` 运输。
- T15-03：四个小件均为独立的 Dynamic Rigid Body + Collider。
- T15-04：四个目标高度满足两层支撑几何。
- T15-05：策略明确调用既有 tight / loose skill；尚未开始 Task15 Bridge、MoveIt 控制器或执行验收。

## 加载场景

在 Isaac Sim 4.5 打开 Task06 双 FR3 Stage、确认 Timeline 为 Stop 后，在 Script Editor 执行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task15-hybrid-palletizing/isaac/scripts/task15_hybrid_palletizing_scene.py").read())
```

预期 Console 末尾显示：

```text
Task15 hybrid palletizing scene ready
Strategy Phase A / TIGHT: ...
Strategy Phase B1 / LOOSE: ...
Strategy Phase B2 / LOOSE: ...
```

此阶段不要启动 Task11 或 Task10 的旧 Bridge 来执行 Task15：二者的对象命名、话题和任务状态机不同。下一步才是基于两个项目级 skill 建立 Task15 专用 Bridge 与协调控制器。

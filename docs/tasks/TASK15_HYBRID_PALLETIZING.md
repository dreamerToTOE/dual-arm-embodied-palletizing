# Task15 — 紧/松协调混合码垛场景与复用策略

状态：🟡 场景、Task15 专用 Bridge 与两个复用控制器已实现；尚未进行 Isaac 实机执行验收。

## 目标

在同一张桌面上验证混合尺寸箱体的协调路由：先用两台 FR3 顶部吸盘对同一个大件做紧协调搬运，再用两臂各自搬运独立小箱体，按两层放到大件顶面。

Task15 新增可重复加载的物理场景、明确的任务分批、两个项目级复用契约、专用 Bridge 与配置驱动控制器。它不复制 Task04、Task10 或 Task11--Task13 的控制逻辑。

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

脚本：[task15_hybrid_palletizing_scene.py](../../isaac/scripts/task15_hybrid_palletizing_scene.py) 是自包含入口。在空白 Isaac Stage 中运行时，它会自动调用已验收的 `task06_dual_fr3_scene.py` 和 `task06_dual_ros_graph.py`，创建 `/World/left_fr3`、`/World/right_fr3`、`/World/Table`、双 compact suction、`/World/Task06ROSGraph` 与 `/physicsScene`；若这些基线已经完整存在则直接复用，不重复创建。Task15 本身只新增/重置任务物体。

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
- T15-05：Task15 Bridge、紧协调复用目标和配置驱动松协调控制器均已构建；尚未进行 Isaac + MoveIt 物理执行验收。

## 加载场景

在 Isaac Sim 4.5 的新 Stage 或已有 Task06 Stage 中，确认 Timeline 为 **Stop** 后，在 Script Editor 执行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task15_hybrid_palletizing_scene.py").read())
```

预期 Console 末尾显示：

```text
Task15 hybrid palletizing scene ready
Task15 基线就绪：Timeline Play 后运行 task15_hybrid_suction_bridge.py。
Strategy Phase A / TIGHT: ...
Strategy Phase B1 / LOOSE: ...
Strategy Phase B2 / LOOSE: ...
```

此阶段不要启动 Task11 或 Task10 的旧 Bridge 来执行 Task15：二者的对象命名、话题和任务状态机不同。

## 控制器实现

- Phase A：`task15_tight_large_cube` 是 `task11_shared_box_grasp.cpp` 的独立编译目标，带有 `TASK15_TIGHT_PHASE` 宏。它直接复用 Task11--13 已验收的双吸附、共同抬升、`+X 0.100 m` 运输、共同放置、释放前 RETREAT 规划和 Ground Truth 回写逻辑；宏只切换为 `/task15/*` 话题与 `task15_large_cube` Planning Scene object id。
- Phase B：`task15_loose_layer_stack` 的两个 `BatchConfig` 包含四条 `ArmAssignment`。每批左右各一条完整 `TaskTrajectoryCandidate`，经过 Task08 FCL、Task09 LocalWait 和共享时钟执行器。它不是四个独立状态机。
- `PrimitiveConfig::target_support_surface_z` 的默认值为 `0.050 m`，所以旧 Task04--Task10 不变。Task15 下层在 LargeCube 最新顶面释放；上层在对应下层小 Cube 最新 Ground Truth 顶面释放。释放前保持 `1 mm` gap，PhysX 落稳后的几何中心对应表中的 `0.145 / 0.175 m`。
- Bridge：`isaac/scripts/task15_hybrid_suction_bridge.py` 发布 `/task15/large_cube_pose` 与按 `[SmallCube1, SmallCube2, SmallCube3, SmallCube4]` 排列的 `/task15/small_cube_poses`，并复用已验收的 `.003 m` 阈值、`1e6` force/torque limit 与 retry-close 参数。

## 单入口与失败重规划

Task15 的紧协调和松协调不是两个需要人工衔接的命令。使用下面的总 launch 后，Phase A 成功退出才会自动启动 Phase B；Phase A 任意失败会以非零退出码停止 launch，绝不会在大件未放稳时启动小件码垛：

```bash
ros2 launch fr3_dual_palletize task15_hybrid_palletizing.launch.py execute:=true
```

`execute:=false` 可作完整无动作预检：它顺序规划紧协调和松协调、检查 FCL 与 LocalWait 调度，但不发布 joint / suction 命令。预检中 Phase B 使用 Phase A 的名义终点建立私有 Planning Scene；实际执行时则严格使用 Phase A 完成后的 Isaac Ground Truth。

所有会受采样随机性影响的 MoveIt Pose / Joint / Cartesian 规划阶段均采用同一规则：一次 MoveIt 调用只做一次内部尝试，控制器在外层最多重新规划 **3 次**，日志会输出 `attempt=1/3` 至 `attempt=3/3`。第三次仍失败会输出明确错误并停止当前任务；不会带着不完整轨迹继续执行。紧协调复用的 Task11--Task13 阶段和松协调复用的 `PalletizePrimitive` 阶段均覆盖此规则。

Phase B 的实际执行仍会检查大 Cube 已在紧协调目标 `(0.650, 0.000, 0.090)` 附近，防止绕过 Phase A 直接进行上层码垛。

每次 Phase A 启动还会清理仅属于本任务的 `task15_large_cube` 与
`task15_small_cube_1`--`task15_small_cube_4` Planning Scene 对象。这避免上一轮
预检或中途失败遗留的小件目标碰撞物阻挡新一轮大件共同运输；不会删除桌面、机器人或其他任务对象。

## 当前本地验证

已完成静态和构建验证，尚未进行物理执行验收：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install
source install/setup.bash
ros2 pkg executables fr3_dual_palletize | grep task15
ros2 launch fr3_dual_palletize task15_hybrid_palletizing.launch.py --show-args
python3 -m py_compile \
  /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task15_hybrid_suction_bridge.py
```

构建结果：`task15_tight_large_cube` 与 `task15_loose_layer_stack` 均已安装；Bridge 与场景 Python 语法检查通过。下一次验收应先以 `execute:=false` 分别验证两个 Phase 的规划/FCL，再在已重置 Isaac 场景中执行 `execute:=true`。

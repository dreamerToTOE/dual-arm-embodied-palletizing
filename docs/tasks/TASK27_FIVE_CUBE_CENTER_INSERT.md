# Task27：第一层五 Cube 的两段式紧协调装箱

状态：✅ **2026-09-29：`606 mm` 内宽、实测通道动态居中版在 Isaac Sim 4.5 + MoveIt2 中单进程连续完成五件实体码垛，exit 0；中心件两侧实测间隙 `1.569 / 1.310 mm`。** 这是该版单轮实体成功，不代表重复运行成功率；历史和失败复测保留在下文。

## 目标

在 Task26 已验证的三面围墙、侧面 L 型阵列吸盘与双臂紧协调工艺上，完成车厢最深 `X` 层的五件横向装箱：四件先建立左右两侧的稳定支撑，最后一件从装料口插入中间余量。

```text
+Y 墙
  Cube +2  <- 先贴 +Y 墙
  Cube +1  <- 再贴住 Cube +2
  Cube  0  <- 最后沿 +X 插入中心余量
  Cube -1  <- 再贴住 Cube -2
  Cube -2  <- 先贴 -Y 墙
-Y 墙
```

所有 Cube 均处在同一最深 X 层；不是逐层向上码垛，也不是现有 Task26 的深/浅两排 X 向布局。

## 几何约束

Task27 单独使用 Y 内腔 `[-0.303, +0.303] m`、Cube 宽度 `0.120 m`；Task26 已验收的 `628 mm` 车厢保持原样：

- 内腔总宽度为 `0.606 m`；五件总宽度为 `0.600 m`；
- 因而五件之间与两侧墙合计仍有 `6 mm` 余量，不可声称零间隙；
- 内侧件直推时与外件预留 `1.5 mm` 角点扫掠余量，到深墙后保持 -X 面吸附再朝外件短侧压 `1.0 mm`，名义邻件间隙降至 `0.5 mm`；
- 若两件外件贴墙、中央件位于 `y=0`，压紧后中央两侧名义余隙各约 `2.5 mm`。实体结果须按 Isaac Ground Truth 复核。

这说明，在不缩小车厢或压缩 Cube 的前提下，中心 Cube 不可能同时与左右邻 Cube 零间隙接触。Task27 的正确目标是让该余量**对称、可预测且尽可能小**，而不是通过扩大碰撞豁免或强行穿透来伪造零缝。

当前方案的名义 Y 中心为（内侧件实体执行时还会跟随外侧件的实际落点微调）：

| 位置 | 名义 Y (m) | 支撑关系 |
|---|---:|---|
| `+2` | `+0.243` | `+Y` 围墙 |
| `+1` | `+0.1215` | 直推时与 `+2` 留 `1.5 mm`，深墙处侧压目标 `0.5 mm` |
| `0` | `0.000` | 两侧目标各约 `2.5 mm` 插入余量 |
| `-1` | `-0.1215` | 直推时与 `-2` 留 `1.5 mm`，深墙处侧压目标 `0.5 mm` |
| `-2` | `-0.243` | `-Y` 围墙 |

最终数值以 Isaac Collider 尺寸和物理 Ground Truth 复核为准。

## 阶段 A：四件边侧基准 Cube

执行顺序必须左右交替，以降低连续占用同侧通道的概率：

```text
+2 -> -2 -> +1 -> -1
```

外侧两件复用 Task26 已通过的侧压链路；内侧两件先对齐已放置外侧件的 Isaac Ground Truth Y 坐标，再沿 +X 直推。旧的另一臂低位 `SIDE_CONTACT` 仅完成约 65% Cartesian 路径，因此改由仍吸住 -X 面的推入臂在深墙处做 1 mm 短侧压：

```text
双臂紧协调侧吸
  -> 共同接触 / 抬升
  -> X/Y 直角共同搬运至预推位
  -> 下降并同步释放
  -> 推入臂重抓 Cube -X 面
  -> +X 分段推入至车厢深端墙
  -> 外侧件：另一臂侧压到对应围墙，推入臂作为反向背挡
  -> 内侧件：预先对齐实际外侧件，+X 直推后沿 Y 短压，逐段检查邻件间隙与关节扭矩
  -> 紧协调放下后双臂同步退出；格位完成后安全预检下一件预吸位，能直达则不回 HOME
```

阶段 A 不引入新选择器或在线调度；复用 Task26 原有分段推入及力矩监督，目标格位固定，内侧通道的 Y 坐标以已放置外侧件的实际位置微调。

## 阶段 B：中心 Cube 插入

中心 Cube 作为独立实验，不与阶段 A 的成功与否混淆：

```text
双臂紧协调搬运至中央预推位
  -> 释放
  -> 单侧推入臂重抓 -X 面
  -> 严格 +X 分段推入至最深层
  -> 不做向 ±Y 的压紧
  -> 读取 Ground Truth，计算左、右间隙及中心偏置
  -> 安全退出
```

中心 Cube 的验收应包括：

1. 不碰撞、不扰动四件已落稳边侧 Cube；
2. X 向与深端墙的接触/间隙符合 Task26 的 `3 mm` 门限；
3. 左右间隙总和接近当前 `606 mm` 内宽、内件短侧压后的名义值 `5 mm`，且中心偏置受限；
4. 两侧间隙尽量均衡。旧 `628 mm` 内宽历史实测为 `12.314 mm` / `14.151 mm`，不作为新几何验收结果；
5. 最后一件完成后双臂回共同 HOME；前四件优先安全直达下一件预吸位，FCL 拒绝时回退共同 HOME。

## 不在本任务内

- 不做运行时 Cube 选择器；
- 不做多层堆垛；
- 不新增 force / impedance control 算法，沿用 Task26 已有的推入监督；
- 不修改 Task26 已验证的碰撞门限或 Allowed Collision Matrix；
- 不把中心插入失败伪装成“允许碰撞”。

## 已实现工程边界（2026-09-24）

- `isaac/scripts/task27_five_cube_center_insert_scene.py`：Task27 场景入口。它复用
  Task26 已验收的 FR3、L 型阵列侧吸、桌面、围墙、关节量程对齐和 ROS 图，但建立
  独立 `/World/Task27` 根、五件 Cube 和五个逐件到料批次；启动时会清理旧的
  `/World/Task26` / `/World/Task27`，避免旧刚体或旧 ROS 图混入。
- `isaac/scripts/task27_five_cube_center_insert_bridge.py`：复用真实 Surface Gripper、
  原子双臂关节下发、Ground Truth 与滑轨互锁，但使用独立 `/task27/*` ROS namespace。
- `task27_five_cube_center_insert`：独立可执行节点。其编译单元复用 Task26 的
  Cartesian 贴线检查、同步 FCL、三批重抓候选筛选、分段推入/力矩监督和物理退出，
  不改变 `task26_truck_box_push_in` 的默认任务布局。
- 外侧 Cube 验收贴 `±Y` 围墙；内侧 Cube 的间隙以外侧 Cube 的 **Isaac Ground Truth**
  面为准；中心 Cube 以两个内侧 Cube 的真实面为准，验收深端墙贴合、总余量、左右
  间隙平衡与中心偏置。
- 支持两段独立进程执行：中心阶段开始时会先验证前四件的 Ground Truth，并重新写入
  新进程的 MoveIt Planning Scene，不能假定上一次进程留下的 CollisionObject 仍存在。

## 构建与验收命令

Isaac Sim 4.5 中先停止 Timeline，在 Script Editor 运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
          "task27_five_cube_center_insert_scene.py").read())
```

点击 Play，等待 FR3 关节归位后，在同一 Script Editor 运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
          "task27_five_cube_center_insert_bridge.py").read())
```

终端 A 启动 MoveIt：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=0

ros2 launch fr3_dual_side_suction_description \
  moveit_dual_side_suction.launch.py use_rviz:=false
```

终端 B 构建并先预检阶段 A：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=0

colcon build --packages-select fr3_dual_palletize --symlink-install
source install/setup.bash

ros2 run fr3_dual_palletize task27_five_cube_center_insert --ros-args \
  -p planning_only:=true \
  -p first_batch:=1 \
  -p max_batches:=4
```

当前版本也已用单进程预检全部五件并在**全新场景**中单进程执行全部五件；希望一次完成时，在终端 B 用以下两条命令替代下方分段执行命令：

```bash
ros2 run fr3_dual_palletize task27_five_cube_center_insert --ros-args \
  -p planning_only:=true \
  -p first_batch:=1 \
  -p max_batches:=5

ros2 run fr3_dual_palletize task27_five_cube_center_insert --ros-args \
  -p first_batch:=1 \
  -p max_batches:=5 \
  -p execution_time_scale:=3.0
```

预检不会移动 Cube；执行前确保场景仍是五件均未入垛的初始状态。以下阶段 A/B 命令保留用于单独定位问题，不要在完整执行之后重复运行。

阶段 A 物理执行（只放四件边侧基准 Cube）：

```bash
ros2 run fr3_dual_palletize task27_five_cube_center_insert --ros-args \
  -p first_batch:=1 \
  -p max_batches:=4 \
  -p execution_time_scale:=3.0
```

**不重建 Isaac 场景**，阶段 A 成功、四件仍在车厢内后，分别预检和执行中心件：

```bash
ros2 run fr3_dual_palletize task27_five_cube_center_insert --ros-args \
  -p planning_only:=true \
  -p first_batch:=5 \
  -p max_batches:=1

ros2 run fr3_dual_palletize task27_five_cube_center_insert --ros-args \
  -p first_batch:=5 \
  -p max_batches:=1 \
  -p execution_time_scale:=3.0
```

`ROS_LOCALHOST_ONLY=0` 必须保持：Isaac GUI 中的 ROS 2 Bridge 与终端节点需要相互发现。

## 实施顺序

1. 为 Task27 建立五件最深层的离线场景布局和 Ground Truth 验收项；
2. 以 `planning_only:=true` 先验证阶段 A 的四件 IK、严格 Cartesian 和同步 FCL；
3. 依次完成阶段 A 的 Isaac 物理验收；
4. 最后仅针对中心 Cube 调试阶段 B 的插入与左右间隙；
5. 阶段 B 成功后才考虑扩展至下一 X 层或后续连续来料。

## 2026-09-24 实体验收记录

在全新 Isaac Sim GUI 中按上方命令加载场景、启动 Play 和 `/task27` bridge，终端启动 MoveIt。阶段 A 先以 `planning_only:=true, first_batch:=1, max_batches:=4` 预检；旧方案的第四件 `SIDE_CONTACT` 仅达到约 `0.65` Cartesian fraction，故改为内侧件在搬运时对齐通道、再沿 +X 直推。修改后四件预检 exit 0；实体执行同样 exit 0。保持同一 Isaac 场景，阶段 B 先以 `planning_only:=true, first_batch:=5, max_batches:=1` 预检，再实体执行，均 exit 0。

独立从 Isaac USD Stage 读取最终方块中心（单位 m）：

| Cube | 最终中心 `(x, y, z)` | 格位误差 | 贴合/余隙 |
|---|---|---:|---:|
| `01` +Y 外 | `(1.099293, 0.253176, 0.260000)` | `1.086 mm` | +Y 墙 `0.824 mm` |
| `02` -Y 外 | `(1.098560, -0.253901, 0.260000)` | `1.444 mm` | -Y 墙 `0.099 mm` |
| `03` +Y 内 | `(1.099321, 0.132679, 0.260000)` | `1.485 mm` | +Y 外侧 Cube `0.497 mm` |
| `04` -Y 内 | `(1.099401, -0.133786, 0.260000)` | `0.636 mm` | -Y 外侧 Cube `0.115 mm` |
| `05` 中央 | `(1.099684, 0.000365, 0.260000)` | `0.483 mm` | +Y / -Y 内侧 Cube `12.314 / 14.151 mm` |

中心 Cube 的深端墙间隙为 `0.316 mm`；左右剩余总间隙 `26.465 mm`，接近几何理论值 `28 mm`。中央件双杯均经 Isaac 确认 `CLOSED`，共同抬升、X 向搬运、下降时保持附着，运输阶段记录的最大倾角约 `0.108°`；释放后双杯 `OPEN`，16 段推入完成，最大记录关节扭矩 `36.39 N·m`，双臂最终回共同 HOME。

关键原始日志摘录：

```text
Task27 阶段恢复 PASS：前 4 件已由 Ground Truth 回写为真实碰撞物。
Task26 启动滑轨同步：left=0.7500 right=0.7500 rest=0.6500 world_shift_x=+0.1000 m。
task27_center_insert COMMON_LIFT GT=(0.500, 0.000, 0.539), suction=(left:CLOSED right:CLOSED).
task27_center_insert PUSH slice 16/16: commanded_cube_x=1.1000, actual=1.0997, lag=0.28 mm, peak_torque=32.83 Nm.
task27_center_insert center Ground Truth: expected=(1.100, 0.000, 0.260), actual=(1.100, 0.000, 0.260), cell_error=0.483 mm, +X_gap=+0.316 mm, +Y_inner_gap=+12.314 mm, -Y_inner_gap=+14.151 mm, total=+26.465 mm.
Task26 batch 5 PASS: 两件已落稳入垛，双臂已回到共同 HOME，等待下一批到料。
```

最后两条日志沿用了 Task26 的旧标签和“两件”措辞，实际 Task27 每批仅一件；源码已修正后续日志文本，不改变本次物理执行逻辑。

### 验收中发现并修复

1. 阶段 A 结束时两基座滑轨在 `x=0.750 m`。阶段 B 用新进程启动，原先 `g_world_shift_x=0` 导致实体杯面与规划目标整体差约 `100 mm`；吸附前几何门禁停止了动作。现在每次启动均等待左右 `/task27/*/rail_state` 的实测、到位标志，并检查两侧一致，再初始化世界偏移，缺失则拒绝规划。
2. 中央件实际接近时两杯与侧面间隙稳定在约 `2.1–2.3 mm`；重复名义位姿、左右中线纠偏及按实测状态尝试的独立毫米级纠偏均未可靠缩小该间隙。最终交付版没有保留未经闭环验证的独立纠偏。Task27 的预吸附上限设为 `2.5 mm`，低于 Isaac Surface Gripper `3 mm` 捕获阈值；Task26 的 `2.0 mm` 不变。无论预门禁如何，实体必须双侧 `CLOSED` 且通过后续附着几何与共同抬升检查，才算真正抓住。
3. 第三件 +Y 内侧推入时出现一次 `63.87 N·m` 的短时关节扭矩峰值（当前硬上限 `80 N·m`），随后回落并完成码垛。这不是本次失败，但应在复现实验中检查接触动力学和峰值来源，不能把单次 PASS 当成额定负载证明。

### 尚未解决 / 下次验收重点

- 2026-09-24 的旧几何版本有一轮分阶段通过；2026-09-26 当前 `1.5 mm` 版本有一轮单进程连续通过。仍需在**当前同一版本**下多次重建场景重复运行，统计规划成功率、每件周期及扭矩峰值。
- 第五件双杯虽确认为 `CLOSED` 且搬运稳定，预吸附实测空气隙可达约 `2.27 mm`。如果验收要求杯面与箱体几何零间隙，而非 PhysX Surface Gripper 的闭合附着，还需单独标定碰撞形状/接触 offset，不能以本次结果声称物理零缝。
- 双臂侧吸紧协调仍可能因 FR3 冗余 IK 的随机候选出现单次规划失败；目前有候选筛选与 FCL 门禁，本次没有出现失控或绕过碰撞检查。

## 2026-09-26 重复性诊断与修复

第一次重跑暴露出预检与执行的起点不一致：旧的侧压预检从 `CELL_EXIT` 末端出发，
而真实侧压从 `PUSH_INTO_BOX` 末端出发。一次外侧件推到深墙后，背挡臂的后续 Y
横移发生 IK 跳支，`Cartesian fraction=1` 仍偏离直线。现在重抓候选必须从**同一
PUSH 末端**通过侧压完整链路才会被选中；滑轨前移后还会重新生成当前坐标系的
FCL 世界与已落稳 Cube 碰撞体。后续试验中，外侧两件均完成实体侧压。

第二个问题来自内侧件的真实角点扫掠。原先两件中心线严格相距 `120 mm`，
重跑的第 4 件在第 `10/16` 段达到 `87.0 N·m`，超过原有 `80 N·m` 保护阈值而安全
停机；方块落后命令 `3.42 mm`。Isaac 停止位的第 4 件出现约 `0.36°` 偏航，
其世界包围范围与外侧件在 Y 上交叠约 `0.98 mm`，解释了接近邻件时的力矩突升。
没有提高扭矩上限、扩大 ACM 或忽略已放置 Cube。

内侧件对齐外侧 Ground Truth 时改为名义留缝后，得到以下实际结果：

| 方案 | 实体进度 | 关键读数 | 结论 |
|---|---|---|---|
| `0 mm` 内侧余量 | 前三件通过，第 4 件中止 | `87.0 N·m`，超过 `80 N·m` | 角点可能卡住 |
| `2.0 mm` 内侧余量 | 前两件通过，第 3 件中止 | 第 3 件邻件间隙 `3.227 mm`，超过原有 `3 mm` 门限；推入峰值 `20.66 N·m` | 通道足够，落稳间隙过大 |
| `1.5 mm` 内侧余量 | 隔离复测第 3–5 件通过；随后完整五件连续通过 | 隔离复测第 3/4 件邻件间隙 `1.300/1.666 mm`；第 4 件推入记录峰值 `30.27 N·m`；中央件两侧余隙 `11.225/12.069 mm`、深墙间隙 `0.537 mm` | 隔离复测与完整运行分别记录如下 |

隔离复测是在全新 Isaac 场景中，把前两件放回上一轮已验收的落点后，从第 3 件
实体执行到第 5 件；不能将其写成“从空场景连续运行五件”。随后另行重建场景，
完成了下方的空场景五件连续运行。`1.5 mm` 余量仅作用于 Task27 内侧直推，Task26 原布局与原有的
`3 mm` 位置/间隙、`80 N·m` 扭矩门限均未改。当前日志在本机
`artifacts/task27_repeat_20260926/`，包括每次失败和通过的完整 ROS 输出。

### 当前版本完整五件实体运行（2026-09-26）

重建 Task27 初始 Isaac 场景后，`planning_only:=true`、`first_batch:=1`、
`max_batches:=5` 的五件预检 exit 0；随后以 `first_batch:=1`、`max_batches:=5`、
`execution_time_scale:=3.0` 单进程连续完成五件，**exit 0**。每批均先物理供料、
双臂侧吸搬运，再由单臂沿 +X 分段推入；两件外侧件还完成另一臂侧压。五次
`batch PASS` 均在双臂返回共同 HOME 后记录。ROS 日志从启动到最后一批 PASS
约 `23 min 33 s`，其中包含 Isaac 物理执行、供料、规划和等待，不能当作纯规划耗时。

| Cube（顺序） | 结束后 Isaac USD 中心 `(x,y,z)` m | 格位误差 | 深端墙间隙 | 侧向墙/邻件间隙 | 分段推入记录峰值 |
|---|---|---:|---:|---:|---:|
| `01` +Y 外 | `(1.099641, 0.252434, 0.260000)` | `1.606 mm` | `0.359 mm` | +Y 墙 `1.566 mm` | `34.71 N·m` |
| `02` -Y 外 | `(1.099375, -0.253999, 0.260000)` | `0.625 mm` | `0.625 mm` | -Y 墙 `0.001 mm` | `24.32 N·m` |
| `03` +Y 内 | `(1.099388, 0.130535, 0.260000)` | `0.731 mm` | `0.612 mm` | +Y 外件 `1.899 mm` | `28.72 N·m` |
| `04` -Y 内 | `(1.099232, -0.131988, 0.260000)` | `0.923 mm` | `0.768 mm` | -Y 外件 `2.011 mm` | `23.03 N·m` |
| `05` 中央 | `(1.099357, -0.000102, 0.260000)` | `0.651 mm` | `0.643 mm` | +Y/-Y 内件 `10.636/11.886 mm` | `32.62 N·m` |

中心件两侧剩余间隙合计 `22.523 mm`，在当前允许的 `25 ± 6 mm` 门限内；
这不是零缝。上述力矩为控制器在推入分段检查中**记录到的峰值**，不能代表
物理仿真的连续时间真实最大力矩。单进程过程中有被拒绝的 RRT/重抓候选，
包括第 4 件一次约 `0.106 mm` 的 FCL 候选碰撞；候选被拒绝并换成安全候选，
不是执行中的碰撞，也未放宽 ACM 或 `80 N·m` 保护阈值。

关键原始 ROS 日志：

```text
Task27 batch 1 PASS: 当前批 1 件已落稳入垛，双臂已回到共同 HOME。
Task27 batch 2 PASS: 当前批 1 件已落稳入垛，双臂已回到共同 HOME。
task27_plus_inner final Ground Truth: expected=(1.100, 0.131, 0.260), actual=(1.099, 0.131, 0.260), cell_error=0.731 mm, +X deep wall_gap=+0.612 mm, +Y outer cube_gap=+1.899 mm.
Task27 batch 3 PASS: 当前批 1 件已落稳入垛，双臂已回到共同 HOME。
task27_minus_inner final Ground Truth: expected=(1.100, -0.132, 0.260), actual=(1.099, -0.132, 0.260), cell_error=0.923 mm, +X deep wall_gap=+0.768 mm, -Y outer cube_gap=+2.011 mm.
Task27 batch 4 PASS: 当前批 1 件已落稳入垛，双臂已回到共同 HOME。
task27_center_insert center Ground Truth: expected=(1.100, 0.000, 0.260), actual=(1.099, -0.000, 0.260), cell_error=0.651 mm, +X_gap=+0.643 mm, +Y_inner_gap=+10.636 mm, -Y_inner_gap=+11.886 mm, total=+22.523 mm.
Task27 batch 5 PASS: 当前批 1 件已落稳入垛，双臂已回到共同 HOME。
```

完整日志位于本机 `artifacts/task27_repeat_20260926/preflight_full_1p5.log` 与
`artifacts/task27_repeat_20260926/full_run_1p5.log`，没有纳入 Git（体积较大）；
最终五件 Cube 中心还通过 Isaac USD Stage 直接读取确认。

### 606 mm 窄车厢、内件短侧压与批间交接（2026-09-26）

老师提出中心件不能离两侧太远，并希望边侧件也压实。在当前左右交替顺序中，
`Cube_01/02` 是外侧件，原有双臂侧压分别使其贴向 +Y/-Y 墙；`Cube_03/04`
是内侧件，新加短侧压使其贴向外侧邻件。因此只对 Task27 缩小车厢：Task26 的 628 mm
内宽完全不变；Task27 从 628 mm 缩到 606 mm（五件本体总宽 600 mm）。不通过
扩大 ACM、提高 80 N·m 力矩阈值或穿透碰撞体消除间隙。

内件沿 +X 直推仍留 1.5 mm 角点扫掠余量；到深墙后，保持 -X 面吸附并由推入臂
沿 Y 短压 1 mm，再从**压后的末端状态**预检并退出。该阶段分四段检查 Isaac
Ground Truth 邻件间隙与关节力矩；规划时同一重抓候选必须同时通过推入、短侧压
和退出的双臂 FCL。紧协调放到预推位后，双臂改为同步抬离。每件完成后预检
下一件从当前实测关节位到预吸高位的直达路径；若与下一件到料位置冲突，则回退
原有同步 HOME，绝不跳过碰撞门禁。第 5 件仍以 HOME 结束。

全新场景中，先以 `planning_only:=true, first_batch:=1, max_batches:=5` 做五件
预检，exit 0；随后按本文完整执行命令单进程运行五件，exit 0。第一次预检曾因
`/move_group/get_parameters` 服务瞬时超时失败，未发送任何控制命令；重试后
五件预检正常。五件实体运行从第一条 batch 日志到最后一条 PASS 约
`20 min 48 s`，旧 628 mm 版本记录为约 `23 min 33 s`。两次不是同条件
受控效率实验，不能把差值全部归因于跳过 HOME；本次第 1/3/4 批确实跳过了
批间 HOME，第 2 批因下一件尚未到料时的假定碰撞预检不通过而安全回退 HOME。

| Cube | Isaac USD 最终中心 `(x,y,z)` m | 格位误差 | 深墙间隙 | 侧墙/邻件间隙 | +X 推入记录峰值 |
|---|---|---:|---:|---:|---:|
| `01` +Y 外 | `(1.099617, 0.242526, 0.260000)` | `0.609 mm` | `0.383 mm` | +Y 墙 `0.474 mm` | `33.89 N·m` |
| `02` -Y 外 | `(1.098807, -0.242999, 0.260000)` | `1.193 mm` | `1.193 mm` | -Y 墙 `0.001 mm` | `36.15 N·m` |
| `03` +Y 内 | `(1.099780, 0.121564, 0.260000)` | `0.582 mm` | `0.220 mm` | +Y 邻件 `0.962 mm` | `27.97 N·m` |
| `04` -Y 内 | `(1.099668, -0.121552, 0.260000)` | `0.336 mm` | `0.332 mm` | -Y 邻件 `1.447 mm` | `29.59 N·m` |
| `05` 中央 | `(1.099520, 0.000171, 0.260000)` | `0.509 mm` | `0.480 mm` | +Y/-Y 邻件 `1.393/1.724 mm` | `33.75 N·m` |

Cube_03 短侧压期间邻件间隙从 `1.634` 降至 `0.978 mm`，释放后为
`0.962 mm`；Cube_04 从 `2.178` 降至 `1.489 mm`，释放后为 `1.447 mm`。
短侧压每段记录到的最大扭矩分别为 `22.96/24.67 N·m`。中心两侧余隙合计
`3.117 mm`。加上两侧外墙 `0.474+0.001 mm` 和两处内外邻件
`0.962+1.447 mm`，合计约 `6.001 mm`，与内腔比五件实体总宽多出的
`6 mm` 闭合；不能解释成 Cube 被压缩或精确零缝。以上读数只代表
分段采样，不是连续时间力矩峰值。

关键原始日志：

```text
task27_plus_inner INNER_SIDE_TRIM slice 4/4: neighbor_gap=+0.978 mm, peak_torque=22.60 Nm.
task27_plus_inner final Ground Truth: ... +Y outer cube_gap=+0.962 mm.
task27_minus_inner INNER_SIDE_TRIM slice 4/4: neighbor_gap=+1.489 mm, peak_torque=24.30 Nm.
task27_minus_inner final Ground Truth: ... -Y outer cube_gap=+1.447 mm.
Task27 batch 4 PASS: Cube 已落稳，双臂已安全退出；下一件 Cube_05 可从当前状态直接到预吸高位，跳过批间 HOME。
task27_center_insert center Ground Truth: ... +Y_inner_gap=+1.393 mm, -Y_inner_gap=+1.724 mm, total=+3.117 mm.
Task27 batch 5 PASS: 当前批 1 件已落稳入垛，双臂已回到共同 HOME。
```

本机完整日志为 `artifacts/task27_repeat_20260926/planning_only_606_optimized_retry.log`
与 `artifacts/task27_repeat_20260926/full_run_606_optimized.log`（未提交 Git）。
紧接着用**同一版**重建场景复测：前四件再次通过，第 3/4 件最终邻件间隙
`0.878/0.819 mm`；中心件在吸附前左右杯面间隙差 `0.902 mm`，虽然低于原有
`1 mm` 门槛，共同 X 搬运后增至 `1.080 mm`，控制器按设计安全停机，进程
exit 1。完整日志为 `full_run_606_optimized_repeat2.log`。这轮失败证明首次完整
通过并不等于稳定；没有通过放宽搬运门槛掩盖问题。当时先尝试**只针对 Task27
中心件**把吸附前对称性门槛收紧到 `0.3 mm`，促使现有未吸附纠偏机制在启用
Surface Gripper 之前消除偏差，运输阶段仍用原 `1 mm` 门槛；下文的复测说明
为什么最终扩展到 Task27 全部 Cube。该修复需要重新
跑完五件实体后才能评价。即使之后通过，仍应继续同配置多轮重启复测；首次
第 4 件最终邻件间隙 `1.447 mm` 距当前 `1.5 mm` 验收上限仅 `0.053 mm`，
不能据此声称可靠的亚毫米压紧。

2026-09-28 追查上一次在后台继续运行的完整日志
`full_run_606_strict_center_preclose.log`：该轮在 `Cube_03` 停机，固定 1 mm
内侧短压后实际邻件间隙仍为 `1.961 mm`（原验收上限 `1.5 mm`），因此尚未
运行到中心件，**不能**把吸附前 `0.3 mm` 门槛称为已完成实体复测。日志同时
显示 Cube_03 推入末端的 X 向跟随滞后 `1.57 mm`，这轮物理伺服残差明显高于
首次通过时的约 `0.5 mm`。当前新增以 Isaac Ground Truth 间隙触发的**最多
1 mm 单次额外侧压**：仅在固定短压后仍超 `1.5 mm` 时触发，目标至少保留
`0.6 mm` 名义间隙；候选预检额外侧压和压后退出，实际执行后仍要求邻件间隙
`[-0.5, 1.5] mm`、深墙门槛及 `80 N·m` 力矩门槛。零命令五件预检已 exit 0，
`Cube_03/04` 的额外侧压及退出均通过 IK/FCL。首次实体复测在 Cube_02
共同 X 搬运时按原 `1 mm` 对称性门槛安全停机：吸附前差 `0.991 mm`，
搬运后差 `1.164 mm`。这证明只收紧中心件不够；Task27 **所有** Cube
的吸附前门槛现改为 `0.3 mm`，Task26 与搬运中的 `1 mm` 门槛均不变。
后续实体运行结果待补。

### 2026-09-28～29：重复运行的安全停机与闭环修正

同配置实体复测暴露了两类随机残差，不能只凭第一次完整 PASS 宣称稳定：

- Cube 双吸盘预吸前的杯面间隙有时不对称，运输时会进一步放大。Task27 的
  `PRE_CLOSE_GEOMETRY` 门槛现为 `0.3 mm`，只在 **SUCTION OFF** 状态下
  利用最新 Isaac Cube/TCP Ground Truth 作短距离纠偏；Task26 行为不变。
  `computeCartesianPath` 曾多次对 0.2～0.35 mm 请求返回 `fraction=1`、
  但实际只有原起点。现要求轨迹终点 FK 与请求相差不超过 `0.05 mm`，
  否则不启动吸盘；最近一次将 Task27 非零纠偏的最小指令改为 `0.65 mm`，
  后续全新场景完整实体运行在 Cube_02/04 各触发一次 `0.65 mm` 纠偏，
  均得到非单点轨迹并通过再次实测；不能由两次成功推出统计可靠性。
- 内侧 Cube 固定 `1 mm` 短侧压有时仍留大于 `1.5 mm` 的邻件间隙。
  现以真实间隙为反馈，必要时最多额外短压一次 `1 mm`；每段仍检查
  Ground Truth、`80 N·m` 力矩门槛及整段双臂 FCL，不扩大 ACM。

`full_run_min_step_20260929.log` 是从全新场景开始的五件复测：Cube_01/02/03
分别完成，Cube_02 对 -Y 侧墙间隙 `0.030 mm`，Cube_03 额外短侧压后对
Cube_01 邻件间隙 `0.642 mm`。Cube_04 在吸附前左右杯面间隙差 `0.662 mm`，
尝试向外纠偏 `0.35 mm` 时 MoveIt 仍只返回单点轨迹，控制器正确在
**SUCTION ON 之前**安全停机；这不是五件通过。

保持三件已落稳的 Isaac 场景、不重新摆物料，以 `first_batch:=4,
max_batches:=2` 续跑，日志 `resume_4_5_0p65_20260929.log` exit 0：Cube_04
短侧压后与 Cube_02 邻件间隙 `0.931 mm`，Cube_05 与两侧内件间隙
`0.381 / 2.244 mm`，双臂最终回 HOME。续跑时 Cube_04 换了 RRT 候选，
吸附前杯面差直接为 `0.099 mm`，所以**没有触发**新 `0.65 mm` 纠偏。
中央件虽然通过原有每侧 `4 mm` 门槛，左右不均衡可见：前四件压紧后的
真实通道中心并非设计 `y=0`。因此当前代码把 Cube_05 的预推位和终点
Y 改为 Cube_03/04 Isaac 实测中心的中点，预检仍用设计中心。

以上日志保存在本机 `artifacts/task27_repeat_20260926/`，未纳入 Git。
`planning_full_dynamic_center_20260929.log` 在重建场景后零命令五件预检
exit 0。对应完整实体结果如下。

### 2026-09-29：动态中央目标完整实体复测

为避免第 3/4 件实际短侧压后通道偏心，Cube_05 在规划前用两件内侧 Cube
最新 Isaac Ground Truth 中心 Y 的中点作预推位和最终 Y；本轮读数为
`plus_y=0.1211`、`minus_y=-0.1218`、`target_y≈-0.0004 m`。这只是
目标位姿校正，原有双臂 IK/FCL、杯面对称性、推入力矩、最终间隙验收均不放宽。

从全新 Isaac 场景，先运行五件零命令预检 exit 0，再以同一源码单进程连续
执行五件，`full_run_dynamic_center_20260929.log` exit 0，批 1 开始至批 5
回 HOME 约 `21 min 16 s`。第 1/3/4 批通过下一件预吸位 FCL 直达检查并
跳过批间 HOME；第 2 批检查未通过，回退共同 HOME。Cube_02/04 在 SUCTION ON
之前执行了 `0.65 mm` 预吸纠偏，其中 Cube_04 杯面差从 `0.516 mm` 降为
`0.134 mm`。第 3/4 件短侧压都成功，没有触发额外 1 mm 修正。

| Cube | Isaac 最终中心 `(x,y,z)` m | 格位误差 | 深墙间隙 | 侧墙/邻件间隙 |
|---|---|---:|---:|---:|
| `01` +Y 外 | `(1.099586, 0.241977, 0.260000)` | `1.104 mm` | `0.414 mm` | +Y 墙 `1.023 mm` |
| `02` -Y 外 | `(1.098567, -0.242947, 0.260000)` | `1.434 mm` | `1.433 mm` | -Y 墙 `0.053 mm` |
| `03` +Y 内 | `(1.099788, 0.121078, 0.260000)` | `0.638 mm` | `0.212 mm` | +Y 外件 `0.898 mm` |
| `04` -Y 内 | `(1.099701, -0.121801, 0.260000)` | `0.463 mm` | `0.299 mm` | -Y 外件 `1.147 mm` |
| `05` 中央 | `(1.099491, -0.000491, 0.260000)` | `0.525 mm` | `0.509 mm` | +Y/-Y 内件 `1.569/1.310 mm` |

中央两侧间隙合计 `2.879 mm`，左右差 `0.259 mm`。与上一轮固定 `y=0`
的 `0.381/2.244 mm` 相比，对称性明显改善；两轮前四件实际落位不同，
不能将全部改善量严格归因于这一行目标校正。真实接触力峰值仍未做连续采样，
且这不是多轮成功率证明。现场截图保存在本机
`artifacts/task27_repeat_20260926/final_dynamic_center_20260929.png`，未纳入 Git。

### 下一步

1. 在当前 606 mm 内宽、1.5 mm 直推余量、1 mm 内件短侧压与相同控制参数下，从全新 Isaac 场景至少再跑数轮完整五件，记录每轮是否 exit 0、各件周期、格位误差和分段推入扭矩；不要把不同内宽版本合并为同配置成功率。
2. 若复测稳定，再扩展到下一 X 墙层；新层必须重新验证供料/退出通道和已放置件的 FCL，不直接复用第一层的 PASS。
3. 若需评价真实接触峰值或夹具受力，应增加更高频率的力/扭矩采样；现有每推入段读数只用于当前安全门禁与趋势判断。

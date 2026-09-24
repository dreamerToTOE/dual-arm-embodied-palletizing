# Task27：第一层五 Cube 的两段式紧协调装箱

状态：✅ **2026-09-24：Isaac Sim 4.5 + MoveIt2 完成一次五件实体码垛验收，阶段 A/B 均 exit 0，双臂返回共同 HOME。** 尚未证明多次重复运行的统计稳定性。

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

沿用 Task26 当前车厢 Y 内腔 `[-0.314, +0.314] m`、Cube 宽度 `0.120 m`：

- 内腔总宽度为 `0.628 m`；五件总宽度为 `0.600 m`；
- 因而中心插入后不可消除的总余量为 `28 mm`；
- 若中心 Cube 位于 `y=0`，两侧理论间隙均为约 `14 mm`。

这说明，在不缩小车厢或压缩 Cube 的前提下，中心 Cube 不可能同时与左右邻 Cube 零间隙接触。Task27 的正确目标是让该余量**对称、可预测且尽可能小**，而不是通过扩大碰撞豁免或强行穿透来伪造零缝。

建议的第一版名义 Y 中心为：

| 位置 | 名义 Y (m) | 支撑关系 |
|---|---:|---|
| `+2` | `+0.254` | `+Y` 围墙 |
| `+1` | `+0.134` | `+2` Cube |
| `0` | `0.000` | 左右均保留约 `14 mm` 插入余量 |
| `-1` | `-0.134` | `-2` Cube |
| `-2` | `-0.254` | `-Y` 围墙 |

最终数值以 Isaac Collider 尺寸和物理 Ground Truth 复核为准。

## 阶段 A：四件边侧基准 Cube

执行顺序必须左右交替，以降低连续占用同侧通道的概率：

```text
+2 -> -2 -> +1 -> -1
```

外侧两件复用 Task26 已通过的侧压链路；内侧两件先对齐已放置外侧件的 Isaac Ground Truth Y 坐标，再沿 +X 直推（第四件的旧低位 `SIDE_CONTACT` 仅完成约 65% Cartesian 路径，因此没有强行执行侧压）：

```text
双臂紧协调侧吸
  -> 共同接触 / 抬升
  -> X/Y 直角共同搬运至预推位
  -> 下降并同步释放
  -> 推入臂重抓 Cube -X 面
  -> +X 分段推入至车厢深端墙
  -> 外侧件：另一臂侧压到对应围墙，推入臂作为反向背挡
  -> 内侧件：在预推位预先对齐实际外侧件，沿 +X 直推，不在深位做侧压
  -> 双臂安全退出
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
3. 左右间隙总和接近车厢理论余量，且中心偏置受限；
4. 两侧间隙尽量均衡。第一版以约 `14 mm` / `14 mm` 为目标；本次实测为 `12.314 mm` / `14.151 mm`，中心偏置约 `0.919 mm`；
5. 完成后双臂回共同 HOME。

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

- 仅有一轮完整实体通过；需要多次重建场景重复运行，统计规划成功率、每件周期及扭矩峰值。
- 第五件双杯虽确认为 `CLOSED` 且搬运稳定，预吸附实测空气隙可达约 `2.27 mm`。如果验收要求杯面与箱体几何零间隙，而非 PhysX Surface Gripper 的闭合附着，还需单独标定碰撞形状/接触 offset，不能以本次结果声称物理零缝。
- 双臂侧吸紧协调仍可能因 FR3 冗余 IK 的随机候选出现单次规划失败；目前有候选筛选与 FCL 门禁，本次没有出现失控或绕过碰撞检查。

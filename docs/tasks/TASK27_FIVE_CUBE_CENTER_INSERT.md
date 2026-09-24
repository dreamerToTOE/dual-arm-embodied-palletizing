# Task27：第一层五 Cube 的两段式紧协调装箱

状态：🟡 **场景、Isaac bridge 与独立执行器已实现并通过 Python 语法检查、ROS 构建；待 Isaac / MoveIt 零命令预检和物理验收。**

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

每一件均复用 Task26 已通过链路：

```text
双臂紧协调侧吸
  -> 共同接触 / 抬升
  -> X/Y 直角共同搬运至预推位
  -> 下降并同步释放
  -> 推入臂重抓 Cube -X 面
  -> +X 分段推入至车厢深端墙
  -> 对应侧的另一臂侧压（外侧 Cube 压墙；内侧 Cube 压向同侧外侧 Cube）
  -> 推入臂作为反向背挡
  -> 双臂安全退出
```

阶段 A 不引入新选择器、在线调度或力控；只把现有已验收工艺改为四个已知目标格位和对应支撑对象。

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
4. 两侧间隙尽量均衡。第一版以约 `14 mm` / `14 mm` 为目标，具体容差在 Isaac 物理基线后确定；
5. 完成后双臂回共同 HOME。

## 不在本任务内

- 不做运行时 Cube 选择器；
- 不做多层堆垛；
- 不做 force / impedance control；
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

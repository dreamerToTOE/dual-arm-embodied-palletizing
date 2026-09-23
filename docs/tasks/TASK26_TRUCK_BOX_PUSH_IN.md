# Task26：车厢三面围墙的双臂紧协调推入与侧压贴墙

状态：✅ **批 1（两件）已在 Isaac Sim 物理执行通过。**

## 目标

验证第一层装箱的“无侧缝”工艺，而不是只把 Cube 放进车厢：

```text
双臂紧协调侧吸搬运至预推安全行
  -> 释放双侧吸附
  -> 一臂改吸 Cube 的 -X 面，沿 +X 推到后向支撑面
  -> 该臂保持 -X 反向背挡
  -> 另一臂从侧面压向 ±Y 围墙
  -> 两臂安全退出，并用 Isaac Ground Truth 验收
```

深位 Cube 的后向支撑面是车厢 `+X` 深端墙；浅位 Cube 的后向支撑面是同一排已经完成的深位 Cube。
因此每排必须“先深后浅”。

## 已验证场景

| 项目 | 数值 |
|---|---:|
| Cube 尺寸 | `0.120 × 0.120 × 0.120 m` |
| 桌面顶面 | `z=0.200 m` |
| Cube 中心高度 | `z=0.260 m` |
| 车厢内腔 X | `[0.910, 1.160] m` |
| 车厢内腔 Y | `[-0.314, +0.314] m` |
| 围墙厚 / 高 | `0.020 / 0.150 m` |
| 装料口 / 推入方向 | `-X / +X` |
| 预推安全行 | `(0.790, ±0.060, 0.260)` |
| 深位格 | `(1.100, ±0.254, 0.260)` |
| 浅位格 | `(0.980, ±0.254, 0.260)` |
| 两条 X 向滑轨 | 静止位 `x=0.650 m`，行程 `[0.450, 1.050] m` |

车厢 Y 向的五通道容量为后续扩展预留；本次物理验收先覆盖 `+Y` 侧墙的一排深位、浅位两件。

## 核心实现

- 初始紧协调搬运仍复用已验证的“共同接触 → 共同抬升 → X/Y 直角搬运 → 下降 → 同步释放”链。
- 空载接近使用 RRTConnect 候选池；候选按关节总行程排序，但只有完整后续链路也通过同步 FCL 时才可采用。
- 抓住 Cube 后的共同搬运、推入、侧压全部是严格 Cartesian 轨迹；若 FK 对目标直线偏离超过 `5 mm`，即使 `computeCartesianPath()` 返回 `1.0` 也拒绝该候选。
- 推入臂在重新吸住 `-X` 面后，将 +X 推入拆为 16 个受监测的短段；每段检查 Cube Ground Truth 跟随误差和腕部力矩。
- 深位 Cube 推到 `+X` 墙后，浅位 Cube 推至深位 Cube 的 `-X` 面；两种支撑关系分别验收，避免把正常的层内 120 mm 距离误判为墙缝。
- 侧压时，+X 推入臂保持吸附并沿 Y 方向同步跟随，作为**反向背挡**；侧压臂从另一侧压向墙。背挡轨迹被拆成 8 段，避免长 Cartesian 路径发生 IK 解支跳变。
- 侧压命令对墙方向增加 `4 mm` 终点超程，使 PhysX Collider 承担止挡；这不是放宽验收，最终仍强制使用 `3 mm` Ground Truth 间隙门限。
- 未扩大 Allowed Collision Matrix，未忽略 Cube-机械臂或 Cube-墙体碰撞。

## 物理验收结果（2026-09-23，批 1）

运行参数：`max_batches:=1`、`execution_time_scale:=3.0`。

| 物体 | 实际中心 `(x,y,z)` m | 位置误差 | 后向支撑间隙 | 侧墙间隙 | 结论 |
|---|---|---:|---:|---:|---|
| `task26_r0_deep` | `(1.099, 0.253, 0.260)` | `1.018 mm` | `+X` 深端墙 `0.636 mm` | `+Y` 墙 `0.795 mm` | PASS |
| `task26_r0_shallow` | `(0.979, 0.254, 0.260)` | `0.591 mm` | 同排深位 Cube `0.580 mm` | `+Y` 墙 `0.116 mm` | PASS |

验收标准：每一处支撑/围墙间隙均不得超过 `3 mm`。两件完成后，系统日志为：

```text
Task26 batch 1 PASS: 两件已落稳入垛，双臂已回到共同 HOME，等待下一批到料。
```

## 构建与复现

先启动 Isaac Sim，并在 Script Editor 加载 `isaac/scripts/task26_truck_box_scene.py` 后点击 Play；再加载
`isaac/scripts/task26_truck_box_bridge.py`。该桥接器发布 `/task26/cube_poses`、双侧吸盘状态、TCP 真实位姿和滑轨状态。

启动 MoveIt：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=0

ros2 launch fr3_dual_side_suction_description \
  moveit_dual_side_suction.launch.py use_rviz:=false
```

另开终端构建并先做零命令预检：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=0

colcon build --packages-select fr3_dual_palletize --symlink-install
source install/setup.bash

ros2 run fr3_dual_palletize task26_truck_box_push_in --ros-args \
  -p planning_only:=true \
  -p max_batches:=1
```

物理执行：

```bash
ros2 run fr3_dual_palletize task26_truck_box_push_in --ros-args \
  -p max_batches:=1 \
  -p execution_time_scale:=3.0
```

`ROS_LOCALHOST_ONLY=0` 是必要条件：Isaac GUI 的 ROS 2 Bridge 与本机终端需要可互相发现；设为 `1` 会造成 `/task26/cube_poses` 超时。

## 修改文件

- `isaac/scripts/task26_truck_box_scene.py`：车厢整体 X 位移、墙边目标格位及中央预推安全行。
- `ros_ws/src/fr3_dual_palletize/src/task26_truck_box_push_in.cpp`：+X 支撑语义、双侧墙压紧、主动背挡、候选全链路预检、吸附前中线纠偏与 Ground Truth 验收。
- `docs/tasks/TASK26_TRUCK_BOX_PUSH_IN.md`：本文档。

## 下一步

本次只验收第一批的 `+Y` 一排两件。后续应在不改变该已验证工艺的前提下，继续验证 `-Y` 一排、五通道扩展与批间连续供料；多层码垛须重新评估围墙高度与前臂净空。

# Task25：分批到料的侧面吸盘紧协调多 Cube 码垛

状态：🟡 设计已确认；Isaac 场景与 Ground Truth bridge 已实现（含休眠区瞬移与到料判定），
执行器待实现，尚未开始任何物理验收。

## 目标与范围

Task24 的八件**静态**供料布局在零命令预检中第一件即失败：未处理的 Cube 会占据另一只
机械臂进入“侧面中心吸附位”的通道。Task25 不改动吸附几何，改用**分批到料**，从源头
消除供料件之间的互相堵塞。

```text
第 1 批：两个 Cube 到达供料槽并停稳
-> 读取 Ground Truth 与到位状态
-> 只把当前批两件加入 MoveIt Planning Scene
-> 对当前批次执行 Task24 已验证的紧协调搬运链路（逐件）
-> 两件都放置完成、双臂同步退回共同 HOME
-> 第 2 批两个 Cube 出现
-> 重复直到 4 批 8 件全部完成
```

每件 Cube 的链路原样复用 Task24，不新增或放宽任何规划语义：

```text
RRTConnect 空载接近
-> 左右侧面对应 CONTACT
-> 双吸盘同步 CLOSED
-> COMMON_LIFT -> COMMON_X_TRAVEL -> COMMON_Y_ALIGN
-> COMMON_DESCENT_TO_ENTRY -> COMMON_SIDE_SHORT_PUSH
-> 同步释放 -> Ground Truth 回写 -> PREPLANNED_COMMON_RETREAT
```

第一版**不包含**：真实相机、在线选件、Python 选择器、连续皮带速度控制、到位光电检测、
力控 / 阻抗控制、以及“单臂无吸附短推”后备策略。批次与顺序全部固定。

## 供料槽与目标垛

每批两件放在**同一 `y` 行、沿 `X` 分隔**的两个槽位，避免任何一件占据另一臂的侧面
中心接近通道：

| 槽 | 位置 | 取件顺序 |
|---|---|---|
| A | `(0.520, -0.070, 0.260)` m | 批内**先取** |
| B | `(0.300, -0.070, 0.260)` m | 批内后取 |

- 两槽 X 相距 220 mm，远大于 L 型阵列面板 70 mm 的 X 向包络，两件在 X 上完全错开。
- 批内先取槽 A 的原因：负载段 `COMMON_X_TRAVEL` 在**与供料同 `y` 的高度**上沿 `X`
  飞行，先取靠目标侧的槽 A，后取槽 B 时其飞行通道上已经没有任何未取供料件。
- 槽位落在 Task24 已验证的供料 `x` 区间（0.240–0.690 m）内，不引入新的可达性边界。

目标垛仍为 Task24 的两面墙，顺序不变（远墙先于近墙、底层先于上层）：

```text
批 1 -> x=0.820 的 YZ 墙底层：y=-0.120 / y=+0.120，z=0.260
批 2 -> x=0.820 的 YZ 墙上层：y=-0.120 / y=+0.120，z=0.380
批 3 -> x=0.640 的 YZ 墙底层：y=-0.120 / y=+0.120，z=0.260
批 4 -> x=0.640 的 YZ 墙上层：y=-0.120 / y=+0.120，z=0.380
```

该顺序同时保证：运往 `x=0.820` 的飞行通道上，`x=0.640` 的近墙尚未建立；运往
`x=0.640` 的飞行通道（止于 `entry_x=0.620`）也不会越过远墙。

## 分批到料机制（休眠区瞬移）

Isaac 场景**一次性**创建 8 件真实 Dynamic Rigid Body，其中 6 件在远离工作区的休眠位；
每批按命令瞬移到供料槽并落稳。这样未到货件在物理上不可能挡通道，也不需要在运行时
动态创建 USD 刚体。

- 休眠位：桌下 `z=-5.000 m`，休眠期间关闭重力；未被唤醒的 Cube 不进入 MoveIt
  Planning Scene，也不作为障碍物参与任何 FCL 采样。
- 到料：瞬移到槽位 `z = 0.200 + 0.002 m`（底面高于桌面 2 mm）、线速度与角速度清零、
  恢复重力，自由落体 2 mm 落稳。
- 到位判定由 bridge 完成：连续若干帧满足 `|z - 0.260| <= 1 mm`、`|v| <= 2 mm/s`、
  `|omega| <= 0.05 rad/s` 后，把该件标记为已到位且静止。
- 具体 API（刚体姿态写入、速度清零、重力开关）在实现时按 Isaac Sim 4.5 的实际接口
  核定，不在本设计中假定。

ROS 接口（Task25 独立命名空间，不复用 Task24 的 topic 名称）：

```text
/task25/cube_poses      geometry_msgs/PoseArray   固定 8 件、顺序与任务表一致
/task25/feed_state      std_msgs/Int32MultiArray   长度 8；1 = 已到位且静止，0 = 未到货
/task25/feed_command    std_msgs/Int32             执行器 -> Isaac，请求释放第 N 批（1..4）
/task25/{left,right}/suction_command  std_msgs/Bool
/task25/{left,right}/suction_state    std_msgs/Bool
/task25/{left,right}/side_suction_tcp_pose  geometry_msgs/PoseStamped
```

`cube_poses` 中未到货件上报的是休眠位（`z=-5`）。执行器**必须**同时读取 `feed_state`，
绝不把未到货件当作障碍物、目标或可抓取对象。

## 执行器与 Planning Scene 规则

1. 一次只把**当前批次两件**加入 MoveIt Planning Scene。
2. 已完成码垛的 Cube 始终保留为真实碰撞物，释放落稳后按最新 Ground Truth 回写。
3. 后续批次在到达之前不得作为 Planning Scene 障碍物。
4. 每批开始前必须确认该批两件都已到位且静止，才允许读 Ground Truth 并规划。
5. 该批两件都完成、并且双臂同步返回共同 HOME 之后，才允许释放下一批。
6. 每一段（空载接近与负载五段）都必须通过完整双臂 RobotState + FCL 同步门禁；没有
   通过 FCL 的候选一律不执行。
7. 载荷段保持显式直角 Cartesian：`Z -> X -> Y -> Z -> X`；吸住后禁止 OMPL 任意绕行。

批间 HOME 采用 Franka 官方 `move_to_start` 空载准备姿态
`[0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]`。该姿态在 Task24 八件全在场的初始状态下已被
实际使用，因此是“有碰撞物时仍然安全”的退出位；每批结束时两臂同步回该位并确认到位。

## 与 Task24 的复用边界

- **不修改** Task24 的任何文件：`isaac/scripts/task24_side_suction_tight_scene.py`、
  `isaac/scripts/task24_side_suction_tight_bridge.py`、
  `ros_ws/src/fr3_dual_palletize/src/task24_side_suction_tight.cpp`、
  `docs/tasks/TASK24_SIDE_SUCTION_TIGHT_OFFLINE.md`。Task24 的单件物理基线与八件失败
  记录作为可复现对照保留。
- Task25 新建独立脚本与可执行文件，复用同一个 L 型阵列侧吸描述包
  `fr3_dual_side_suction_description` 与同一个 MoveIt launch。
- 几何常量（Cube 120 mm / 0.8 kg、桌面顶面 0.200 m、Cup 面 1 mm 名义接近、3 mm 捕获
  阈值、2 mm 贴合门限、280 mm 共同抬升、20 mm 短推、20 mm 释放间隙）全部沿用 Task24，
  不重新标定。

## 验收门槛

1. 场景与 bridge 就绪后，只有批 1 的两件进入槽位，其余 6 件处于休眠位；`feed_state`
   只在两件落稳后置 1。
2. 零命令预检：`planning_only:=true` 下 4 批 8 件的完整链路（空载接近 + 负载五段）
   全部通过同步 FCL 门禁，且**不发布**任何 joint / suction 命令。
3. 单批物理验收：`max_batches:=1` 两件完成，最终 Cube Ground Truth 误差均不超过 10 mm，
   两个吸盘明确 `OPEN`。
4. 完整验收：4 批 8 件连续完成 8/8；每批之间确认双臂已回到 HOME，且已完成件始终在
   Planning Scene 中。
5. 任一件失败即停止，保留失败阶段与误差日志。不得用提高 TCP Z、把 TCP 上移到 Cube
   边缘、扩大 ACM、提前移除已完成件或跳过该批来掩盖失败。
6. `execution_time_scale` 正式物理验收固定 `3.0`；`1.0` 只用于无物理的快速检查。

## 启动与实际验收

以下路径是本设计约定的 Task25 文件。Isaac 场景与 bridge 已实现，可以单独验证
“分批到料 + 到料判定”本身；执行器尚未实现，下面的 `ros2 run` 命令在它提交前不生效。

```text
isaac/scripts/task25_batched_feed_scene.py        已实现
isaac/scripts/task25_batched_feed_bridge.py       已实现
ros_ws/src/fr3_dual_palletize/src/task25_batched_side_suction.cpp   待实现
```

场景与 bridge 的独立验证方法：加载场景、Play、运行 bridge，bridge 会自动释放批 1；
终端核对 `/task25/feed_state` 从 `[0,0,...]` 变成前两位为 `1`，并且 `/task25/cube_poses`
的 `Cube_01`、`Cube_02` 稳定在槽 A `(0.520, -0.070, 0.260)` 与槽 B `(0.300, -0.070, 0.260)`
的 1 mm 以内。随后手动发一批命令可验证下一批到料：

```bash
ros2 topic pub --once /task25/feed_command std_msgs/msg/Int32 "{data: 2}"
```

Isaac Sim 中先停止 Timeline 再加载场景；点击 Play 后单独运行 bridge：

```python
scene_path = "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task25_batched_feed_scene.py"
exec(compile(open(scene_path, "rb").read(), scene_path, "exec"))

bridge_path = "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task25_batched_feed_bridge.py"
exec(compile(open(bridge_path, "rb").read(), bridge_path, "exec"))
```

MoveIt 复用 Task24 的侧吸 launch；Isaac 与全部 ROS 终端的 `ROS_LOCALHOST_ONLY`
必须一致（Task24 基线为 `0` / 不设置，两者不可混用）：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=0

ros2 launch fr3_dual_side_suction_description moveit_dual_side_suction.launch.py use_rviz:=false
```

先做四批零命令预检：

```bash
ros2 run fr3_dual_palletize task25_batched_side_suction --ros-args \
  -p batches:=4 \
  -p planning_only:=true \
  -p execution_time_scale:=3.0
```

单批通过后再做完整四批物理验收：

```bash
ros2 run fr3_dual_palletize task25_batched_side_suction --ros-args \
  -p batches:=4 \
  -p execution_time_scale:=3.0
```

## 待确认事项

- 工程记录写回远端分支：已确认为 `侧面吸盘紧协调码垛`。
- “单臂、无吸附、短距离推送调整 Cube 到共同可达位”的后备策略：已确认**不做**，
  本任务不含该路径，也不把“调整供料位”作为可行性的补救手段。
- 本地已有待上传提交；远端 `侧面吸盘紧协调码垛` 与本地基线分叉，必须按
  fast-forward 方式重建提交后再推，不能直接 push 或 pull/merge 到当前工作区。
- README 顶部“路线冻结（2026-09-15）：主线切换为 FR3 二指夹爪”一段是否改为
  L 型阵列侧吸口径。

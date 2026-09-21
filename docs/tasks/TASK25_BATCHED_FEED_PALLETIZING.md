# Task25：分批到料的侧面吸盘紧协调多 Cube 码垛

状态：🟡 场景、bridge 与执行器均已实现并通过编译与冒烟检查；Isaac 物理验收待执行。

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
批 1 -> x=0.820 的 YZ 墙底层：y=-0.150 / y=+0.150，z=0.260
批 2 -> x=0.820 的 YZ 墙上层：y=-0.150 / y=+0.150，z=0.380
批 3 -> x=0.640 的 YZ 墙底层：y=-0.150 / y=+0.150，z=0.260
批 4 -> x=0.640 的 YZ 墙上层：y=-0.150 / y=+0.150，z=0.380
```

> Task25-B：YZ 墙的 Y 向中心距由 240 mm 改为 **300 mm**。这不是审美调整，而是
> L 型阵列的几何下限：TCP 相对法兰侧向偏置 155 mm，把 Cube 放到 `y=+pitch/2`
> 一格时支架尾端在 `pitch/2 - 216`，邻件朝向落点的那一面在 `-pitch/2 + 60`；
> 要在邻件旁边下降就必须 `pitch >= 276 mm`。240 mm 会侵入 36 mm，并已在
> `COMMON_DESCENT_TO_ENTRY` 处以真实 FCL 碰撞被拒（见下面的 Task25-B 实测修正）。
> 取 300 mm 后余量为 24 mm。

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
/task25/feed_state      std_msgs/Int32MultiArray   长度 8；0 = 未到货（桌下休眠），1 = 正在落稳，2 = 已到位且静止
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

## 执行器实现要点

```text
node:  task25_batched_side_suction
参数:  max_batches（默认 4 = 完整 4 批 8 件；单批验收用 1）
       planning_only（默认 false；零命令预检用 true）
       execution_time_scale（正式物理验收固定 3.0）
订阅:  /task25/cube_poses、/task25/feed_state、/task25/{left,right}/suction_state、/{left,right}/joint_states
发布:  /task25/feed_command、/{left,right}/joint_command、/task25/{left,right}/suction_command
```

批循环：

```text
批 b：请求到料（b > 1）
   -> 等到该批两件都“到位且静止”
   -> 读到新一帧 Ground Truth 并校验槽位偏差
   -> 只把当前批两件加入 Planning Scene
   -> 槽 A 那件走完整紧协调链路
   -> 槽 B 那件走完整紧协调链路
   -> 双臂同步回共同 HOME
   -> 才允许下一批到料
```

- 槽位是设计常量（槽 A `x=0.520`、槽 B `x=0.300`，同 `y=-0.070`）。启动时用批 1 的真实
  Ground Truth 校验“场景 - bridge - 执行器”三方对同一槽位的理解；任何批次的实际落点偏差
  超过 3 mm 即停止，不再用错误几何规划。
- 到料是瞬移 + 落稳，因此每次都必须在 bridge 判定“到位且静止”之后再读**新一帧** Ground
  Truth，绝不使用瞬移前的缓存 pose。
- `planning_only` 不发布 joint、suction，也**不发布 feed_command**（它会改变物理世界）；
  它按设计槽位虚拟推进全部批次，最后一段还包含共同 HOME 退出轨迹的 FCL 校验。
- 共同命令使用 `frame_id = task25_dual_sync`，bridge 只在收齐同一 stamp 的左右消息后，
  于同一 physics callback 内成对下发，不存在独立 Action Graph 的帧级先后差。
- 批内先取槽 A 是硬要求：负载段 `COMMON_X_TRAVEL` 在“与供料相同的 y”高度沿 X 飞行，
  先取槽 A 后取槽 B 时这条通道上不再有任何未取供料件。

## Task25-B 实测修正（零命令预检定位）

首轮四批零命令预检暴露了三个问题，全部以 FCL 日志和 MoveIt 服务实测定位，未用任何
放宽碰撞规则的方式绕过。

### 1. 共同负载段的“冻结搭档臂”假碰撞

现象：每个右臂候选的 `COMMON_Y_ALIGN left Cartesian fraction` 恒为 `0.6354`。

定位：`computeCartesianPath` 只能规划单臂路径，并把搭档臂冻结在该段起点。task25 需要
把 Cube 从 `y=-0.070` 搬到 `y=+0.150`，两臂要一起走 +220 mm；冻结的搭档臂会被真实
运动的另一臂扫到，MoveIt 在 63.5% 处把路径截断。用 `/check_state_validity` 实测该处
碰撞对为 `left_fr3_side_suction <-> right_fr3_side_suction`，接触点 `(0.824,-0.007,0.509)`
正好是左臂杯面追上右臂杯面的位置（120/190 = 63.2%）。

修正：共同负载段的两条单臂路径不做“对冻结搭档”的碰撞判断，真正的门禁交给紧随其后的
`validateSync` —— 它按同一时间参数采样左右两条轨迹，对完整双臂 RobotState 做
robot--robot 与 robot--world 的 FCL 检查。ACM 未扩大，世界障碍物未移除。

### 2. YZ 墙 Y 向中心距：240 mm 不可行，改为 300 mm

现象：修正 1 之后，全部候选改在 `COMMON_DESCENT_TO_ENTRY` 失败，碰撞对恒为
`left_fr3_side_suction <-> task25_cube_1`，接触点 `(0.809,-0.060,0.319)`，正是已落稳
邻件的顶棱。

定位：L 型阵列的支架尾端（法兰侧）在 TCP 后方 155 mm。判据：
支架尾端 `y = pitch/2 - 216`，邻件面 `y = -pitch/2 + 60`，要求前者仍在后者之外
=> `pitch >= 276 mm`。240 mm 时支架尾端落在 `-0.096 m`、邻件面在 `-0.060 m`，
侵入 36 mm；实测 FCL 深度 1.383 mm 只是第一次采样到的浅接触，继续下降会插进 50 mm。

修正：Y 向中心距取 300 mm（`y=±0.150`），支架尾端 `-0.066 m` 对邻件面 `-0.090 m`，
余量 24 mm。该值同时满足上层垛（上层 Cube 的支架 z 区间 0.391–0.409 m 与邻件
y 向不重叠）。

### 3. 零命令预检下后续批次误用休眠位

现象：批 2 的左臂 HIGH_PRE_CONTACT 连续 8 个 RRTConnect 候选全部 `plan() failed`。

定位：日志显示 `source=(-0.200, -0.300, -5.000)` —— 零命令预检不请求到料，后续批次的
Cube 仍停在桌下休眠位，而 per-cube 起点仍在读实时 Ground Truth，于是左臂去规划一个
5 m 深的目标位姿。

修正：预检分支必须使用该批的设计槽位作为预演起点；物理分支才使用到料后读到的真实
Ground Truth。

### 4. `computeCartesianPath` 会报出「离线」轨迹

现象：批 3 的 slot_b 在**执行阶段**通过同步 FCL 时发现左臂支架撞桌面，
接触点 `(0.118,-0.296,0.198)`，而同一阶段在候选预检里是通过的。

定位：用 `/compute_cartesian_path` 复现同参数规划，再用 `/compute_fk` 逐点复算工具 TCP，
发现轨迹的中间状态完全脱离命令直线 —— TCP 从 `y=+0.02` 一路甩到 `y=-1.2 m` 再绕回来，
而 MoveIt 仍然报 `fraction=1.0000`。原因是冗余臂在个别路径点的 IK 解跳到了别的解支，
MoveIt 随后把这个绕行写进了关节插值轨迹。Task24 一直把 `jump_threshold` 传 `0.0`
（等于关闭跳变检查），所以这个隐患一直存在，只是此前 Y 向位移只有 50 mm 没有触发。

修正：新增**贴线校验**。每条笛卡尔轨迹都用 FK 逐点复算末端位置，要求它到命令直线的
垂距不超过 5 mm（正常轨迹实测在 0.1 mm 量级）；不贴线就换采样步长
（0.002 / 0.0015 / 0.003 / 0.001）重规划，仍不贴线则拒绝该候选。

`jump_threshold` 保持 `0.0`：实测把它设成 `1.0` 会让正常下降段的 fraction 掉到
`0.9789`，因为 MoveIt 的 jump 判据把「手腕快速转动」和「解支跳变」混在一起，
无法用来区分两者。

效果：零命令预检里可以看到它实际拦下了 74.3 / 119.2 / 696.9 mm 级的离线轨迹，
并在换步长后找到贴线解。

### 修正后的实测结果

```bash
ros2 run fr3_dual_palletize task25_batched_side_suction --ros-args \
  -p max_batches:=4 -p planning_only:=true -p execution_time_scale:=3.0
```

```text
########## Task25 batch 1/4: Cube_01 (slot A, picked first) then Cube_02 (slot B) ##########
Task25 planning-only batch 1 PASS: ...
########## Task25 batch 2/4: Cube_03 ... Cube_04 ##########
Task25 planning-only batch 2 PASS: ...
########## Task25 batch 3/4: Cube_05 ... Cube_06 ##########
Task25 planning-only batch 3 PASS: ...
########## Task25 batch 4/4: Cube_07 ... Cube_08 ##########
Task25 planning-only batch 4 PASS: ...
（进程退出码 0；全程未发布任何 joint / suction / feed_command）
```

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

以下三个文件均已实现：Isaac 场景与 bridge 可以单独验证“分批到料 + 到料判定”本身；
执行器已完成编译与参数/接线冒烟检查，但尚未进入任何 Isaac 物理验收。

```text
isaac/scripts/task25_batched_feed_scene.py                          已实现
isaac/scripts/task25_batched_feed_bridge.py                         已实现
ros_ws/src/fr3_dual_palletize/src/task25_batched_side_suction.cpp   已实现（已编译、已冒烟）
```

编译注意：本机 `/tmp` 是 **10 MB 的 tmpfs**，直接编译千行级源文件会报
`error writing to /tmp/ccXXXX.s: 设备上没有空间`；必须先把 `TMPDIR` 指到大盘。
完整命令见下面运行手册的“终端 1”。

场景与 bridge 的独立验证方法：加载场景、Play、运行 bridge，bridge 会自动释放批 1；
终端核对 `/task25/feed_state` 从 `[0,0,...]` 变成前两位为 `1`，并且 `/task25/cube_poses`
的 `Cube_01`、`Cube_02` 稳定在槽 A `(0.520, -0.070, 0.260)` 与槽 B `(0.300, -0.070, 0.260)`
的 1 mm 以内。随后手动发一批命令可验证下一批到料：

```bash
ros2 topic pub --once /task25/feed_command std_msgs/msg/Int32 "{data: 2}"
```

执行器的静态检查（不需要 MoveIt，只验证二进制、参数与接线）：

```bash
# 参数越界：立即以非零码退出，不进入任何等待
ros2 run fr3_dual_palletize task25_batched_side_suction --ros-args -p max_batches:=9

# 缺少 /move_group：必须明确报 “Task25 无法连接 /move_group。” 后退出，而不是静默卡住
ros2 run fr3_dual_palletize task25_batched_side_suction --ros-args -p max_batches:=1 -p planning_only:=true
```

### 完整验收运行手册（逐终端复制即用）

全部 ROS 终端保持 `ROS_LOCALHOST_ONLY` **一致**：本手册统一 `unset ROS_LOCALHOST_ONLY`
（等价于 `0`）。绝不能在部分终端用 `1`，否则 MoveIt 与 Isaac bridge 会落在互不可见的
DDS 发现集合里。

**终端 1 —— 编译（只有改过代码才需要）**

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export TMPDIR=/home/ubuntu2004/.tmp_build
colcon build --packages-select fr3_dual_palletize --symlink-install
```

**终端 2 —— 启动 Isaac Sim（必须先有 ROS 2 环境）**

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib"
unset ROS_LOCALHOST_ONLY
/home/ubuntu2004/isaacsim-4.5.0/isaac-sim.sh
```

**Isaac Script Editor 第 1 步 —— 恢复本机控制执行器（每次重启 Isaac 后一次）**

```python
import carb
import omni.kit.app

HOST = "127.0.0.1"
PORT = 8226
EXTENSION = "isaacsim.code_editor.vscode"

settings = carb.settings.get_settings()
settings.set("/exts/isaacsim.code_editor.vscode/host", HOST)
settings.set("/exts/isaacsim.code_editor.vscode/port", PORT)

manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate(EXTENSION, True)
print(f"[Project] Isaac local executor ready at {HOST}:{PORT}")
```

**Isaac Script Editor 第 2 步 —— 停止 Timeline，重建 Task25 场景**

```python
scene_path = "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task25_batched_feed_scene.py"
exec(compile(open(scene_path, "rb").read(), scene_path, "exec"))
```

**第 3 步 —— 点击 Timeline 的 Play**

**Isaac Script Editor 第 4 步 —— 运行 bridge（它会自动释放批 1）**

```python
bridge_path = "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task25_batched_feed_bridge.py"
exec(compile(open(bridge_path, "rb").read(), bridge_path, "exec"))
```

**终端 3 —— 启动 MoveIt（全局只能有一个 `/move_group`）**

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY

ros2 launch fr3_dual_side_suction_description moveit_dual_side_suction.launch.py use_rviz:=false
```

**终端 4 —— 先单独确认“分批到料 + 到料判定”（建议先做）**

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY

ros2 topic echo /task25/feed_state
```

批 1 到料后应看到前两位为 `1`：

```text
data: [1, 1, 0, 0, 0, 0, 0, 0]
```

手动放下一批，验证瞬移与落稳：

```bash
ros2 topic pub --once /task25/feed_command std_msgs/msg/Int32 "{data: 2}"
```

```text
data: [1, 1, 1, 1, 0, 0, 0, 0]
```

**终端 5 —— 零命令预检（不发布 joint / suction / feed_command）**

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY

ros2 run fr3_dual_palletize task25_batched_side_suction --ros-args \
  -p max_batches:=4 \
  -p planning_only:=true \
  -p execution_time_scale:=3.0
```

**终端 5 —— 单批物理验收（只做批 1 两件）**

```bash
ros2 run fr3_dual_palletize task25_batched_side_suction --ros-args \
  -p max_batches:=1 \
  -p execution_time_scale:=3.0
```

**单批与完整验收之间必须重置场景**：停止 Timeline → 重新执行第 2 步的场景脚本 →
点击 Play → 重新执行第 4 步的 bridge → 再做完整验收。

**终端 5 —— 完整四批物理验收（8 件）**

```bash
ros2 run fr3_dual_palletize task25_batched_side_suction --ros-args \
  -p max_batches:=4 \
  -p execution_time_scale:=3.0
```

**终端 4（可选）—— 观察每批到料与落稳**

```bash
ros2 topic echo /task25/feed_state
```


## 待确认事项

- 工程记录写回远端分支：已确认为 `侧面吸盘紧协调码垛`。
- “单臂、无吸附、短距离推送调整 Cube 到共同可达位”的后备策略：已确认**不做**，
  本任务不含该路径，也不把“调整供料位”作为可行性的补救手段。
- 本地已有待上传提交；远端 `侧面吸盘紧协调码垛` 与本地基线分叉，必须按
  fast-forward 方式重建提交后再推，不能直接 push 或 pull/merge 到当前工作区。
- README 顶部“路线冻结（2026-09-15）：主线切换为 FR3 二指夹爪”一段是否改为
  L 型阵列侧吸口径。

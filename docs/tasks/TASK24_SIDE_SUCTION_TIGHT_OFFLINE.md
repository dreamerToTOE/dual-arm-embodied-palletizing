# Task24：侧面吸盘紧协调离线码垛

状态：🟡 单 Cube 物理基线已通过；八件静态供料全量预检第一件失败，已决定改为分批到料（Task25）（教师已确认 L 型阵列侧吸工具）

## 目标

验证两个 FR3 通过对应的左右侧面 L 型吸盘，共同搬运 `120 mm / 0.8 kg` Cube。
当前第一版是固定离线八件任务：不使用 Cube 选择器，`Cube_01` 至 `Cube_08` 的
source、entry 和 target 均已知。单件物理闭环已通过；多件阶段先做零命令预检，
随后才做一次连续物理验收。

每件 Cube 的流程为：

```text
RRTConnect 空载接近
-> 左右侧面对应 CONTACT
-> 双吸盘同步 CLOSED
-> 共同 Z 抬升
-> 共同 X 运输
-> 共同 Y 对齐
-> 共同 Z 下降到 entry
-> 共同 +X 20 mm 短推
-> 同步释放
-> 共同 Z 退出
```

吸住 Cube 后禁止 OMPL 任意绕行，负载段严格采用 `Z -> X -> Y -> Z -> X`。
空载阶段允许 RRTConnect；每个同步候选均由完整双臂 RobotState + FCL 采样验证。
为处理 FR3 的冗余肘部构型，空载阶段采样至多 8 个候选；完整负载链路预检只接受
首个全部通过 FCL 的候选，绝不以候选数量替代碰撞检查。

## 垛型与场景

- 当前 Stage 创建八个真实 Dynamic Rigid Body：`/World/Task24/Supply/Cube_01` 至
  `Cube_08`；其 PoseArray 顺序与离线任务表严格一致。
- 目标按墙优先执行：远侧 `YZ` 墙 `x=0.820` 的四个单元先完成，再执行近侧
  `YZ` 墙 `x=0.640` 的四个单元；每面均为 `y/z` 的 2×2 排列。
- Cube 边长为 120 mm、质量为 0.8 kg；不使用灰色等待区或中转台。
- 八个供料 Cube 位于桌面中部 `x=0.20/0.36/0.52/0.68`、`y=±0.12 m` 的 4×2
  网格，中心距为 160 mm、净间隙为 40 mm；避免把后续供料放到两臂共同可达范围外。
- 桌面底面落在世界 `z=0`，顶面为 `z=0.200 m`，中心和尺寸分别为
  `(0.550, 0, 0.100) m`、`(1.20, 0.80, 0.20) m`。这不是悬空桌子。
- 两个 FR3 基座固定在 `(0.650, -0.600, 0)` 与 `(0.650, +0.600, 0) m`，
  间距为 1.20 m。该外扩使侧吸构型优先采用较伸展的肘/腕姿态；每次布局改动
  仍须以完整同步 FCL 预检确认，不能假定它自动消除所有自碰。
- `/World/Task24` 可从空 Isaac Stage 重建；两个 FR3、桌面、L 型工具、物理 Cube 和 ROS bridge 均由 Task24 脚本配置。

多 Cube 阶段的前提是：Isaac 中每一个真实 Cube、bridge 发布的 PoseArray、MoveIt
Planning Scene 中的 CollisionObject 三者数量与顺序完全一致。执行器会在抓取当前件
前移除对应 World CollisionObject，在释放落稳后按最新 Isaac Ground Truth 加回；
其余未抓取与已放置 Cube 始终保留为真实障碍物。

## FR3 姿态与桌面高度依据

本轮不采用“运行失败后逐个改桌高”的方式。依据为：

- Franka 官方 `franka_ros2` 的 FR3 `move_to_start` 示例使用
  `[0, -π/4, 0, -3π/4, 0, π/2, π/4]`；Task24 的 Isaac drive target 与 MoveIt
  `home` 都采用该空载准备姿态，避免旧版 `joint4=-0.1518 / joint6=0.5445`
  贴近关节行程边界的初始构型。
- FR3 的 shoulder 高约 `0.333 m`；FR3 Duo 官方手册将 shoulder 周围约
  `0.7 m` 描述为具有良好可操作性的工作区域。桌面顶面选择 `0.200 m` 后，底层
  Cube 的侧吸接触高度是 `0.261 m`，相对 shoulder 仅低 `72 mm`；上层接触点
  `0.381 m` 也贴近 shoulder 平面。供料接触点距各自 shoulder 的水平距离约
  `0.54 m`，因此不是靠近最大伸展边界的构型。

参考：[Franka Research 3 产品页](https://franka.de/franka-research-3-arm)、
[Franka `move_to_start` 示例源码](https://github.com/frankarobotics/franka_ros2/blob/humble/franka_example_controllers/src/fr3/move_to_start_example_controller.cpp)、
[Mobile FR3 Duo 手册](https://franka.de/hubfs/Prototype%20Manual%20and%20Datasheets%20_Mobile%20FR3%20Duo_R04245_EN.pdf?hsLang=en)。

高度变更后必须重新经过本文件的 FCL 与 Isaac 物理验收；此前低桌面的通过日志不能
直接替代新工作空间的验收结果。

## 关键几何修正

早期短工具会迫使 FR3 wrist/flange 贴近 Cube 和桌面。教师确认后，Task24 只保留
下列 L 型侧吸阵列；此前的竖直顶吸外观对照版已从源码移除：

- 竖直段：80 mm
- 横向支臂：130 mm
- 阵列面板：70 x 70 mm（四个 Cup 的约 68 x 68 mm 外包络之外保留 1 mm 边缘）
- `side_suction_tcp` 相对 `fr3_link8` 的侧向偏置：155 mm

Isaac 工具、MoveIt URDF 与 bridge 的吸盘约束偏置使用同一尺寸。闭合前读取 Isaac Ground Truth，检查两个 TCP 到 Cube 的 X/Z、跨距、间隙与姿态；不满足阈值时不允许吸附。

单件基线以 Cup 面到 Cube 侧面的 `1 mm` 名义接近位姿发出命令。该 1 mm 空隙避免
在尚未建立真空 D6 约束时，Cup Collider 先把 Dynamic Cube 推偏；它仍明显小于 bridge
的 `3 mm` 捕获阈值。Isaac Ground Truth 要求左右真实间隙各不大于 `2 mm`、两侧间隙
差不大于 `1 mm`，因此不会在明显浮空时创建约束。共同载荷运输高度为 `280 mm`，以保证 L 型支架
在 `COMMON_X_TRAVEL` 全程避开桌面。最终 `COMMON_DESCENT_TO_ENTRY` 停在目标中心
上方 `20 mm`：这是 L 型阵列本体相对 Cup 面的桌面净空，而非扩大侧面吸附间隙。
同步释放后 Cube 低速落稳，必须由 Isaac Ground Truth 回读验证其最终位置；目标
堆放高度本身不变。

Cup 阵列中心在命令上相对 Cube 中心上移 `1 mm`。这仍处于 `120 mm` Cube 的侧面
有效吸附区，且已用于消除 L 型面板与桌面的亚毫米擦碰；该偏置同时应用于接触、
共同抬升、运输、下降与短推，不能只在单一阶段补偿。

双侧吸附使用 Fixed grasp（`bendAngle=0`）与相同的有限 D6 阻抗
（`stiffness=1e4`、`damping=1e3`），并保留 `1e6` 的断裂力/力矩阈值。共同负载
关节目标由同一个 Isaac physics callback 成对下发，避免两个独立 Action Graph
在不同物理帧覆盖左右目标。该修正不改变 `3 mm` 捕获阈值、不关闭任何 Cube--工具
或工具--环境碰撞，接触间隙仍受 Ground Truth 硬门限约束。

## 当前验收边界

多 Cube 验收不复用任何“Isaac 有其它真实 Cube、MoveIt 却只建模 Cube_01”的旧配置。
本轮场景、bridge 与执行器统一使用 `active_cube_count:=8`；若数量或顺序不一致，
执行器会拒绝开始。

本轮单件验收要求：双吸盘对应接触检查通过、`COMMON_LIFT -> COMMON_X_TRAVEL ->
COMMON_Y_ALIGN -> COMMON_DESCENT_TO_ENTRY -> COMMON_SIDE_SHORT_PUSH` 全部通过同步 FCL
门禁、两个吸盘稳定 CLOSED、最终 Cube Ground Truth 误差不超过 10 mm。通过前不得
开始多 Cube 连续搬运。

## Task24-H 实测记录（官方准备姿态 + 0.200 m 桌面）

在 Isaac Sim 4.5 重建场景、重新加载 bridge、重启 Task24 MoveIt 后完成一次真实单件
闭环。关键结果如下：

- Isaac 实测官方准备姿态收敛误差：左臂 `0.042°`、右臂 `0.031°`。
- `PRE_CLOSE`：TCP--Cube 的 X/Z 误差 `0.569 / 0.724 mm`；左右实际间隙
  `1.242 / 1.314 mm`，差 `0.072 mm`。
- `COMMON_LIFT`、`COMMON_X_TRAVEL`、`COMMON_Y_ALIGN`、`COMMON_DESCENT_TO_ENTRY`、
  `COMMON_SIDE_SHORT_PUSH`：均为 Cartesian fraction `1.0000`，且同步 FCL 通过。
- 最终 Ground Truth：期望 `(0.820, -0.075, 0.260) m`，实际
  `(0.820, -0.075, 0.260) m`，误差 `0.184 mm`。
- 在完全重建 Isaac 场景与 bridge 后的第二次独立运行同样通过；最终误差为
  `0.115 mm`，且正常释放明确记录为两个物理吸盘均 `OPEN`。

结论：这证明“提高桌面 + 官方 FR3 空载准备姿态”在单 Cube 侧吸紧协调任务中是可用的
工作空间基线。它不等价于多 Cube 验收；恢复其它 Cube 前仍须把所有动态障碍物同步加入
MoveIt Planning Scene 并重新验证完整垛墙。

## Task24-M 实测记录（八件静态供料全量预检）

在 Isaac 同时创建 8 件真实 Dynamic Rigid Body、bridge 发布 8 件 Ground Truth、MoveIt
Planning Scene 同时包含桌面 / 当前供料件 / 已完成码垛件的配置下，执行八件零命令预检：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=0

ros2 run fr3_dual_palletize task24_side_suction_tight --ros-args \
  -p max_cubes:=8 \
  -p active_cube_count:=8 \
  -p planning_only:=true \
  -p execution_time_scale:=3.0
```

结果：**第一件即失败**。失败原因不是放宽碰撞规则后被拒绝，而是未处理的 Cube 占据了
另一只机械臂进入“侧面中心吸附位”的通道。该次预检没有发布任何 joint 或 suction 命令，
未产生物理执行风险。

结论与边界：

- 这是几何 / 通道问题，不能用提高 TCP Z、把 TCP 上移到 Cube 边缘或扩大 ACM 来掩盖。
- 八件同时静态摆放在供料区（`SUPPLY_POSES` 的四列 X、两行 Y 镜像布局）本身是未完成的
  实验布局，不作为验收版本；它也不推翻已通过的单件物理结论。
- 下一步改为**分批到料**：一次只把当前批次两件加入 Planning Scene，到位停稳后再读取
  Ground Truth 并规划。这从源头消除静态供料互相堵塞，另立 Task25。

## 验收命令

先启动 Task24 场景和 bridge，再启动 Task24 专用 MoveIt：

> Task24 的 Isaac 内嵌 `rclpy` bridge 当前以 `ROS_LOCALHOST_ONLY=0` 运行。
> 因此 MoveIt 与执行器终端也必须统一设置为 `0`；不能和此前部分任务使用的
> `ROS_LOCALHOST_ONLY=1` 混用，否则 MoveIt 与 Task24 bridge 会落在互不可见的
> DDS 发现集合中。

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=0

ros2 launch fr3_dual_side_suction_description moveit_dual_side_suction.launch.py use_rviz:=true
```

先做八 Cube 零命令预检：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=0

ros2 run fr3_dual_palletize task24_side_suction_tight --ros-args \
  -p max_cubes:=8 \
  -p active_cube_count:=8 \
  -p planning_only:=true \
  -p execution_time_scale:=3.0
```

预检全部通过后，才执行八 Cube 连续物理验收：

```bash
ros2 run fr3_dual_palletize task24_side_suction_tight --ros-args \
  -p max_cubes:=8 \
  -p active_cube_count:=8 \
  -p execution_time_scale:=3.0
```

`execution_time_scale=1.0` 仅可用于无物理验收的快速检查；正式验收固定使用
`3.0`。单 Cube 已通过，但 Task24 仍是 🟡：这表示八件连续码垛尚未验收，而不是
重新降低已经通过的单件物理结论。

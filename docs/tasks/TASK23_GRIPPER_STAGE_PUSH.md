# Task23 — 二指夹爪暂存后平推二层验证

状态：🟡 已实现可运行原型；待 Isaac Sim 物理验收。

## 目标与范围

本任务验证教师提出的夹爪策略：不在密集目标格中直接夹取或插入，而是先把一个
Cube 放到二层旁的暂存台，再由另一只机械臂将其沿固定方向推入目标格。

```text
左臂：供料区抓取 -> 抬升 -> 暂存台释放 -> 安全退出
右臂：闭合 finger -> 暂存 Cube 后侧接触 -> +X 受控平推 -> 安全退出
```

这是一项最小可行性验证，不扩展已冻结的 Task04--Task22 吸盘路线，也不声称解决
动态多层垛整体稳定性。

## 场景和物理约束

- 第一层：`4 x 4`、30 mm Cube，中心坐标 `x={0.615,0.645,0.675,0.705}` m、
  `y={-0.045,-0.015,0.015,0.045}` m、`z=0.065` m。
- 为隔离“上层受控推送”这一问题，第一层为**静态 Collider**；其顶面为 `z=0.080` m。
- 供料区：16 个 30 mm、50 g 的**动态** Cube，Prim 为
  `/World/Task23/Supply/Supply_01...Supply_16`。
- 第二层中心高度为 `z=0.095` m。
- 暂存台：中心 `(0.565, 0, 0.065)` m，顶面 `z=0.080` m。
- 物理材料：静摩擦 `0.90`、动摩擦 `0.75`、无弹性。

因此，本任务证明的是“刚性支撑面上的上层平推”可否由现有 FR3 二指夹爪实现；第一层
Cube 之间以及下层与桌面的动力学稳定性，是下一阶段必须单独验收的对象。

## Python 选择器

`task23_gripper_push_selector.py` 只读取 Isaac Ground Truth
`/task23/box_states`，不加载 MoveIt、不发布关节命令。

它按供料 Cube ID 选择下一件，并固定输出：

```text
supply_arm = left
push_arm   = right
staging    = (0.565, target_y, 0.095)
push       = +X
```

每一行目标格的顺序为 `x=0.705 -> 0.675 -> 0.645 -> 0.615`。也就是先填推送方向
最远端，再向暂存台逐格回填，避免已放 Cube 占据后续直线推送通道。

只有执行器发布同一对象的 `TaskWorldCommit` 后，选择器才会发布下一件；因此场景状态
和 MoveIt World 状态不会在上一个 Cube 未完成时提前混合。

## 夹爪执行与碰撞策略

`task23_gripper_stage_push` 自包含地复用了 Task01 的真实 finger 接触语义：

```text
OPEN -> 垂直抓取 -> CLOSE -> AttachedCollisionObject -> LIFT
-> 暂存 -> OPEN -> Ground Truth 写回 -> RETREAT
```

双臂 description 中的 `hand_tcp` 也显式采用官方 `0.1034 m` 指尖 TCP 偏置，
与 Task01/Task03 的 Franka Hand 语义一致；不能把 TCP 错设在 hand 壳体原点，
否则相同的抓取高度会让 finger 碰撞桌面。

右臂的 TCP 高度遵循已验证的抓取关系：TCP 比 Cube 中心高 10 mm，避免 finger 下探到
暂存台内。推送阶段中，只临时从 MoveIt World 删除**当前受控推送的一个 Cube**，以允许
预期的 finger--Cube 接触；桌面、暂存台、固定第一层、已提交第二层 Cube 及另一只机械臂
始终保留在碰撞检查中。没有扩大 ACM，也没有永久忽略 Cube--robot 碰撞。

每个 RRTConnect 或 Cartesian 规划最多尝试 3 次；三次失败立即停止，绝不继续下一个
Cube。

释放与推送后的验收只接受该动作开始后收到的**新一帧** Isaac Ground Truth；不会把缓存中
的抓取前或推送前 pose 当作 settle pose。

## 验收门槛

每一件必须依次满足：

1. 暂存后 Isaac Ground Truth 与暂存点的 XY/Z 误差均不大于 10 mm；
2. 推送后 Ground Truth 与目标格的 XY/Z 误差均不大于 10 mm，yaw 误差不大于 5°；
3. 发布该 Cube 的 `TaskWorldCommit`；
4. 第一件以 `max_tasks:=1` 通过后，才运行完整 `max_tasks:=16`。

若推送因接触形状、摩擦、可达性或上层 Cube 之间的机械干涉失败，执行器会停止在当前
Cube。这是方法不可行或参数不足的证据，不能通过扩大碰撞豁免来掩盖。

## 启动与实际验收

先关闭其他 Task 的 MoveIt launch（全局 ROS 图中只能有一个 `/move_group`），然后启动
Isaac Sim：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_dual_gripper_description fr3_dual_palletize --symlink-install
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

/home/ubuntu2004/isaacsim-4.5.0/isaac-sim.sh
```

在 Isaac Script Editor 中，时间线停止时依次执行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task23_gripper_push_stack_scene.py").read())
```

点击 **Play**，再执行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task23_gripper_ground_truth_bridge.py").read())
```

在一个干净 ROS 终端，先运行单 Cube 物理门槛：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch fr3_dual_palletize task23_gripper_stage_push.launch.py \
  max_tasks:=1 execution_time_scale:=1.5 use_rviz:=true
```

预期先看到左臂抓取 `Supply_01` 并放上灰色暂存台，左臂退出；随后右臂以闭合 finger
从暂存 Cube 的 `-X` 侧向 `+X` 推入第一行最远格 `(0.705,-0.045,0.095)`。日志应包含：

```text
... stage Ground Truth PASS ...
... final Ground Truth PASS ...
... PASS: gripper pickup -> stage -> single-arm +X push -> Ground Truth commit.
Task23 PASS: 1/1 ...
```

单件通过后，停止 launch 和时间线，重新运行场景脚本、点击 Play、重新运行 Ground Truth
bridge，再运行：

```bash
ros2 launch fr3_dual_palletize task23_gripper_stage_push.launch.py \
  max_tasks:=16 execution_time_scale:=1.5 use_rviz:=true
```

完整验收要求 `16/16` 全部 commit；任一失败均应保留失败对象和误差日志，作为下一轮接触
策略调整的输入。

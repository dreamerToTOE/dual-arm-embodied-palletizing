# Task20：Multi-size Palletizing / 多尺寸连续码垛

状态：🟡 Task20-A 已完成；Task20-B 的 loose 运行时 Motion / FCL preflight 已完成。
通用紧协调 planner、真实执行与自动路由仍待后续阶段（2026-09-14）。

## 目标

在 Task17--19 的通用数据接口上，完成至少 6 类不同尺寸箱体的连续码垛，不再依赖“30 mm 小 Cube + 1 个固定大件”二分类 Demo。

## 尺寸集合

第一版至少准备 6 类尺寸，具体数值作为配置而不是写死在控制器中。

建议覆盖：

```text
小 / 中 / 大
细长 / 扁平 / 近方形
单臂明显可搬 / 双臂更合适
```

质量可先与体积按简化规则关联，后续再独立随机化。

## Episode

每个 episode：

```text
随机 Box 类型组合
+ 随机初始位置
+ Placement Planner 自动生成目标
+ Task21 选择松/紧协调模式
+ Task16 鲁棒运动规划
+ Task08/FCL 安全验证
+ 执行
```

## Task20-A：运行时多尺寸连续 Placement

新增 `task20_multi_size_episode`。它只订阅 Task18：

```text
/task18/box_states  fr3_dual_palletize/msg/BoxStateArray
```

将每个动态 `BoxState` 转为 Task17 `BoxSpec`，按体积从大到小处理，并为每个物体
调用 Task19 `PlacementPlanner`。它只生成连续 `PlacementSpec` 和统计，不启动
MoveIt、不发布关节/吸盘命令、不伪装为执行成功。

Task18 catalog 现在有六类可随机抽取的尺寸：

```text
small_cube / medium_box / slim_box / flat_box / tall_box / large_shared_box
```

场景采样器在 512 个 seed 上通过无重叠、桌边和可达 envelope 检查。每个 episode
日志记录 `episode / seed / input_boxes / payload_classes / planned /
planning_failures / candidates_total`，可按 seed 重放。

### 已验证

使用一个六类尺寸的 `BoxStateArray` fixture：

```text
episode=1
seed=20260914
input_boxes=6
payload_classes=6
planned=6
planning_failures=0
candidates_total=1322
```

所有目标由 Task19 自动产生，没有固定物体数量、尺寸或目标坐标。大件优先落在
低层；松/紧模式选择仍明确留给 Task21，不在 Task20-A 以尺寸 if/else 冒充 router。

### 运行命令

先按 Task18 启动 Isaac 场景与 bridge；随后在 ROS 终端运行：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 run fr3_dual_palletize task20_multi_size_episode --ros-args \
  -p timeout_sec:=60.0 \
  -p expected_seed:=20260912 \
  -p episode_index:=1
```

可以通过 `pallet_min_x/max_x/min_y/max_y/support_height` 覆盖自动放置区域；这些
是输入参数，不是控制器内固定目标点。

## Task20-B：运行时 Motion / FCL Preflight（loose）

新增 `task20_motion_preflight`，将每个运行时 loose 任务接入已验收组件：

```text
BoxStateArray
  -> BoxSpec
  -> Task19 PlacementSpec
  -> left/right 各自 Task16 RobustPlanner candidate
  -> 按完整候选时长选择可行臂
  -> Task08/FCL：活动臂 + 被动另一臂 + 携带物
  -> ACCEPT 或 SAFE_REJECT
```

它为每个允许 `loose_left` / `loose_right` 的箱体分别尝试允许的机械臂；每次尝试
都生成 `HOME -> PRE_PICK -> CONTACT -> ATTACH -> LIFT -> PLACE -> RETREAT -> HOME`
完整候选。只有 MoveIt 的全场景规划与 Task08/FCL 均通过时，才接受候选。一个臂的
RRTConnect 三候选全部失败时，会尝试另一允许的臂；这是基于实际完整路径可行性，而
不是按箱体类别或 y 坐标硬编码分配。

运行时随机 yaw 现在也随 `PlacementSpec` 传递到 `PalletizePrimitive` 的取放顶部姿态
与 `planned_release_pose`；旧 Task04--Task16 默认为 yaw=0，行为保持不变。

这个节点严格是 preflight：**不会发布 `joint_command`、suction command，也不会调用
执行接口。** 为避免把现有 Task12/13 的固定共享搬运误当成通用算法，收到仅允许
`tight_shared_object` 的箱体时会 `SAFE_REJECT` 并以非零状态退出；不会声称它已经
通过 loose 安全门禁。

### 验证结果

在隔离 ROS domain 中启动 Task06 双 FR3 MoveIt 环境，使用合成 HOME joint state 和
5 件不同 loose 尺寸 fixture（`tall / flat / medium / slim / small`）验证：

```text
seed=20260916
tasks=5
loose_safe=5
tight_deferred=0
executed=0
Task20-B PASS
```

每个接受的候选都输出 `Task16 robust + Task08/FCL PASS; NOT_EXECUTED`。其中某些
source/arm 组合的 RRTConnect 会在 3 次候选后超时，系统未放行该组合，而是选择另一臂
的完整可行候选。另以 `large_shared_box` fixture 验证了紧协调路径的安全拒绝：

```text
SAFE_REJECT ... modes=tight_shared_object
generic shared-object planner is not available; no trajectory accepted
```

这两个结果分别验证了“可行时接受”和“没有通用 tight solver 时拒绝”，不把不完整能力
伪装为成功。

### 运行命令

先启动 Task06 双 FR3 MoveIt；实际 Isaac 联调时必须使用 bridge 给出的真实
`/joint_states`，保持 `publish_home_joint_state:=false`：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 run fr3_dual_palletize task20_motion_preflight --ros-args \
  -p timeout_sec:=60.0 \
  -p expected_seed:=20260912
```

仅用于没有 Isaac 的 MoveIt 回归测试时，才显式设置
`-p publish_home_joint_state:=true`。该开关会发布合成 HOME 起点，因此绝不能与真实
Isaac joint-state bridge 同时使用。

## 重点指标

```text
episode success rate
box success rate
planning failure rate
IK failure rate
collision-rejection rate
coordination strategy distribution
mean makespan
mean replans
placement utilization
```

## 验收标准

```text
T20-01  至少 6 类尺寸均可由同一 BoxSpec 数据模型描述
T20-02  核心执行代码不按具体尺寸写 if/else 特例
T20-03  随机初始位置下可连续处理多个 Box
T20-04  目标由 Task19 自动生成
T20-05  每个任务均通过 Task08/FCL 或紧协调几何安全检查
T20-06  批量 episode 输出统计结果，而不是只保存单次成功演示
T20-07  失败可按 seed 复现
```

## 当前验收状态

- ✅ T20-01：六类尺寸均由同一个 `BoxSpec / BoxState` 数据模型表达。
- ✅ T20-02：Task20-A 核心按通用尺寸计算，无特定尺寸分支。
- ✅ T20-03（规划层）：随机运行时输入可连续处理所有收到的 Box。
- ✅ T20-04：每个目标由 Task19 自动生成。
- ✅ T20-06（规划统计层）：每个 episode 输出 seed 与规划统计。
- ✅ T20-07：输入 seed 随 BoxStateArray 记录，随机布局可重放。
- 🟡 T20-05：所有 loose runtime task 已通过 Task16 / Task08-FCL preflight；
  `tight_shared_object` 必须等待通用共享物 planner 后再验收，当前会安全拒绝。
- 🟡 物理执行：仍待后续执行层；不能以 Task20-A/B 的只读 PASS 代替。

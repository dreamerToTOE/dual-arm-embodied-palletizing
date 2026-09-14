# Task20：Multi-size Palletizing / 多尺寸连续码垛

状态：🟡 Task20-A 运行时多尺寸自动 placement 与统计已完成；执行、FCL 与自动路由待后续阶段（2026-09-14）。

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
- 🟡 T20-05：待 Task20-B 把每个任务接入 Task16 / Task08-FCL 或紧协调安全门禁。
- 🟡 物理执行：待 Task20-B，不能以 Task20-A 的只读 PASS 代替。

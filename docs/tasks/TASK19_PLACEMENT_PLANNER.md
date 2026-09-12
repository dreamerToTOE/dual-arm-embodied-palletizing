# Task19：Automatic Placement Planner / 自动码垛点规划

状态：🟡 几何 / height-map 第一版完成并通过离线回归；待接入 Task18 实时输入与 MoveIt 快速可达性（2026-09-12）。

## 目标

把当前人工写死的码垛目标坐标升级为由规划器根据箱体尺寸、托盘区域和当前已放置物体自动生成 `PlacementSpec`。

输入：

```text
BoxSpec[]
Pallet / support region
Current placed-object state
```

输出：

```text
PlacementSpec[]
```

## 第一版建议方法

优先使用可解释、确定性的几何 / height-map 方法，不先引入 RL。

候选生成：

```text
可用支撑平面 / height map
  ↓
生成候选 XY + yaw
  ↓
边界检查
  ↓
箱体间不重叠检查
  ↓
支撑面积 / 稳定性检查
  ↓
机械臂可达性预检
  ↓
候选评分
```

## 候选评分

可逐步加入：

```text
总高度
空隙率
支撑率
重心稳定性
机器人可达代价
预计运动距离
松/紧协调代价
未来可放置空间
```

第一版只需少量明确指标，避免一次做成复杂全局优化。

## Placement Planner 与运动规划分层

```text
Placement Planner
  只决定“箱子应该放哪里”

Robust Motion Planner
  决定“机器人怎么到那里”
```

如果某个 placement 几何上合理但机械臂不可达，应返回失败原因并尝试下一个 placement candidate，而不是让 RRTConnect 无限重试。

## 第一版实现

新增通用 `PlacementPlanner`（`placement_planner.hpp/.cpp`）和只读回归程序
`task19_placement_planner_demo`。输入为 Task17 `BoxSpec`、`PalletRegion` 与
`PlacedBox[]`，输出按 cost 从低到高排序的 `PlacementCandidate[]`，每一项含完整
`PlacementSpec`、support ratio 和 cost。

实现的可解释步骤：

```text
table height-map + 已放箱体顶面 height-map
  -> 0 / 90 deg yaw 候选
  -> 托盘边界（旋转矩形四角）检查
  -> 3D 高度区间 + 2D SAT 无重叠检查
  -> 单一支撑面完整覆盖（support ratio = 1）
  -> height + 中心距离 + support 的确定性 cost 排序
  -> 可选 reachability callback；拒绝后自动检查下一 candidate
```

这保持职责分离：Task19 决定“放哪里”；Task16 RobustPlanner 仍决定“如何运动到
那里”。`PlacementReachabilityCheck` 是后续 MoveIt 快速预检的接入点，当前 demo
用可控 callback 验证 fallback，不向 MoveIt 或 Isaac 发布命令。

针对 Task15 兼容配置，planner 得到与历史大件目标等价的
`(0.650, 0.000, 0.090)`，随后在大件顶面为四件小箱分别生成 50/46/44/42 个完整
支撑、无重叠候选。遗留 Task15 executor 保持不改，仍作为已验收基线；Task20 的
通用批处理将使用本 planner 的输出替代其固定 BatchConfig target。

## 已验证

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 run fr3_dual_palletize task19_placement_planner_demo --ros-args \
  -p job_config:=/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws/src/fr3_dual_palletize/config/task15_legacy_job.yaml
```

实际末行：

```text
Fallback PASS: rejected=28, support=task15_large_cube
Task19 PASS：自动生成 PlacementSpec；无固定目标坐标，且可达性失败会回退到下一候选。
```

## 验收状态

- ✅ T19-02：support top height 自动计算 candidate z。
- ✅ T19-03：旋转 footprint 边界、3D/2D overlap 检查。
- ✅ T19-04：候选必须完整位于单个 support top，support ratio=1。
- ✅ T19-05：生成并按确定性 cost 排序多个候选。
- ✅ T19-06：callback 拒绝 28 个 table candidate 后，自动回退到大件顶面 candidate。
- ✅ T19-07：Task15 fixture 自动产生等价大件目标和合理小件顶面布局。
- 🟡 T19-01：新通用链没有固定 target；历史 Task15 BatchConfig 为回归基线暂保留，
  将在 Task20 的通用执行器中彻底替换。

## 验收标准

```text
T19-01  目标点不再写死在 Task15 / BatchConfig 中
T19-02  能根据当前支撑物高度自动计算 z
T19-03  候选不越托盘边界且不与已放物重叠
T19-04  至少包含基本支撑稳定性判定
T19-05  可生成多个候选并按 cost 排序
T19-06  MoveIt 不可达时可回退到下一 placement candidate
T19-07  固定 Task15 场景可由 Planner 自动生成等价或合理的新布局
```

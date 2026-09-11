# Task19：Automatic Placement Planner / 自动码垛点规划

状态：⚪ 待实现。

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

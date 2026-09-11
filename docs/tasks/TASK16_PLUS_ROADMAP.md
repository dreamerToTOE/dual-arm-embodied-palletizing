# Task16+ 路线图：从固定 Demo 到鲁棒、随机、多尺寸、自主码垛

> 本路线图用于承接 Task15 之后的工程主线。Task00--Task15 保持既有验收结果不变。
>
> 核心目标：不再继续围绕固定初始位姿、固定目标点和固定两类尺寸堆叠 Demo 功能，而是逐步把系统升级为“可重复验证、可随机化、可扩展到多尺寸箱体、自动生成码垛点并自动选择松/紧协调模式”的通用双臂码垛系统。

## 总体顺序

```text
Task16  Planning Robustness
  ↓
Task17  Box / Task Parameterization
  ↓
Task18  Randomized Scene + Ground Truth Task Input
  ↓
Task19  Placement Planner
  ↓
Task20  Multi-size Palletizing
  ↓
Task21  Loose/Tight Coordination Router
  ↓
Task22  Learning-based Coordination
        （远期讨论项；当前不实施）
```

推荐原则：

1. 先提高当前 MoveIt / RRTConnect 的稳定性，再增加任务随机性；
2. 在引入相机之前，先用 Isaac Ground Truth 模拟“感知输出”，把任务规划问题与视觉问题解耦；
3. Box 尺寸、质量、初始位姿、抓取候选、目标位姿不得继续写死在 C++ 主逻辑中；
4. Task19 起，码垛点由 Placement Planner 生成，不再人工硬编码；
5. Task20 起至少支持 6 类尺寸；增加新尺寸时不修改核心执行代码；
6. 松/紧协调模式由 Task21 根据任务属性自动选择；
7. **Task22 只作为远期研究备忘，不进入当前实施计划。只有 Task08/FCL 安全链和 Task16--21 经典主线全部稳定后，且时间允许时，才重新评估是否开展 DRL。**

## Task 关系

| Task | 主题 | 关键输出 | 主线优先级 |
|---|---|---|---|
| 16 | 规划鲁棒性 | RobustPlanner / 候选评分 / IK 冗余实验 / benchmark | 最高 |
| 17 | Box 参数化 | BoxSpec / TaskSpec / 场景配置驱动 | 高 |
| 18 | 随机场景 | 随机初始位姿 + GT 输入接口 | 高 |
| 19 | 自动码垛点 | Placement Planner / support map | 高 |
| 20 | 多尺寸连续码垛 | ≥6 类尺寸、随机任务连续运行 | 高 |
| 21 | 松紧协调自动路由 | LOOSE / TIGHT 自动选择与调度 | 高 |
| 22 | DRL 协调研究 | 仅讨论：未来可输出高层动作/航点/时间协调，FCL 兜底 | **不实施 / 最后再评估** |

## 当前正式主线终点

当前正式开发主线截至 Task21：

```text
Task16
Planning Robustness
  ↓
Task17
Box / Task Parameterization
  ↓
Task18
Randomized Scene
  ↓
Task19
Placement Planner
  ↓
Task20
Multi-size Palletizing
  ↓
Task21
Loose/Tight Coordination Router
```

Task22 不计入近期交付，也不应占用当前开发时间。

## 系统最终期望结构

```text
Camera / Isaac Ground Truth
        ↓
Object State Estimator
        ↓
BoxSpec[]
        ↓
Placement Planner
        ↓
Pick Pose + Place Pose
        ↓
Coordination Router
   ↙                 ↘
LOOSE                 TIGHT
   ↓                    ↓
Robust Planner       Shared-object planner
   ↓                    ↓
Task08/FCL / geometry safety verification
        ↓
Task09 scheduling
        ↓
Execution
        ↓
Ground Truth / perception feedback
```

> 只有未来确实启动 Task22 时，才在 `Task09 scheduling` 周围增加 learning-based proposal / coordination 分支；当前经典链路保持完整独立。

## 重要边界

- RRTConnect 仍可作为当前主 planner；Task16 暂不要求引入备用 planner。
- 不采用“冻结固定轨迹”作为最终方案，因为最终 pick / place 位姿会变化。
- FR3 第 7 自由度是否限制，不预设结论；Task16 用实验比较自由 7-DOF、硬固定 1 个冗余关节、软偏好冗余姿态。
- Task16 的 RobustPlanner 是经典候选路径生成与评分模块，不依赖 DRL。
- Task22 中讨论的 waypoint/timing policy 若未来实施，定位为“主动提出候选轨迹骨架”，不是简单替代 Task16 的路径评分器。
- 学习策略输出的任何候选轨迹，在真正执行前仍必须通过几何/碰撞安全门禁。
- **Task22 当前状态为 discussion only：不编码、不训练、不阻塞 Task16--21。**

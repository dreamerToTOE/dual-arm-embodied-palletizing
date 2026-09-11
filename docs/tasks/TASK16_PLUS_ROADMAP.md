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
Task22  Learning-based Coordination（研究增强线，可选）
```

推荐原则：

1. 先提高当前 MoveIt / RRTConnect 的稳定性，再增加任务随机性；
2. 在引入相机之前，先用 Isaac Ground Truth 模拟“感知输出”，把任务规划问题与视觉问题解耦；
3. Box 尺寸、质量、初始位姿、抓取候选、目标位姿不得继续写死在 C++ 主逻辑中；
4. Task19 起，码垛点由 Placement Planner 生成，不再人工硬编码；
5. Task20 起至少支持 6 类尺寸；增加新尺寸时不修改核心执行代码；
6. 松/紧协调模式由 Task21 根据任务属性自动选择；
7. 深度强化学习优先作为高层协调或轨迹/航点生成增强器，不直接替换 Task08/FCL 安全验证。

## Task 关系

| Task | 主题 | 关键输出 | 主线优先级 |
|---|---|---|---|
| 16 | 规划鲁棒性 | RobustPlanner / 候选评分 / IK 冗余实验 / benchmark | 最高 |
| 17 | Box 参数化 | BoxSpec / TaskSpec / 场景配置驱动 | 高 |
| 18 | 随机场景 | 随机初始位姿 + GT 输入接口 | 高 |
| 19 | 自动码垛点 | Placement Planner / support map | 高 |
| 20 | 多尺寸连续码垛 | ≥6 类尺寸、随机任务连续运行 | 高 |
| 21 | 松紧协调自动路由 | LOOSE / TIGHT 自动选择与调度 | 高 |
| 22 | DRL 协调研究 | policy 输出高层动作/航点/时间协调，FCL 兜底 | 研究增强 |

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
Task09 scheduling / future learning-based coordinator
        ↓
Execution
        ↓
Ground Truth / perception feedback
```

## 重要边界

- RRTConnect 仍可作为当前主 planner；Task16 暂不要求引入备用 planner。
- 不采用“冻结固定轨迹”作为最终方案，因为最终 pick / place 位姿会变化。
- FR3 第 7 自由度是否限制，不预设结论；Task16 用实验比较自由 7-DOF、硬固定 1 个冗余关节、软偏好冗余姿态。
- 学习策略输出的任何候选轨迹，在真正执行前仍必须通过几何/碰撞安全门禁。
- Task22 不阻塞 Task16--21 主工程路线。

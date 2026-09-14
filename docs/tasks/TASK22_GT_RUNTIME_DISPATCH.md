# Task22：Ground-Truth 在线任务分派与预规划框架

状态：🟡 架构已冻结；Task22-A 将先搭建只读的 ROS2 分派接口，不接入相机、不会执行机器人。

## 背景与目标

Task20-D 的串行执行器在每件物体落稳后，才为下一件依次枚举 placement candidate、
左右臂完整 MoveIt 轨迹和 FCL。这个保守流程正确地保护了运行时场景一致性，但把高代价
的 RRTConnect 重复放在物理执行空档中，造成紧协调结束后明显的等待。

Task22 不引入相机。Isaac 现有 `/task18/box_states` Ground Truth 是本阶段唯一的物体
输入；它在接口上等价于未来的感知结果，但不会把相机检测、深度误差和遮挡问题混入
调度/规划效率实验。

目标是将“选什么、放哪里、谁执行”移到 Python 高层分派器，令 MoveIt 只为已选择的
候选执行最终运动规划与安全验证。

```text
Isaac Ground Truth (/task18/box_states)
        + TCP Ground Truth
                 |
                 v
Python Dispatch Selector
  object + placement candidate + preferred arm / tight mode
                 |
                 v
MoveIt Dispatch Gateway
  scene-version / freshness / IK / one primary-arm plan / FCL
                 |
                 v
Task08-B / Task09-B,C
  full-task spatiotemporal FCL -> PASS / local wait / time scaling / replan
                 |
                 v
Isaac execution -> settle Ground Truth -> World Commit -> next dispatch
```

## 计算边界与效率原则

Python selector 只做低成本计算：

```text
observation freshness
box 类型与允许模式
placement candidate 的离线/几何 cost
TCP -> pick 的欧氏距离
pick -> target 的欧氏距离
简单的队列、负载和工作区惩罚
```

这些是 `O(N * K * A)` 的标量运算（`N` 个物体、`K` 个候选、`A<=2` 个手臂），通常为
微秒至毫秒级；它们不调用 IK、OMPL 或 FCL，不会成为当前瓶颈。

TCP 到位“估计时间”也只使用距离除以保守名义速度，作为排序特征，不是仿真或轨迹规划。
真实 TCP 到位仍由 Task20-E 的 release feedback gate 负责。

MoveIt 的高代价部分改为：

```text
1. 先只为 Python 选择的 primary arm 规划完整轨迹；
2. 该轨迹通过 Task08-B FCL 后即执行；
3. primary arm 规划/验证失败，才为 fallback arm 或下一 placement candidate 规划；
4. 不再对每一件默认同时构建左右两条完整 RRTConnect 候选。
```

这不会删除安全门禁，只避免为了路由评分而预先计算必然不会执行的一条完整轨迹。

## 分派决策契约

Task22-A 新增 typed ROS2 合同，Python 不能直接发布关节或吸盘命令：

```text
TaskDispatchCandidateArray
  scene_version
  candidates[]:
    object_id
    pick_pose / target_pose / support_surface_id
    placement cost
    allowed coordination modes

TaskDispatch
  scene_version
  object_id
  target_pose
  mode = LOOSE | TIGHT_SHARED_OBJECT
  preferred_arm = left | right | none
  fallback_arm = left | right | none
  selector score / policy name
```

每个决定必须附带 `scene_version`。MoveIt Gateway 在接收时检查物体 pose、已放置支撑面和
双臂状态是否仍与版本一致；若不一致，拒绝旧决定、刷新候选，不执行陈旧轨迹。

## Python 选择规则（第一版）

对 loose 物体，先用下面的可解释低成本评分选臂；并不是仅用“离哪只臂近”。

```text
score(box, target, arm) =
  w1 * estimated_tcp_to_pick_time
+ w2 * estimated_pick_to_target_time
+ w3 * placement_cost
+ w4 * workspace/queue penalty
+ w5 * fallback penalty
```

其中 workspace/queue penalty 只使用预定义工作区与另一臂的已保留任务状态。MoveIt Gateway
仍要检查实际 IK、关节限位、碰撞和 payload；Python 的选择只是让昂贵规划优先从最可能成功
的一臂开始。

对仅允许 `tight_shared_object` 的物体，selector 输出 `TIGHT_SHARED_OBJECT` 与
`preferred_arm=none`，复用 Task11--13 / Task21 的共同物体 Planner 与 Executor，绝不把
共同搬运错误降级成单臂“最近”规则。

## 双臂冲突处理：复用既有 Task

Task22 不新建碰撞策略：

```text
选定单臂轨迹 + 另一臂未来轨迹
       |
Task08-B full-task 10 ms FCL
       |
       +-- SAFE      -> 执行
       +-- 时间重叠  -> Task09-B/C：最小 local wait 或合法 time scaling
       +-- 空间冲突  -> fallback arm / 下一 placement candidate / MoveIt replan
```

单纯“降速”只对时间重叠有效。若两条几何路径本身相交，必须更换臂、目标或路径；不得通过
扩大 ACM、忽略 cube-robot 碰撞或盲目延长执行时间绕过。

## 预规划与真实回写

紧协调物体执行期间，Gateway 可以基于其 **预测的** release pose 与 terminal joint state
为下一件 loose 物体后台预规划 primary-arm trajectory。共同物体落稳后：

1. 用最新 Isaac Ground Truth 比较已放置物 pose 和预测 pose；
2. 误差在阈值内时，只做 scene-version / FCL / current-joint 轻量复核，复用缓存轨迹；
3. 误差超阈值时，废弃缓存并从真实状态局部重规划。

因此“预规划”绝不跳过 Task20-E 真实 release gate、World Commit 或 FCL。Task22-A 先建立
数据契约与只读选择器；后台线程和缓存复用属于 Task22-C，必须在接口验证后实施。

## 实施阶段

```text
Task22-A  Ground Truth Dispatch Contract
  - Candidate / Dispatch typed message
  - Task19 placement candidate provider（只读）
  - Python selector（只发布 decision；不执行）

Task22-B  MoveIt Dispatch Gateway
  - scene-version / freshness / primary-arm-first final validation
  - fallback arm only after primary failure
  - no physical command before preflight PASS

Task22-C  Predictive Pipeline
  - tight execution期间后台预规划下一 loose primary-arm candidate
  - settle后轻量复核或失效重规划

Task22-D  Runtime dual-arm execution
  - 复用 Task08/09 FCL、local wait、time scaling
  - 运行时连续多对象物理验收
```

## 非目标

- 本 Task 不加入 RGB、深度相机、检测网络或相机标定；
- 本 Task 不使用 DRL；
- Python selector 不替代 MoveIt、FCL、Task20-E TCP release gate；
- 不修改 Task20-D 的失败结论，也不把 Task22-A 的只读输出当作物理执行通过。

## Task22-A 验收

1. `/task18/box_states` 的每个 runtime box 都能生成带 `scene_version` 的候选；
2. Python selector 对同一 observation snapshot 只发布一个可重复的 `TaskDispatch`；
3. loose 决策含主臂与 fallback 臂；tight 物体输出 `TIGHT_SHARED_OBJECT`；
4. selector 全程不创建 MoveGroup、不调用 OMPL/FCL、不发布 joint/suction command；
5. 输出候选数量、selector wall time 和最终 decision，证明高层筛选不成为性能瓶颈。

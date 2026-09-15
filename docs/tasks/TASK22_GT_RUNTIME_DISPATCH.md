# Task22：Ground-Truth 在线任务分派与预规划框架

状态：🟡 Task22-A 已实现并通过隔离 ROS2 回归；Task22-B 的 primary-arm-first
MoveIt/FCL 预检已实现、编译并完成隔离启动检查。两者均不执行机器人；Task22-B 尚未进行
真实 MoveIt/Isaac 规划验收。

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

## Task22-B 已实现边界

`task22_moveit_dispatch_gateway` 仅在 launch 参数
`enable_moveit_preflight:=true` 时启动。它订阅 transient-local candidate/dispatch
snapshot，并且：

1. 必须先匹配同一个 `scene_version`、`candidate_id`、物体 ID 与支撑面 ID；
2. 将 snapshot 中每个唯一 runtime box 回写为 MoveIt collision object；
3. loose 物体仅对 Python 指定的 primary arm 生成一次完整候选（快速配置为
   `candidate_count=1`、2 s RRTConnect 预算）；只有 primary 被 MoveIt 或 10 ms
   full-task FCL 拒绝时，才调用 fallback arm；
4. tight 物体不走最近手臂规则，直接复用 `SharedObjectPlanner` 的双臂共同物体规划与 FCL；
5. 输出 `ACCEPT` 或 `REJECT`，**不发布关节轨迹、吸盘命令或执行请求**。

Gateway 启动时会通过 `/move_group` 参数服务复制 `robot_description` 与
`robot_description_semantic` 到自身节点（复用 Task20 已验证方式）。若 MoveIt 尚未启动，
它会在 10 秒后清晰报错并安全退出，不再由 `MoveGroupInterface` 因缺失 SRDF 直接崩溃。

这个 Gateway 会调用 `PlanningSceneInterface::applyCollisionObjects()` 建立预检 collision
world；因此它不能与 Task20 物理执行器并行运行。当前只允许在一个空闲、已启动的 MoveIt
实例上做预检。真正执行、World Commit 和 release feedback 仍属于 Task22-D。

已完成的静态验证：

```text
colcon build --packages-select fr3_dual_palletize --symlink-install   PASS
ros2 launch ... enable_moveit_preflight:=true（隔离域、无输入）      PASS
```

这只证明节点、消息和启动依赖正确；在获得真实 MoveIt planning/FCL 日志之前，Task22-B
不能标记为规划通过。

## Task22-A 已实现组件

```text
/task18/box_states
    -> task22_gt_candidate_provider（C++）
    -> /task22/dispatch_candidates
    -> task22_gt_dispatch_selector（Python）
    -> /task22/dispatch
```

- `TaskDispatchCandidate.msg`、`TaskDispatchCandidateArray.msg`、`TaskDispatch.msg`：
  typed ROS2 合同；candidate 与 decision 都带 `scene_version`。
- `task22_gt_candidate_provider`：复用 `boxStateToBoxSpec()`、`composePose()` 与
  Task19 `PlacementPlanner`，输出每件最多三个经过几何 placement 门禁的 preview
  candidate。它不链接 MoveIt 库。
- `task22_gt_dispatch_selector`：Python 节点订阅 candidate 与双 suction TCP Ground
  Truth，按距离/名义时间/placement cost/workspace penalty 评分。对 loose 输出
  `preferred_arm` 和 `fallback_arm`；对 tight 输出 `TIGHT_SHARED_OBJECT` 和两个
  `none`。它不导入 MoveIt，且没有 joint/suction publisher。
- candidate 与 decision 使用 transient-local QoS；Task22-B Gateway 即使晚启动也能读取
  最后一个 snapshot，并根据 `scene_version` 决定接受或拒绝。
- selector 对 candidate 的订阅同样是 transient-local：即使 `/task18/box_states` 在 launch
  过程中先到达、provider 已先发布 snapshot，selector 仍会补收当前候选，随后等待 TCP
  Ground Truth 再发布 dispatch。这个启动顺序回归已经单独验证。

provider 为生成完整 batch preview 会以 Task19 的当前最优 candidate 暂时构建后续物体的
height-map；这不是物理 World Commit。Task22-B 收到 Python 选中的 candidate 后仍必须以
真实已放置物、真实关节状态和当前 scene version 重算/验证，不可直接执行 preview。

## Task22-A 隔离回归结果

在独立 `ROS_DOMAIN_ID` 中发布手工 `BoxStateArray` 和 TCP Ground Truth，不存在 MoveIt、
FCL、Isaac joint 或 suction publisher：

```text
loose box（最近一次复测）:
  provider placement_wall = 0.207 ms
  selector_wall           = 157.2 us
  result                  = LOOSE, primary=left, fallback=right

tight shared box:
  provider placement_wall = 0.148 ms
  selector_wall           = 95.7 us
  result                  = TIGHT_SHARED_OBJECT, primary=none, fallback=none
```

这些数字仅说明高层候选生成与 Python 评分不是效率瓶颈；它们不是 MoveIt 规划时间，
更不是物理执行通过结论。

另做了“provider 先发布、selector 后启动”的 QoS 回归：provider 先生成
`scene_version=1` 的 snapshot，selector 后启动时先输出等待 TCP 的提示；收到 TCP 后输出
`Task22-A DISPATCH READY`。因此 Ground Truth bridge、provider、selector、Gateway 的启动
先后不再影响分派。

## Task22-A Isaac 预览步骤

本阶段不需要启动 MoveIt。Isaac 已加载任意 Task18 Ground Truth 场景后，保持 Timeline
**Play**，依次运行已有 bridge：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
          "task18_ground_truth_bridge.py").read())

exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
          "task20_runtime_suction_bridge.py").read())
```

另开终端：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch fr3_dual_palletize task22_gt_dispatch_preview.launch.py
```

检查输出：

```bash
ros2 topic echo /task22/dispatch --once
ros2 topic echo /task22/dispatch_candidates --once
```

预期看到 `Task22-A CANDIDATES READY` 和 `Task22-A DISPATCH READY`。本 launch 不会
发送 `/left/joint_command`、`/right/joint_command`、`/task20/*/suction_command`，所以可
在 Isaac 正常运行时安全地做只读验证。

## Task22-B MoveIt 预检步骤（暂不执行机器人）

只在 Task22-A 输出正确、且没有运行 Task20 执行器时进行。先按上节启动 Isaac 的两个
Ground Truth bridge，并保持 Timeline **Play**。另开 MoveIt 终端：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch fr3_dual_compact_suction_description \
  moveit_dual_compact_suction.launch.py
```

再开分派预检终端：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch fr3_dual_palletize task22_gt_dispatch_preview.launch.py \
  enable_moveit_preflight:=true
```

预期日志为 `Task22-B LOOSE ACCEPT`、`Task22-B TIGHT ACCEPT` 或明确的 `REJECT` 原因，且
整个过程中 Isaac 中的关节和吸盘都不应运动。此步骤是后续 Task22-C 缓存规划的安全基线，
不是物理执行验收。

## 非目标

- 本 Task 不加入 RGB、深度相机、检测网络或相机标定；
- 本 Task 不使用 DRL；
- Python selector 不替代 MoveIt、FCL、Task20-E TCP release gate；
- Task22-B 尚未取得真实 MoveIt/Isaac preflight PASS，不把隔离启动成功误记为规划通过；
- 不修改 Task20-D 的失败结论，也不把 Task22-A 的只读输出当作物理执行通过。

## Task22-A 验收

1. `/task18/box_states` 的每个 runtime box 都能生成带 `scene_version` 的候选；
2. Python selector 对同一 observation snapshot 只发布一个可重复的 `TaskDispatch`；
3. loose 决策含主臂与 fallback 臂；tight 物体输出 `TIGHT_SHARED_OBJECT`；
4. selector 全程不创建 MoveGroup、不调用 OMPL/FCL、不发布 joint/suction command；
5. 输出候选数量、selector wall time 和最终 decision，证明高层筛选不成为性能瓶颈。

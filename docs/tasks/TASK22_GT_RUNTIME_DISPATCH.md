# Task22：Ground-Truth 在线任务分派与预规划框架

状态：🟡 Task22-A 已实现并通过隔离 ROS2 回归；Task22-B 的 primary-arm-first
MoveIt/FCL 预检已在真实 MoveIt + Isaac Ground Truth 环境完成 tight shared-object
预检通过；Task22-C1 的预测缓存 Ground Truth/终端状态门禁已通过无执行回归；Task22-C2
独立 planning sandbox 的场景隔离、实时状态接入和一件 loose object 的完整候选生成已通过。
当前先以**稳定串行执行**为主线：每件真实 release/settle/GT World Commit 后才为下一件
调用 MoveIt/FCL。运行中 tight attachment 的预测状态复制与 sandbox lookahead 暂缓，作为
后续性能优化；Task22-D 的连续多对象物理执行仍未完成验收。

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

紧协调物体执行期间，系统最终需要基于其 **预测的** release pose 与 terminal joint state
为下一件 loose 物体后台预规划 primary-arm trajectory。共同物体落稳后：

1. 用最新 Isaac Ground Truth 比较已放置物 pose 和预测 pose；
2. 误差在阈值内时，只做 scene-version / FCL / current-joint 轻量复核，复用缓存轨迹；
3. 误差超阈值时，废弃缓存并从真实状态局部重规划。

因此“预规划”绝不跳过 Task20-E 真实 release gate、World Commit 或 FCL。Task22-C1 已先
实现 cache 复用门禁；真正后台规划必须使用与执行器隔离的 MoveIt planning sandbox，不能
将预测 object pose 写入正在执行的 `/move_group` shared planning scene。

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

Task22-C1 Predictive Cache Contract  ✅
  - cache entry 绑定 source scene_version、release/source pose 与双臂 terminal q
  - settle后按最新 Ground Truth 判定 cache hit / invalidate；hit 仍需最终 FCL

Task22-C2 Isolated Predictive Pipeline  🟡
  - 独立 MoveIt planning sandbox 的 scene isolation 与全局 joint-state 接入已通过
  - execution world snapshot -> sandbox 的一件 loose 完整候选生成已通过（非执行）
  - 暂缓：tight execution期间复制预测 terminal/attachment 并预规划 next loose candidate
  - settle后轻量复核或失效重规划；不得改写执行 `/move_group` 的 shared scene

Task22-D  Runtime dual-arm execution
  - 当前主线：真实 GT 串行 dispatch -> MoveIt/FCL -> execution -> release/settle/World Commit
  - 复用 Task08/09 FCL、local wait、time scaling；不复用预测轨迹
  - 稳定通过连续多对象物理验收后，才回到 C2/C3 做性能优化
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
`robot_description_semantic` 到专用的 MoveIt worker node（复用 Task20 已验证方式）。若
MoveIt 尚未启动，它会在 10 秒后清晰报错并安全退出，不再由 `MoveGroupInterface` 因缺失
SRDF 直接崩溃。即使 launch 已预声明同名空参数，Gateway 也会用 `/move_group` 的有效
字符串覆盖；启动日志会输出 `RobotModel copied ... URDF=... bytes SRDF=... bytes` 作为验收
证据。

Task22-B 启动/并发问题修复记录：

```text
问题 1：transient-local 的 candidate/dispatch 可早于模型参数复制到达。
风险   ：回调在空 SRDF 下构造 MoveGroupInterface，导致
         robot_description_semantic 缺失并异常退出。
修复   ：robot_model_ready gate 只缓存早到消息；完成有效 URDF/SRDF 写入后才开始预检。

问题 2：在 Gateway 的单线程订阅回调内同步执行规划时，MoveIt 的
         /joint_states listener 无法被调度，getCurrentState() 只看到时间戳 0。
修复   ：Gateway 与持有 MoveGroupInterface 的 worker node 分离，使用三线程 executor；
         dispatch callback 采用独立可重入 callback group，worker node 可并行接收
         /joint_states。预检期间仍不发布任何执行命令。
```

这个 Gateway 会调用 `PlanningSceneInterface::applyCollisionObjects()` 建立预检 collision
world；因此它不能与 Task20 物理执行器并行运行。当前只允许在一个空闲、已启动的 MoveIt
实例上做预检。真正执行、World Commit 和 release feedback 仍属于 Task22-D。

已完成的验证：

```text
colcon build --packages-select fr3_dual_palletize --symlink-install   PASS
真实 MoveIt + Isaac Ground Truth：
  RobotModel copied from /move_group: URDF=35800 bytes, SRDF=6330 bytes
  Task21 tight：6 个阶段全部 MoveIt/Cartesian/FCL PASS
  Task22-B TIGHT ACCEPT: object=task18_box_01, stages=6,
    duration=21.827 s, FCL_samples=856, wall=1.189 s, NOT_EXECUTED
```

该结果证明 Task22-B 可以从 Ground Truth 经 Python 决策进入真实 MoveIt/FCL 预检，且不会
发送机器人或吸盘命令；它不是 Task22-D 的连续多对象物理执行验收。

## Task22-C1 预测缓存门禁

`predictive_plan_cache` 是不依赖 MoveIt action 的线程安全库。它只保存已经由未来独立
planning sandbox 生成的 `TaskTrajectoryCandidate`，并为该候选保存：

```text
source scene_version
完成的 tight object 预测 release pose
lookahead loose object 预测 source pose
left/right 预测 terminal joint positions
selected arm
```

release 后最新 runtime snapshot 必须全部满足以下条件才能得到 `HIT`：

1. `scene_version > source_scene_version`，确认已收到 release 后的新 GT World；
2. completed object 的实际 release pose 与预测 pose 在 5 mm / 0.05 rad 内；
3. lookahead object 的 source pose 未漂移超过同一阈值；
4. left/right 各七个关节都在预测 terminal state 的 0.01 rad 内。

任一条件失败都会返回明确 `INVALIDATE` 原因；即使 `HIT`，缓存候选仍必须以最新 world
objects 和 current joint state 通过最终 Task08-B FCL，才可提交给未来执行器。

已完成无执行回归：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run fr3_dual_palletize task22_predictive_cache_demo
```

结果：`CACHE_HIT`、`STALE_SCENE`、`RELEASE_DRIFT`、`SOURCE_DRIFT` 与
`JOINT_DRIFT` 五种路径均得到预期 verdict，最终输出 `Task22-C1 PASS`。本回归不连接
Isaac、MoveIt action、joint command 或 suction command。

### Task22-C2：独立 predictive planning sandbox

当前 `PalletizePrimitive` / `SharedObjectPlanner` 通过 `PlanningSceneInterface` 更新
`/move_group` 的 shared scene。若在紧协调物理执行期间把预测 release object pose 写入该
scene，将污染执行器的碰撞世界；因此不能伪装成“后台加速”。

已新增 `task22_predictive_sandbox.launch.py`：它只启动 namespace 为
`/task22_sandbox` 的第二个 `move_group`。sandbox 仅 remap 读取 Isaac 的全局
`/joint_states`、`/tf`、`/tf_static`；其 action、service 和 planning scene 全在私有
namespace，且 `allow_trajectory_execution=false`。它不启动 robot_state_publisher、RViz、
环境发布器、joint bridge 或控制器。

`PrimitiveConfig` 与 `SharedObjectPlanConfig` 新增 `move_group_namespace` 和
`planning_scene_namespace`。默认空字符串保持所有既有 Task 使用执行 `/move_group` 的行为；
未来后台候选必须显式设为 `/task22_sandbox`，从而令 `MoveGroupInterface` 与
`PlanningSceneInterface` 都不改写执行规划场景。

已完成实际隔离探针：只向 sandbox 写入并删除远离工作区的临时 `CollisionObject`，确认
execution `/move_group` 完全不可见；随后 sandbox 的 `MoveGroupInterface` 成功读取全局
`/joint_states` 的 7 个 `left_arm` 关节。探针不调用 `plan()` / `execute()`，也不发布
joint/suction command。

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

# 需在既有执行 MoveIt 和 Isaac joint bridge 已启动时，另开一个终端运行：
ros2 launch fr3_dual_palletize task22_predictive_sandbox.launch.py

# 再开一个终端：
ros2 run fr3_dual_palletize task22_predictive_sandbox_probe \
  --ros-args -p sandbox_namespace:=/task22_sandbox
```

实际输出：`Task22-C2 PASS：sandbox scene 与 execution /move_group 隔离，且已读取 7 个
left_arm joint states；未调用 plan/execute 或发布任何控制命令。`

### Task22-C2C：execution snapshot 到 sandbox candidate

新增 `task22_predictive_sandbox_candidate_demo`，它订阅 Task22-A 的
`/task22/dispatch_candidates`，按 `object_id` 和 `selected_arm` 选择一件具有
`loose_left` / `loose_right` 权限的候选。它先只读获取执行 `/move_group` 的 world
CollisionObject；清空并镜像到 `/task22_sandbox` 后，复用 `PalletizePrimitive` 生成完整
`HOME -> PRE_PICK -> CONTACT -> LIFT -> PLACE -> RETREAT -> HOME` 候选。

候选 primitive 的 `move_group_namespace`、`planning_scene_namespace` 都强制为
`/task22_sandbox`，控制主题也指向没有 bridge 订阅者的 `/task22/sandbox_never_*` 名称。
因此它可以调用 sandbox 的 `plan()` / Cartesian 服务，但无法执行，且不会发布 joint/suction
command。完成后会把 sandbox object 恢复为镜像快照，并断言 execution world object ID 集合
没有变化。

安全边界：若 execution scene 已含 `AttachedCollisionObject`，C2C 会明确拒绝，而不是把
真实紧协调抓持物漏掉后仍生成“安全”候选。下一阶段 C3 需要从 tight planner 输出的预测
release pose、left/right terminal q 和 attachment 语义构建完整 predicted snapshot，才允许在
紧协调物理执行期间运行。

无执行集成回归中，Task18 Ground Truth bridge 当时未运行，因此通过 ROS2 发布了一个与
`BoxStateArray` 合同一致的确定性 `task18_box_03` fixture，并持续发布带有效当前时间戳的
双臂 joint-state fixture。真实 `/move_group` 与 sandbox MoveIt 均在运行；结果为：

```text
Task22-C2C SNAPSHOT READY: execution world objects=4 -> /task22_sandbox
Task22-C2C PASS: task18_box_03
  points   = 232
  duration = 23.087 s
  events   = 4
  execution /move_group world 未被写入
  未调用 execute，未发布 joint/suction command
```

这证明 snapshot、namespace 路由、真实 OMPL/Cartesian candidate 生成和清理链路成立，
但**不是 Isaac 现场验收**，也不构成 tight 期间后台预测通过结论。现场回归必须由 Isaac
Timeline Play 后的 `/task18/box_states` 与实时 `/joint_states` 驱动，绝不能使用 fixture。

现场回归命令如下（已有执行 MoveIt、Isaac joint bridge 和 Task18 Ground Truth bridge 时）：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

# 终端 1：独立、不可执行的 sandbox
ros2 launch fr3_dual_palletize task22_predictive_sandbox.launch.py

# 终端 2：由真实 GT 生成 Task19 placement candidate；不要开 MoveIt preflight
ros2 launch fr3_dual_palletize task22_gt_dispatch_preview.launch.py

# 终端 3：只在 sandbox 内生成候选
ros2 run fr3_dual_palletize task22_predictive_sandbox_candidate_demo \
  --ros-args \
  -p sandbox_namespace:=/task22_sandbox \
  -p object_id:=task18_box_03 \
  -p selected_arm:=left
```

`task22_predictive_sandbox.launch.py` 的 sandbox node 使用独立名称
`/task22_sandbox/sandbox_move_group`；服务/action 仍位于 `/task22_sandbox`。在本机
MoveIt Humble 上，手工 SIGINT 停止第二个 `move_group` 曾出现一次第三方
`class_loader` 析构警告并以 `-11` 退出，另一次为正常 SIGINT `-2` 退出；两次都发生在
候选回归已完成后，执行 `/move_group` 不受影响。该非确定性 teardown 问题已记录，不能
作为 C2C 运行时规划通过或失败的依据；后续需在隔离 ROS domain 中复测 MoveIt lifecycle。

因此 C2 的**隔离安全前置条件**和空闲场景 candidate integration 已经成立；下一步才可在该
sandbox 中创建 tight 的 predicted world。C1 仍负责在真实 settle 后按 Ground Truth/terminal
state 决定缓存命中或失效，命中后仍必须通过最终 Task08-B FCL。

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
- 不修改 Task20-D 的失败结论，也不把 Task22-A 的只读输出当作物理执行通过。

## Task22-A 验收

1. `/task18/box_states` 的每个 runtime box 都能生成带 `scene_version` 的候选；
2. Python selector 对同一 observation snapshot 只发布一个可重复的 `TaskDispatch`；
3. loose 决策含主臂与 fallback 臂；tight 物体输出 `TIGHT_SHARED_OBJECT`；
4. selector 全程不创建 MoveGroup、不调用 OMPL/FCL、不发布 joint/suction command；
5. 输出候选数量、selector wall time 和最终 decision，证明高层筛选不成为性能瓶颈。

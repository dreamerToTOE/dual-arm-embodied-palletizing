# TASK28 Stage B0：单臂重抓与 +X 推送

## 2026-11-03 快照与规划复核：B0_PARTIAL

在 `02629ef` 上复跑聚焦测试：Python **55 passed**；C++ 快照合约及隔离 ROS 域的订阅缓存测试通过；完整场景 FCL 重放再次得到实测起点无碰撞、旧名义起点碰撞深度 **1.43391 mm**、人工插值第 34/100 步首次碰撞，旧侧压终点的简化几何相交 **3 mm**。这些是离线及合成消息结果，不能证明 Isaac 发布器到执行器的整条在线链或侧压物理执行。物理源结论仍依据下述原场景步后诊断，本次没有重复该诊断。

后续非穿入预接触轨迹的完整场景采样检查已通过，但它尚未覆盖接触推送或完整侧压候选路径；现有 B0 保护门仍在实际侧压规划前中止。因此本次未运行有条件的完整 GUI 实验，未变更原验收阈值或碰撞模型。继续保持 **`B0_PARTIAL`**；下一步需在保留原几何及安全约束下解决接触阶段的表示并验证完整侧压路径，之后才能检查完整物理循环。

## 后续离线接触规划进展

[Stage B 接触感知规划基础](TASK28_STAGE_B_CONTACT_AWARE_PLANNING.md) 已在完整静态规划场景中生成非穿入的自由接近与预接触对齐轨迹；对旧穿入终点和路径中途碰撞的负对照均被拒绝。这不代表已完成物理侧压、力闭环、最终释放或退避；本阶段仍为 **`B0_PARTIAL`**，旧 Cube 移除保护门继续生效。

## 2026-11-03 步后物理快照与侧压接触冲突：B0_PARTIAL

**结论：原场景的步后物理源通过有界自由运动交叉检查；完整侧压路径未通过前置验证，未运行完整 GUI 循环。** B0 新增了明确的 `TASK28_B0_SIDE_PATH_UNVERIFIED` 中止：侧压候选预检和执行规划均在现有代码移除 Cube 前拒绝继续，而本工单要求完整场景路径检查。该中止是暂时的失败状态，不代表侧压或整循环成功。

### 物理步状态源

在原 Task27 场景的休眠供料区，对 Cube_01 施加一次 0.020 m/s 初速度，采样 100 个 `subscribe_physics_on_step_events(..., False, 0)` 步后回调，未执行抓取、码垛或完整物理循环。Cube 通过物理状态演化移动 32.998 mm；该诊断没有写 Cube 位姿。两个受检句柄/API 为 `SingleRigidPrim` 的 rigid-view PhysX tensor `get_transforms`，以及 `omni.isaac.dynamic_control.get_rigid_body_pose`。两个 API 途径共享 PhysX 仿真后端，不是两个独立仿真器。

| 诊断 | 物理句柄 | tensor 与 dynamic-control 最大 XYZ 差 | USD 与 dynamic-control 最大 XYZ 差 | 同步快照采集最长墙钟时间 |
|---|---|---:|---:|---:|
| [v1 自由运动](../../offline/task28_task27_integration/evidence/task28_b0_poststep_probe_v1/summary.json) | 100/100 有效 | 0 mm | 0.333 mm | 未采集双臂快照 |
| [v2 双臂同帧采集](../../offline/task28_task27_integration/evidence/task28_b0_poststep_probe_v2/summary.json) | 100/100 有效 | 0 mm | 0.667 mm | 0.560 ms |

[v2 逐步原始样本](../../offline/task28_task27_integration/evidence/task28_b0_poststep_probe_v2/poststep.jsonl)包含步号、按回调 dt 累计的仿真时间、墙钟、有效句柄、tensor 位姿、另一 API 位姿和速度、USD 位置，以及两个 PhysX articulation 的七关节值。USD 在此条件下存在可测落后；旧负载/接触运行的 0.904 mm 差异和更大瞬态的具体机制仍未独立复现，不能据此追认旧验收通过。候选物理源在读取前检查 rigid 和两臂 articulation 的 physics handle，拒绝 `get_world_pose()` 的 USD fallback、无效/非有限数据及不匹配关节顺序。本诊断不证明抓持接触和高负载下的完整闭环。

### 候选在线快照与新鲜度

仅 `stage_b0_only` 在双侧吸附后进入 release-retreat 模式（依据吸盘命令，物理打开另由反馈确认）之后，每三个物理步（名义 0.05 s 仿真时间）发布 `/task27/b0_poststep_snapshot`：同一步的 Cube 世界位姿、左右实测关节、步号、累计仿真时间、来源及采样起止墙钟，同时保留 USD 值供审计。Stage A 原有 `/cube_poses` 和控制器未改。B0 的推送片后监督、深墙验收、侧压规划起点和最终落位读取此候选快照；深墙验收等待在推送及原等待结束后才开始采样的消息。左右关节在同一同步步后回调内依次读取，按原 FR3 七关节顺序验证。

接收器拒绝错误 frame/source、句柄无效、缺失或非有限数据、四元数无效、采样跨度超过 **50 ms**、来源时间在未来、样本年龄超过 **50 ms**、步号/仿真时间/来源时间倒序，以及早于相关动作完成标记的样本。50 ms 大于原场景探针最长 0.560 ms，单回调内仿真步不会前进；旧 QP 运行的两次同步记录中，发布到接收约 0.24–0.30 ms，50 ms 为其约 166 倍；慢速仿真中相邻发布最多约 170.1 ms，因此使用 2 s 等待新帧，而不把发布间隔当作可接受的消息年龄。50 ms 在本诊断的 20 mm/s 速度下对应 1 mm 位移；这不是对其他速度的误差保证，也可能在机器负载高时保守拒绝。该期限是仿真快照的新鲜度界限，不改变原 3 mm 几何门限。

C++ 合约测试覆盖实测 v2 快照解码、旧样本/动作前样本、乱序、错误 frame、缺失或非有限关节及关节顺序；隔离 ROS domain 的实际 C++ 订阅缓存测试覆盖发布/接收、动作前、乱序和错误 frame。Python 测试覆盖句柄回退拒绝和非有限关节。**Isaac 发布器到执行器的完整 ROS 链尚未通过完整 GUI 验证；重抓等旧代码仍有 USD 读取路径，因此尚不能把整个 B0 在线状态一致性标为完成。** 侧压规划快照已准备，但由于下述阻塞，保护门在实际 `planSideCompaction` 前停止；实际规划器起点与模拟状态的运行时对应尚未验证。

### FCL 与侧压终点

C++ 离线重放已加载原双臂模型、Cube、桌面和三面墙，保持原碰撞几何和 ACM。旧名义起点重新复现右吸盘/Cube 的 **1.434 mm** 接触，而记录的双臂实测起点无碰撞；对实测起点到名义起点的人工插值，101 个 FCL 样本中第 **34** 个首次碰撞。该插值不是侧压候选路径。

原侧压的 1 mm 命令间隙及 4 mm 超程意味着：对按名义位姿贴住 +Y 墙的 Cube，辅助杯面终点比 Cube 的 −Y 面深入 **3 mm**。使用原 120 mm Cube 和原 10 mm 半径、10 mm 长的吸杯圆柱，单独 FCL 几何测试得到接近点无碰撞、该名义终点相交 **3.000 mm**。这说明现有命令的预期接触在当前禁止工具/Cube 相交的模型下会被拒绝；不能用从场景删除 Cube 或放宽 ACM 来声称通过。它不是对所有替代路径不存在的证明，也不是对完整侧压候选路径的验证。下一步需先确定与不放宽碰撞规则相容的接触表达和完整路径验证，再运行一次 GUI 物理循环。

### 本轮验证与结果

- 原场景中的有界 headless 状态诊断：v1/v2 共各 100 个物理步样本；没有新的 Stage A 或 B0 完整 GUI 尝试。
- 聚焦 Python 回归：**55 passed**；C++ 快照合约、ROS 订阅缓存及 FCL 重放/终点接触测试通过。ROS 缓存测试使用合成快照，实际 Isaac→ROS→执行器整链未测试。
- 原 Stage A 证据、3 mm 深墙验收、力矩/QP 与碰撞几何没有放宽。旧修正运行仍按原 USD 验收值 3.800 mm 失败；本轮无新的物理推送、最终释放或双臂退避结果。
- 复现物理诊断：`DISPLAY=:1 ROS_DOMAIN_ID=109 /home/ubuntu2004/isaacsim-4.5.0/python.sh isaac/scripts/task28_b0_poststep_probe.py --run-id NEW_UNIQUE_ID`。快照 C++ 合约测试用 `g++ -std=c++17 -I ros_ws/src/fr3_dual_palletize/src offline/task28_task27_integration/tests/b0_snapshot/test_snapshot.cpp -lyaml-cpp -o /tmp/task28_snapshot_test && /tmp/task28_snapshot_test`；完整快照 C++/ROS 测试脚本见 [run.sh](../../offline/task28_task27_integration/tests/b0_snapshot/run.sh)，需先按下节加载 ROS 工作区并使用隔离的 `ROS_DOMAIN_ID`；FCL 复现命令见下节。

## 2026-11-03 状态一致性与侧压起点离线复核：B0_PARTIAL

**决定：不启动本工单的一次完整 GUI 运行。** 本工单要求先验证可用于在线验收的物理状态源、消息新鲜度及双臂同步；现有记录不足以验证这些前提。此前所有失败结果保持原判，3 mm 验收阈值与 Stage A 基线未变。本次没有改变在线位姿源、执行器的门限或 FCL 碰撞规则。

### USD / 候选 PhysX 状态与时间

- 桥在 `subscribe_physics_step_events` 回调中先运行 torque step，随后运行原 feed callback；后者每约 0.05 仿真秒用 USD `ComputeLocalToWorldTransform(Default)` 生成 `world` frame 的 `/task27/cube_poses`，接着 audit wrapper 在同一 callback 读取同一 Cube 的 `SingleRigidPrim.get_world_pose()`。本机 Isaac Sim 4.5 的实现：当 physics handle 有效时，后者经 rigid view 的 PhysX tensor `get_transforms()` 取得世界位置和 wxyz 姿态；无有效 handle 时会退回 USD。旧 trace 没记录 handle 是否有效，也未独立记录物理引擎对该张量刷新时刻的确认。
- 旧修正验证的同一发布消息，USD x=1.096200466 m、rigid API x=1.097104073 m，相差 0.903606 mm；以原中心几何计算 gap 分别为 3.799534 mm 与 2.895927 mm。这里证实两个读数不同，**尚不能确定哪一个是该时刻已完成物理步的真实位置，也不能把旧失败改判为通过**。
- 验收附近 physics step 15312–15342（记录的仿真时间 255.20–255.70 s），两个位置分别保持上述精确值，rigid API 的 X 速度却持续约 +1.69 mm/s。这是**可能的采样或缓存不同步**的证据，并非仅凭速度就证明某一位置错误；需要在原场景用已确认的步后时刻、物理句柄状态及另一独立物理查询交叉核对。所用记录见[修正验证同步窗口](../../offline/task28_task27_integration/evidence/task28_b0_pose_sync_corrected_v1/pose_sync/acceptance_pose_window.jsonl)。
- `CubeBuffer::waitNew` 只检查 revision 大于推送前的 `source/release_revision`，所以不能保证消息在推送和等待完成后才发布，也没有对消息时间、frame、延迟和乱序进行在线检查。`06386a8` 的候选 `Arm::measuredPositions` 分别读取左右最新缓存，未检查来源时间、左右同步或缓存年龄；该旧候选已由本轮同帧快照赋值替代。旧两源时间关联是离线证据，不是运行时 freshness 契约。新增的离线关联回归拒绝 frame 不匹配、发布/接收/验收墙钟顺序错误及不匹配的消息 header，但**不等于完成在线 stale 消息防护**。

### 侧压 FCL 离线重放

使用诊断运行 physics step 13845 捕获的左右实测关节、右臂参考关节和同一帧 Cube 位姿；模型沿用仓库 xacro/SRDF，Cube 尺寸 0.12 m，按当时导轨的 0.100 m 世界偏移转换到规划坐标。该轮只装入 Cube；后续重放已增加桌面和墙体，见上节。执行 [C++ FCL 重放回归](../../offline/task28_task27_integration/tests/fcl_replay/replay.cpp) 的结果：

| 起点 | Cube / 右吸盘 FCL | 右 TCP x（规划坐标） |
|---|---:|---:|
| 记录的右臂参考终点 | collision，首接触深度 **1.43391 mm** | 0.939000 m |
| 同帧双臂实测关节 | 此静态场景无碰撞 | 0.936610 m |

这复现了原日志的 1.434 mm 接触，也支持规划终点与实测状态不一致是该次预检失败的原因；它不能证明旧 C++ `measuredPositions` 调用满足时间同步，不覆盖完整世界或整条侧压轨迹，也不证明后续侧压安全。该候选代码仍未执行到，也尚无针对其在线缓存与规划初始化调用的 C++ 回归。没有扩大 ACM 或禁用吸盘/Cube 碰撞。

复现（先确保外部工作区提供 `franka_description`）：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
source ros_ws/install/local_setup.bash
offline/task28_task27_integration/tests/fcl_replay/run.sh
```

### 当前结果与下一步

本轮 FCL 重放通过；Python 聚焦回归 **54 passed**。本轮没有新的物理推送、侧压、最终释放或退避；最近一次完整物理尝试仍在原 USD 验收读数 3.800 mm 处失败，最终无释放/退避。下一步需要在原仿真场景独立验证 post-step 刚体 pose 与步号、双臂关节及时间戳之间的对应关系，建立可拒绝旧/乱序消息的在线快照，并对真实 side-planning 初始化做覆盖和完整 FCL；只有这些前提通过后才能执行新的单次完整循环。

## 状态：PARTIAL

## 2026-11-03 同步诊断与一次修正验证：B0_PARTIAL

本轮基线 `01f18e700fae3fb5787e03ec8a900a615a29da8a`。按工单仅执行一次同步诊断和一次修正后的完整 GUI 运行；修正运行在原深墙门限失败后停止，**没有达到 B0_COMPLETE**。先前 v1–v5 和 Stage A 原始证据均保留。

| 项目 | 同步诊断 | 唯一修正验证 |
|---|---:|---:|
| Stage A、双侧释放、共同退避、右臂 −X 面重抓 | 完成 | 完成 |
| 实际 +X 位移，首个至最后单侧 CLOSED 帧 | 327.876 mm | 326.691 mm |
| 原执行器验收时 USD / ROS 深墙 gap | **2.530 mm，通过 3 mm** | **3.800 mm，失败** |
| 同一发布回调 PhysX 刚体 gap | 2.530 mm | 2.896 mm |
| 侧压、最终释放与退避 | 侧压 FCL 起点碰撞，中止；安全中止打开两吸盘，未退避 | 未进入侧压；验收失败时右吸盘仍 CLOSED，未完成最终释放或退避 |
| QP optimal / infeasible，力矩锁存 | 13889 / 0，无 | 15343 / 0，无 |
| 峰值施加力矩左 / 右，最大变化率 | 36.001 / 36.053 N·m，1023.676 N·m/s | 36.159 / 35.152 N·m，1193.317 N·m/s |
| 最大实测关节速度 / QP 加速度 | 0.287 rad/s / 10.529 rad/s² | 0.259 rad/s / 10.647 rad/s² |
| 单侧吸附期关节误差峰值 / RMS | 0.02008 / 0.00422 rad | 0.01851 / 0.00407 rad |

两次实际推送单侧 QP 帧分别为 1844 / 2073；该阶段共同和相对目标均关闭，旧双臂对象参考、G3 内力和双臂 payload 前馈未激活。两次逐关节施加力矩与力矩变化率未超各自原上限，实测速度与 QP 加速度的最大值低于原配置上限；位置预测约束仍在 QP 中。机械臂 position-arm 写入计数均为 0。原 Task27 场景抓取前的供料唤醒仍有直接 Cube 位姿写入；物理推送阶段未新增这类写入。

### 同步位姿根因证据与限制

- 执行器路径：原 Task27 feed 桥以约每 0.05 s **仿真时间**发布 `/task27/cube_poses`，`_pose` 对同一 `/World/Task27/Supply/Cube_01` prim 使用 `UsdGeom.Xformable.ComputeLocalToWorldTransform(Default)`。消息 `frame_id=world`，`header.stamp` 来自 ROS node 时钟；本轮数值与系统墙钟在同一 epoch。执行器 `CubeBuffer` 保存最新消息；本轮新增记录其原消息时间戳、收到消息的系统墙钟和验收系统墙钟。旧 v2/v3 不包含这些关联字段。原 `waitNew` 所依据的 release/source revision 在推送前取得，因此不能保证其返回的是验收调用后才发布的新消息。
- 轨迹路径：同一 Cube prim 的 `SingleRigidPrim.get_world_pose()` 在本 Isaac 版本有有效物理句柄时通过 PhysX tensor `get_transforms()` 读取刚体 actor 的世界位姿；四元数为 wxyz，ROS 为 xyzw。两个路径都是世界坐标下同一 prim／刚体原点，两个报告使用的原深墙准则均为 `1.160 − (x + 0.060) = 1.100 − x`，没有另一种立方体尺寸或额外 rail 偏移。该原准则采用名义半边长，不是按倾角修正的包络面间距；本轮没有改变它，也没有把几何间距冒称深墙接触力。
- 同步诊断验收消息 `header_ns=1793693320865796193`，仿真时间 230.750 s，ROS 接收比发布记录晚 0.300 ms，验收比接收晚 69.356 ms。发布回调先记录实际 ROS 消息内 USD 位姿，再紧邻读取刚体位姿；二者 X 均为 `1.097469687 m`，所以同刻 gap 均为 2.530313 mm。之后 FCL 中止与该门限通过是两个独立事实。
- 修正验证的验收消息 `header_ns=1793694324924568181`，仿真时间 255.650 s；发布到接收 0.243 ms，接收到验收 66.547 ms。**同一消息的发布回调中** USD X=`1.096200466 m`、刚体 X=`1.097104073 m`，差 `0.903606 mm`；原执行器验收分别对应 3.799534 mm 和刚体几何 2.895927 mm。这次差异不依赖事后末帧、ROS 传输延迟或 gap 算式，在验收前 0.75 s 仿真时间内两读数各自维持该差值。速度接口同期报告约 +1.69 mm/s，而两位姿读数近似不变，故不能仅由速度推断真实位移。
- 诊断运行的运动窗口中，USD X 在 228.750–228.950 s 连续 0.20 s 仿真时间保持 `1.077546358 m`，同回调刚体 X 从 `1.077546358` 前移至 `1.080938578 m`，最大差 **3.392 mm**。这证实运行时两条状态更新路径可产生毫米级不一致；验收消息直接携带的数值与执行器记录逐位相符。USD／PhysX 写回或缓存的具体内部机制、以及旧 v2/v3 每次各 0.915 / 0.856 mm 差异的确切来源，仍无法由当时缺少同步时间戳的日志证明。不得用刚体读数替代原 ROS 验收或把本轮结果追认成功。

### 侧压 FCL 修正及本次验证范围

诊断运行在深墙通过后，全部 8 个执行期侧压高位入场候选均在 `t=0` 检测到 `right_fr3_side_suction <-> task26_cube_1`，模型穿入 1.434 mm，执行器按原条件中止并确认双吸盘 OPEN。原重抓链预检使用名义 Cube `x=1.100 m` 和名义推送末端关节；执行期重规划使用实测 Cube `x=1.097470 m`，却仍把名义关节终点作为另一臂起点。同期实测右 TCP X 约 1.036610 m，名义推送 TCP X 为 1.039 m，差约 2.39 mm。该起点／物体状态不一致能解释模型穿入，但 FCL 读数本身不证明物理穿透，也不能因此放宽碰撞规则。

最小候选修正为：仅在 Stage B0 侧压执行规划前，从现有各臂 JointState 读取完整有限的实测关节位置作为两个规划起点；缺失时安全中止；保留 Cube、墙体、工具和双臂的原有 FCL 检查。修正后的唯一完整运行在 **3.800 mm** 深墙门限处提前停止，**没有执行到该修正分支**，故不能声称 FCL 问题已修复或路径安全；现有 Python 聚焦测试也未覆盖该 C++ 侧压起点分支。继续工作需先解决两个位姿源更新一致性及原验收下的剩余间隙，再单独验证该候选。

### 本轮证据和构建

- [同步诊断消息关联](../../offline/task28_task27_integration/evidence/task28_b0_pose_sync_diagnostic_v1/pose_sync/acceptance_pose_sync.json)、[验收前后双源时序](../../offline/task28_task27_integration/evidence/task28_b0_pose_sync_diagnostic_v1/pose_sync/acceptance_pose_window.jsonl)、[诊断执行日志](../../offline/task28_task27_integration/evidence/task28_b0_pose_sync_diagnostic_v1/task27_executor.log)、[QP/位姿指标](../../offline/task28_task27_integration/evidence/task28_b0_pose_sync_diagnostic_v1/compact/b0_metrics.json)。
- [修正验证消息关联](../../offline/task28_task27_integration/evidence/task28_b0_pose_sync_corrected_v1/pose_sync/acceptance_pose_sync.json)、[验收前后双源时序](../../offline/task28_task27_integration/evidence/task28_b0_pose_sync_corrected_v1/pose_sync/acceptance_pose_window.jsonl)、[修正验证执行日志](../../offline/task28_task27_integration/evidence/task28_b0_pose_sync_corrected_v1/task27_executor.log)、[QP/位姿指标](../../offline/task28_task27_integration/evidence/task28_b0_pose_sync_corrected_v1/compact/b0_metrics.json)、[GUI 场景初始化截图](../../offline/task28_task27_integration/evidence/task28_b0_pose_sync_corrected_v1/scene_loaded.png)。初始化图不是最终状态图；本轮失败后没有最终退避截图，原始 60 Hz 轨迹保留本地。诊断运行有 8 条侧压起点 FCL 碰撞记录，修正运行未进入该预检且未记录 FCL collision。
- runner 使用原始任务配置、唯一 batch、GUI、隔离 ROS domain 104 / 105 和新运行 ID；参数与文末 B0 命令相同。诊断执行器 build/install SHA-256 为 `99837ec2833f5a040d43fe5257d4123ea411b392b76af7fd5e41f556536d23a0`；修正验证版本为 `276685c499b46c7edc0b8ec45d052d2c0da389ca77df01153a7d941c7cdd88fe`，均校验 build 和 install 一致。两次均只运行 Cube_01。
- 聚焦回归 `PYTHONPATH=offline/task28_g1_1:. python3 -m pytest -q offline/task28_task27_live_qp/tests offline/task28_g3_stage_a/tests`：**53 passed**。同步工具的测试验证精确匹配消息、而非取较晚样本；没有扩大碰撞允许、改变 3 mm、力矩或其他硬约束。


在原 Task27 Isaac Sim GUI 场景中，三次完整尝试均完成冻结 Stage A、右臂在 Cube_01 的 −X 面单侧重新吸附，并用 live torque QP 物理执行原有 16 段 +X 推送。**三次均未通过执行器的深墙 3 mm 支撑间隙门限，因此 B0 未完成**：没有执行后续侧压、最终单侧释放和最终退避，也没有证据证明释放后稳定。不能把 QP 求解成功或实际推动误报为完整 B0 PASS。

冻结基线为 `966fda6195723b45c63c04a182d4c800085d9a89`。它记录三次 Stage A GUI 物理周期复现，0.25 mm XY 精度指标仍未满足；本任务不重新优化它。原 Task27 场景几何和夹爪保留。原场景抓取前仍通过直接设置位姿将休眠 Cube_01 唤醒到供料槽；该供料行为与其后的物理抓取、推送应分开理解，本轮没有在物理推送期间直接写 Cube 位姿。机械臂力矩接管后的 position-arm 写入计数为 0。

| 尝试 | 单臂反馈 Kp/Kd | 推送后的执行器深墙 gap | 结果 |
|---|---:|---:|---|
| v1 | 12/5，既有自由臂反馈 | +9.722 mm | 16 段实推后门限失败；executor 1 |
| v2 | 48/10，仅已确认吸附的 B0 推送臂 | +3.714 mm | 16 段实推后门限失败；executor 1 |
| v3 | 48/10；末端等待从 1.5 s 暂延至 4 s | +3.679 mm | 16 段实推后门限失败；executor 1 |
| v4 | 96/14 预定配置 | 未进入 B0 | GUI 进程收到 SIGTERM，退出码 143；原因未知 |
| v5 | 96/14 预定配置 | 未进入 B0 | GUI 进程收到 SIGTERM，退出码 143；原因未知 |

v4 在执行器启动前中断，v5 在 Stage A 左臂接近期间中断；都没有 summary 或 B0 结果。中断后仅停止了对应 ROS domain 的孤儿进程。96/14 未在 B0 中验证，未保留；4 s 等待未使 v3 通过，亦已撤回。当前代码保留 v2/v3 测试过的 48/10，以及原始 1.5 s 等待。该控制配置只是部分物理验证，不能视为最终方案。v2/v3 后进一步加入仅 B0 模式、且已先观察到双侧吸附和释放时才允许该增益的范围防护；这一防护经单元测试，尚未重新完成物理运行。

## 原工艺与本次接入

- `task27_five_cube_center_insert.cpp` 编译包含 Task26 执行器；复用 `PREPLANNED_COMMON_RETREAT`、`REGRASP_SWING`、`preplanPush()`、原 RRTConnect 候选、IK 和同步 FCL 检查、`executePushWithSupervision()` 的 16 段 +X 轨迹、物体 GT 与吸附反馈。Cube_01 为第一批唯一物体，执行器和 runner 均限制 `first_batch=1, max_batches=1`，不会到 Cube_02。原 Task27 外侧件需要后续 +Y 侧压，其原有完整链已做预检，但由于深墙门限未通过，本轮没有物理执行侧压。
- `isaac/scripts/task28_task27_one_pass.py` 新增 `--stage-b0-only`，要求集成 live QP、唯一批次、冻结的对象参考与 G3 载荷配置；向执行器传递 B0 标志并把失败退出记为失败。Stage A 的对象中心参考、世界系旋转修正、G3 和 QP 配置未改变。
- `isaac/scripts/task28_task27_torque_bridge.py` 离开 `DUAL_ATTACHED` 时清除对象参考、双臂保持状态及末端积分；原模式切换以吸盘**命令**触发单臂模式，物理关闭由独立 Surface Gripper 反馈逐帧确认。仅 `--stage-b0-only`、已经完成双侧释放且推送臂的吸盘实际 CLOSED 时，将该臂的独立关节反馈从 12/5 改为 48/10；辅助臂不改。独立模式的共同/相对 QP 软权重为零，不施加双臂闭链目标；无双臂 payload 或 G3 内力项。
- `ros_ws/src/fr3_dual_palletize/src/task26_truck_box_push_in.cpp` 增加 B0 单件边界、落桌／双侧打开检查、实际重抓几何和辅助臂吸盘状态记录，以及完整结束时的 B0 标记。该路径没有改变原 Task26 默认工艺；执行器每次代码变更后重新编译，v1/v2 的 build 与 install 二进制 SHA-256 均为 `6e38ab822f6cae9fa444609bcf1af9e0aebaaf893a475a4f77e4b0fd70a6e879`；v3 的 4 s 版本为 `8341bc30a8156e2bad9e2301799c96498c74e792e1fa6c128c03a85918da595b`。该等待修改已撤回并重新编译为前者。
- 新增 `offline/task28_task27_integration/analyze_b0.py`，保留完整 60 Hz 原始数据在本地，将状态、数值和 10 Hz 时序保存为紧凑证据；`offline/task28_g3_stage_a/tests/test_full_cycle_integration.py` 增加模式切换、单件范围和证据检查。

## 实测转换、重抓和推送

三个运行均记录 `DUAL_ATTACHED → RELEASE_RETREAT → SINGLE_ARM_PUSH`。Stage A 双侧 OPEN 首帧的 Cube/Table 竖向力分别为 **7.863 / 7.858 / 7.816 N**；随后执行器记录实际桌面支撑、双吸盘 OPEN 和共同退避完成。原有释放后静默互锁有效，两个滑轨从 **0.650 → 0.750 m** 重新站位，规划世界按原工艺偏移重建。随后根据实际 Cube 位姿规划重抓及整条推送／退出链。

| 实测指标 | v1 | v2 | v3 |
|---|---:|---:|---:|
| 吸附前右 TCP 至 Cube −X 面 gap (mm) | 0.970 | 0.862 | 0.990 |
| 吸附前 TCP 相对 Cube 的 Y / Z 偏差 (mm) | −0.012 / −0.127 | +0.049 / −0.070 | +0.133 / −0.162 |
| 实际关闭反馈：左 / 右 | OPEN / CLOSED | OPEN / CLOSED | OPEN / CLOSED |
| 首次单侧 CLOSED 到最后单侧 CLOSED 的实际 Cube ΔX / ΔY / ΔZ (mm) | +320.866 / +3.188 / +0.331 | +328.220 / +0.001 / +0.107 | +327.523 / −0.039 / +0.078 |
| 16 段末：命令 X / 执行器记录实际 X (m) | 1.1000 / 1.0903 | 1.1000 / 1.0963 | 1.1000 / 1.0963 |
| 执行器验收时深墙 gap / 高度误差 (mm) | +9.722 / 0.332 | +3.714 / 0.106 | +3.679 / 0.076 |
| 最后轨迹帧 Cube XYZ (m) | `(1.090670, 0.063382, 0.260331)` | `(1.097201, 0.059769, 0.260107)` | `(1.097177, 0.060013, 0.260078)` |
| 最后轨迹帧相对目标 `(1.100, 0.243, 0.260)` XYZ (mm) | `(-9.330, -179.618, +0.331)` | `(-2.799, -183.231, +0.107)` | `(-2.823, -182.987, +0.078)` |
| 最后轨迹帧姿态 XYZ 欧拉角 (°) | `(-0.00004, 0.316, 1.753)` | `(-0.00000, 0.102, 0.117)` | `(-0.00003, 0.074, 0.154)` |
| 最后轨迹帧 Cube/Table Fz (N) | 1.337 | 1.328 | 1.311 |
| 最后轨迹帧吸盘左 / 右 | OPEN / CLOSED | OPEN / CLOSED | OPEN / CLOSED |

相对目标 Y 偏差较大是因为原工艺要求先在中央安全行 +X 推入，之后再侧压至 +Y 目标；本轮侧压未开始。最后帧 Fz 是吸附状态下的瞬时桌面接触力，不证明释放后稳定，也不证明深墙接触。

执行器的 gap 使用原 Task27 ROS `/cube_poses`（原桥由 USD world transform 发布），紧凑轨迹使用 `SingleRigidPrim.get_world_pose()`；两者分别采样于不同数据路径及时间。v2/v3 最后一帧按中心几何计算分别为 2.799 / 2.823 mm，但执行器原验收时读到 3.714 / 3.679 mm，且已经失败。**不能用末帧数值追认原门限通过**；这两条状态路径的偏差及时间对应仍需专项核对。中心几何 gap 也不是独立的深墙接触力测量。

v1/v2/v3 单侧实际 CLOSED 期间有 1857 / 1931 / 2177 个 QP 帧，均记录独立关节跟踪、相对目标关闭、对象参考不活跃、双臂 G3 与 payload 关闭。16 段的 Cube 实测逐段位置和偏差见执行器日志；右臂实际吸附、左臂 OPEN，与规划结果分开核验。执行器对三次均在深墙门限停止，没有将单侧吸盘执行最终 OPEN，因此仅能报告当时保持吸附；仿真停止不等于完成受控释放。

## QP、约束与诊断

| 指标 | v1 | v2 | v3 |
|---|---:|---:|---:|
| QP optimal / infeasible | 14277 / 0 | 13391 / 0 | 13873 / 0 |
| 最大施加力矩：左 / 右 (N·m) | 35.824 / 36.197 | 33.666 / 33.588 | 36.315 / 31.669 |
| 最大逐关节力矩变化率 (N·m/s) | 942.239 | 1581.264 | 1462.525 |
| 最大实测关节速度 (rad/s) | 0.301 | 0.224 | 0.295 |
| 最大 QP 关节加速度 (rad/s²) | 10.751 | 10.376 | 10.436 |
| 单侧吸附期间全关节跟踪峰值 / RMS (rad) | 0.06568 / 0.01427 | 0.01946 / 0.00423 | 0.01849 / 0.00400 |
| 力矩锁存错误 / 接管后 position-arm 写入 | 无 / 0 | 无 / 0 | 无 / 0 |

每臂关节力矩限制 `(87,87,87,87,12,12,12) N·m`，变化率限制 `(2400,600,240,2400,240,240,240) N·m/s`，各关节记录峰值均未超出各自限制。QP 的位置和速度约束、20 rad/s² 加速度上限均未放宽。紧凑 JSON 保留逐关节峰值；力矩为仿真施加命令而非真实关节力传感。FCL 为已有采样检查，不是连续碰撞证明；采样的接触诊断未构成全场景碰撞力认证。

v1 最后一帧右臂第 7 关节参考误差约 0.0481 rad，QP 预测的关节加速度约 0.567 rad/s²，有限差分实测约 −0.006 rad/s²，且该帧无力矩／变化率饱和指标。单臂反馈提高后，v2/v3 的跟踪峰值和深墙验收间隙均下降，但数据不足以将残差完全归因于摩擦、模型误差或任何单一因素；提高增益也未达成完整任务。v3 延长等待后仍未通过原验收，故不继续参数扫描。下一步应首先同步核对 USD／PhysX 的 Cube 位姿和执行器验收时刻，再针对剩余间隙制定单一受控修正；不得放宽原 3 mm 标准。

## 证据与回归

- v1：[指标](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v1/compact/b0_metrics.json)、[10 Hz 时序](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v1/compact/b0_series.jsonl)、[执行器日志](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v1/task27_executor.log)、[summary](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v1/summary.json)。
- v2：[指标](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v2/compact/b0_metrics.json)、[10 Hz 时序](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v2/compact/b0_series.jsonl)、[执行器日志](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v2/task27_executor.log)、[summary](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v2/summary.json)。
- v3：[指标](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v3/compact/b0_metrics.json)、[10 Hz 时序](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v3/compact/b0_series.jsonl)、[执行器日志](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v3/task27_executor.log)、[summary](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v3/summary.json)、[GUI 场景截图](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v3/scene_loaded.png)。截图为场景初始化，不是最终释放截图；各次原始大型 60 Hz trace 留在本地。
- 中断记录：[v4](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v4/interruption.json)、[v5](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v5/interruption.json)；v5 的执行器日志记录中断前进度。信号来源未确定。
- 聚焦回归命令：`PYTHONPATH=offline/task28_g1_1:. python3 -m pytest -q offline/task28_task27_live_qp/tests offline/task28_g3_stage_a/tests`，本轮最近一次结果 **51 passed**。保留原 Stage A 基线证据，未开始 B1、五 Cube 连续码垛或 Sim2Real。

复现须使用不存在的独立 run ID 与 ROS domain。v2 命令示例：

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=104 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 --stage-b0-only \
  --consistent-object-reference --assume-internal-contact-load \
  --internal-force-target-n 1 --record-joint-reactions \
  --internal-force-weight 1000 --internal-force-kp 0.5 \
  --local-sweep-limit-m 2.0 --execution-time-scale 16 \
  --timeout-s 2400 --run-id NEW_UNIQUE_RUN_ID
```

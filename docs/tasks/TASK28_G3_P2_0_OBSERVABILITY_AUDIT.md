# Task28 G3-P2-0｜有限侧吸与内部载荷可观测性审计

**结论：`G3_P2_0_PARTIAL`。** 已完成源码与论文对应关系审计；当前系统没有经验证、带明确坐标和时间语义的双侧接触 wrench，因此不能从现有观测唯一确定双臂物理内部载荷，也不能支持 P2 内力闭环。这里的 PARTIAL 指观测能力和后续实验前提尚未建立；本次是只读审计，不是未成功的物理实验。

仓库 `dreamerToTOE/dual-arm-embodied-palletizing`，分支 `side-suction-palletizing`。本地与远端起点均核实为 `b24b5ccd15c36fc355d9577628cd1d8b9aa5d3b4`，审计前工作树干净。所有源码结论为静态审查，未运行 Isaac、ROS 或机器人；历史资料明确标记为历史。源码和本机安装头文件 SHA-256 见 [source_manifest.json](../../offline/task28_p2_0/evidence/source_manifest.json)。

## 1. P2 论文与本系统的对应关系

原论文为 De Simone、Costanzo、De Maria、Natale，*Cooperative Object Transport and Assembly: Pose/Force Control by Visual-Tactile Feedback*，IEEE RA-L，2026，DOI [10.1109/LRA.2026.3665445](https://doi.org/10.1109/LRA.2026.3665445)。作者稿第 3–5 页建立接触模型与抓取映射、估计接触载荷，并在式 (20) 将经接触模态选择的 wrench 分解为物体净载荷和内部载荷；视觉与触觉观测经 SE(3) 上的滤波器融合。作者稿中用弹性/阻尼接触模型重构 wrench，不能推断论文的每个接触均由六轴传感器直接测得。原实验为异构机械臂与带触觉的平行夹爪，接触及观测假设与 Task28 的双 FR3、单侧聚合侧吸不同。[作者公开稿](https://www.researchgate.net/publication/400852785_Cooperative_Object_Transport_and_Assembly_PoseForce_Control_by_Visual-Tactile_Feedback)

独立复现仓库 `/home/ubuntu2004/lmy/dual-arm-coordinated-manipulation-reproduction` 的只读 HEAD 为 `79ad22c7fded963e0b8e9cc50caadb242bca8108`。其 [P2 方法卡](https://github.com/dreamerToTOE/dual-arm-coordinated-manipulation-reproduction/blob/79ad22c7fded963e0b8e9cc50caadb242bca8108/references/P2_pose_force.md) 将接触 wrench/真值替代视觉触觉滤波明确列为 adaptation；TASK07 和 TASK09 均为 TODO，不能写成已有 P2 复现。该仓库原有 4 个未跟踪 results 目录保持不动。任务库原有 [G0-A §7](TASK28_G0A_INTERFACE_AUDIT.md) 与本次直接检查的源码对照如下；本机安装源码版本和被检查的文件哈希都在机器证据中，未把未检查的 API 全局归为不可用。

## 2. 信号与能力矩阵

| 信号 / 数据 | 当前连接和来源 | 可支持的结论与限制 |
|---|---|---|
| suction desired | Task24/26 bridge 订阅 `std_msgs/Bool`，缓存 `_desired`；Task26 [`_physics_step_inner`](../../isaac/scripts/task26_truck_box_bridge.py#L700) 按变化调用 `close/open` | 是请求，非已附着证据；命令 Bool 无 source stamp 或请求 ID。Task27 bridge 复用 Task26。 |
| actual CLOSED | Task24/26 的 `is_closed()` 发布 `Bool`；Task26 [`_publish`](../../isaac/scripts/task26_truck_box_bridge.py#L782) | 是 gripper 状态，不包含 body ID、六维受力、成功密封或时间戳。闭合不能证明抓住期望 Cube。Task28 fixture 默认命令关闭时无 gripper，发布 `False`，不能解释为已测 OPEN（[fixture bridge](../../fixtures/task28_g1_0_new_single_cube_v0/bridge/task28_single_cube_bridge.py#L350)）。 |
| attached body / 相对抓取 | 安装 `SurfaceGripper.h` 的 raycast 选择命中刚体并建立 body1/anchor；任务 bridge 未对外发布 attached body 或约束双方 anchor | 期望的 Cube index 和 GT 几何对齐不是 D6 实际 body ID 或捕获时 `T_OL/T_OR`。实际身份/抓取变换为 **UNKNOWN**。 |
| break / detach / retry / recapture | 安装 SurfaceGripper 检测 D6 broken 后 open；Task26 desired ON 时调用 update，未 CLOSED 约 0.05 仿真秒重试 close，成功只打印（[源码](../../isaac/scripts/task26_truck_box_bridge.py#L720)） | 无独立事件时间、原因、体 ID、序号或故障锁存；自动重抓可能遮蔽短时断裂。显式 open 与 break 不能由低频 Bool 唯一分辨。 |
| SurfaceGripper / D6 配置 | Task24/26 每臂一个聚合 gripper；`gripThreshold=0.003 m`、`forceLimit=torqueLimit=1e6`、`bendAngle=0`、stiffness `1e4`、damping `1e3`、`retryClose=True`、`disableGravity=False`（[Task26](../../isaac/scripts/task26_truck_box_bridge.py#L307)） | 几何含四杯不等于四个独立吸附/密封传感器。破断值是后端阈值，刚度/阻尼是配置；不是测得吸力或已标定的物理真空容量。 |
| Cube/TCP pose | Task26 发布 `PoseArray`、`PoseStamped`，同一次 `_publish` 使用 node-clock stamp、`world` frame；仿真调度累计 `dt>=0.05 s`（[源码](../../isaac/scripts/task26_truck_box_bridge.py#L742)） | 是仿真派生的 pose，不是接触 wrench；标称仿真周期不代表实测墙钟频率或多信号新鲜度。消费侧部分 buffer 丢弃 stamp。 |
| joint effort | Task26/27 优先 `get_measured_joint_efforts()`，发布 `JointState.effort`；Task24 bridge 无同等 force topic（[Task26](../../isaac/scripts/task26_truck_box_bridge.py#L747)） | 本机 API 描述为求解器关节力在 DOF 方向投影。它不是 D6 反力、实测硬件 F/T 或已扣动力学的外力矩；`frame_id=world` 不能把关节广义量变成 world wrench。 |
| incoming joint wrench | 本机安装 API `get_measured_joint_forces()` 返回 `(num_joints+1,6)` incoming joint wrench；第 0 行为 base，关节行映射需 index+1，前三项力、后三项矩 | API 存在不说明其直接等于外部 suction D6 reaction；坐标、正负、作用点、加载/动力学归因须验证。 |
| effort fallback | Task26 主 API 异常时调用上条 API；二维结果取 `array[:,0]`，再按 `fr3_joint1..N` 命名并填入 effort（[源码](../../isaac/scripts/task26_truck_box_bridge.py#L747)） | 可能将包含 base 行的力分量当作关节 effort。没有 source/shape/valid 标志；未知历史运行是否触发，不能把此 fallback 当合格力矩。 |
| 诊断 wrench estimator | `task26` 的 `WrenchEstimator` 用静止 bias 与手写 Jacobian 求 `(Jᵀ)^+ (tau-bias)`，只保留前三维投影；仅 `wrench_probe` 分支调用（[估计器](../../ros_ws/src/fr3_dual_palletize/src/task26_truck_box_push_in.cpp#L1817)） | 单臂诊断，未证明双侧接触力或内力；缺少动力学/重力/摩擦补偿、同步和可观测性验证。G0-A 指出手写 Jacobian 的 parent-link 轴/原点问题。 |
| D6 reaction wrench | 当前任务 bridge 未读/发布。检查的安装 SurfaceGripper D6 handle 为私有，所查 DynamicControl D6 接口暴露 create/destroy/properties/isBroken，未见 reaction getter | **未连接、未验证**。其他 PhysX 接口是否能可靠读取该外部约束为 UNKNOWN；不能泛称 Isaac 全部 API 不支持。 |
| 时间及失败锁存 | Cube/TCP/effort 发布带 node-clock stamp；suction Bool 不带 stamp。Task26 [`ForceBuffer`](../../ros_ws/src/fr3_dual_palletize/src/task26_truck_box_push_in.cpp#L1782) 只保存最新 effort 和最大绝对分量 | 无双臂 source/receive 同步、age/skew、事件连续性或 break latch。`peak()` 是最新消息跨关节最大值，默认 0；它不是连续时间峰值或已验证新鲜反馈。 |

实际 Task26 推入段使用位置误差和最新 effort 监控，不能把这些判据称为吸附内力控制。`waitSuction` 依赖之前收到的 Bool，未要求与该次命令匹配的新样本；因此其等待成功也不构成完整时序握手。

## 3. 能否估计双侧内部 wrench

当前答案为 **不能以现有数据可靠、唯一地估计**。在约定物体参考点与作用方向后，令堆叠的接触 wrench 为 `w=[w_L;w_R]`，抓取映射为 `G`，则物体净接触 wrench 为 `G w`，某个明确定义的 `ker(G)` 分量可作为内部 wrench。内部与净载荷的分解依赖接触可传递子空间、权重/广义逆、坐标变换及测量模型，并非仅由物体净 wrench 或闭链几何唯一恢复。

最低前提是：确认两个实际约束连接同一物体及其抓取作用点/姿态；取得左右同步、带单位、符号、作用点、坐标系、时间和有效性的接触 wrench，或经过模型补偿、可观测性分析及独立标定的等效估计；将其变换到同一物体参考系，并定义 grasp map 和允许接触子空间。还需处理重力、惯性、工具自重、其它环境接触/支撑以及不确定度。若只有部分力分量，只能报告经验证的可观测子空间，不能凭伪逆创造未测维度。

Task26 的 estimator 还用 joint-state 名称索引另一条 effort 向量，而 ForceBuffer 不保留 effort 名称；这隐含两流顺序一致的未验证假设（[源码](../../ros_ws/src/fr3_dual_palletize/src/task26_truck_box_push_in.cpp#L1940)）。就算输出一个数值，也需要独立 Jacobian、模型补偿、静/动态校准、秩与残差检查，以及同步观测，才可作为候选估计。

“有限阈值代理”与“物理吸附力”必须分开：给聚合 D6 设置有限 break-force/torque，仅能研究该仿真约束在特定后端、积分步、参数和事件策略下的**合成有限附着代理**。只有经可追溯真空压差、有效密封面积、法向/剪切/力矩、偏载和动态标定，并验证与该代理的映射，才能讨论真实侧吸承载余量。四杯几何也不提供逐杯载荷或泄漏模型。

## 4. G2 QP-3 handoff 遗留风险

QP-3 的 5 步结果是保存 NR-0 节点处由合成 twist 和 dt 生成的离线候选；它不是 ROS 消息的发送或真实反馈。必须收紧 QP-3 报告中“joint order / topic 对齐”的表述：它只验证了离线结构，**未证明可以直接交给任一 bridge**。

1. QP-3 [`handoff.py`](../../offline/task28_p5_qp_3/handoff.py#L30) 输出 `left_fr3_joint1..7` / `right_fr3_joint1..7`；Task28 fixture state merger 期望底层 `fr3_joint1..7` 并添加 side 前缀（[merger](../../fixtures/task28_g1_0_new_single_cube_v0/moveit/task28_single_cube_description/scripts/task28_joint_state_bridge.py#L11)），scene 也引用 `fr3_joint*`（[scene](../../fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py#L140)）。fixture bridge 对命令名直接 `articulation.get_dof_index(name)`，没有剥离前缀（[bridge](../../fixtures/task28_g1_0_new_single_cube_v0/bridge/task28_single_cube_bridge.py#L261)）。这是具体潜在不匹配；静态路径强烈提示风险，但未运行 articulation，实际 DOF 清单仍须验证。历史 Task24 publisher 明确去掉左右前缀（[Task24](../../ros_ws/src/fr3_dual_palletize/src/task24_side_suction_tight.cpp#L679)）。
2. 历史 sink 为 `/left/joint_command`、`/right/joint_command`；fixture 是 `/task28_g1_0/{left,right}/joint_command`，sync frame 也各自命名。不同 topic/frame 不会自动互通；离线记录没有任何 publisher。
3. fixture `start_bridge(enable_robot_commands=False)` 默认不建立机器人/吸附命令订阅或 articulation command 写入（[源码](../../fixtures/task28_g1_0_new_single_cube_v0/bridge/task28_single_cube_bridge.py#L77)）。不可将默认遥测模式中的 False 视为真实 OPEN，也不可启用它来“验证”本次审计。
4. 双臂同 stamp 只触发同 physics callback 的配对，不证明原子物理生效、测得 servo period 或确认反馈新鲜。merger 重打 now、缓存左右最新值，缺少源时钟和 age/skew 门；nominal 10 ms 不是已测 100 Hz。无丢包/重复/超时/watchdog、实时延迟或失败停止的系统证据。

因此，实际 position-only bridge 的端到端兼容性、suction 与 QP 的状态互锁仍为 **UNVERIFIED**；不修改历史报告、bridge 或 fixture 来掩盖这些差异。

## 5. 最小下一阶段设计（仅建议，未执行）

建议下一项单独授权的实验仅解决“一个聚合吸附约束的事件和载荷是否可观察”，先不实现双臂力控制。保留现有高阈值基线，在独立仪表化副本中：

- 只读捕获 actual attached body ID、两个 anchor、desired/actual 状态、detach/break 原因、retry/recapture、单调事件 ID，以及 simulation step、source/receive 时间；break 后锁存并记录重试策略。必要的只读接入若无法识别或可靠获得，结果为 BLOCKED。
- 首先证明一个候选 D6 reaction 或独立参考传感接口的 frame、作用点、力/矩单位和正负；用零负载、已知重力方向和已知偏心力矩分别验证，保存源 API、shape、版本、标定、残差和不确定度。若 D6 reaction 不可访问，可评估 incoming-wrench/动力学 observer，但只有完成独立标定与可观测性检查后才作为估计；否则报告 UNKNOWN。
- 在隔离场景用预先冻结的合成有限 D6 阈值做最小事件观察，比较人工 detach、失效/断裂、自动重试与重抓；记录所有事件，禁止将关闭重试或调整阈值伪装为历史行为。实际阈值、载荷、预算及数值容差在具体实验获准后预注册，不能从结果反选。
- 负控包括错误 body、故意缺失/过期或不匹配时间戳、frame/符号/作用点错误、重复消息、break→recapture、未校准和 fallback shape 错误；必须拒绝或显式标 UNKNOWN，不能仅依 `CLOSED` 宣称成功。
- 在工具/状态不明、对象身份不符、连续事件缺失、力值非有限、frame/时间不匹配、标定残差超预设界或未预期断裂时停止并保留全部记录；不借此生成自动释放或实机动作。是否可由单臂仪表结果扩展到双侧 internal-wrench 分解，需之后单独审查可观测性及抓取映射。

此次没有验证候选 API、标定数值、物理容量或运行代价；上述是实验设计，不是 P2 控制器、仿真成功或物理授权。NR-1 困难场景/动态 Cube 维持延期。到 G3-P2-0 停止，等待审阅。

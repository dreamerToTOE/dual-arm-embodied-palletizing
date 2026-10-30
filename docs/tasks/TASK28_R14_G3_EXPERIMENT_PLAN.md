# Task28 R14：G3 最小力控实验与 Isaac 集成路线

基线为 `72fbb8d5ea09ac9edb0a3869a07bed01e243741c`。R14 将研究恢复到 G3；G2 新增扰动是否全部通过，不是独立 G3 离线研究或隔离无接触诊断的前置条件。本文件是源码与历史证据约束下的实验设计，逐阶段的新运行结果另见 R14 主报告和机器证据；未把计划中的实验记为完成。

R12 的 common/relative gain 缩放 `0.75`、启动速度捕获 `τ=0.10 s`、虚拟物体峰值 `0.250 mm`、相对 TCP 平移 `0.150 mm` 及原 QP、关节速度/力矩/变化率约束继续冻结。原始 Cube 接近路径保持 **`CONTINUOUS_SAFETY_NOT_ESTABLISHED`**。这条路径不能用于接触、抓取或搬运；离线 G3 数学和独立单臂探针不依赖其通过。

## 1. 复用哪些成果

| 已有成果 | 本次可复用的功能 | 不能据此授予的能力 |
|---|---|---|
| [P2-0](TASK28_G3_P2_0_OBSERVABILITY_AUDIT.md) | 信号身份、frame/作用点/符号、附着事件和 source provenance 审计 | `CLOSED`、joint effort 或未掉落不能当作接触 wrench；D6 reaction 未连接、未校准 |
| [P2-1](TASK28_G3_P2_1_GRASP_WRENCH_BASELINE.md) | 已测试的 `GraspModel`、加权分解、作用/反作用和力臂变换；不重写 G/W/nullspace | 名义 61 mm 力臂不是实际附着 anchor；数学零空间不是实测内力或吸附容量 |
| [P2-2-R1](TASK28_G3_P2_2_R1_SIGN_AND_STATIC_6D.md) | 六轴静态 oracle、符号负控和 `Jᵀ` 最小范数估计器 | loaded/unloaded 两次 reset 差分不是因果在线反馈；position-drive 结果不迁移为 explicit-effort 校准 |
| [P2-3 / R1](TASK28_G3_P2_3_R1_VIRTUAL_WRENCH_REPLAY.md) | 双侧时序/frame 适配、独立 KKT 和虚拟净载荷/内载分解对照 | 原 P2-3 实际稳定后 G1 几何拒绝仍有效；虚拟回放 PASS 不证明共同接触 |
| [P2-4 observer](../../offline/task28_p2_4/observer.py) | 因果历史、完整 9-DOF 模型、质量矩阵、时间/身份、秩/条件数及残差拒绝门 | 不能自行辨认模型错误、任意多点接触或真实 actuator torque；输出仍是候选 TCP 等效 wrench |
| [P2-4-R3](TASK28_G3_P2_4_R3_STABLE_EXPLICIT_ONLINE_OBSERVER.md) | 独立单 FR3、全九维重力加 PD、240 步零载 hold、已有 ±0.5 N 外力探针协议 | 单姿态模型候选不覆盖原 L 工具、双臂、实体接触、慢速已载运动、真实 FR3 信号 |
| [QP-6 数学基线](../../offline/task28_p5_qp_6/MATHEMATICAL_SPECIFICATION.md) | 已有 G/H 虚功、机器人/物体理想动力学和内部 wrench 目标的离线 HQP 对照 | 自由 18 坐标动力学未通过实际源验证；理想 contact box 未标定，不能打开物理命令路径 |
| [R12 QP](../../offline/task28_p5_qp_6_r12/qp_math.py)、[R8 adapter](../../offline/task28_p5_qp_6_r8/effort_adapter.py) | 共同/相对运动任务、速度/位置前视硬约束，独占 effort 写入、原限幅/变化率和 readback | R12 是速度 QP 后接 torque impedance，不是已经实现了接触力分配的逆动力学 HQP |

## 2. 在线观测器进入接触阶段前的缺口

以下结论来自实际源代码，区别于报告中的研究意图。历史文件不改写；后续修复使用新的实验身份和输出目录。

1. **源语义。** `get_applied_joint_efforts` 是显式命令缓冲回读，`get_measured_joint_efforts` 是 solver projected force，均未证明等价于 FR3 `tau_J`。历史 implicit position-drive source 的 `BLOCKED` 不因 R3 新模式而消失。当前只能用来源明确的完整显式输入计算模型候选残差。
2. **完整模型与时序。** `M qdd+c+g−tau_cmd` 必须对应相同 articulation、坐标顺序和施加区间，保留手指耦合；后向差分不能使用未来帧。原 L 工具/手指 mimic/约束反力不能由隔离单 FR3 的结果代替。保留原始 M，不为通过正定/对称门静默对称化或换成其它姿态模型。
3. **逐步施力 oracle。** [R3 collector 第 54 行](../../offline/task28_p2_4_r3/collector.py#L54) 将最后一次 application point 重复写到整段。非零外力时，这不是逐步实测施力点日志；不能据此升级完整动态力矩参考精度。新的采集器必须在每次施力时记录当步 force/torque、point、frame、step 和作用区间，且 estimator 调用不能读取该 oracle。零外力 hold 不依赖此力臂。
4. **门禁不可只写在报告。** [R3 online evaluator](../../offline/task28_p2_4_r3/online_evaluate.py#L28) 计算 rank/condition，但拒绝条件主要是样本数与三项误差；没有调用 P2-4 的完整 observer contract，也未把 rank/condition 实际接入拒绝逻辑。下一实验应复用已有严谨 API 并测试空数据、NaN、时钟、错序、病态 J、非对称/非正定 M、缺失 oracle 的拒绝，不再另写简化伪通过路径。
5. **控制模式与 rate 的实际含义。** R3 collector 的 `pre_gains` 在清零后采集，`runtime_drive_gains_zero=true` 是赋值；不是独立的前后 gain 证明。其 `effort_rate_bounds` 在第 46 行直接作为每步 `Δeffort` 上限使用，没有乘 dt，不能称为已执行的 N·m/s 门。R8 adapter 的 rate gate 才是明确的 `rate×dt`。下一 G3 采集必须记录实际 gain/mode 前后值、逐关节原始/限幅/施加命令及 `Δeffort/dt`，并在新预注册中明确单位，不能追溯修改旧配置。
6. **标量已知外力不等于接触传感。** ±Fx candidate 通过不覆盖六轴在线响应、受约束的工具动力学、接触位置歧义、瞬时冲击和反馈延迟。当前没有经验证的左右实际接触 wrench 及 body/anchor/event 组合，因此不能宣称闭环物理内部力已实现。

## 3. 按顺序执行的四个阶段

用户已允许无真实机器人命令的 Isaac 研究；该授权与实验是否具备准入证据分别记录在 [g3_stage_admission.json](../../offline/task28_r14/evidence/g3_stage_admission.json)。以下新增要求是 R14 提议的实验合同，不伪称历史 V10-D/FORMAL 的注册条件。V10-D/FORMAL 不是恢复独立 G3 的必要步骤，本任务不自动启动它们。

| 阶段 | 最小实验与复用 | 进入条件与当前状态 | 可得结论 / 必须记录 |
|---|---|---|---|
| S1 无接触源与重力补偿 | 原 P2-4-R3 单臂构型和原控制参数；先重做 240 步零外力 hold；原 collector 外加 R14 拒绝型保护，严谨 observer 独立审计 | 隔离进程、已绑定 USD/版本/配置、无 Cube/吸盘/D6/ROS；输出完整退出与原始数据。本阶段有直接执行依据；R14 新结果按实际 receipt 报告 | q/dq/M/c/g、完整九维命令/回读/projected 值、预/后时刻、重力/PD/饱和/rate、因果零载残差、读/估计耗时；任何严格门拒绝原样保留。仅一个无接触姿态，不是双臂验证 |
| S2 受控仿真接触与单臂力反馈 | 先修复 S1 仪表缺口并独立已知载荷验证，再在单独受控接触夹具中逐步比较开环/闭环 | **当前未就绪。** 需新场景哈希、初始无穿透/关节合法/稳定证据、局部运动连续包络、已定义接触对与接触源的符号/参考点/时序验证、接触力/功率/运动/时间预算和失败停止预注册；没有合格初始构型+局部边证据就不进入接触 | 已知外力探针与 PhysX 接触力 oracle 分开。接触反力候选须独立校准，不能以控制器命令或同一动力学方程残差同时充当真值。记录力响应、偏差、超调、耗时、能量和每一失败 |
| S3 双臂载荷与内部力 | 复用 P2-1 和 QP-6 的 G/W/N；零内力、相同净 wrench 下非零 nullspace 目标、左右非对称分配作最小对照 | **当前未就绪。** 两侧 S2 结果、真实仿真 body/anchor、同 step 的双侧合格 wrench、共同物体 full SE(3)/碰撞/支持状态与安全局部运动证据必须齐全；力矩可行性与接触可传力集合均需验证 | 对齐实际 grasp 几何后计算净 wrench 与定义明确的内部量；独立检查 `G w_int≈0`、重构/功率一致性、物体净力不被内部目标污染、关节 torque/rate、约束激活与失效。仅数学内部目标命中不能当真实内力调节 |
| S4 双臂 Cube 吸附搬运 | 从合格 supported 初始化、附着、短垂直 lift、短 transport、place/release 顺序逐段集成 | **当前未就绪。** S1–S3、有限吸附模型与事件/身份观测、阶段对应 G1/支持/碰撞门和整段连续安全证据先通过；原始接近路径仍禁止。初始场景直接构造只能标明 isolated initialization，不能冒充已完成 approach | 同时记录 Cube 目标/反馈、双 TCP/相对误差、左右六维 wrench、净 wrench、内部量、effort/rate、附着变化/断裂/重抓、任务分母与失败原因；每个真实尝试均计入，不能用理想连接稳定替代吸附稳定性 |

S2 可以首先在不含 Cube 的独立接触夹具研究；这样不会假设原始 Cube 接近路径已通过。初始状态直接在合格构型创建是一种单独的仿真实验初始化，不建立任何到达该构型的路径证据。创建时就有穿透、未审查的隐藏 contact、人工关掉碰撞或修改 ACM，均不满足这一路线。必须同时证明初始状态和实验全程局部运动范围；单个安全样本不证明整个接触过程。

受控无接触已知外力（例如既有 ±0.5 N 探针）可作为 S1 后续 source 检查，但它不产生实体接触约束，不能被命名为 S2 接触验证完成。

R14 S1 的新保护层见 [no_contact.py](../../offline/task28_r14/no_contact.py)：保持原 R3 控制量计算不变，只允许或拒绝写入；arm 绝对 effort 取 R3 与原 R8 研究限值的交集 `[35,12,8,35,4,4,4] N·m`，从第二次写入开始同时检查 R8 的每秒 rate 与原 R3 的每步增量。第一条命令保留原 R3 明确使用的 full-gravity bootstrap，不按后续 rate 从零爬升；因此这不是原 L 场景 R8 的首次命令准入通过，也不能迁移其控制权交接结论。新保护层还读取实际 gain/mode/cursor 并检查注册 acceleration；它不能补造历史 collector 未采到的观测，也不能让 observer 的模型源拒绝消失。Isaac 初始化停滞/超时必须单列为运行失败，即使尚未写命令也不能计作 240 帧完成。

## 4. 最小闭环与分配方法

先完成严格观测器的只读运行，使 `tau_ext` 的输入、时间区间、frame 和不确定性可追踪，再让同一估计进入一个有明确限幅的单轴接触反馈环。外部已知施力/接触 oracle 在独立通道保存，只供评估；力控制器只能读取经过门禁的 estimator 输出。先固定目标位姿/接触轴做力目标从零开始的平滑参考，比较相同场景下“反馈关闭”与“反馈打开”，并保留响应曲线。不能使用 oracle 直接闭环后宣称观测器闭环有效。

最小架构优先复用现有运动层：独立 G3 外层把合格的接触力误差转换为有界任务修正，送入原有共同/相对任务与 QP/effort adapter；R12 已通过的控制器文件不改。具体外层质量/阻尼/增益、力参考、爬升时间、行程、接触刚度/阻尼、能量与停止阈值须依据 S1 噪声/延迟、夹具动力学和剩余力矩预算在运行前预注册。本次没有这些数据，故不提供看似可直接执行的任意数值，也不把旧 ±0.5 N 自动当成接触安全限值。

在 S3，对作用于 Cube、关于实际接触原点的局部 wrench `w=[w_L;w_R]` 使用已有约定：

```text
w_object = G w
w_external = G_W# w_object
w_internal = (I − G_W# G) w
```

实际 contact→object 变换、作用/反作用和 `r×F` 必须来自当前具身份的 grasp 记录。P2-1 的 W 用于度量定义和单位归一化，不能解释为吸附容量。先在离线重算中保证同一净 wrench 目标下修改内部目标不会改变 `G w`，并检查完整动力学/关节 torque 可行性，之后才进入已通过 S2 的实体仿真。

R12 qdot QP 本身没有 contact wrench 或 tau 决策变量，不能把 P2-1 分配结果直接视为可执行力矩。若选择现有 54 维 inverse-dynamics HQP，须先解决原始 constrained articulation 的动力学 source gate；不能借用单 FR3 静态 M/c/g 或伪造约束反力解除该阻塞。另一条外层力反馈方案也必须经过自身稳定性、力矩限幅和接触约束验证。两条方案只选最小可验证的一条，避免同时重写两套控制器。

## 5. 门限、停止规则和吸附假设

| 数值/门 | 权威来源及适用范围 |
|---|---|
| G2 `.250 mm` 物体、`.150 mm` 相对 TCP；arm `.12 rad/s`，研究 effort `[35,12,8,35,4,4,4] N·m`，rate `[2400,600,240,2400,240,240,240] N·m/s` | [R12 contract](../../offline/task28_p5_qp_6_r12/source_contract.json)，冻结不改；G3 使用该运动层时仍须遵守。不是吸附承载认证 |
| G1 双侧物体推定/闭链 `.100 mm`、`.001 rad` | [ClosurePolicy](../../offline/task28_g1_1/task28_validator/closed_chain.py#L9)，在 bilateral 附着阶段适用；G2 的 `.150 mm` 不能替代或放宽 `.100 mm`。detached approach 不伪造附着闭链要求 |
| S1 历史 `dt=1/120 s`、240 步、q1 `≤1.2 rad`、速度 `≤3 rad/s`、九维 effort `[60×7,10,10]`、注册 acceleration `150 rad/s²` | [P2-4-R3 preregistration](../../offline/task28_p2_4_r3/experiment_preregistration.json)，仅同一隔离探针历史协议。R14 实际 arm effort 进一步受上文交集和 R8 rate 保护，不沿用较大的历史 60 N·m 作为有效限值；首条 gravity bootstrap 单独声明。不能迁移为原双臂控制器的新限制 |
| R3 oracle 误差 `.2 N·m` 关节、`.5 N` 力、`.2 N·m` 力矩 | [R3 online evaluator](../../offline/task28_p2_4_r3/online_evaluate.py#L32) 的历史候选验收，不是接触闭环稳定性或真实传感器精度 |
| 新接触力参考、接触容量、力控增益、接触冲击/能量、完成率目标 | **未预注册/未验证**。须来自具体实验来源和不确定性/可行性分析，不能从成功结果反选或为 PASS 放宽 |

现有 [ContactModeValidator](../../offline/task28_g1_2a/task28_contact/validator.py) 和状态/支持规则可复用数值逻辑，但其 `INDEPENDENT_SYNTHETIC` attachment 记录是离线测试类型，不能直接把 command echo 或模拟器 Bool 包装为已验证实际 feedback。仿真接入须建立新的来源适配，证明 request/body/anchor/step、attach/break/release/recapture 事件真实独立。endpoint transition gate 的名称和代码均明确不证明边安全，须另证连续碰撞和附着阶段连续闭链。

每阶段在写命令前检查 source identity、相同步号/时钟、有限性、关节原限位、原速度/effort/rate、控制权与意外 contact；拒绝后不向前推进一个“试试看”的控制步。模拟器故障停止、结果不完整、超预算、断裂、重抓、observer 不可用均保留原始证据并计入失败分母，不能通过重置隐藏。R13 已指出的预命令独立 cursor 绑定问题应在新 G3 adapter 里解决；原 G2 文件保留。

吸附实验必须声明：现有每臂一个聚合 SurfaceGripper/D6 不等于四个独立吸盘；理想 fixed joint 或极高 break 阈值只代表数值连接。有限代理需明示法向/切向/弯矩/扭矩可传力集合、刚度/阻尼、破断/重试逻辑及时间步依赖，并通过已知载荷和 break-event 对照验证。若没有真空压差、有效密封面积、泄漏和动态标定，其结果只能写作**有限附着仿真代理**，不能宣称真实吸附稳定性。

## 6. 下一项可直接开始的 G3 工作

建议任务名：**`G3-P2-4-R4：严格观测器源适配与受控接触预检`**。已授权范围内可直接开始源码/归档分析、负控和隔离无接触诊断；接触运行仍以 S2 准入证据为条件，不用等待 G2 扰动全部通过。

1. 接收 R14 S1 原始数据与严格审计；逐项处理实际失败，尤其是原始质量矩阵、求解器速度时序、显式命令区间与完整 observer contract，禁止只因简化 evaluator PASS 就跳过。
2. 用原 P2-4 API 建立新的 explicit source adapter；为 NaN/空数据/过期/错序/rank/非 SPD/错误施力点建立有意义的负控。修复逐步 oracle、gain readback、rate 单位和 pre-command cursor 记录，仅在新的目录中实现。
3. 先复查一姿态零载和同一已知载荷协议；多轴/慢速动态是后续小矩阵，须预注册而不自动扩大。输出估计可用率、因果残差与源限制，无接触通过只准入进一步预检。
4. 为单独接触夹具输出 scene/config hash、candidate 初始状态、全部接触对、碰撞/支持证据、局部连续运动包络、独立接触 oracle 接口以及有依据的载荷/能量/停止合同。先输出可审查的入场判定；任何未满足项仍 BLOCKED，不能运行 Cube/吸盘。
5. 只有上述证据具备时执行 S2 的最小开环/闭环对照，再审查 S3；S4 不因前一阶段测试函数返回 PASS 自动启动。

这条路线使 G3 能独立推进，同时保留原始接近路径、双臂物理 grasp、接触力来源和吸附容量的未完成状态。真实机器人命令在所有阶段均禁止。

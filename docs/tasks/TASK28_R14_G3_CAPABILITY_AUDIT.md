# Task28 R14：G3 P2-0 至 P2-4-R3 能力复用与遗留问题审计

基线为 `72fbb8d5ea09ac9edb0a3869a07bed01e243741c`。这份审计只读取已有代码、注册文件和机器证据；它本身没有启动 Isaac、接触、Cube、吸附、D6、ROS 或机器人。R14 新实验的执行结果由主报告单列，不能把本表中的历史数据算作 R14 新运行。源文件及证据 SHA-256、准确指标和审计发现见 [g3_capability_audit.json](../../offline/task28_r14/evidence/g3_capability_audit.json)。

**G3 已有可复用的严格观测器、六维 wrench 估计、抓取矩阵、加权内载分解及合成耦合 HQP。尚未建立的是原始双臂 L-suction 场景中的合格接触力观测和内力反馈闭环。** 不需要重写这些数学模块，也不需要等待 G2 全部新增扰动通过才开展独立 G3 离线研究。

## 1. 历史成果逐项核对

| 阶段 | 历史最终结论 | 已有证据与复用范围 | 仍不能声称的能力 |
|---|---|---|---|
| [P2-0](TASK28_G3_P2_0_OBSERVABILITY_AUDIT.md) | `G3_P2_0_PARTIAL` | 明确 desired/CLOSED、实际 attached body、接触变换、D6 reaction、source/timing 缺口；审查真实 libfranka 与 Isaac 信号区别。 | CLOSED 不是载荷测量，也不证明连接了期望 Cube；不能观测物理内力。 |
| [P2-1](TASK28_G3_P2_1_GRASP_WRENCH_BASELINE.md) | `G3_P2_1_MATH_PASS` | 冻结 G1 名义 grasp，6×12 G、rank 6，坐标/参考点转换、加权分配及 `ker(G)` 分解；独立手算/KKT 对照。 | 全六维聚合接触是数学假设；没有吸附容量或实际约束锚点。 |
| [P2-2](TASK28_G3_P2_2_B_ONLY_TORQUE_WRENCH_VALIDATION.md) | `G3_P2_2_PARTIAL` | SVD wrench estimator 和严格输入合同；实际单臂静态零载通过，8 个非零载荷符号不一致被拒绝。 | 不能隐式翻号后追溯改写该失败。 |
| [P2-2-R1](TASK28_G3_P2_2_R1_SIGN_AND_STATIC_6D.md) | `G3_P2_2_R1_STATIC_6D_PASS` | 两构型共 20 工况，独立 joint1 力臂/正负载符号控制；六轴静态估计通过。 | loaded/unloaded 是不同 reset，不是在线同步基线；符号只适用于该原生 position-hold 模式。 |
| [P2-3](TASK28_G3_P2_3_BILATERAL_STATIC_INTEGRATION.md) | `G3_P2_3_BLOCKED` | 双臂同步采集和独立符号通过；九个共同物体集成工况全部因冻结 G1 几何拒绝。 | 不能因 FK 实现一致就忽略 TCP 对名义 grasp 的漂移。 |
| [P2-3-R1](TASK28_G3_P2_3_R1_VIRTUAL_WRENCH_REPLAY.md) | `G3_P2_3_R1_VIRTUAL_REPLAY_PASS` | 九组既有观测的 CPU 虚拟分解；力臂、符号、加权零空间有独立 oracle/KKT 对照。 | 原 P2-3 几何 BLOCKED 保留；没有物体、接触力观测、在线双臂力控。 |
| [P2-4](TASK28_G3_P2_4_ONLINE_B_OBSERVER.md) | `G3_P2_4_PARTIAL` | 严格因果动力学 observer；7 个静态候选通过；无载运动 960 帧可回放。 | 无载运动的 projected effort 动态源门拒绝；已载运动未运行。 |
| [P2-4-R1](TASK28_G3_P2_4_R1_DYNAMIC_SOURCE_AUDIT.md) | `G3_P2_4_R1_BLOCKED` | 历史无载运动时序/差分审计、只读 mode probe；force mode 不等于能读出隐式 drive torque。 | 更小残差、换 lag 或 mode 字符串不能证明隐式 actuator torque 来源。 |
| [P2-4-R2](TASK28_G3_P2_4_R2_EXPLICIT_EFFORT_VALIDATION.md) | `G3_P2_4_R2_SOURCE_PASS_ONLY` | 显式零增益输入源：zero/±0.25 N·m joint1，共 60 步，命令 readback 误差为零。 | 在线阶段在 zero-load 第 36 步 bounded-motion abort，尚未施加 ±0.5 N，不能称为已知载荷估计失败或通过。 |
| [P2-4-R3](TASK28_G3_P2_4_R3_STABLE_EXPLICIT_ONLINE_OBSERVER.md) | `G3_P2_4_R3_ONLINE_EXPLICIT_CANDIDATE_PASS` | 单臂、无 Cube 的九维 gravity+PD 显式 hold 240 步；zero/±0.5 N Fx 各 120 步的因果候选残差评估。 | 仅是该孤立场景的候选模型证据；不等于装有 L-tool 的双臂接触力闭环。下节记录采集/准入缺口。 |

几项重要量化边界：P2-3 零载稳态左/右名义 TCP 偏差为 **0.441309/0.333326 mm**，相对闭链 **0.169917 mm、0.00133642 rad**，超过当时冻结的 **0.100 mm、0.001 rad**。P2-3-R1 最大物体合力/合力矩 oracle 误差约 **0.00265585 N、0.000861043 N·m**，这是虚拟回放值，不能覆盖该几何失败。

P2-4 的无载运动 959 个候选帧中最大关节残差 **0.0091080523 N·m**，仍明确含 `INDEPENDENT_DYNAMIC_SOURCE_SEMANTICS_NOT_VALIDATED`。P2-4-R3 三工况归档最大关节/力/矩误差为 **0.0175200063 N·m、0.0473526515 N、0.0117887716 N·m**，记录的最小 rank 为 6、最大 condition 为 8.6773301。这些正向历史指标没有被本审计重分类为实机精度或接触安全认证。

## 2. 六项能力及复用入口

1. **FR3 力矩接口。** Isaac `get_applied_joint_efforts()` 是显式命令输入缓冲，`get_measured_joint_efforts()` 是 solver DOF 投影力；均不是 libfranka `tau_J`。P2-0/P2-2 已记录 `get_measured_joint_forces()` 的 base 行与六维形状，旧 Task26 fallback 取首列不能作为合格力矩源。真实 `tau_J`、`tau_ext_hat_filtered`、`O_F_ext_hat_K` 仅被只读审计，没有硬件接入验证。R8 [effort_adapter.py](../../offline/task28_p5_qp_6_r8/effort_adapter.py#L55) 可复用完整九维 buffer、双臂身份、无竞争 sink、finger 保持、命令 readback 和 per-joint effort/rate 门。其 Level 1 证书仍是诊断 actuation，不是传感器或完整动力学证书。
2. **无接触外力矩 observer。** 复用 P2-4 [observer.step](../../offline/task28_p2_4/observer.py#L146)：`M qdd+c+g-tau`，完整九维耦合，后向速度差分，第一帧 WARMUP，历史只在有效帧推进。已有 DOF/单位、state/model/J 一致、SPD、时间/age/skew、rank/condition/重构残差合同。输出始终是 TCP-equivalent candidate；模型误差、多处外扰及作用点未知仍可能投影成假想 TCP wrench。
3. **末端六维估计。** 复用 P2-2 [estimate](../../offline/task28_p2_2/estimator.py#L80)，不直接向它传 raw total effort。已有最小范数解、奇异值、可观测投影、不可观测基和残差。P2-2-R1 [adapter.py](../../offline/task28_p2_2_r1/adapter.py) 仅用于该静态配对来源；不能替代在线动力学补偿。单臂隔离 Fx 候选通过不代表原始场景六轴动态验证。
4. **抓取矩阵与内载。** 复用 P2-1 [GraspModel](../../offline/task28_p2_1/grasp.py#L113)、`decompose` 和 `allocate`。输入约定为作用在 Cube 上、各 TCP 原点和局部轴的 `[f;moment]`；输出净 wrench 关于 Cube 中心。`G_W#` 和 `N=I-G_W#G` 依赖声明的 W；`Nw` 不是由净 wrench 唯一测得的物理内力。实际 grasp/anchor、左右观测、动作反动作和参考点移位必须先通过。
5. **协同任务与分配。** 最初 QP-6 [coupled.solve](../../offline/task28_p5_qp_6/coupled.py#L15) 已有 54 维 `[qdd18,tau18,w_local12,a_object6]` 模型、硬 robot/object/contact 等式、严格层级、P3 内载参考和可选线性接触约束。它通过的是合成 SPD/理想刚性接触模型，输出 `physical_contact_established=false`。可作为 G3 离线结合入口；真实场景的同状态动力学、工具惯量、finger 约束、对象惯量和力源尚未满足，不可直接接物理 sink。
6. **QP 与安全。** R12 沿用 R11 形式的 [solve_pose_aware_qp](../../offline/task28_p5_qp_6_r12/qp_math.py#L213)，优化变量是 14 维 `qdot`，硬相对位姿反馈、速度 box、位置 lookahead；源码明确 `inverse_dynamics_claim=false`。R8 gravity+PD 生成 torque，再由 adapter 拒绝越限；这不是力矩直接参与求解的约束 QP。未来 G3 力分配须把总 torque（原任务项与 `J^T` 接触作用项，按统一符号）纳入同一预算，同时保留末端执行前独立 effort/rate 门；不得在 QP 解后无约束叠加内力命令。

## 3. P2-4-R3 新发现与未解决问题

以下是基线源码可直接复核的事实；历史文件和历史 PASS 原样保留，但它们不足以作为下一阶段的完整准入程序。

| 发现 | 证据支持程度 | 对后续实验的影响 |
|---|---|---|
| `collector.py` 把最后一次 `point` 复制成整段 oracle application points | **源码直接支持，归档验证**：[collector.py:54](../../offline/task28_p2_4_r3/collector.py#L54)。±Fx 各 120 帧 point 只有一个唯一值，而 arm q 最大范围分别 0.03570747/0.03569603 rad。 | 缺失实际逐步施力点，不能把该数组当完整逐时刻真值。无法只靠此归档精确恢复所有真实施力点；后续必须逐步记下 pre-step point、力矩、body 和时间，之后才比较 oracle。 |
| R3 collector 没有在采集循环内调用 observer；`online_evaluate.py` 事后按因果差分重算 | **源码直接支持**：[collector.py](../../offline/task28_p2_4_r3/collector.py)、[online_evaluate.py:24](../../offline/task28_p2_4_r3/online_evaluate.py#L24)。 | 能支持因果候选重算；未证明 estimator 在 live 循环中的 deadline、丢帧处理和 estimate→force-command 闭环。 |
| R3 known-load evaluator 记录 rank/condition，却没有 rank、condition、SPD、freshness、正 dt 的显式验收门，也未调用严格 P2-4 observer | **源码直接支持**：[online_evaluate.py:28](../../offline/task28_p2_4_r3/online_evaluate.py#L28)、门判定在第 31–34 行。collector 有正向有限性检查，但不能替代 evaluator 的拒绝负控。 | 归档记录为有限、满秩，不推翻该正向数值事实；该 evaluator 本身不能作为新接触实验的 fail-closed 证书。复用严格 observer，独立验证缺失/NaN/乱序/低秩时拒绝且无后续写入。 |
| R3 `effort_rate_bounds` 被逐步裁剪，未乘 dt；R3 effort bounds 前七维均 60 | **源码和注册直接支持**：[collector.py:46](../../offline/task28_p2_4_r3/collector.py#L46)、[preregistration](../../offline/task28_p2_4_r3/experiment_preregistration.json)。R8 source ceilings 为 `[87,87,87,87,12,12,12]`，并明确 `rates*dt`。 | 旧单位/配置不得静默迁移到原始场景。新 G3 注册必须定义 Nm/s 与每步增量，继承当前实际逐关节 source/research limit 的交集，并包括第一条写入。保留原始历史配置，不通过修改其阈值“修复”证据。 |
| `pre_gains` 记录在 zero-gain 设置后；`runtime_drive_gains_zero` 是直接写入的 true | **源码直接支持**：[collector.py:38–39](../../offline/task28_p2_4_r3/collector.py#L38)。 | 该字段不是驱动切换前后完整实测证明。新采集要分别读入前后 gains/modes，并按值验收。 |
| 单臂裸资产和原始 L-tool 双臂场景不等价 | **既有源码及 G2 源审计支持**：[R6](TASK28_G2_P5_QP_6_R6_ACTIVE_TORQUE_IDENTIFICATION.md)、[R9](TASK28_G2_P5_QP_6_R9_COOPERATIVE_QP_AND_APPROACH.md)。 | mounted-tool 惯量、native finger/mimic/limit reaction、TGS 末步速度与时间区间 residual 不能由 R3 单姿态代替。无接触非零残差应先作模型偏差，不能先认作外力。 |
| R3 只覆盖 zero/±Fx；缺少六轴动态、双侧同步、接触位置识别和 wrench uncertainty | **实验矩阵直接支持**。 | P2-2-R1 的静态六轴结论不能跨控制模式补齐它；新阶段需分开标明已观测轴与未知轴。 |
| P2-3 geometry BLOCKED；吸附 body/anchor/事件/有限容量、D6 reaction 未建立 | **既有正负证据支持**。 | 只读离线 G3 可继续；同一 Cube 闭链/实际接触/内部力闭环与搬运仍须分别通过对应门。 |

上述问题不应通过重新解释历史 oracle、增大容差或拟合最佳 lag/sign 消除。新的 R14 诊断如无法满足某门，应保留 rejection 和原始记录；可以继续不依赖该门的离线数学研究。

## 4. 最小可行验证顺序

**第一步：无接触源/observer。** 先对已有 zero-load 归档独立重算，明确其 replay 身份；再以新的实验身份执行一个有界 fresh zero-load hold，复用已有姿态稳定器，不加入期望外力或内力。登记当前实际 asset、scene、DOF、units、dt、gravity、drive ownership 和每关节 effort/rate/运动界；在第一条写入前核对它们。保存 pre-command 和 post-step 状态、当前 M/c/g、requested/applied/projected 三类量和完整 source interval。采用严格 P2-4 observer 只读估计，拒绝状态不推进历史、不触发力命令；分别报告总 residual、惯性/重力/控制项、TCP rank/condition、bias/噪声/漂移与所有拒绝。静止零残差只能支持零载一致性，不能识别外载符号或校准六轴。

**第二步：单臂受控虚拟载荷，再决定接触。** 先修复新采集器的逐步作用点和强验收合同并做 CPU 负控；使用已核验安全的初态，单一预注册小载荷正负对照，observer 不接收 oracle。已知外力 API 是加载诊断，不等于真实接触响应。若要引入真实模拟接触，需另有实验专属安全初始化/局部运动包络、接触对及作用点、接触材质/刚度/阻尼与稳定 timestep、独立力真值、硬限值及 abort 证据。原始 HOME→Cube 路径保持 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`，不能执行它来取得初态；可研究新的独立安全初态，但必须事先验证其构型、初始重叠、闭链/支撑和局部全过程。

**第三步：双臂分配与内力。** 先用已有 G/weighted split/HQP 做 CPU 接口和 sign/frame/point 测试；同一物体实际双侧 source 合格后，运行零内载参考、同向净载、相反接触线和偏置力偶等可辨识对照。净 wrench 与内载要分别有 oracle，`G Nw≈0` 仅是代数检验。逐关节计算总 effort 预算、速率以及相对位姿，闭环增益和目标必须在观察结果前登记。只有观测器、接触模型和力分配均合格，才运行有界内载阶跃/释放；不得把分配值当测得值。

**第四步：Cube 吸附搬运。** 单独审查初态/路径连续安全、有限附着模型和事件管线。现有每臂一个 SurfaceGripper、`forceLimit=torqueLimit=1e6`、retryClose 的历史设置是聚合理想连接近似，不表示真实四杯密封、真空容量、抗剪/抗弯或漏气稳定性。必须声明连接可传递子空间、有限 break 规则、实际 body/anchor、break/detach/recapture 序列、反力/估计来源及重力/惯性假设；未掉落不能替代这些验证。通过后才讨论短距离 Cube 搬运，并同时报告 Cube/相对位姿、左右 wrench、净/内载、effort、连接事件、完成率和失败原因。

目前可以直接开展的 G3 工作是：**新身份的无接触严格 observer 适配及零载检查、P2-4-R3 新采集器的 oracle/时间合同负控、已有 P2-1/初始 QP-6 数学接口联合离线验证。** 它们不要求 G2 大规模扰动继续通过。实际接触、吸附和 Cube 运动属于后续有独立门禁的阶段；R14 授权仿真研究没有消除这些证据前提。

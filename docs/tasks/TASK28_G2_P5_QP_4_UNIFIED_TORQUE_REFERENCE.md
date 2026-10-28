# Task28 G2-P5-QP-4：统一 torque-control reference handoff

最终状态：**`G2_P5_QP_4_TORQUE_REFERENCE_PASS_OFFLINE`**。

本任务完成了来源锁定的双臂运动 reference 合同：从已接受 QP-3 的 5 步 NR-0 节点 → QP-2 数值结果，精确复制左右 `q_next/qdot`、物体位姿和原始接触模式，供未来 torque controller 的接口研究使用。独立重验全部通过，原 Cube/table 穿透负例仍被拒绝。PASS 仅限离线合同，不表示 torque 可行、动态跟踪、实际双臂动作或接触力控制通过。

## 1. 起点、来源和范围

开始核对的本地 HEAD、origin tracking ref 和 `git ls-remote origin refs/heads/side-suction-palletizing` 均为 `44710179f2ff24ba5c8eea62c3b2818d66026821`，分支为 `side-suction-palletizing`，起始工作树干净。后续中断续作时仅存在本任务新增文件。所有原有受 Git 跟踪文件、fixture、ACM、Task24/26/27、G1/G2/G3 算法和历史证据保持不变。

来源链为：

```text
NR-0 seed28/result.json.gz : transport[40] (共 81 个 transport 节点)
    → QP-2 preregistration : seed28:transport:midpoint40
    → QP-3 run_handoff : 从该节点出发的固定 object twist、dt=.02、5 步
    → QP-3 archived handoff_records.json : accepted q_next/qdot/geometry
    → QP-4 双臂同步运动 reference，execution_authorized=false
    → 未来 torque backend（仅定义接口要求）
```

**这五步是从 NR-0 一个保存节点出发的局部 QP-2 示例，不是整条 NR-0 几何路径的时间参数化或完整路径跟踪。** `recheck.py` 逐值验证 NR-0 原节点、QP-2 预注册节点和 QP-3 初态相同；没有重新规划或替换路径。

信任根为 [source_binding.json](../../offline/task28_p5_qp_4/source_binding.json)。它固定输入、模型和关键源码的 SHA-256；缺文件或哈希不符立即拒绝，不自动更新绑定。fixture 由原 `verify_fixture` 验证 manifest 和每个内部文件。换轨迹、模型或合同需显式修订此绑定，当前 adapter 不声称支持任意外部轨迹。

## 2. 代码级迁移审计

分类：A＝控制模式无关、原样复用；B＝可复用但需下游接口变更；C＝缺少动力学/torque 前提；D＝旧 position-only 接口特有。详细源文件和行号保存在 [migration_matrix.json](../../offline/task28_p5_qp_4/migration_matrix.json)。

| 模块/部分 | 分类 | 源码依据与判断 |
|---|---|---|
| NR-0 planner / projection | A | `Planner.plan` 保存 `(T_WO,q14)`、父边和几何回溯；验证中的合成时间没有定义 actuator timing。 |
| QP-0 core / QP-1 driver | A | `velocity_maps`、14-DOF box/equality QP 与有限步几何门可保留；输出单位是 rad/s。QP-1 的步进仍是离线预测。 |
| QP-2 feedback / independent driver | A | 完整 SE(3) 误差反馈与仿射速度兼容性在运动学域有效，独立有限差分与数值验收不依赖采用位置还是力矩低层。 |
| QP-3 Euler adapter | B | `make_position_targets` 中 `q_next=q+dt*qdot` 保留为离线参考；新增独立下游 schema。 |
| QP-3 PositionTarget、旧位置 sink | D | 位置目标字段是旧桥语义；本任务不提供其发送路径或隐式 fallback。 |
| G1 模型、grasp、FCL/G1/B0 门 | A | 原 FK、SE(3)、限位、payload/robot/table/ACM 几何验收继续使用；没有新增动力学结论。 |
| P2-1 grasp-wrench 分解 | A | 六维 wrench 的参考点、坐标变换、`G` 和 weighted decomposition 数学可复用。 |
| P2-1 物理内力估计 | C | `request_physical_estimate` 仍以缺少标定的双侧接触 wrench 返回 BLOCKED。 |
| R3 单臂姿态保持/观测器 | C | 可作为隔离已知负载证据；运动跟踪、双臂时序和完整观测拒绝门尚缺。 |
| 历史 Task24 / frozen fixture bridge | D | `_joint_command` 读取 `name/position`；`_apply_joint_command` 创建 `ArticulationAction(joint_positions=...)`。 |

QP-0/1 中，`B_i=H_i^-1 J_i` 将关节速度映射到物体中心世界 twist，`C=[B_L,-B_R]`。QP-2 使用实际隐含物体位姿 `X_i=FK_i G_i^-1`，令 `A_R=R_L R_R^T`、`phi=Log(A_R)`：

```text
p_L_dot - p_R_dot = -k_p (p_L-p_R)
omega_L - A_R omega_R = -k_R phi
d(phi)/dt = J_l(phi)^-1 (omega_L - A_R omega_R)
```

标量增益下 `J_l(phi)^-1 phi=phi`，因此反馈方向与其误差微分一致。QP-2 以该仿射关系取代齐次 `C*qdot=0`；旧 `C*qdot` 只作对照诊断，不能要求反馈时仍为零。object-level tracking correction 与 SO(3) 主值分支限制保持原实现。该数学成立不保证 torque backend 能实现期望速度，必须重新测量真实闭链误差。

关节运动学 null space 的变量为 `qdot∈R^14`；P2-1 的内部 wrench 满足 `G w_int=0`，变量为左右各 6 维 contact wrench、共 `R^12`，单位为 N/N·m。两者不是同一空间；转换到 joint torque 还需正确的 `J^T`、力方向、TCP 参考点、接触模型和动力学。新增测试用原模型验证 `C.shape=(6,14)`、`G.shape=(6,12)`、两者秩为 6，并拒绝把 14 维 joint velocity 输入 `G`。本任务不提供任何这样的错误转换。

## 3. Reference 数据与时间合同

实现：[reference.py](../../offline/task28_p5_qp_4/reference.py)。产物：[reference_contract.json](../../offline/task28_p5_qp_4/evidence/contract_v1/reference_contract.json)、[accepted_reference_sequence.json](../../offline/task28_p5_qp_4/evidence/contract_v1/accepted_reference_sequence.json)。

| 字段 | 精确语义 |
|---|---|
| Cube / fixture | `Cube_01`；`task28_g1_0_new_single_cube_v0`；绑定原 URDF、fixture manifest、G1 grasp 源码、NR-0 和 QP-3 archive 哈希。 |
| joint order | 左 `left_fr3_joint1..7`，右 `right_fr3_joint1..7`；不依赖 unnamed array 顺序猜测。 |
| `left/right_q_rad` | 保存 QP-3 的区间终点 `q_next`，精确复制，不重投影或 reset。 |
| `left/right_qdot_rad_s` | 区间起点 `t_k` 计算并用于该步 Euler 积分的 QP 速度。它与终点位置一起描述原区间，**不是已认证光滑轨迹在终点的导数**。 |
| `time_s/stamp_s` | 合成 `[t_k,t_{k+1}]`，dt=.02 s，总数值时长 .1 s；共享 pair sequence、左右 stamp 和源计算时刻。 |
| object pose | `T_WO`，列向量、右乘局部点变换至 world，米；完整 4×4 SE(3) 旋转/平移，来自该步 `desired_object_pose_next`。 |
| `frame_id` | `task28_g1_0_dual_sync` 仅保留 QP-3 来源帧。新 schema 无 ROS message 或 bridge 路由。 |
| motion intent / contact mode | `A_STAGE_TRANSPORT_REFERENCE_UNCONFIRMED` / 原始 `bilateral_transitioning`。 |
| confirmation / release | `NOT_RUN` / false。QP-3 没有生成 R1 confirmation observation，不能升级为已确认 airborne 或 supported。 |
| acceleration | `null`、`UNAVAILABLE_NO_VALID_TIME_PARAMETERIZATION`；零填充和有限差分值均拒绝。 |
| geometry / safety | 原 accepted state、sampled edge、closure、QP status；`HASH_BOUND_ARCHIVED_ACCEPTANCE_NOT_LIVE_FEEDBACK`；连续安全 `NOT_CERTIFIED_FINITE_SAMPLING_ONLY`。 |
| authorization | 全部 `execution_authorized=false`。 |

没有插值器、伺服 resampling、加速度轨迹或 torque 字段。QP-1/2 的 `(qdot_k-qdot_(k-1))/dt` 仍仅是数值诊断，不能认证 actuator acceleration。保持原 `bilateral_transitioning` 也不意味着物体受到桌面支撑；实际 Cube 几何和支撑例外继续由 G1/R1 门判断。

输入必须匹配已固定来源，验证成功后再构造 typed reference。输出校验逐字段与 hash-bound 来源重建值比较，拒绝改值、身份/模式混淆、来源冒充、错误模型、安全标签升级和执行开启。sequence 必须连续、时间严格递增、左右 stamp 同值。`validate_sequence(now_s,max_age_s)` 可检查**合成回放游标相对于末帧的年龄**，二者必须同时显式给出；这不测量 actuator feedback freshness，既没有默认物理 age 阈值，也不将历史 archive 过期与实时反馈过期混淆。

## 4. 未来 torque backend 合同（设计，未实现）

机器可读设计：[torque_backend_contract.json](../../offline/task28_p5_qp_4/torque_backend_contract.json)。R3 实际记录仅来自单个 `/World/ProbeFR3` 的 9-DOF；双臂部署仍需在初始化时逐个核验实际 articulation names、资产哈希和 frame，失败即禁止启动。

| reference 索引 | reference 名字 | 未来 fixture articulation root | 实际 DOF 名称 | 核验后的期望索引 |
|---|---|---|---|---|
| 0..6 | `left_fr3_joint1..7` | `/World/Task28G10SingleCube/left_fr3` | `fr3_joint1..7` | 0..6 |
| 7..13 | `right_fr3_joint1..7` | `/World/Task28G10SingleCube/right_fr3` | `fr3_joint1..7` | 0..6 |
| 无（左） | 无 | 同左 root | `fr3_finger_joint1/2` | 7/8 |
| 无（右） | 无 | 同右 root | `fr3_finger_joint1/2` | 7/8 |

arm 为 revolute：位置 rad、速度 rad/s、torque N·m。四个 finger DOF 为 prismatic：m、m/s、effort N；不允许把 14 维数组直接当两套 9 维命令，不允许隐式补零。finger 的保持/禁用/受控策略、模型耦合和边界需要独立确认；当前 reference 明确没有 finger 输出。

Isaac R3 `collector.py` 设置 force effort mode、runtime implicit drive gains 为零，并提交全 9 维 `g(q_pre)+PD`。applied-effort readback 是 command/input provenance，measured projected effort 是求解器投影力，均不能自动视为真实 `tau_J`。真实 libfranka 用户 torque 排除由机器人补偿的重力/摩擦；官方接口文档明确此语义，因此不能把 Isaac 的含重力完整命令原值转发到真实机器人，否则存在重复补偿风险。部署仍须锁定实际 libfranka/控制器版本与负载配置。[官方 Torques 接口](https://frankarobotics.github.io/libfranka/0.20.2/classfranka_1_1Torques.html)

未来同步输入至少包含左右 q/dq、joint names、clock domain/epoch、源采样 timestamp、接收 monotonic time、观测序列和 applied command sequence。需定义并测量 age/skew/截止时间、暂停恢复和乱序策略；任一侧 missing/non-finite/stale/async/rank-invalid 必须锁存双臂故障，禁止一侧继续。不能只用同一个包的 stamp 就宣称测量同步。

commanded torque、applied readback、measured joint torque、estimated external torque 必须分别记录。模型式 `tau_ext=M*qdd+c+g-tau_cmd_total` 需要同一状态/时刻的 M/c/g、全部驱动力矩、已知符号和 payload/finger 模型；`J^T w=tau_ext` 还需秩/条件数、TCP 参考点和时间门。真实接口若只提交除重力的命令，估计器必须按其语义构造 total actuation，不能重复加减重力。

移动双臂的 torque limits、torque-rate limits、physical acceleration limits、servo period、feedback age/skew 阈值均为 **UNKNOWN/未认证**，机器合同使用 null。R3 的隔离研究 bounds 不自动推广。后续需记录请求/实际饱和和限速值，超界停止且保存失败；不得把静默 clipping 当成原参考仍可行。故障停止策略必须双臂协同，保持载荷/吸附的安全状态；未设计的保持上次 torque 或归零 torque 都不能作为默认 fallback。

实际闭链验收必须基于反馈 q/dq 和物体观测；几何预测仅是前置条件。真实 attachment identity、抓取变换、break/detach、支撑确认与有限吸附能力仍需采集和标定。CLOSED 或 configured break threshold 不等于力测量。post-hoc sampled FCL 不提供 QP collision avoidance，也不提供连续安全证书。

旧桥风险完整保留：side-prefixed 名称与 articulation 无前缀 DOF 不匹配；Task24 legacy `/left|right/joint_command` 与 fixture `/task28_g1_0/left|right/joint_command` 命名空间不同；fixture 命令处理默认 `False`；nominal 10 ms 不是实测 servo period；反馈年龄、左右 skew、实际应用延迟仍缺测量。本任务没有修改或激活桥。

## 5. R3 历史结果与集成限制

R3 的 `G3_P2_4_R3_ONLINE_EXPLICIT_CANDIDATE_PASS` 作为历史单臂候选结果保留；本任务只静态检查其源码和保存数据，不重新运行物理场景。

1. `collector.py:54` 在循环结束后将最终 `point` 重复写入全部 `application_points_world_m`，没有保存真实逐步施力点。未来运动场景需每步记录施力点、刚体姿态和前后时刻。
2. `online_evaluate.py` 计算 rank/condition 并输出，但没有完整 rank/conditioning/source-freshness rejection gates；不能将历史报告的 rank≥6 描述扩展为已实施的完整门禁。还需 source time 单调性、完整帧/有效估计行数及非有限值等拒绝策略。
3. R3 验证 posture hold 和已知外力估计；未证明 moving trajectory tracking、实际 Cube 吸附、双臂内力调节或硬件 torque-sensor 等价性。

这些问题列为未来 observer/controller 接入前提；不改写 R3 代码或历史结论。

## 6. CPU 独立验证和量化证据

[recheck.py](../../offline/task28_p5_qp_4/recheck.py) 读取保存 `q0/q_next/qdot`，调用原 QP-2 独立仿射关系重建、原 frozen bounds、原 G1/B0 current/predicted/edge 门。没有调用 optimizer solve、FeedbackDriver.run、NR search 或 reset。聚焦测试还把 solve/run 替换成必抛异常，证明 archive recheck 不依赖新求解。历史回归可调用原有 CPU 数值测试；所有历史文件保持原样。

| 指标 | 本次重验 |
|---|---:|
| 同步 reference 数 | 5 pairs / 14 DOF |
| q_next/qdot 相对保存结果 | 逐元素完全相同 |
| 最大 Euler 重构误差 | `1.8046545161021221e-16 rad` |
| 最大硬仿射等式残差 | `2.4109839931527998e-17` |
| 最大速度 tracking 平移/旋转误差 | `1.5946256646312367e-7 m/s` / `1.297499619014373e-5 rad/s` |
| 最小位置/速度裕度 | `0.4954808167383442 rad` / `2.597554536475771 rad/s` |
| 最大 predicted 闭链平移/旋转误差 | `5.8785074337285e-7 m` / `1.1470538213457013e-6 rad` |
| original G1 current/predicted | 5/5 对均有效 |
| B0 sampled edges | 5/5 有效，共 15 个 evaluated samples |
| 原 Cube/table 负例 | 拒绝，保留 `COLLISION:Cube_01/box\|task28_table/box` |

[negative_controls.json](../../offline/task28_p5_qp_4/evidence/contract_v1/negative_controls.json) 保存 26 项控制：错误/缺失来源身份、错误 Cube/帧/关节序、异步左右时间和源时间、错误 step、错误 q/qdot、断裂初态链、伪造 provenance/phase、虚假 confirmation、开启执行、上游 QP infeasible/missing qdot、current/predicted geometry rejected、bounds invalid、edge inconclusive、伪造 acceleration、重复/stale/future stamp，以及**原 QP-3 穿透记录**。26/26 按指定原因拒绝；除原负例外均明确是软件合同 fault injection，不声称新物理失败实验。

聚焦测试另覆盖无效 SE(3)、non-finite 值、模型/安全状态更改、缺依赖、archive 哈希篡改、kinematic/wrench null-space 域区分和无 ROS/Isaac/actuation import/call。失败 candidate 不发往任何 sink；函数整批失败，CLI 不输出 accepted batch。

历史回归 **789 passed，97.72 s**，退出码 0；包含 731 项原跨任务测试和 P2-4/R1/R2/R3 的 58 项。完整 argv、PYTHONPATH、Python/平台、日志哈希和退出码保存于 [regression_historical/record.json](../../offline/task28_p5_qp_4/evidence/regression_historical/record.json)。早期两次导入路径错误及一次中断后无法回收结果的运行在 `development_checks.json` 中如实记录，不算通过。测试启动器补全历史 module/test-helper 路径，没有修改旧测试。

QP-4 最终聚焦 **53 passed，5.61 s**、退出码 0；独立目录复现的 7 个文件科学数据全部相同、退出码 0。详见 `regression_focused/record.json`、`reproducibility_record.json` 和 manifest。复现只排除 `computation_time_s` 比较科学数据，reference 数值、状态和拒绝理由要求相等。临时目录不作为持久证据；完整首轮数据、比较结果、哈希和日志已归档。

## 7. 复现与交付

从 repository root 执行，下列 output 目录须不存在：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=offline:. python3 -m offline.task28_p5_qp_4.generate_evidence --output /tmp/qp4-review-new
PYTHONDONTWRITEBYTECODE=1 python3 -m offline.task28_p5_qp_4.run_checks --suite focused --output /tmp/qp4-focused-new
PYTHONDONTWRITEBYTECODE=1 python3 -m offline.task28_p5_qp_4.run_checks --suite historical --output /tmp/qp4-historical-new
```

运行环境为 Python 3.10.12、NumPy/SciPy、原冻结 FCL CPU backend；版本、命令、源与产物 SHA-256 在 `artifact_manifest.json` 和各运行 record 中。全新输出目录拒绝覆盖；科学输出不依赖随机新样本。B0 源、geometry 和 ACM 继续由已有验证链约束。

新增范围只有本报告和 `offline/task28_p5_qp_4/`：typed contract/来源绑定、fault controls、原门重验、证据生成、回归启动器、测试、迁移矩阵、未来 backend 合同和机器证据。没有新的动力学控制器或执行桥。

## 8. 剩余边界与停点

本次证明 source-identified、同步、可复现的离线运动 reference；没有证明整个 A-stage 路径跟踪、可执行时间参数化、动态 torque 可行性、双臂真实反馈时序、有限吸附能力或内力控制。fixture 继续标记 `NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`，连续闭链/碰撞安全仍未认证。

下一步必须独立授权并验证真实/Isaac torque backend 的名称映射、四个手指处理、时钟/序列、动力学、限幅/限速及双臂故障停止；之后才能评估移动跟踪和接触控制。此处停止 QP-4，不自动开始双臂 torque controller、Isaac、ROS、NR-1、SVM、PCS/RSS 或 RL。

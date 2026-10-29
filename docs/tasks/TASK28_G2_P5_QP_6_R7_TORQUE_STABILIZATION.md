R7 的反馈控制没有达到预注册的稳定诊断条件：S1 和 S3 在原始 L 形吸盘场景中完成了 30 步、双臂各 30 次的九维 effort 写入，但实际 TCP 平移漂移为 2.05–3.67 mm，超过 1.0 mm 诊断门槛；S2 两次均在 Isaac 启动阶段中止，未写入 effort。正式静态保持没有运行，移动跟踪没有运行，任务停在诊断阶段。下一步需要先解决独立进程启动/生命周期问题，并重新建立满足门槛的状态转换与平滑时间参数化；本 R7 不再调参或重跑正式阶段。

# Task28 G2-P5-QP-6-R7：Original L-Suction Dynamics Qualification and Bilateral Torque Stabilization

- 仓库：`dreamerToTOE/dual-arm-embodied-palletizing`
- 分支：`side-suction-palletizing`
- R7 起始 HEAD：`759293ad862dac74a35b979007929ef59227b29f`
- 场景入口：`fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py::build_scene`
- 运行日期：2026-10-29（Asia/Seoul）
- 运行范围：CPU Isaac Sim、原始双 FR3、原始 L 形 side-suction 几何、60 Hz TGS、无 ROS、无吸盘闭合、无 Cube 接触/搬运、无硬件。

## 最终状态键

| 键 | 值 |
|---|---|
| `ORIGINAL_SCENE_PRESERVED` | `PASS` |
| `R6_ACTIVE_TORQUE_EVIDENCE_PRESERVED` | `PASS` |
| `DYNAMIC_MODEL_SCOPE` | `SCOPED_PARTIAL_MEASURED_M_C_G` |
| `TOOL_INERTIA_STATUS` | `UNKNOWN` |
| `DIAGNOSTIC_STATIC_STABILIZATION_STATUS` | `FAIL_OR_INCOMPLETE` |
| `FORMAL_STATIC_HOLD_STATUS` | `SKIPPED_DIAGNOSTIC_PREREQUISITE` |
| `SHORT_MOVING_TRACKING_STATUS` | `SKIPPED_FORMAL_STATIC_PREREQUISITE` |
| `COMPUTED_TORQUE_STATUS` | `SKIPPED_MODEL_NOT_QUALIFIED` |
| `COOPERATIVE_QP_STATUS` | `SKIPPED_NO_MOVING_CONTROLLER` |
| `ACTUAL_CUBE_SUCTION_NOT_TESTED` | `true` |

机器可读副本：`offline/task28_p5_qp_6_r7/evidence/final_status.json`。

## 保护范围和来源审查

R7 只在新目录 `offline/task28_p5_qp_6_r7/` 增加代码和证据。本次没有修改原始 fixture、USD、机器人描述、G1/NR/QP 历史代码、ACM、FCL 或 R6 证据。原始 FR3 资产哈希仍为 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`；场景 builder 的冻结校验记录在每个诊断结果的 `scene_before` 中。

来源审查先于新实验，并记录在 `offline/task28_p5_qp_6_r7/literature_and_source_review.md`。审查了 R3–R6、R6 effort adapter/source gate、原始 G1 fixture/FK/FCL、安装的 Isaac articulation 调用，以及 Franka torque/impedance 和 Pinocchio RNEA/CRBA/ABA 资料：[franka_interactive_controllers](https://github.com/nbfigueroa/franka_interactive_controllers)、[libfranka overview](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html)、[Pinocchio dynamics](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_a-features_g-dynamic.html)、[Isaac articulation API](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.prims/docs/index.html)。这些资料用于确认 torque-domain gravity/impedance 的形式、硬件 FCI 时序不能直接转移到 60 Hz TGS、以及 end-step solver `dq` 不能直接当作独立测得的加速度。

## R6 独立重算

`reanalyze_r6.py` 只读取 R6 A/B/C/D 原始 trace，没有发送物理命令。每个 cell 完成 18 个 command step、36 次双臂九坐标写入（合计 144 次），command 与立即 readback 字节级一致；B/D 重复轨迹逐关节完全一致。所有 cell 的 ownership transfer 在 physics step 182，首个 effort command 在 step 183。接管时的 arm `dq` 已保存；例如左臂第二关节约 `-0.0107249 rad/s`、第四关节约 `0.0050073 rad/s`，右臂第二关节约 `-0.0105853 rad/s`、第四关节约 `0.0051315 rad/s`。因此 A 的漂移包含从 native position drive 到 effort ownership 的状态转换影响，不能作为静态保持通过。

R6 的原始 A（gravity-only）TCP 平移漂移约为左 `2.427642 mm`、右 `1.721612 mm`。B/C 的正负 joint-4 注入分别为左 `+0.0270386/-0.0248871 rad`、右 `+0.0266037/-0.0243301 rad`；最大 arm 关节漂移分别达到约 0.0374/0.0364 rad。基线扣除后的符号对称量很小：左臂各关节的 `B + C - 2A` 最大绝对值约 `2.06e-4 rad`，右臂约 `1.98e-4 rad`；同时存在明显的 cross-joint motion，不能把 joint-4 注入视为只作用于 joint 4。

R6 末步模型残差估计沿用历史定义 `||M * ((dq_post-dq_pre)/dt) + c + g - tau||₂`（只取七个 arm 坐标）：独立重算最大值为 `0.05492453116950269 N·m`，与历史值一致。该量使用 TGS end-step `dq` 的有限差分，只是估计量，不是 torque sensor 残差。

## 动力学范围与工具惯量

每个 articulation 按 `q9=[q_arm7,q_finger2]` 分块，检查了

`Maa*qdd_arm + Maf*qdd_finger + c_arm + g_arm = tau_arm + external_generalized_effort`。

检查包括 `M` 的有限性、对称性和 SPD，same-step `q/dq/M/c/g`，whole-step `dq` 有限差分、由 `M` 解出的 integration prediction，以及 `Maf` 的手指耦合。R6 四个 cell 的 18 个控制间隔全部通过数据形状和 SPD 检查；重算结果保存在 `evidence/model_inspection.json`。典型 integration-prediction 速度误差最大值为 A 左 `2.91e-4 rad/s`、B 左 `8.08e-4 rad/s`、C 左 `1.19e-3 rad/s`，随扰动增大而增大；这支持“模型观测可作限定范围诊断”，不足以证明同状态双臂 computed torque 可行。

工具惯量保持 `UNKNOWN`。冻结 builder 在 link8 下创建了 L 形 collider 子几何，但没有可唯一归属的 payload mass/inertia 声明；独立 PhysX `MassAPI/RigidBodyAPI` 只读检查在启动阶段中止，没有改动场景。运行时可见的 9×9 `M` 仍记录了 SPD 最小特征值约 0.00901（左）和 0.00895（右），但不能把它反推出工具质量。没有人为添加 payload、修改 USD 或修改物理场景。因此模型结论是 `SCOPED_PARTIAL_MEASURED_M_C_G`，computed torque 保持跳过。

五个负例均被拒绝：错误动力学符号、非 SPD mass、伪造传感器形状、非法时间戳和错误手指假设。机器证据位于 `evidence/model_inspection.json`。

## R7 控制器和门控

控制器在 ownership transfer 读取实际 source-validated `q_hold`，使用

`tau_arm = g_arm(q) + Kp*(q_hold - q_arm) - Kd*dq_arm`。

固定增益为 `Kp=[1.2,1.2,1.0,0.8,0.5,0.4,0.3] N·m/rad`，`Kd=[0.18,0.18,0.16,0.14,0.10,0.08,0.06] N·m·s/rad`。S1 只使用 gravity 加小阻尼；S2/S3 使用 gravity 加 PD hold。控制器保留 native finger gains、mimic policy、targets 和 force modes，并通过 R6 的完整九维 effort adapter 写入和立即读回。每次 command 前执行 source、时间戳、effort/rate、竞争 position-drive、FCL/clearance 和场景身份检查，再推进恰好一个 60 Hz physics step。

冻结 source arm effort 上限为 `[87,87,87,87,12,12,12] N·m`。初始预注册的 `[6,6,6,6,4,4,4] N·m` 在任何闭环写入前被 S1 cap gate 拒绝，因为观测到 joint-4 gravity 约 `28 N·m`；该负例保存在 `evidence/diagnostic/S1_cap_reject/`。随后在保持 source 上限的条件下，将研究 cap 修订为 `[35,12,8,35,4,4,4] N·m`，并在 `preregistration.json` 记录 revision。该修订不是调参结果，而是允许 source-valid gravity compensation 表示实际接管状态。

## 诊断实验结果

预注册间隔为 ownership transfer 后 30 个 physics steps；单独定义的 TCP 平移门槛为 1.0 mm，arm 关节位置门槛为 0.02 rad，速度门槛为 0.30 rad/s。冻结 G1 的 0.100 mm grasp threshold 没有用于 FREE_HOME。

| cell | 结果 | effort commands | TCP 平移漂移（左/右） | 最大关节漂移（左/右） | 说明 |
|---|---|---:|---:|---:|---|
| S1 | `DIAGNOSTIC_STABILIZATION_COMPLETE` | 30/臂 | 3.6668 / 2.0825 mm | 0.005171 / 0.005110 rad | source-valid、无意外碰撞；TCP 门槛失败 |
| S2 | `PROCESS_ABORTED` | 0 | 未知 | 未知 | 两次启动失败：SIGSEGV(139)、SIGABRT(134)；无 serialized frame、无 command write |
| S3 | `DIAGNOSTIC_STABILIZATION_COMPLETE` | 30/臂 | 3.6426 / 2.0522 mm | 0.004988 / 0.004925 rad | source-valid、无意外碰撞；TCP 门槛失败 |

S1 和 S3 的关节位置、速度、effort/rate、手指策略和碰撞门均通过，失败点是实际 L-tool TCP 平移漂移。两次有效 PD 运行的 TCP 结果接近，说明反馈写入路径可重现，但在本注册的 1 mm diagnostic TCP 条件下不能声明稳定。S2 没有物理结果，不能用来比较 S1/S3。

独立评估输出为 `evidence/evaluation.json`：`diagnostic_eligible=[]`、`diagnostic_stabilization_status=FAIL_OR_INCOMPLETE`。完整的启动日志位于 `evidence/process_logs/`，S2 结果位于 `evidence/diagnostic/S2/result.json`。

## 正式阶段和停止理由

正式静态保持要求所有诊断 cell stable、source-valid，并且只运行一次 60-step fresh hold。S1/S3 的 TCP 漂移超门槛，S2 没有有效状态，因此正式静态运行被预注册 gate 跳过；没有正式失败后的调参或重跑。对应记录为 `evidence/formal_static_hold_skip.json`。

由于正式静态没有 PASS，短移动 tracking、平滑 C2 reference、Cube 接触、吸盘闭合和 transport 均没有发出。由于 mounted-tool inertia 未被独立识别且同状态双臂 model qualification 未完成，computed torque 也被跳过。协同 QP 状态为 `SKIPPED_NO_MOVING_CONTROLLER`。对应 skip 证据为 `moving_tracking_skip.json` 和 `cooperative_qp_skip.json`。

## 回归与可复现性

R7 复用了独立回归 runner，未启动 Isaac physics，历史套件总计 `1015 passed`：历史基础 789、QP-4 53、QP-5 5、QP-6/R1 30、QP-6/R2 12、QP-6/R3 7、QP-6 54、QP-5/R1 6、QP-5/R2 22、R4 focused 12、R5 focused 13、R6 focused 12。证据和 SHA256 位于 `evidence/regressions/`。R7 自身的四个离线测试为 `4 passed`，验证 source cap、负例、评估门和状态文件结构。

## 证据索引

- 报告配套目录：`offline/task28_p5_qp_6_r7/`
- 预注册和 source contract：`preregistration.json`、`source_contract.json`、`registrations.json`
- 实验运行时：`runtime_stabilization.py`、`effort_adapter.py`、`source_gate.py`
- R6 重算：`reanalyze_r6.py`、`evidence/r6_reanalysis.json`
- 动力学检查：`model_inspection.py`、`evidence/model_inspection.json`
- 诊断评估：`evaluate.py`、`evidence/evaluation.json`、`evidence/run_manifest.json`
- 工具惯量：`evidence/tool_inertia_inspection.json`
- 正式/移动/QP skip：`evidence/formal_static_hold_skip.json`、`evidence/moving_tracking_skip.json`、`evidence/cooperative_qp_skip.json`
- 回归：`evidence/regressions/summary.json`

R7 在此停止。后续工作若获新授权，应先修复 fresh Isaac 进程启动稳定性，重新注册同一场景和 source gate，再单独建立 ownership transition 后的平滑时间参数化；只有满足新的诊断门槛，才能重新考虑一次正式静态 hold。

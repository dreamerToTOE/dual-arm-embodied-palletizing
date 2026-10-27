# Task28 G3-P2-4-R1：动态 joint-effort 源与时序审计

**最终状态：`G3_P2_4_R1_BLOCKED`。** 审计确认了当前单 FR3 位置驱动配置的运行时模式和时间支持，但安装的 Isaac Sim 4.5 wrapper 没有提供独立的隐式 position-drive actuator 广义力矩读数。`get_measured_joint_efforts()` 是 solver projected joint force，`get_measured_joint_forces()` 是包含多种贡献的 incoming joint force；二者都不能直接作为

\[
\tau_{actuator}=M(q)\ddot q+c(q,\dot q)+g(q)-\tau_{external}
\]

中的纯 `\tau_actuator`。因此原 P2-4 观测器的动态解释仍无法被防御性地建立，不能运行已载动态阶段，也不能把残差较小称为校准通过。

## 起点与范围

本任务要求的起点已核对：本地 HEAD、`origin/side-suction-palletizing` 和 GitHub 远端均为 `2c459c50ec9e70e2437a2c4b53de6236d0d95ab8`；开始时工作树干净。只新增 `offline/task28_p2_4_r1/` 和本报告，没有修改 P2-4 报告、原始观测、冻结 fixture、G1/NR/QP、ACM 或 Task24/26/27。

所有运行均为隔离单 FR3；没有 Cube、吸盘、D6 约束、ROS、生产 bridge、实体机器人或力控制。`execution_authorized=false`、`physical_robot_tested=false` 始终保持不变。

## Phase 1：安装源和控制模式

安装源证据与行号、哈希保存在 [source_semantics.json](../../offline/task28_p2_4_r1/evidence/source_semantics.json)。关键事实如下：

| API/路径 | 安装版 4.5 实现 | 语义结论 |
|---|---|---|
| `SingleArticulation.get_measured_joint_efforts()` | 委托 `ArticulationView._physics_view.get_dof_projected_joint_forces()`（`single_articulation.py:598–631`，`articulation.py:1279–1347`） | solver 计算的 DOF 运动方向投影力，不是文档保证的 actuator-only torque |
| `get_applied_joint_efforts()` | 读取 `get_dof_actuation_forces()`（`single_articulation.py:633–665`，`articulation.py:1211–1277`） | 只返回显式 `set_joint_efforts` 的 buffer，不能代替隐式位置驱动输出 |
| `get_measured_joint_forces()` | 读取 `get_link_incoming_joint_force()`（`articulation.py:1350–1458`） | 可读 `(M,num_joint+1,6)` incoming force/torque，位于 child incoming-joint frame；它仍是 transmitted solver force |
| `get_mass_matrices()`、`get_coriolis_and_centrifugal_forces()`、`get_generalized_gravity_forces()` | wrapper 的广义质量、状态相关 `c` 和 `g`（`articulation.py:3901–4105`） | 可用于模型项，但不是驱动器力矩观测 |
| `get_effort_modes()` | 读取每个 DOF 的 `UsdPhysics.DriveAPI.TypeAttr`（`articulation.py:2571–2637`） | 只能报告 `force/acceleration` 模式，不报告实际 drive torque |

原 P2-4 的 `ArticulationAction(joint_positions=...)` 走的是 implicit PD position-target 路径。只读 mode probe 在冻结 USD 上实际报告 9 个 DOF 全部为 `force`，没有设置或切换任何模式；初始 `get_applied_joint_efforts()` 全为零，而 projected effort 有非零重力/驱动相关值。这证明了 `force` 是 DriveAPI 模式属性，不能把它误读成已经暴露了 position-drive torque。

官方 [PhysX Articulations 文档](https://nvidia-omniverse.github.io/PhysX/physx/5.3.0/docs/Articulations.html)也明确区分了两者：隐式 drive 的约束相对于时间步结束求解；incoming joint force 是仿真步平均值，并包含重力/科氏、外力/约束、摩擦、速度限幅、drive 和 joint limit 等贡献。该链接是 5.3 文档，只作语义佐证；本地 4.5 wrapper 源码和实际记录优先。

`fr3_link8` 与 `fr3_hand_tcp` 的负质量/无效惯量警告来自原 P2-4/P2-2-R1 证据，PhysX 使用了 small-sphere inertia approximation。没有修改资产，也没有把该模型当成已校准的真实 FR3 动力学。

## Phase 2：960 步 motion_zero 归档重算

只读使用 P2-4 已归档的 `motion_zero` 压缩原始 JSON。每个 observer 输入仅包含当前/上一帧 `q,dq,M,c,g,projected_effort,J`；零载 oracle 在分析之后读取，只作为独立的零外力参照，绝未作为 observer 输入。时间戳、DOF 顺序、有限性、FK 和 rank 仍沿用原评估门。

分析代码为 [analyze_archive.py](../../offline/task28_p2_4_r1/analyze_archive.py)，机器结果为 [motion_archive_diagnostics.json](../../offline/task28_p2_4_r1/evidence/motion_archive_diagnostics.json)。预注册 dt 为 `1/120 s`，实际中位差为 `0.00833333376795 s`；时间严格递增，effort 区间为 `[t-dt,t]`，状态/模型项在 `t` 查询，effort 区间中点比状态早约 `4.1667 ms`。

使用原观测器的因果后向差分

```text
qdd[k] = (dq[k] - dq[k-1]) / (t[k] - t[k-1])
candidate[k] = M[k] qdd[k] + c[k] + g[k] - tau_projected[k]
```

得到的 arm residual infinity norm 按阶段为：

| 阶段 | 样本范围 | 最大残差 |
|---|---:|---:|
| 静止前 | 1–239 | `0.00133136 N·m` |
| 加速 | 240–480 | `0.00907694 N·m` |
| 减速 | 481–720 | `0.00910805 N·m` |
| 停止后 | 721–959 | `0.000648216 N·m` |

加减速段显著高于静止段，符合未建模 implicit drive、摩擦/限位、solver 贡献、惯量警告或差分噪声的多种解释；该数据不能把误差归因于单一因素。

对相同归档分别计算了当前模型/effort、上一状态模型、中心差分、前向差分以及前后一个 effort 样本的诊断比较。全序列最大 infinity norm 为：

| 诊断假设 | 最大残差 |
|---|---:|
| 因果当前模型 + 当前 projected effort | `0.00910805 N·m` |
| 因果上一状态模型 + 当前 effort | `0.00910823 N·m` |
| 中心差分（使用未来样本） | `0.00919301 N·m` |
| 前向差分（使用未来样本） | `0.0133058 N·m` |
| 当前模型 + 前一 effort | `0.0131456 N·m` |
| 当前模型 + 后一 effort | `0.0136862 N·m` |

这些是事后敏感性诊断，不是调参结果；没有选择残差最小的 lag、符号或差分器作为标定。零载数据下 `lhs-tau` 和 `tau-lhs` 的绝对残差相同，符号无法由该运动单独识别。多步后向差分窗口 `1,2,3,4,5,8,10` 的最大残差分别约为 `0.009108、0.009031、0.011698、0.014403、0.017715、0.026599、0.028732 N·m`，简单平滑窗口没有消除中段残差。

q/dq 连续性和时间门通过，但这只说明归档记录可重放；它没有建立 projected effort 与 actuator torque 的等价性。P2-4 的 959 个 candidate estimates 仍保留为 candidate，未升级为动态 source validation。

## Phase 3：最小独立 source test

由于 Phase 1 已证明没有独立的 position-drive torque API，Phase 2 也没有出现可通过换符号或 lag 解决的单一假设，本次只运行了**只读 mode probe**，没有运行 explicit-effort 控制实验：

- 单 FR3、冻结 USD、CPU/headless、零外力、无运动命令；
- 读取 DOF 名称、`get_effort_modes()`、初始 projected effort 和 applied-effort buffer；
- 不调用 `set_effort_modes()`、`set_joint_efforts()`，不改变控制模式；
- 结果：9 个 DOF 均为 `force`，`applied_effort_initial` 全零，`measured_projected_effort_initial` 非零；probe exit code 为 0。

证据见 [mode_probe.json](../../offline/task28_p2_4_r1/evidence/mode_probe.json)、[mode_probe.log](../../offline/task28_p2_4_r1/evidence/mode_probe.log) 和 [phase3_decision.json](../../offline/task28_p2_4_r1/evidence/phase3_decision.json)。显式 `set_joint_efforts` 模式可以另行做 source characterization，但它是 DIFFERENT CONTROL MODE，不能替代历史 position-drive 观测，也没有在本次自动切换或执行。

## 决策、建议与边界

本任务判定 `G3_P2_4_R1_BLOCKED`，原因是当前安装 backend 没有提供可独立读取和时间对齐的历史隐式 position-drive actuator generalized torque；可读的 projected/incoming force 明确包含 solver/drive/约束等混合贡献。静态 oracle 一致、无载运动残差较小、或 `force` mode 属性均不足以改变这一判定。

若继续研究，最小替代路线是另行预注册一个 explicit-effort-only 单 FR3 source characterization，记录 `set_joint_efforts`、实际控制 mode、已知力矩、incoming force、时间区间和模型残差；其结论只能适用于该新模式。要把它用于历史 position-drive observer，必须再建立两种控制模式的等价性，不能静默替换。

本任务没有建立 Cube 附着、D6 反力、有限吸附阈值、双臂 internal wrench、ROS timing、production bridge 或物理执行能力，也没有放宽任何 G1 几何/碰撞门。

## 回归与证据

新增独立审计测试 **5 passed**，覆盖归档样本数/时序、分段残差、lag/差分诊断、符号不可识别、只读 mode probe 和 source-mode BLOCKED 决策。原有 P2-4 **46 passed**（本次重跑 0.12 s）和跨任务 **731 passed**（本次重跑 100.68 s）的历史结果保持不变；本任务不修改它们。所有命令、环境、源文件哈希、mode probe 结果和逐帧诊断见 `offline/task28_p2_4_r1/evidence/` 的 `artifact_manifest.json`。

本报告停止于 G3-P2-4-R1，不自动开始 G3-P2-5 或任何 ROS、机器人、吸附、力控制、NR/QP/RL 实验。

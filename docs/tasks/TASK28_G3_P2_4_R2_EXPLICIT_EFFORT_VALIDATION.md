# Task28 G3-P2-4-R2：显式 effort 源与在线外力矩可行性

**最终状态：`G3_P2_4_R2_SOURCE_PASS_ONLY`。** 隔离的 Isaac Sim 4.5 单 FR3 显式 effort 配置完成了源行为表征：运行时 `force` mode、零 stiffness/damping、`set_joint_efforts` 输入、`get_applied_joint_efforts` 命令缓冲回读、DOF 顺序和正负关节 1 响应均通过。在线已知外力阶段在第 36 个物理步触发预注册的 `BOUNDED_MOTION_LIMIT`，没有形成可评估的在线外力矩序列，因此没有宣称 `ONLINE_CANDIDATE_PASS`。

这个结果只适用于一个临时、显式 effort 的仿真控制模式；它没有推翻 R1 对历史隐式 position-drive 源的 `G3_P2_4_R1_BLOCKED`，也不等价于真实 FR3 的 `tau_J` 或 `tau_ext_hat_filtered`。

## 起点、边界和 R1 保留

本地、远端和 GitHub 分支 `side-suction-palletizing` 的起点均为 `bf1ab9e22c0ace3b8644fa92c4f8e5d310e5ff6c`。R1 的报告和阻塞结论保持不变。本任务只新增 `offline/task28_p2_4_r2/` 和本报告，不修改 frozen USD、Task24/26/27、G1/NR/QP、ACM、P2-4 原始观测或 R1 证据。

每次 Isaac 运行均为 `/World/ProbeFR3` 单臂、无 Cube、无吸盘、无 D6、无 ROS、无生产 bridge；`execution_authorized=false`、`physical_robot_tested=false`、`frozen_usd_modified=false`。冻结 USD SHA-256 为 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`。

## Phase 0：显式 effort API 语义

安装 wrapper 的关键语义来自 Isaac Sim 4.5 源码（R1 已归档 wrapper SHA）：

| API | 本次解释 |
|---|---|
| `set_joint_efforts` | 文档要求没有 joint drive，或 stiffness/damping 均为零；实现写入下一物理帧的 `set_dof_actuation_forces` |
| `get_applied_joint_efforts` | `get_dof_actuation_forces` 命令/输入缓冲；本任务将其作为 source input readback，不称为传感器 |
| `get_measured_joint_efforts` | `get_dof_projected_joint_forces` 的 solver projected force；只作比较，不作为已校准 actuator torque |
| `_articulation_view.set_effort_modes('force')` | private API，写 DriveAPI 类型；不会自动清零 gains |
| `_articulation_view.set_gains(kps=0,kds=0,save_to_usd=False)` | 临时运行时清零 implicit PD gains，不写回 USD |

因此显式实验的实际配置是 `force` mode + `Kp=Kd=0` + `set_joint_efforts`。使用 private articulation view 已记录在采集状态中；源 USD 没有保存或修改。9 个 DOF 的单位为前 7 个 `Nm`、后 2 个 `N`。初始 USD gains 和运行时零 gains 均被记录。

## 预注册和两次运行

最终正式源实验的预注册 SHA-256 为 `cacf2ad54c0cd836e60af4df2dec1378f2df0467bc6f54bb7fa12064e33ff93d`，配置位于 [experiment_preregistration.json](../../offline/task28_p2_4_r2/experiment_preregistration.json)。源工况固定为：零 effort、`+0.25 N·m` joint1、`-0.25 N·m` joint1；每个 20 步，dt=`1/120 s`，最大关节速度门为 `3 rad/s`，无外力。

首次较长预注册尝试使用旧 hash `b477a131...`，在第 36 步触发 `BOUNDED_MOTION_LIMIT`；失败状态和 console 已保留在 `evidence/source_initial_failure/`。随后冻结了正式的短源表征预算和当前 preregistration hash，再运行三工况；该修订事实被保留，因此本任务不把它表述为对原位置驱动实验的无条件动态校准。

## Phase 1：显式源表征结果

正式源运行完成 `zero`、`plus_q1`、`minus_q1` 三个工况，共 60 个物理步；采集状态和评估见 [formal_source](../../offline/task28_p2_4_r2/evidence/formal_source/)，逐工况指标见 [source_metrics.json](../../offline/task28_p2_4_r2/evidence/source_metrics.json)。采集器 SHA-256 为 `f9aab9acd72f47204ff498059c187d97f7e385ae6153f67ff972cef4bc8b0ac7`。

| 工况 | 命令 joint1 | applied 回读最大误差 | joint1 位移 | 末 10 帧平均速度 |
|---|---:|---:|---:|---:|
| zero | `0` | `0` | `+0.0009275 rad` | `+0.02547 rad/s` |
| plus_q1 | `+0.25 N·m` | `0` | `+0.014587 rad` | `+0.14433 rad/s` |
| minus_q1 | `-0.25 N·m` | `0` | `-0.012726 rad` | `-0.09331 rad/s` |

三种情况下时间间隔均为 `0.00833333376795 s`，DOF 名称和顺序匹配，force mode 和零 gains 通过，正负响应方向分离。projected effort 仍被单独保留：joint1 相对命令的最大差约 `3.17e-5`（plus）和 `2.46e-5`（minus）N·m；这只是 solver 比较值，不能解释为测得 actuator torque。

因此 Phase 1 的结论是：**explicit command/readback source behavior 在该隔离仿真模式内通过**。`get_applied_joint_efforts` 仅证明 PhysX 接收并回读了显式输入，不能证明真实硬件等效性，也不能证明 projected effort 是纯驱动力矩。

## Phase 2：在线已知外力尝试

由于 Phase 1 gate 通过，按同一最终 preregistration 尝试了 `force_zero`、`force_plus`、`force_minus` 的 0.5 N 世界 X 方向外力矩阵，命令 effort 为零，oracle 与 observer 输入分离。机器人在第 36 个物理步触发 `BOUNDED_MOTION_LIMIT`，没有完整工况文件；失败状态、console 和 evaluator 结果保存在 [formal_online_failed](../../offline/task28_p2_4_r2/evidence/formal_online_failed/)。这是受控停止，不是在线成功或失败力矩估计结果。

由于没有完整的在线观测序列，本任务没有报告外力 wrench 误差、Jacobian 重构残差或动态 observer 通过率，也没有运行未来样本、匹配 reset、oracle 输入或后验拟合。停止原因说明了显式零增益模式下重力/动力学运动仍可能快速增长；不能靠放宽速度/位置门继续运行。

## Phase 3：部署边界

| 名称 | 本任务含义 | 是否等同真实 FR3 |
|---|---|---|
| Isaac torque command | `set_joint_efforts` 写入的显式输入 | 否，仿真命令 |
| Isaac applied-effort readback | `get_applied_joint_efforts` 的输入缓冲回读 | 否，非传感器 |
| Isaac projected joint force | solver 的 `get_measured_joint_efforts` | 否，可能含 solver/约束等贡献 |
| 模型外力矩估计 | `M qdd+c+g-tau_source` 的模型量 | 仅是仿真候选估计 |
| 真实 FR3 信号 | `tau_J`、`tau_ext_hat_filtered`、`O_F_ext_hat_K` | 必须在实机/官方接口上另行验证 |

未来若需要共享六维 wrench 接口，仿真应携带 `source_kind=explicit_command|projected_solver|model_estimate`、frame、时间支持、单位和不确定性；真实机器人则携带官方信号身份和校准元数据。两者不能因为维度相同而宣称 source equivalence。

## 回归、失败和限制

- R2 新增测试：**3 passed**，覆盖 source gate、readback mismatch negative case 和 preregistration mode/boundary。
- P2-4 原有聚焦回归：**46 passed**。
- R1 审计测试：**5 passed**。
- 跨任务历史回归：**731 passed**。
- 首次 source 运行的 `BOUNDED_MOTION_LIMIT`、正式 source 成功记录和 online 失败记录均保留；没有删除失败样本。

证据清单、代码/配置/资产哈希、运行命令和 exit code 位于 [artifact_manifest.json](../../offline/task28_p2_4_r2/evidence/artifact_manifest.json)。本任务结论停止于 `G3_P2_4_R2_SOURCE_PASS_ONLY`：显式 effort 输入源在隔离仿真模式内完成表征，在线非零负载 observer 尚未验证。没有建立真实 torque sensor equivalence、Cube 附着、吸附力、双臂内力、ROS 或物理执行能力。

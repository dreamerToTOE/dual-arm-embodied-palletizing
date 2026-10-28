# Task28 G3-P2-4-R3：稳定显式 torque 与 B-only 在线 wrench 验证

**最终状态：`G3_P2_4_R3_ONLINE_EXPLICIT_CANDIDATE_PASS`。** 本任务在不修改 frozen FR3 USD 的临时 Isaac Sim 4.5 场景中，完成了全 9-DOF gravity-feedforward + PD 显式 effort 姿态保持，并通过了独立已知载荷的因果在线外部 wrench 候选评估。结果仅是隔离仿真中的 model-based candidate，不等同真实 FR3 `tau_J`、`tau_ext_hat_filtered` 或实际 Cube 吸附力控制。

## 起点与边界

本地、远端和 GitHub 分支 `side-suction-palletizing` 起点均为 `def302dcd7446de6c8dd62c0e0c3562bd4ce92d9`。R2 的 `SOURCE_PASS_ONLY` 和 R1 的 implicit position-drive `BLOCKED` 结论保持不变。

本任务仅新增 `offline/task28_p2_4_r3/` 和本报告。场景为单个 `/World/ProbeFR3`，无 Cube、吸盘、D6、ROS、生产 bridge 或实体机器人；`execution_authorized=false`、`physical_robot_tested=false`、`frozen_usd_modified=false`。冻结 USD SHA-256 仍为 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`。

R2 的在线失败发生在 `force_zero` 第一工况、第 36 个物理步，**尚未施加 ±0.5 N 外力**；本任务没有把它描述成已知载荷估计失败。R3 开发期间的 collector bookkeeping bug 和两个低 effort-bound stability abort 也完整保存在 `evidence/development_failures/`，没有作为正式通过证据。

## 显式 torque 控制合同

正式预注册为 [experiment_preregistration.json](../../offline/task28_p2_4_r3/experiment_preregistration.json)，最终 SHA-256 为 `18abd8d41674b1be28f7e1f9c3bc5c69e9f338f5581205079d71f385278796d5`。控制命令对所有九个 DOF 使用：

```text
tau_cmd = gravity_feedforward(q) + Kp * (q_ref - q) - Kd * dq
```

其中 `gravity_feedforward` 是当前 Isaac articulation 的完整 9 维 generalized gravity，两个手指保持 prismatic 单位和 full 9×9 模型耦合。正式配置的显式 effort mode 为 `force`，临时 runtime drive `Kp=Kd=0`；姿态稳定器的 `Kp/Kd`、effort bound、rate bound、q/dq 限制和读延迟门均冻结并记录。第一步允许完整 gravity feedforward，后续才施加 effort-rate 限制，避免人为的初始重力欠补偿。

每步记录 q、dq、full command、applied readback、projected effort、M/c/g、重力项、PD 项、跟踪误差、饱和标记、时间和物理步号。`get_applied_joint_efforts` 始终被标为 command/input provenance；`get_measured_joint_efforts` 仍单独保留为 solver projected force。

## Stability gate

正式 zero-load posture hold 完成 240 个物理步。独立 evaluator 结果为 [stability_pass/evaluation.json](../../offline/task28_p2_4_r3/evidence/stability_pass/evaluation.json)，状态 `STABILITY_GATE_PASS`：

- 最大 arm 速度：`1.05e-6 rad/s`；
- joint1 位移范围：约 `6.3e-9 rad`；
- applied-command 最大误差：`9.48e-7`；
- effort saturation：`0`；
- 非有限值、DOF 顺序、force mode、runtime zero gains、时间和延迟门均通过。

这证明了当前临时显式控制配置的姿态保持稳定性，不证明真实 torque-sensor 等价性。

## 因果在线 estimator 与独立已知载荷

稳定门通过后，运行 3 个 120 步工况：`zero_external`、`positive_fx`（世界 X `+0.5 N`）、`negative_fx`（世界 X `-0.5 N`）。外力仅写入独立 oracle，未进入 estimator。estimator 使用：

- 当前/上一帧 q、dq；
- 当前 full `tau_cmd`（包括 gravity feedforward 和 PD feedback）；
- 当前 M/c/g；
- frozen RobotModel 的当前 TCP Jacobian；
- 因果后向 qdd；

每个工况的 oracle 是施力点处的世界系力，评估时转换为当前 TCP 参考点的六维 wrench，并计算 `J.T*w`。在线 evaluator [online_evaluate.py](../../offline/task28_p2_4_r3/online_evaluate.py) 在独立已知载荷记录之后才比较结果；它没有使用 oracle 作为输入、没有 matched reset、未来样本或后验 sign/lag fitting。

结果文件：[online_known_load/evaluation.json](../../offline/task28_p2_4_r3/evidence/online_known_load/evaluation.json)。三个工况均通过：

| 工况 | 最大关节外力矩误差 | 最大 wrench force 误差 | 最大 wrench moment 误差 | 最小 rank | 最大 condition |
|---|---:|---:|---:|---:|---:|
| zero_external | `9.69e-6 N·m` | `2.53e-5 N` | `4.72e-6 N·m` | 6 | 8.66 |
| positive_fx | `0.01386 N·m` | `0.04735 N` | `0.00841 N·m` | 6 | 8.68 |
| negative_fx | `0.01752 N·m` | `0.04668 N` | `0.01179 N·m` | 6 | 8.66 |

冻结 acceptance gates 为关节误差 `0.2 N·m`、力误差 `0.5 N`、力矩误差 `0.2 N·m`、rank≥6；全部通过。没有执行慢速运动扩展，因为 R3 的最小已知载荷矩阵已完成且任务要求在此停止。

## 失败记录和回归

开发期间保留了三类失败：早期 collector 在首步 bookkeeping bug、低 effort bound 下的 bounded-motion abort，以及 R2 的 force-zero abort。它们没有被覆盖或删除。R3 正式实验使用最终 preregistration hash，稳定门和在线门均有独立输出。

测试结果：

- R3 新增测试：**4 passed**；
- P2-4 原有聚焦测试：**46 passed**；
- R1 审计测试：**5 passed**；
- R2 测试：**3 passed**；
- 跨任务历史回归：**731 passed**。

所有正式/失败的 Isaac 数据、oracle、运行状态、console、评估、源和资产 hash 见 [offline/task28_p2_4_r3](../../offline/task28_p2_4_r3)，证据清单为 [artifact_manifest.json](../../offline/task28_p2_4_r3/evidence/artifact_manifest.json)。

## 解释边界

本结果是“显式 effort 输入 + 仿真动力学模型 + 已知外力 oracle”的 B-only 在线候选验证。它没有验证真实 FR3 `tau_J`、`tau_ext_hat_filtered`、硬件时间延迟、实际吸盘接触、Cube 负载、双臂 internal wrench 或 ROS/生产桥接。R1 的历史 position-drive source 阻塞仍然有效；R3 的显式模式不能替换历史 position-drive 路径，也没有实现双臂力控制。

任务在 `G3_P2_4_R3_ONLINE_EXPLICIT_CANDIDATE_PASS` 停止，不自动进入 force control 或其他任务。

# G1-A-FR3 实际控制切换验证报告

## 结论

本研究分支的接受状态为 **`G1A_FR3_PARTIAL`**，控制优势结论为 **`INCONCLUSIVE`**。

F0 已在 Isaac Sim 4.5 中证明真实 FR3 articulation 能接收并执行有限关节目标：测量关节和 TCP 均发生有限、连续的物理仿真运动，随后在第 90 个物理步切换到静止目标。这个结果验证的是 **position-drive reference transition**，不是经过验证的阻抗或柔顺控制。

F1 未完成物理工具/Cube 接触。完整 Task27 场景和 MoveIt 已启动，候选 pre-contact、OUTER_DESCENT、CONTACT 轨迹均完成 FCL 检查；现有 live-QP 在 20 mm `LOCAL_SWEEP_LIMIT` 处停止，执行器随后被有界地终止。因此没有可归因的 Cube 位移、loaded wall contact、接触冲量或控制策略比较结果。没有把离线 FCL 通过当作物理接触，也没有把 Cube 的初始 table support 当作 tool/Cube 或 wall 接触。

## 范围和隔离

- 基线：`20b55e065a38212eab031d1621895f1e3a670eb5`。
- 分支：`research/stage-b-contact-switch-fr3`。
- 仿真：Isaac Sim 4.5.0；硬件、RL、G2--G4 均未使用。
- 资源检查：16 CPU、约 46 GiB RAM、RTX 3070 8 GiB；实验期间只运行一个 Isaac Sim 实例。
- 主开发分支和 G0/G1-A 旧 worktree 未修改。
- 复用 `task26_truck_box_scene.py` 的 FR3/side-tool 构造器，并审计后复用 `task28_task27_torque_bridge.py` 和 `offline/task28_task27_live_qp/` 的现有接口。

协议和源文件 SHA256 记录在 [`configs/f0_f1_protocol.json`](configs/f0_f1_protocol.json)。

## F0：实际 FR3 articulation 控制接口

隔离场景只包含左侧 FR3、原始 side-suction tool 几何和物理场景设置，不包含 Cube、桌面或墙。控制写入为：

`SingleArticulation.apply_action(ArticulationAction(joint_positions=...))`

测量执行不使用关节 teleport，也未写入 Cube 的世界力。流程为 30 个 warm-up 步、settle、J1 目标从当前值平滑移动约 0.05 rad，然后保持固定目标。完整 JSONL 和汇总见 [`evidence/f0_smoke/`](evidence/f0_smoke/)，诊断图见 [`figures/f0_tracking.png`](figures/f0_tracking.png)。

| 指标 | 结果 |
|---|---:|
| 物理步数 | 150 |
| `dt` | 0.0166667 s |
| 目标切换步 | 90 |
| TCP 位移 | 0.004815 m |
| 最大关节速度 | 0.08046 rad/s |
| 最大跟踪误差 | 0.00857 rad |
| 最大目标步长 | 0.000833 rad |
| measured effort 可用 | 是 |
| teleport | 否 |
| 接触力写入/读取 | 未使用 |

F0 的结论是 `PASS`：命令确实到达实际 FR3 articulation，关节和 TCP 有限跟踪目标，并在第 90 步转入静止位置目标。由于本试验使用 position-drive target，不能称为“阻抗切换”；要验证柔顺性，仍需一个有明确刚度/阻尼、动力学读回和饱和记录的 FR3 控制模式。

## F1：FR3 到 Cube 的物理交互 smoke attempt

使用原始 Task27 场景入口，包含双 FR3、side tool、table、Cube_01 和深墙；Cube 运行时尺寸为 0.12 m、质量约 0.8 kg。未对 Cube 施加直接世界力，也未在测量阶段 teleport Cube。

### Attempt 01

`g1a_fr3_f1_attempt01` 已创建 Isaac 场景并进入 torque bridge，但 MoveIt 在启动时找不到 `ros_ws/install/setup.bash`，退出码为 1。该失败被保存在 [`evidence/f1_attempt01/`](evidence/f1_attempt01/)，不计为接触试验。

### Attempt 02

在当前 worktree 构建 ROS overlay 后重跑。MoveIt 正常启动（退出码 0），executor 读取 Cube_01，并记录了 8 个候选和以下规划/FCL 结果：

- `HIGH_LEFT`、`HIGH_RIGHT`：synchronized FCL PASS；
- `OUTER_DESCENT`、`CONTACT`、`LEFT_LOADED_LIFT_PROBE`、`LEFT_LOADED_X_PROBE`：synchronized FCL PASS；
- 选择 `LEFT_CANDIDATE_1/8: HIGH_PRE_CONTACT -> OUTER_DESCENT -> CONTACT FCL PASS`。

这些是规划和几何检查结果，不是实际接触结果。live-QP 共完成 606 次 `QP_OPTIMAL` 求解，随后触发 `LIVE_QP_LOCAL_SWEEP_LIMIT`（20 mm）；executor 以 `-15` 结束，运行被超时/停止。没有记录到由 FR3 工具推动的 Cube 位移或 loaded wall contact，因此未计算接触前速度、动能、峰值力、冲量或任务完成率。完整 receipt、MoveIt/executor 日志和压缩 torque bridge 日志在 [`evidence/f1_attempt02/`](evidence/f1_attempt02/) 中。

### 接触可观测性审计

本次运行只看到初始 Cube/table 支撑：runtime bridge 报告 table support force 约 7.848 N，质量为 0.8 kg。这不等于 tool/Cube 或 Cube/wall 接触。日志同时明确记录：

- `observer_status=MODEL_ONLY_CANDIDATE_NO_PHYSICAL_TAU_J`；
- `contact_oracle=NOT_CONNECTED`；
- `SurfaceGripper_D6` runtime `valid=false`；
- contact wrench 未直接从 D6 暴露。

因此 Fuller Task27 的力观测问题在该场景没有被宣称解决；本报告不提供任何定量力控收益。

## F2：C0/C1/C2 比较状态

因为 F1 没有建立真实 FR3-to-Cube 物理交互，三种策略均没有进入匹配评估：

| 策略 | 状态 | 原因 |
|---|---|---|
| C0 conventional position insertion | `NOT_RUN` | F1 执行器交接先失败 |
| C1 geometry-triggered switching | `NOT_RUN` | 没有实际 wall contact 和可用 contact state |
| C2 predictive deceleration/compliant transition | `NOT_RUN` | 未实现/验证动态柔顺读回，不能用 reference smoothing 代替 |

前一阶段 proxy 中约第 19--20 步发生的切换没有被外推到本次 FR3 试验；F0 的第 90 步只是自由空间 position target hold transition。没有证据证明任何平滑命令降低了碰撞力，也没有构造人工失败来支持层级控制。

最小追加 primitive 是：在不触发 20 mm 局部扫掠门限的前提下，让 MoveIt 的分段 joint trajectory 进入同一个实际 FR3 控制 sink（或提供单臂 position-drive trajectory executor），并保留每段 target、实际 q/TCP、饱和和终止原因。若要继续声称“柔顺切换”，还需一个明确的 reduced-stiffness/impedance 模式及其动力学读回。完成该 primitive 后，才能冻结协议并运行匹配的 C0/C1/C2 小样本试验。

## 可复现性和验证

```bash
python3 research/stage_b_contact_switch_fr3/tests/test_receipts.py
python3 research/stage_b_contact_switch_fr3/scripts/analyze_receipts.py
python3 research/stage_b_contact_switch_fr3/scripts/make_f0_plot.py
```

接收检查输出 `receipt checks: PASS`；分析 receipt 保存在 [`evidence/analysis.json`](evidence/analysis.json)。F0 原始运行失败尝试和 ROS overlay 构建日志也保留在 `evidence/` 下。

## 最终判定

- `G1A_FR3_PARTIAL`
- `CONTROL_ADVANTAGE=INCONCLUSIVE`
- G1-B force-feedback readiness：`NOT_READY`（contact oracle 未连接，且 C0/C1/C2 未形成匹配数据集）
- 当前提交：由 `git rev-parse HEAD` 复现；最终 SHA 在交付记录中给出。

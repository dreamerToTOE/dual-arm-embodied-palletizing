# TASK28 R14：G2 冻结与 G3 双臂力控研究恢复

基线是 R13 HEAD `72fbb8d5ea09ac9edb0a3869a07bed01e243741c`。本任务没有真实机器人命令、ROS 控制输出、Cube 接触/运动、吸盘、D6 或 V10-D/FORMAL 执行。

## G2 收尾

R12 控制器和门限保持冻结：common/relative feedback scale `0.75`、启动速度捕获 `τ=0.10 s`、Cube 峰值 `0.250 mm`、相对 TCP 平移 `0.150 mm`，以及原 QP、速度、力矩、变化率和场景约束。原始 Cube 接近路径仍为 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`，不能用于接触、抓取或搬运。

R14 对 B/C 做了预登记的少量 CPU 近似模型检查。B 和 C 各有名义、初始 q 的正一 float32 ULP、以及观测质量矩阵最大偏差的正负两侧，共 8 次、1680 次 QP 求解。每次都是新的状态重置和重新求解；没有 Isaac 进程、Cube 命令或控制器修改。

| 工况 | 名义峰值 | q ULP+ | M 偏差− | M 偏差+ | 单次相对 TCP 最大值 |
|---|---:|---:|---:|---:|---:|
| V10-B | 0.231312 mm | 0.231357 mm | 0.232017 mm | 0.230603 mm | 0.060626 mm |
| V10-C | 0.246598 mm | 0.246657 mm | 0.247019 mm | 0.246174 mm | 0.056099 mm |

8 个回放均为 `QP_OPTIMAL`，没有新增近似模型阈值超限。但 C 的 Isaac R13 剩余裕量只有 `0.002383 mm`，而同一时刻的近似模型 TCP 偏差为 `0.040304 mm`；B 为 `0.046321 mm`。因此这些结果只能作为代表性敏感性检查，不能建立真实 Isaac 或硬件鲁棒性，状态仍是 `MODEL_NOT_QUALIFIED_FOR_MARGIN` / `NOT_ESTABLISHED`。

机器证据：[g2_results.json](../../offline/task28_r14/evidence/g2_results.json)、[g2_preregistration.json](../../offline/task28_r14/evidence/g2_preregistration.json)、[g2_baseline_freeze.json](../../offline/task28_r14/evidence/g2_baseline_freeze.json)。

## G3 已有能力与遗留问题

详细审计见 [TASK28_R14_G3_CAPABILITY_AUDIT.md](TASK28_R14_G3_CAPABILITY_AUDIT.md)。现有代码可复用：

- P2-4 `observer.step` 的九维 `M qdd+c+g−tau`、因果后向差分、完整 DOF/单位/时钟/质量矩阵/秩/条件数/残差合同。
- P2-2 的六维 TCP 等效 wrench 最小范数估计器。
- P2-1 的抓取矩阵、加权分配和内力零空间分解。
- QP-6 已有 54 维合成 inverse-dynamics HQP 数学实现；它使用理想 SPD/接触模型，不能直接连接原始双臂执行器。
- R8 effort adapter 的双臂独占写入、九维 buffer、手指保持、readback、逐关节 effort/rate 门。

P2-4-R3 的结论是单 FR3、无 Cube 的显式 effort 模型候选通过。其已知外力 evaluator 是采集后的因果重算，没有在采集循环内调用严格 observer；非零力的 application point 是最后一个点重复到整段；rank/condition 等项也未形成完整 fail-closed evaluator 门。因此它不能等同真实 `tau_J`、`tau_ext_hat_filtered`、接触 wrench 或双臂力控闭环。Isaac 的 `get_applied_joint_efforts` 是命令缓冲，`get_measured_joint_efforts` 是 solver projected force，二者均不是 FR3 torque sensor。

## R14 无接触 S1

R14 使用新的隔离进程和新的 evidence 身份，保留原 R3 zero-load hold 的计算，并在写入前增加拒绝型保护：实际 gain/mode、cursor、有限状态、q/dq 和 acceleration 检查；arm effort 取 R3 配置与当前 R8 research limits 的交集 `[35,12,8,35,4,4,4] N·m`；从第二次写入起检查 R8 `rate × dt` 以及旧 R3 每步增量。第一条命令保留 R3 明确注册的 full-gravity bootstrap，故不能把本结果描述为原始双臂 R8 控制权交接通过。

该隔离运行完成 240 步，进程正常退出并验证了独立 cache、源快照和无 ROS bridge。没有调用外力 API，记录 240 个 guarded commands。严格 observer 审计为：1 个 `WARMUP`、239 个 `CANDIDATE_ESTIMATE`；最大关节速度 `1.050532e-6 rad/s`，最大 applied-command 差 `9.476943e-7`，最大零载关节残差 `9.685773e-6 N·m`，最大等效力误差 `2.529155e-5 N`，最大等效力矩误差 `4.959968e-6 N·m`，最小秩 6，最大条件数 8.662224。

这只证明单臂、单姿态、无接触的模型候选一致性；不证明双臂、L-tool、接触、吸附、实时 observer→力反馈或硬件传感器。机器证据：[no_contact/attempt_01/evaluation.json](../../offline/task28_r14/evidence/no_contact/attempt_01/evaluation.json)、[lifecycle.json](../../offline/task28_r14/evidence/no_contact/attempt_01/lifecycle.json)、[runtime_scope.json](../../offline/task28_r14/evidence/no_contact/attempt_01/runtime_scope.json)。

## Isaac Sim 集成路线

1. **S1 无接触源与重力补偿：** 复用 R3 单臂探针和 P2-4 严格 observer，先通过源身份、时钟、命令区间、质量矩阵和 fail-closed 负控。R14 已完成一姿态 zero-load 候选检查。
2. **S2 受控虚拟接触：** 当前未就绪。必须先修复逐步接触 oracle、增益前后 readback、rate 单位、pre-command cursor 和独立 contact-force frame/sign/time 记录；另建带场景 hash、无初始穿透、局部连续包络、停止条件和失败分母的接触夹具。已知外力探针不能冒充接触验证。
3. **S3 双臂载荷与内力：** 当前未就绪。需双臂同状态有效 wrench、实际 body/anchor、当前抓取矩阵、物体净 wrench 与内力独立 oracle，再用既有 `G`/加权分配/`ker(G)` 和 HQP 做零内力、同净载荷异分配、非零内部目标对照；代数零空间命中不等于测得内力。
4. **S4 Cube 吸附搬运：** 当前未就绪。原始接近路径不能重用。只有 S1–S3、有限吸附代理、attach/break/release/recapture 事件、连续碰撞和闭链证据都满足后，才能从独立安全初态分段执行短 lift/transport/place。理想 fixed joint 或高 break limit 只能表示有限仿真代理，不能证明真实吸附稳定性。

阶段准入机器表见 [g3_stage_admission.json](../../offline/task28_r14/evidence/g3_stage_admission.json)。S2、S3、S4 当前均 `can_start_with_current_evidence=false`，不自动启动 V10-D 或 FORMAL。

## 下一项可直接执行的 G3 任务

`G3-P2-4-R4：STRICT_OBSERVER_SOURCE_ADAPTATION_AND_CONTACT_PREFLIGHT`：

- 在新目录中修复显式 source adapter，保持 `tau_J` 与 projected effort 的来源区分。
- 为 NaN、空数据、过期/乱序、非 SPD、低秩 Jacobian、错误作用点和缺失逐步 oracle 建立负控；拒绝后不推进 observer history、不写控制命令。
- 先复查一姿态 zero-load 和同一已知载荷协议，再决定是否建立独立接触夹具；不得将 R3 ±Fx candidate 直接升级为接触力真值。
- 输出接触场景的安全初态、局部连续运动包络、接触对、独立 oracle、力/能量/时间预算和失败停止合同。任一项缺失都保持 `NOT_READY`。

## 交付状态

R14 新增测试覆盖 G2 冻结、S1 无接触证据和 S2–S4 未准入检查：`13 passed`。R13 已归档的完整历史回归保持 `1152 passed`，合计 `1165 passed`；汇总见 [regressions/summary.json](../../offline/task28_r14/evidence/regressions/summary.json)。最终 Git HEAD 在交付终端报告中给出；工作树必须保持干净，且远端分支与本地 HEAD 一致。

状态：`R14_G2_FROZEN_G3_S1_NO_CONTACT_CANDIDATE_PASS_S2_S4_NOT_READY`。

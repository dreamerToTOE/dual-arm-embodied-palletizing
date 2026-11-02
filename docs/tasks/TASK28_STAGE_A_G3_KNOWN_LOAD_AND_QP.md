# TASK28 Stage A / G3：已知载荷标定与内部力矩分配

基线 `49e0cd9ad8571af37242457d808053ac768100bd`。目标是在原 Task27 场景中标定独立仿真反力通道，检查既有 QP 的力矩分配，并只在悬停实验中测试有明确假设的前馈修正。

## 已知载荷标定

使用原双臂 FR3、L 型侧吸工具和 Cube 场景。两吸盘保持打开，每臂先用现有的独立关节目标保持当前姿态；对 joint8 下游的原有 `fr3_hand_tcp` 刚体，沿世界 Y 方向依次逐步施加 `0, +0.1, +0.2, 0, -0.1, -0.2, 0 N`，每段 3 s，另一臂不施力。施力调用 Isaac 4.5 `IPhysxSimulation.apply_force_at_pos(..., mode="Force")`，逐物理步以当时的刚体原点作为世界施力点。命令作为注入真值单独记录，完成区间的反力、q/dq、M/c/g、当前 Jacobian 和已应用 effort 对应**上一物理步**施力命令；未将该注入力供给控制器或估计器。每段的最后 1 s 为固定统计窗口。

v3 GUI 完整运行，两臂 14 段合计 2521 个配对样本全部 `QP_OPTIMAL`，力矩饱和和力矩变化率激活均为 0。稳态均值：

| 注入世界 Y 力 | 左模型估计 | 左 joint8 Fy | 右模型估计 | 右 joint8 Fy |
|---:|---:|---:|---:|---:|
| +0.1 N | +0.100002 N | -0.099806 N | +0.100001 N | -0.099803 N |
| +0.2 N | +0.200004 N | -0.199806 N | +0.200003 N | -0.199802 N |
| -0.1 N | -0.099999 N | +0.099802 N | -0.099999 N | +0.099803 N |
| -0.2 N | -0.199998 N | +0.199803 N | -0.199998 N | +0.199804 N |

零载窗口模型 Fy 最大绝对均值 `0.0000043 N`，joint8 Fy 最大绝对均值 `0.000395 N`；各稳态窗口最大关节速度小于 `0.000982 rad/s`。校准支持该姿态和 `±0.2 N` 范围内的传递力符号、增量与时间对齐；joint8 是父对子关节传递力，因此其变化与施加在远端刚体的外力相反。这不是对 1–2 N 的标定，也不是独立测得吸附 D6 力或真实接触力。窗口平均力矩残差最大约 `0.000020 N·m`，并非整个瞬态的最大误差。

[v3 metrics](../../offline/task28_task27_integration/evidence/task28_g3_known_load_v3/known_load_metrics.json)、[图表](../../offline/task28_task27_integration/evidence/task28_g3_known_load_v3/known_load_comparison.png)、[GUI summary](../../offline/task28_task27_integration/evidence/task28_g3_known_load_v3/summary.json)、[完整压缩 trace](../../offline/task28_task27_integration/evidence/task28_g3_known_load_v3/task27_torque_bridge.jsonl.xz)。

初次 v1 使用共同任务保持，+1 N 施力后局部运动保护停止；v2 改用双臂独立关节保持，+1 N 仍触发 `LOCAL_SWEEP_LIMIT`，因此在 v3 改用更小的 ±0.1/±0.2 N；失败均保留 [v1](../../offline/task28_task27_integration/evidence/task28_g3_known_load_v1/summary.json)、[v2](../../offline/task28_task27_integration/evidence/task28_g3_known_load_v2/summary.json) 和各自 trace，未计入完成结果。v3 没有通过调整保护阈值绕过这两个停止。

## 基线 QP 分配与最小实验改动

对上次 GUI 悬停 v4 的固定稳态窗口独立重算，每段 224 帧，结果见 [机器证据](../../offline/task28_task27_integration/evidence/task28_g3_known_load_v3/qp_allocation.json)。1 N 段的反馈后目标均值 1.430 N、内部目标力矩范数 0.618 N·m，QP 结果与完整力矩目标的 L2 残差为 0.573 N·m；2 N 段分别为 2.869 N、1.239 N·m 和 1.154 N·m。QP 输出减模型偏置后，在目标内部力矩方向的投影均值分别仅 **7.73% 和 7.18%**。QP 输出与 applied 命令缓冲区差小于 `9.54e-7 N·m`；稳态没有力矩饱和和变化率激活。这是求解器层的力矩投影，不是吸盘接触力实现率。

原因有直接代码证据：原 QP 消元时使用 `tau=M*qdd+h`，没有把闭链反作用项放进动力学，而内部力矩只是软目标。因此它要通过关节加速度变化实现所需力矩，又与高权重的静止共同/相对加速度目标竞争。对记录的同一帧，原 QP 可重算至力矩误差 `1.2e-6 N·m`；把**假定**目标闭链负载加入 QP bias 的单帧反事实，投影从 6.95% 提高到 95.20%，力矩目标无穷范数残差从 0.529 降至 0.091 N·m。这只是离线、同状态的模型反事实，不是物理力验证。

为实际实验增加 `--assume-internal-contact-load`：只在用户指定的固定悬停序列且内部目标激活时，把 `Jᵀ f_target` 作为**假设已存在的闭链载荷前馈**加入 QP 偏置；目标力矩及 torque/rate 硬边界仍由原 QP 处理。默认关闭；无目标、非该实验和其他执行入口保持原行为。日志标明 `assumed_commanded_wrench_not_measured`。这是实验性模型修正，只有在实际反应、跟踪和吸附状态都可接受时才可考虑后续使用。

## GUI 悬停对比

原配置 v4 与前馈实验各自重启原 Task27 场景，在 Cube_01 被两个原 Surface Gripper 吸附并共同抬升后执行相同的 `0 → 1 → 2 → 0 N` 目标：每段过渡 2 s、保持 5 s，采用相同的固定稳态窗口。两次姿态有差异，因此这些运行是对照实验，不能单独分离姿态差异带来的影响。

| 指标 | 基线 v4 | 假定接触载荷前馈 |
|---|---:|---:|
| 0／1／2／0 N 段模型候选均值 | 0.0232／0.1408／0.2615／0.0232 N | 0.0195／1.0112／2.0118／0.0195 N |
| 1／2 N 段 joint8 左侧世界 Fy 均值 | 0.1408／0.2615 N | 1.0112／2.0119 N |
| 1／2 N 段 joint8 右侧世界 Fy 均值 | -0.1408／-0.2615 N | -1.0112／-2.0119 N |
| 1／2 N 段完整目标力矩残差 L∞ 均值 | 0.2635／0.5286 N·m | 0.00683／0.00718 N·m |
| 1／2 N 段目标方向力矩投影均值 | 7.73%／7.18% | 101.41%／100.74% |
| 固定段 Cube 峰值漂移 | 2.790 mm | 0.0322 mm |
| 固定段相对 TCP 平移峰值 | 0.0535 mm | 0.0797 mm |
| 固定段力矩饱和／变化率激活帧 | 0／0 | 0／0 |

前馈运行完成 7249 次 `QP_OPTIMAL`，7250 帧关节反力有效；四个稳态窗口吸盘 `is_closed` 均为 true，回零后两侧反力和模型候选均回到起始基线附近。完整 trace、关节反馈与运行截图见 [summary](../../offline/task28_task27_integration/evidence/task28_g3_contact_load_ff_v1/summary.json)、[metrics](../../offline/task28_task27_integration/evidence/task28_g3_contact_load_ff_v1/reaction_metrics.json)、[力矩分配](../../offline/task28_task27_integration/evidence/task28_g3_contact_load_ff_v1/qp_allocation.json)、[图表](../../offline/task28_task27_integration/evidence/task28_g3_contact_load_ff_v1/reaction_comparison.png)、[窗口/进程证明](../../offline/task28_task27_integration/evidence/task28_g3_contact_load_ff_v1/process_proof.txt) 和 [压缩 trace](../../offline/task28_task27_integration/evidence/task28_g3_contact_load_ff_v1/task27_torque_bridge.jsonl.xz)。

该实验表明所测试的前馈能显著减小 QP 内部目标力矩残差，模型候选和 joint8 传递力在 1／2 N 目标下呈正向、可逆响应，且本次 Cube 与相对 TCP 运动保持有界。它**不证明 1／2 N 的吸盘接触力跟踪已验证**：上面的独立注入力标定只覆盖吸盘打开时的 ±0.2 N，且 D6 约束反力仍无独立读取；关节反力可能混合子树惯性、重力和其他约束贡献。因此维持 `G3_JOINT_REACTION_CANDIDATE`，前馈默认关闭。

下一步应在真实悬停姿态上扩大受控已知载荷的可验证范围，或取得独立的仿真约束反力参考，尤其检查 1／2 N 与载荷分配；之后才能判定接触力闭环是否可用于运输实验。

## 复现

```bash
# 在有 GPU 的 Isaac Sim 4.5 X11 桌面中。
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-}
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=73 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh isaac/scripts/task28_task27_one_pass.py \
  --mode smoke --live-qp --record-joint-reactions --known-load-calibration \
  --local-sweep-limit-m 0.15 --timeout-s 900 --run-id <fresh_id>
PYTHONPATH=. python3 -m offline.task28_g3_stage_a.analyze_known_load \
  offline/task28_task27_integration/evidence/<fresh_id>
PYTHONPATH=offline/task28_g1_1:. python3 -m offline.task28_g3_stage_a.analyze_qp_allocation \
  offline/task28_task27_integration/evidence/task28_g3_reaction_v4_corrected

DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=79 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 --fixed-suspended-force-sequence \
  --record-joint-reactions --assume-internal-contact-load --fixed-force-transition-s 2 \
  --fixed-force-hold-s 5 --internal-force-weight 1000 --internal-force-kp 0.5 \
  --local-sweep-limit-m 2.0 --execution-time-scale 16 --timeout-s 900 --run-id <fresh_id>
```

回归测试 159 passed，Python 编译检查通过。未启动 Stage B、REGRASP/PUSH、五 Cube continuation、真实机器人或 Sim2Real；原 HOME→Cube 路径的 `CONTINUOUS_SAFETY_NOT_ESTABLISHED` 沿用。没有改写原场景、USD、Cube、工具、吸盘配置或 QP 硬限制。

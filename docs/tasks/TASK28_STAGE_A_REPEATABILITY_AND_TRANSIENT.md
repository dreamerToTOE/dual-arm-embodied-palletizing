# TASK28 — Stage A 重复性和瞬态跟踪诊断

## 决定与适用范围

**Stage A 的抓取、搬运、真实桌面支撑、释放和退避物理周期已复现：v9、v10、v11 均完成。** 这只建立三次仿真运行的功能基线，不代表统计可靠性或精密放置达标。最终 XY 误差分别为 **0.624513、0.900556、0.384371 mm**，原有 **0.25 mm** XY 研究指标在三次中均未达到。

**严格的“整个运行均无直接 Cube 位姿写入”条件未通过。** 为保持用户要求的原 Task27 场景，三次均保留了其供料初始化：原场景的 `task26_truck_box_bridge.py` 中 `_activate_batch` 调用 `self.rigids[index].set_world_pose(...)`，将休眠区 Cube_01 唤醒到供料槽；这发生在吸附之前。它是直接物体位姿写入，不能把整个运行称为无瞬移。吸附后至落稳的物理周期中没有增加直接 Cube 位姿写入；双夹爪闭合期间相邻 60 Hz 物体位置样本的最大位移分别为 1.325、2.567、1.392 mm，连续样本本身不是对所有代码路径的证明。力矩桥记录的机械臂位置写入计数三次均为 0，限定于力矩控制接管期间；不将原场景接管前的初始化描述成零位置驱动。完整无位姿写入验证需另行处理供料初始化，本轮未改变原场景。

基线代码为 `51b048cb4c037d6e7b8d1ef9e85ad51c115c5b9c`。v10 和 v11 是新 GUI 进程、独立场景重置、全新运行 ID，顺序执行。两者与 v9 的记录配置、场景入口和原场景源文件 SHA-256 一致：原对象中心参考、世界系旋转修正、协作力矩 QP、G3 载荷前馈均保留；不同的是运行 ID 和隔离用的 ROS domain（v9/10/11 分别为 96/97/98）。没有重新设计 G1、G2、G3，没有针对最终 XY 调参，也没有开始 Stage B。

## 三次物理运行

最终目标中心为 `(0.770, 0.060, 0.260) m`；XYZ 误差定义为**实际减目标**。闭合误差是实际左右 TCP 对物体抓取闭合的平移误差峰值，不是期望参考的闭合误差。全程跟踪统计仅包含双侧闭合且对象参考激活的 60 Hz 帧，误差为期望与实测 Cube 中心的三维距离。

| 项目 | 原 v9 | 新 v10 | 新 v11 |
|---|---:|---:|---:|
| 吸附 → 抬升 → 运输 → 支撑 → 双侧释放 → 双臂退避 | 完成 | 完成 | 完成 |
| 最终中心 XYZ (m) | `(0.769377947, 0.059944630, 0.260000020)` | `(0.769100010, 0.059968062, 0.260000020)` | `(0.770378292, 0.060068090, 0.260000020)` |
| 最终 XYZ 误差 (mm) | `(-0.622053, -0.055370, +0.000020)` | `(-0.899990, -0.031938, +0.000020)` | `(+0.378292, +0.068090, +0.000020)` |
| 最终 XY 范数 (mm) | 0.624513 | 0.900556 | 0.384371 |
| 实际双 TCP 平移闭合误差峰值 (mm) | 0.123773 | 0.128975 | 0.105464 |
| Cube 跟踪峰值 / RMS (mm) | 20.385 / 5.238 | 22.170 / 6.280 | 21.762 / 4.948 |
| 末端控制激活至首次双 OPEN (仿真 s) | 5.950 | 6.183 | 4.850 |
| 最后双 CLOSED 帧 Cube 速度 XYZ (mm/s) | `(0.248, -0.113, 0.173)` | `(0.270, -0.021, -0.737)` | `(-0.244, -0.172, -0.283)` |
| 最后双 CLOSED 帧 / 首次双 OPEN / 最终桌面竖向力 Fz (N) | 2.275 / 7.837 / 7.848 | 3.163 / 7.879 / 7.848 | 3.346 / 7.857 / 7.848 |
| 最后双 CLOSED 高度 − 最终高度 (mm) | +0.003040 | +0.000685 | −0.000298 |
| 首次双 OPEN 高度 − 最终高度 (mm) | −0.000030 | −0.000030 | −0.000060 |
| 首次双 OPEN 后至最终帧 / 期间最大 XY 漂移 | 37.117 s / 0.000446 mm | 35.067 s / 0.000403 mm | 40.467 s / 0.000367 mm |
| QP optimal / infeasible | 9475 / 0 | 9009 / 0 | 9517 / 0 |
| 最大施加关节力矩：左 / 右 (N·m) | 35.322 / 35.927 | 37.316 / 37.337 | 36.164 / 35.270 |
| 最大关节力矩变化率 (N·m/s) | 1512.374 | 1067.586 | 1584.120 |
| 最大实测关节速度 / QP 关节加速度 | 0.275 rad/s / 11.127 rad/s² | 0.303 / 10.806 | 0.285 / 11.064 |
| 执行器 / 力矩锁存错误 / 位置写计数 | 0 / 无 / 0 | 0 / 无 / 0 | 0 / 无 / 0 |
| 释放后 G3 接触负载与 payload 前馈 | 持续关闭 | 持续关闭 | 持续关闭 |
| 最终两侧 Surface Gripper | OPEN / OPEN | OPEN / OPEN | OPEN / OPEN |

桌面力为 PhysX `RigidContactView` 的 Cube/Table 接触力，而非用高度近似代替测力。退避结束时速度范数约 0.136 mm/s；三次释放后两夹爪持续 OPEN、Cube 保持桌面支撑并稳定。最后双 CLOSED 到最终的高度差与首个双 OPEN 到最终的高度差分别列出；负号只表示落稳高度略高，数值接近仿真浮点分辨率。末端持续时间是仿真时间，不是墙钟时间，也不是仅统计达到位置误差阈值的时间。G3 内部力是 1 N 目标与模型候选，**不是独立测得的 D6 接触内力**。

力矩绝对值硬上限每臂为 `(87, 87, 87, 87, 12, 12, 12) N·m`，逐关节变化率上限为 `(2400, 600, 240, 2400, 240, 240, 240) N·m/s`；表中最大变化率发生在 2400 N·m/s 限制的关节。各关节逐帧峰值均未超过各自上限，部分变化率达到上限。QP 的位置预测约束保持原设置；速度上限每臂为 `(2.17, 2.17, 2.17, 2.17, 2.61, 2.61, 2.61) rad/s`，加速度上限为 `20 rad/s²`。逐关节峰值见下方 JSON；这些约束均未移除。

## 分阶段对象参考跟踪

各格为 **RMS / 峰值 mm**，括号内为峰值仿真时间（s）。60 Hz 帧按对象参考高度推断分段：接近最高参考高度前为抬升，保持高位至参考开始下降为运输，其后至末端修正启用为下降（包含原 Stage A 的短距离侧向移动），末端修正激活后为末端放置。不同运行的分界会随规划和实际时序变化；这不是按 ROS 日志墙钟时间划分。末端结束于释放前最后一个双 CLOSED 帧。

| 阶段 | v9 | v10 | v11 |
|---|---:|---:|---:|
| 抬升 | 2.234 / 8.221 (83.117) | 2.243 / 11.031 (83.417) | 2.131 / 9.759 (83.417) |
| 运输 | 0.939 / 3.032 (93.717) | 4.166 / 14.043 (93.283) | 0.914 / 3.711 (95.200) |
| 下降及短距离侧向移动 | 2.481 / 12.997 (114.650) | 2.429 / 13.207 (108.750) | 2.668 / 15.091 (113.183) |
| 末端放置 | 12.621 / **20.385** (119.317) | 12.925 / **22.170** (113.733) | 12.515 / **21.762** (116.500) |

v10 运输阶段另有以 X 方向为主的误差增大，参考当时仍在移动；从这些观测不能确定是参考时序、实际动力学还是两者相互作用导致。它与三次均出现的末端竖向峰值应分开研究。

## 20.385 mm 峰值与可复现缺陷

**直接观测：** v9 的峰值在 `t=119.316667 s`，末端修正已于 `114.850 s` 启用。该帧期望中心为 `(0.770007189, 0.060013715, 0.259978824) m`，实际为 `(0.771720648, 0.059934769, 0.280291975) m`，期望减实际为 `(-1.713459, +0.078946, -20.313150) mm`。期望速度和加速度均为零，Cube 底部距桌面约 19.994 mm，桌面 Fz 为 0；QP 的共同 Z 加速度残差为 `+0.000890 m/s²`，该帧无力矩或力矩变化率激活指标。v10 和 v11 同样在末端静止参考下达到峰值，Z 分量分别为 −22.131 和 −21.680 mm，桌面力均为零，峰值帧共同 Z QP 残差分别为 0.002620 和 0.001812 m/s²。

三次末端共同 Z QP 残差 RMS 为 0.01433、0.01394、0.01419 m/s²；共同 Z 指令达到 ±0.15 m/s² 限幅的帧比例为 68.9%、68.7%、66.3%。v9 在下降末段 `t≈114.5 s` 已有约 10.8 mm 误差，说明参考转换之前已出现滞后；末端阶段随后反复振荡。以双 TCP 速度的 60 Hz 差分估计共同加速度，并与上一帧 QP 预测比较，末端 Z 差值 RMS 为 0.193、0.188、0.193 m/s²；它混有差分噪声、采样时序、接触与模型误差，不能直接作为独立的动力学误差测量。

**可复现的模型假设不符：** `_payload_gravity_term` 根据 `clip((cube_z - 0.260) / 0.010, 0, 1)` 衰减重力补偿，不检查是否实际接触桌面。以 `bottom_gap > 1.5 mm`、实测 `Fz < 0.2 N`、前馈比例 `< 0.99` 定义“明显未支撑但已减载”，三次分别有 1.500、1.733、1.167 仿真秒的 60 Hz 帧。最严重样本的载荷比例分别为 0.208、0.177、0.203，底面仍距桌 1.872、1.622、1.734 mm，测得 Fz 均为 0。例如 v9 的 `t=116.500 s`，底面距桌 8.329 mm、Fz 为 0，补偿已降至 85.46%。这说明控制模型在无支撑时提前减少了部分载荷补偿，不能把这部分视为桌面已真实承重。

**判断与未证实假设：** 三次峰值的主要表现是末端闭环的共同 Z 跟踪与负载动力学问题，而非双臂相对闭合误差；静止参考排除了“峰值当刻参考仍在移动”的解释，但下降参考转换可能引入初始偏差。峰值当刻 QP 残差很小且没有力矩或力矩变化率激活，故没有证据把峰值主要归因于当刻 QP 软目标竞争或硬力矩饱和；这不排除此前的竞争、执行动力学和模型误差产生影响。提前卸载、积分项积累、加速度限幅与末端闭环耦合均是可检验的候选机制，当前相关性不构成单一根因证明；尤其峰值时 Cube 高于 0.270 m，补偿比例已回到 1。

建议后续只进行一个受控、配对的仿真对照：保持轨迹、增益和全部硬约束不变，在 Cube 未受桌面支撑时保持完整 payload 重力补偿，仅在独立接触力确认支撑后按明确规则衰减，并比较相同分段误差、接触力和闭环状态。该方案是待验证提议，本轮没有实现或声称其有效，也没有开展参数扫描。

## MoveIt 退出

| 运行 | launch 进程退出码 | `move_group` 子进程退出码 | 情况 |
|---|---:|---:|---|
| v9 | 0 | −11 | 任务完成后的 shutdown segmentation fault |
| v10 | 0 | −2 | 子进程收到 SIGINT；未见 v9 的 −11 |
| v11 | 0 | −2 | 子进程收到 SIGINT；未见 v9 的 −11 |

三个 `dual_joint_state_bridge.py` 子进程均记录 −2。launch 为 0 不意味着子进程全部正常退出；v9 的故障在这两次复验中未以 −11 重现，但不足以证明已修复。本轮没有改动 ROS/MoveIt 关闭逻辑。

## 证据与复现

- 三次统一指标及 10 Hz 紧凑时序：[v9 指标](../../offline/task28_g3_stage_a/evidence/stage_a_repeatability_51b048c_checked/task28_reference_placement_v9.json)、[v9 时序](../../offline/task28_g3_stage_a/evidence/stage_a_repeatability_51b048c_checked/task28_reference_placement_v9_series.jsonl)、[v10 指标](../../offline/task28_g3_stage_a/evidence/stage_a_repeatability_51b048c_checked/task28_stage_a_repeatability_v10.json)、[v10 时序](../../offline/task28_g3_stage_a/evidence/stage_a_repeatability_51b048c_checked/task28_stage_a_repeatability_v10_series.jsonl)、[v11 指标](../../offline/task28_g3_stage_a/evidence/stage_a_repeatability_51b048c_checked/task28_stage_a_repeatability_v11.json)、[v11 时序](../../offline/task28_g3_stage_a/evidence/stage_a_repeatability_51b048c_checked/task28_stage_a_repeatability_v11_series.jsonl)。峰值和 RMS 使用完整 60 Hz 原始轨迹计算，图和紧凑时序降采样到 10 Hz，因此图中最高点可略低于表中峰值。
- [末端跟踪对比图](../../offline/task28_g3_stage_a/evidence/stage_a_repeatability_51b048c_checked/terminal_tracking_comparison.png)；[v9 原报告与证据](TASK28_STAGE_A_FINAL_PLACEMENT_AND_RELEASE.md)。
- v10：[summary](../../offline/task28_task27_integration/evidence/task28_stage_a_repeatability_v10/summary.json)、[执行器日志](../../offline/task28_task27_integration/evidence/task28_stage_a_repeatability_v10/task27_executor.log)、[参考一致性](../../offline/task28_task27_integration/evidence/task28_stage_a_repeatability_v10/reference_consistency.json)、[原验收分析](../../offline/task28_task27_integration/evidence/task28_stage_a_repeatability_v10/placement_release_metrics.json)、[最终 GUI](../../offline/task28_task27_integration/evidence/task28_stage_a_repeatability_v10/final.png)。
- v11：[summary](../../offline/task28_task27_integration/evidence/task28_stage_a_repeatability_v11/summary.json)、[执行器日志](../../offline/task28_task27_integration/evidence/task28_stage_a_repeatability_v11/task27_executor.log)、[参考一致性](../../offline/task28_task27_integration/evidence/task28_stage_a_repeatability_v11/reference_consistency.json)、[原验收分析](../../offline/task28_task27_integration/evidence/task28_stage_a_repeatability_v11/placement_release_metrics.json)、[最终 GUI](../../offline/task28_task27_integration/evidence/task28_stage_a_repeatability_v11/final.png)。新运行的大型 60 Hz 原始轨迹保留本地，不提交到 Git；统一指标包含 shutdown 日志摘录。
- [统一分析脚本](../../offline/task28_g3_stage_a/analyze_repeatability.py)要求输出目录尚不存在，避免覆盖旧证据。两个新运行的原放置验收 `--require-complete` 与对象参考一致性 `--assert-consistent` 均通过。聚焦回归：`PYTHONPATH=offline/task28_g1_1:. python3 -m pytest -q offline/task28_task27_live_qp/tests offline/task28_g3_stage_a/tests`，**48 passed**。

运行命令和固定参数见 [v9 复现命令](TASK28_STAGE_A_FINAL_PLACEMENT_AND_RELEASE.md#复现和回归)；复跑必须使用新的 run ID、空的证据目录和隔离 ROS domain。此结论仅限 Stage A 仿真，未进行真实硬件或 Sim2Real 评估。

# R3：多源信息融合状态估计与双臂技能衔接研究规划

> 状态：**STAGED_SIMULATION_RESEARCH / P0 PASS / P1 PARTIAL / P2A PARTIAL**
> 日期：2026-11-07  
> 仓库：`dreamerToTOE/dual-arm-embodied-palletizing`  
> 规划基线：`research/stage-b-r2b-runtime-integration`，`4e040d9ea21281762cf29499278cb57aac96e15e`  
> 本文保留分阶段规划；P0/P1/P2A 实现与数值以对应报告为准，不代表接触检测或完整 Task27 已通过。


## 2026-11-07 用户批准的 P2A 范围修订

- P0 PASS、P1 PARTIAL 的已完成报告及失败证据保持原样，不因缩小范围而升级历史结论。
- 当前科学实验只覆盖 Cube 已落桌后的 Stage B，实验中假设持续桌面支撑；不研究
  Stage-A 搬运恢复、小幅离桌检测或侧翻。保留 P1 平面模型及其既有门限。
- 视觉是间歇观测，不要求连续检出；先解决动态时间对齐，再开始在线多源融合。
- 次序为 **P2A：时序与遮挡审计 → P2B：基于带时间戳接口的桌面融合研究**。
  本轮只授权 P2A；P2B、吸附/接触分类和技能切换均未开始。
- 仍只在 Isaac Sim 内工作；不增加真机、Sim2Real、RL 或 live switching 要求。

## 0. 用户决定与工程边界

1. Stage B 大部分时间以 FR3 关节状态和正运动学推算吸盘与 Cube 的相对位姿；视觉及可用的模拟末端力/力矩、吸盘反馈用于校验附着状态和异常恢复。**不能以 FK 所依赖的刚性附着假设再反证附着可靠**。
2. 创新点一改为**多源信息融合的机器人—物体交互状态估计**，同时估计 Cube 位姿、吸附置信度、接触关系及不确定性。
3. 当前实验仅覆盖持续桌面支撑的 Stage B，对视觉求解保留 `z=z_table+h/2`、`roll=pitch=0`，估计 `(x,y,yaw)`。支撑是受控实验假设，不新增离桌/侧翻检测器；P1 已记录的反例和门限保持原样，缺帧/残差异常仍标记不确定。
4. 沿用 Isaac Sim 4.5、双 FR3、原侧吸工具和 Task27 几何。遵守仓库 `AGENTS.md`（2026-10-31）：**仿真优先、功能优先**；不把真机传感器、硬件认证、Sim2Real 作为当前研究前置条件。允许有界、可重建的实际 Isaac 动作实验，保留必要碰撞、关节、力矩和数值检查，不做无止境预检。
5. 历史 Task26/Task27 依赖 Isaac `/cube_poses` Ground Truth 完成规划和分段推入。R3 的状态估计要有**无 Cube 真值在线输入的独立实验路径**；Ground Truth/PhysX 接触回调允许作为离线评价 oracle。若把 PhysX 力值明确建模为模拟传感器输入，必须与评分标签隔离，不能用同一字段自证准确。
6. 现有 R2-B 研究状态为 PARTIAL/BLOCKED：静态 `cal24` 四 marker 留出视图位置误差中位数 5.01 mm / P95 10.86 mm（并非 Cube 在线定位精度）；早期 ID23 Cube 静态转移曾得到 47.2 mm 中位数。`physical07` 有 1,550 组 RGB-D，writer 延迟中位数 0.166 s、最大 0.731 s，但没有曝光级对齐证明。E0/E1 对加载深墙接触的区分能力不足；不能把 proximity 当 loaded-contact。Stage B 侧压/FCL 等完整执行问题仍需保留为独立工程问题。

## 1. 统一研究问题与三项创新点

| 创新点 | 科学/工程问题 | 候选算法与最小基线 | 核心评价 |
| --- | --- | --- | --- |
| I1 多源状态估计 | 视觉缺失/偏差、吸附滑移、墙接触下，能否准确估计 Cube、附着与接触状态？ | FK+固定抓取变换；平面约束视觉；残差/滞回规则；随后按证据考虑 EKF、HMM/IMM/GRU | 位姿误差、接触 F1/延迟、吸附异常识别、UNKNOWN 校准、鲁棒性 |
| I2 技能平滑切换 | 双臂搬运→释放→单臂重吸附→推入等切换，如何减少 `Δq`、速度、加速度、jerk 和力矩突变？ | 时间参数化/最小 jerk/优化过渡基线；有必要再比较 PPO/SAC 过渡策略 | 末端误差、峰值 jerk/力矩、切换耗时、成功率、接触冲击 |
| I3 动态技能组合 | 状态异常时，何时继续、保持、后退、调整或重吸附？ | 规则状态机/行为树/技能图；有数据优势再考虑分层 RL | 完成率、恢复率、恢复时间、误切换和不必要动作 |

**职责分离**：I1 输出当前状态及置信度；I3 选择下一技能；I2 负责从当前技能到目标技能的运动/力平滑过渡。不要把 I1 直接写成带控制权的 selector。

## 2. 状态定义及可观测性

- Cube 连续状态：在桌面支撑的 Stage B 优先用 `s=(x,y,yaw,vx,vy,omega)` 和协方差/误差上界；本轮只输出独立 q→FK 的机器人 TCP；Cube 与工具变换及融合留给 P2B，不研究搬运。
- 吸附离散状态：`ATTACHED / SLIP_SUSPECTED / DETACHED / UNKNOWN`。可用输入包括模拟吸盘反馈、FK 预测与独立视觉残差、执行命令与负载运动一致性、可辨识的模拟受力信息。`suction_command=ON` 或 `is_closed()` 单独不足以证明无滑移刚性连接。
- 接触状态至少区分：`FREE_PUSH / DEEP_WALL_NEAR / DEEP_WALL_LOADED / STALLED_AMBIGUOUS / UNKNOWN`。保留 Cube–Table 和 Tool–Cube 等关系边；接近墙、非零受力与真正卡滞不能同义替换。
- 深墙几何量：对轴向对齐 Cube，可用 `g=x_wall-(x_cube+L/2)`；有 yaw 时用变换后的 Cube 支持函数/凸包最前端计算。`g≈0` 只能生成接近或接触候选；加载接触另需力/运动残差或独立模拟接触测量。
- 状态输出需要来源、物理时间戳、观测时延、数据新鲜度、置信度和 `UNKNOWN`。视觉缺帧或吸附证据不足，不允许由估计模型自行填成高置信 `ATTACHED`。

## 3. 分阶段研究计划与验收

### R3-P0：数据与因果接口核查（立即开始）

**动作**：读取当前真实代码及证据，确认 RGB/Depth、ArUco corners、K/T、q/dq/TCP、suction state/command、controller reference、模拟 effort/force、Cube GT、PhysX contact oracle 是否实际可获取及其采样频率/时间戳；追踪哪些数据进入旧 Task27 控制器、哪些仅用于 R2-B shadow/离线评分。先检查 physical07 原始数据是否存在于 Codex 运行环境，不把 GitHub 上没有的本地大文件假定为可用。

**交付**：`R3_P0_SENSOR_AND_CAUSAL_AUDIT.md`，一份可机读的字段/来源/消费者/时序表及最小数据集 manifest。清楚区分 **online observable / optional simulated sensor / evaluation oracle / unavailable**。

**门槛**：所有计划输入有确切代码来源或明确 `UNAVAILABLE`；不要求真实 FR3 硬件数据。不改 live 控制器。

### R3-P1：平面约束视觉定位的可证伪 A/B（首个算法实验）

**适用条件**：仅对 Cube 已落桌、桌面高度与 Cube 几何已知的帧采用 `T_WC(x,y,yaw)`；对于搬运和明显离桌/倾斜帧拒绝该约束，仍由吸附 FK 推算或输出 UNKNOWN。

**实现**：在相同图像、marker corners、相机 K/T 和 marker-to-Cube 几何条件下，比较：

- V0：现有无平面约束的 6DoF ArUco/PnP；
- V1：固定 `z=z_table+h/2`、`roll=pitch=0`、优化 `(x,y,yaw)` 的重投影最小化（鲁棒损失/边界检查）；
- V2（可选）：V1 与 FK/上一帧状态融合，但需作为独立组，不能混入 V1 的纯视觉比较。

**评估**：冻结同一训练/标定集和独立 holdout（按场景/轨迹分组，不能将同一轨迹邻帧随机拆分为训练和测试）；用 Cube GT 仅离线评分。记录 XY median/P95/max、yaw 误差、深墙 gap 误差、检测率、拒绝率、CPU 耗时、动态误差与延迟。单独区分系统标定偏差与运动滞后。不可把 `cal24` 的相机外参误差当作 Cube 在线误差。

**门槛**：报告 V1 相对 V0 的真实改进或退化，附失败样本及可重复脚本。不预设“必然大幅提升”；若因 K/T、marker 尺寸/位置等根因失败，做一次有界诊断和针对性修复，再如实给出 PARTIAL。旧 3 mm 最终落位门槛与感知误差评价必须分开报告。

### R3-P2A：动态传感器时间对齐与 Stage-B 视觉缺失审计（本轮）

**动作**：逐帧审计 physical07 侧压区间 107 次未检出，区分成功采集、未检出及
未执行的名义采样时隙，保留联系表、原图和 UNKNOWN。只以 executor receipt 定义阶段。
核查本机 Isaac 4.5 camera/annotator 的 frame/time 语义；建立同 render-product 的
RGB/depth 采集与有界 q/dq→独立 FK 历史，离散 suction/command 只从过去保持。

**实验**：短时、可重建的桌面 Cube 运动与停止/启动刺激，比较同一图像的旧 request
receipt 与新 render-time 关联；GT 只在评分侧计算误差。一个 physics tick 是目标而非
强制 PASS。保留启动、几何或渲染失败，不复跑完整 Task27、不改控制器。

**交付**：`R3_P2A_VISUAL_OCCLUSION_ROOT_CAUSE.md`、
`R3_P2A_DYNAMIC_TIME_ALIGNMENT.md`、时间字段映射、冻结配置、脚本、逐帧结果与最终决定。

### R3-P2B：带渲染时间的 Stage-B 间歇视觉/机器人状态接口（后续，未授权执行）

以 P2A 的 `frame_id/render_time/arrival_time/pair_id` 和有界 robot-history 接口为入口，
再研究桌面平面位姿的 FK/视觉融合。q→FK 与工具几何独立核验；图像缺失时保留观测
年龄和不确定性，不将缺帧自动等同脱附，不将 `closed` 当作附着证明。实验仍从落桌后
开始。附着分类、加载接触分类、Stage-A 恢复及技能选择均为另行授权的后续工作。

### R3-P3：深墙加载接触和卡滞识别

**动作**：融合由 FK/视觉估计的几何 gap、运动进度、机器人力/力矩残差以及经过定义的模拟受力传感器。明确 `NEAR`、`LOADED`、`STALLED` 的不同定义。独立 PhysX pair、impulse、penetration 和接触持续时间用于评价；若某个 PhysX 力通道被用作输入，则 oracle 必须使用未泄漏的独立标签或对照设计。

**实验**：正常推进、接近但未接触、真实加载深墙接触、摩擦/控制造成的非墙面停滞、模拟卡滞；至少覆盖不同初始位置/命令速度等受控变化。对比几何规则、现有 E0/E1、R3 融合版，报告混淆矩阵、precision/recall/F1、转移延迟与 UNKNOWN。不得用一条正常 episode 声称卡滞恢复成功。

**交付**：I1 技术评审：决定是否确有必要引入 EKF/HMM/IMM/学习模型。未完成复杂状态识别也允许继续做独立、受控的功能实验，但不得宣称 I1 PASS。

### R3-P4：技能切换平滑性

**动作**：先在已能复现的仿真技能之间建立最小 jerk/轨迹时间化/优化式过渡基线，保留关节/扭矩/碰撞检查；再根据证据判断是否引入 PPO 或 SAC 过渡策略。重点是抑制速度、加速度、jerk、力矩跳变而非改变整个搬运 PPO 主策略。

**评价**：关节轨迹/力矩连续性、任务成功率、接触冲击、轨迹误差和切换时间；相同初态和扰动对照。

### R3-P5：动态技能组合与恢复

**动作**：在 I1 的状态/置信度基础上，先使用行为树或技能图选择 `CONTINUE_PUSH / HOLD_CHECK / BACKOFF / RELOCALIZE / REATTACH / SIDE_ADJUST`。保持 I2 平滑控制与 I3 技能选择分离。先 shadow，对通过局部碰撞与数值检查的短动作逐步做实际 Isaac 执行；不将历史 side-compaction FCL 失败冒称成功，也不要求真机级形式化证明。

**评价**：重复的正常/异常 episode、完成率、恢复率、时间、错误技能切换、碰撞和接触指标。最终综合对比 I1/I2/I3 消融。

## 4. 历史 P0/P1 执行边界（已完成，证据保留）

1. 从本规划分支继续，先检查 `AGENTS.md`、基线 SHA、已有修改和 `research/stage_b_vision_contact_r2b`。不要盲目覆盖其他研究分支或未提交用户改动。
2. 实施 P0 审计和 P1 **最小可重复对照**。优先重用现有 R2-B 图像、标记和脚本。若缺少原始 RGB/corners/对应 GT，仅执行有限的可复现 Isaac 采集补齐；不得伪造数据或以旧静态相机 holdout 替代 Cube 实验。
3. 新增代码与报告放在 `research/stage_b_multisensor_r3/`；保留旧任务和控制器行为，不修改 Task26/27 成功判据，不开启 RL/GPU 长训、自动技能切换或真实硬件工作。
4. 执行必要单元测试与短实验，记录输入、配置、commit、误差统计、缺失数据、失败原因与最终 PASS/PARTIAL/INCONCLUSIVE。遵循仿真功能优先，避免过度形式化门禁。
5. 在当前 R3 工作分支提交和推送，**不合并 main 或 side-suction-palletizing**。最终以中文汇报关键数值、有效证据、未知项和下一步建议。

## 4.1 当前 P2A 执行边界

从 `c01e18a1e30e7118a77e8e51f0856c304a74d08f` 继续，只提交 P2A 研究文件。
不改 Task26/27 行为、几何或成功判据，不部署融合或技能选择。记录全部尝试，完成
必要 CPU/有界 Isaac 检查，提交推送当前 R3 分支，停止并以中文报告 P2B 建议。

## 5. 相关已有证据（不得提高历史结论级别）

- `research/stage_b_vision_contact_r2b/R2B_CAMERA_PRECISION_FINAL_DECISION.md`
- `research/stage_b_vision_contact_r2b/R2B_RUNTIME_INTEGRATION_FINAL_DECISION.md`
- `research/stage_b_vision_contact_r2b/R2B_SENSOR_PIPELINE_FINAL_DECISION.md`
- `research/stage_b_vision_contact_r2b/R2B_E0_E1_SHADOW_EVALUATION.md`
- `research/stage_b_vision_contact_r2b/R2B_CONTACT_ORACLE_LABEL_AUDIT.md`
- `docs/tasks/TASK26_TRUCK_BOX_PUSH_IN.md`
- `docs/tasks/TASK27_FIVE_CUBE_CENTER_INSERT.md`
- `AGENTS.md`

**总原则：** I1 的估计结果供 I3 做技能决策，I2 平滑执行；当前先把平面约束视觉和因果数据接口验证清楚，再推进 FK/吸附与加载接触融合。每个实验的真实性优先于算法名称和漂亮的数字。

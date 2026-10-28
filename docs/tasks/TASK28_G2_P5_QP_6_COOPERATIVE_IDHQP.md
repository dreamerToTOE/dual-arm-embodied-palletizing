# Task28 G2-P5-QP-6：协作物体空间逆动力学 HQP 研究

**总状态：`QP6_PARTIAL`。** 理想耦合模型的独立数学与数值验证通过；实际完整双 9-DOF 逆动力学源为 `SOURCE_BLOCKED`。正式运行了 A/S05、A/S10，两格均在原冻结 G1 左绝对平移门首次失败时停止。其余八格明确跳过。没有证据证明物体空间协作改善了实际跟踪，也没有实际 Cube/D6、物理接触 wrench、有限吸附容量或内力闭环通过的结论。

## 1. 起点、文献和历史证据

任务目录最初没有仓库；在本聊天 `work/dual-arm-embodied-palletizing` 检出指定远端分支。开始时本地 HEAD、upstream 和 `git ls-remote` 均为 `ce185a38efff2a93a1f8b9c29d564535bbb019c9`，分支 `side-suction-palletizing`，工作树干净。起点 **1104 个已跟踪文件**逐文件与 Git blob 比较，全部未改变。绑定见 [source_binding.json](../../offline/task28_p5_qp_6/source_binding.json)。本次只新增 `offline/task28_p5_qp_6/` 和本报告。

核查实际 G1 FK/闭链/碰撞代码、NR-0 投影及验证器、QP-2 反馈、QP-4 归档、QP-5/R1 控制采集与 R2 诊断、G3-P2-1 抓取映射、P2-3 阻断记录、P2-4-R3 显式 effort 代码及归档。既有 NR/QP 运动学、原移动拒绝、G3-P2-3 BLOCKED、R3 单臂候选结论均保留；未运行 QP-5-R3 五控制器 benchmark。

文献核查仅获得出版商摘要和机构书目记录，未取得全文：Dio、Völz、Graichen，IROS 2023，643–648，DOI `10.1109/IROS55552.2023.10341854`。摘要直接支持采用耦合动力学、分层优化、力矩/内部载荷约束及双臂仿真。本文没有声称复现其全部公式、参数或实时实现。物体中心 HQP、耦合逆动力学和内载优化本身已有先例，不能作为 Task28 的独立新颖性。[IEEE 摘要](https://ieeexplore.ieee.org/abstract/document/10341854)、[FAU 机构记录](https://cris.fau.de/publications/306133864/)。

## 2. 数学模型与独立验证

完整定义和推导见 [MATHEMATICAL_SPECIFICATION.md](../../offline/task28_p5_qp_6/MATHEMATICAL_SPECIFICATION.md)。采用世界表达的 Cube 中心几何 twist `[v;ω]` 及普通时间导数 `[a;α]`，接触 wrench 关于原冻结名义 TCP、以各接触局部轴表达、方向为**作用到 Cube 上**。反作用到机器人取负。固定抓取变换仍是原 G1 的旋转和 `±0.061 m` 偏置，没有新造抓点。

理想自由广义坐标模型为：

```text
M_robot*qdd + c_robot + g_robot = tau - J_contact.T*f
M_object*a + h_object            = G*f + w_environment
J_contact*qdd + Jdot_contact*dq  = H*a + Hdot*V
```

`G=H.T` 保证虚功一致。局部接触坐标随物体旋转，Hdot 和 Jdot 均包含旋转坐标项；不能只保留世界系向心加速度。支持 COM 偏移的物体惯性和重力项，由 COM 加速度与力矩平移独立推导。两个机器人保留全部 **18 个 actuator DOF**；arm effort 是 N·m、finger effort 是 N，没有把七维命令当作九维命令。

Cube 质量 0.8 kg、尺寸 0.12 m 来自冻结 fixture。COM 位于几何中心、均匀密度和 `I=0.00192 I₃ kg·m²` 是明确的工程假设；本次没有实际 Cube stage 数据来独立验证它们。因此它们只进入理想数学样例，不能满足实际 Level B 的物体源门。

最终离线数值后端为 OSQP 1.0.5。每级优化后将已达到的任务向量冻结为下一级等式，保持严格优先级；没有把跨级 weighted sum 改称 HQP。P1 为共同物体加速度，P2 为相对协调，P3 为内载偏差、effort、effort 变化及显式小加速度择优项。严格刚性兼容时 P2 是冗余任务，不能靠软化物理等式制造协调收益。使用原 G3-P2-1 的 W 度量及其加权零空间分解；度量不是吸附容量。

独立测试覆盖全部十二类要求：机器人/物体平衡、抓取/Jacobian 一致性、Jdot/Hdot、作用反作用、参考点平移、重力、秩和条件、精确度量内载、凸性/可行性/KKT、不可行负例及不软化硬约束。含手算一维载荷、独立 FK 位姿差分、active-face 枚举、小规模直接 KKT 和独立零空间最小二乘；并非只重复调用生产函数。

[math_final](../../offline/task28_p5_qp_6/evidence/math_final/summary.json) 的 **12 个合成耦合案例**结果：

| 数值检查 | 最大值 |
|---|---:|
| 与独立 oracle 的完整 54 维解差 | `1.73965e-6`，门 `2e-6` |
| 原始硬等式残差 | `1.08765e-11` |
| P3 平方目标差 | `6.01e-16` 以下 |
| 内载零空间残差 | `1.29e-15` |

合成例的 G/H/J/等式秩分别为 6/6/12/36。离线完整 HQP 墙钟 p50 约 7.77 ms、p95 8.41 ms、peak 8.43 ms；包含 Python 装配和数值精化，不是正式控制周期，也不证明 1 kHz 实时性。矩阵、变量、优先级、KKT 和计时全部归档。

开发记录保留在 `math_v1` 至 `math_v6`。早期弱正则方向的解差约 `0.00168169`，不能通过逐变量门；曾探索的 objective-only 检查不作为最终依据。最终采用更严格精度和关闭精化阶段 polishing，在去除数值重复等式行后再次检查**全部原等式**；没有修改目标权重或放宽原 `2e-6` oracle 门。最终 `coupled.py` 只用于离线数学，从未作为实际物理控制器运行。

## 3. 实际 Isaac 源表征与阻断

实际安装版本为 `4.5.0-rc.36+release.19112.f59b3005.gl`。冻结 USD SHA-256 为 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`，articulation wrapper SHA-256 与 R2 一致。版本、源码、模型哈希、API 名称、形状、单位、时间和命令缓冲均保留。

有界源运行使用两个隔离 FR3、CPU physics backend，每侧实际顺序为七个 arm 加两个 finger。两侧完整 M 为 `1×9×9`，c/g 为 `1×9`；M 有限、对称、正定，合并所有样本的最小/最大特征值约 `0.0109527 / 2.74362`。这些混合坐标数值不能解释为物理容量。

先运行 12 步重力保持，再运行 18 步完整坐标小幅加速度脉冲。每步记录同一个 pre-step cursor 的 q/dq/M/c/g 和 proposed effort，随后推进一次。读取时间标签来自同步 World physics cursor；API 没有独立的每关节硬件时间戳，不能把该标签当作实机 freshness 认证。

| 源门 | 结论 |
|---|---|
| 冻结 USD、wrapper、DOF 次序、形状、有限性、M 正定 | 通过本次检查 |
| 显式 force mode、runtime drive Kp/Kd=0 | 通过 |
| 重力保持 | 通过；arm 峰值速度约 `6.26e-7 rad/s` |
| command buffer / applied readback | 通过；最大差 `9.33e-7`，来源仍是命令读取 |
| 完整 9D/18D 自由逆动力学执行一致性 | **SOURCE_BLOCKED** |
| 实际 Cube mass/COM/inertia 与 D6/contact 语义 | 未到达，不能通过 |

七个 arm 加速度误差峰值约 `2.885e-5 rad/s²`；finger 峰值约 **`0.00441594 m/s²`**，超过事前 `0.002 m/s²` 门。独立 raw evaluator 首次拒绝为 row 18、右侧、physics step 21、post time `0.1750000091 s`。完整失败帧见 [evaluation_v1/source.json](../../offline/task28_p5_qp_6/evidence/evaluation_v1/source.json)。

额外 pxr-only 检查确认：原 USD finger2 包含 `PhysxMimicJointAPI:rotX`、gearing `-1` 和 finger1 引用；finger upper limit 约 `0.0399999991 m`，初始命令是 0.04 m。mimic、上限约束与数值求解可能引入未建模的约束反力，**这只是需要进一步辨识的解释线索，不是唯一因果证明**。没有删除 mimic、移除手指、替换另一姿态动力学或直接转为 14D 来绕过失败。

运行控制台仍包含 frozen link8/hand_tcp 的负质量/近似惯量警告，未修复或改写资产。这限制实际模型外推，不能以“返回 SPD M”解释成真实 FR3 动力学标定通过。`get_applied_joint_efforts` 始终是 command provenance；`get_measured_joint_efforts` 单独保存为 solver-projected effort，均未冒称硬件 actuator-only torque sensor。

源测试有明确协议局限：加速度验收在完整 30 步采集后离线进行，首次失败之后的 **11 帧仅供诊断**。因此本源 probe 不能称为每步动力学 fail-closed 的实测实现。该问题与完整动力学源失败一起保留，未重跑或调门。B/C/D/E 均没有执行。

## 4. 新平滑参考与事前几何检查

S05 为五段各 0.10 s、总 0.50 s；S10 为五段各 0.20 s、总 1.00 s。两者采用 quintic stop-to-stop law，保留原初始几何及五个 accepted QP-4 端点和顺序。世界平移、旋转指数、twist、acceleration 均按同一 s 的解析导数生成；desired TCP 从同一个 Cube pose 和 frozen grasp 得到。A/B 的独立 joint quintic 使用原 q 端点，原 QP-4 qdot 没有被当作 qdd。

每条正式前完成 **501 个样本**的冻结 FK/G1/FCL、位置界和导数/时间检查。最大左/右绝对几何残差约 `0.2643 / 0.4379 μm`，相对 `0.5879 μm`，均低于原门。解析 twist/acceleration 与独立差分一致；所有段边界停止并有零一阶、二阶导数。端点取自原矩阵；终点采样存在约 `4.34e-19` 的浮点旋转差，不涉及几何重拟合。

首轮 `profiles_v1` 因适配器误传 `bilateral_transitioning` 给只接受 `bilateral/detached` 的 FCL 接口而拒绝，未查询 shape pairs；保留原输出和源码。`profiles_v2` 只修正接口参数，未改变轨迹、G1、碰撞模型或 ACM，全部样本通过。

参考速度的显式“dq 对有效速度界”独立汇总在正式运行后补充，峰值 S05/S10 为 `0.0137572 / 0.00687860 rad/s`，确实低于冻结有效界。**不能把这份后补汇总追认为事前已完成的独立速度界验收。** 正式执行的实际状态速度门始终逐步有效。本点列为流程局限。所有有限采样结论均不证明连续碰撞/闭链安全。

## 5. 冻结矩阵、实际运行与首次失败

物理前冻结 [preregistration.json](../../offline/task28_p5_qp_6/preregistration.json) 和 [formal_registration.json](../../offline/task28_p5_qp_6/formal_registration.json)。正式注册 SHA-256 为 `a9b460e998a02c1d9080707318d7831c19cf4caca58da88ee363f9abe582e2a8`；其绑定的全部源码在两次运行后仍匹配。没有开发型 Isaac controller 实验，使用 1 次有界源运行、2 个 distinct formal cells；没有失败后调参重跑同一格。

A 使用历史 gravity+PD；B 为完整 18D joint computed torque；C 为共同 implied-object acceleration → relative acceleration → effort/change 的严格层级，使用完整 18D 逆动力学，且不含物理 Cube reaction。B/C 的差异是任务耦合、目标和 qdd 生成方式，不是改变共同源或几何门。由于全 DOF 源未过，B/C 都没有物理比较数据。

Level A：无 Cube、无物理接触。

| Controller/profile | 状态 | 初始保持 | 运动步 | 末端保持 | 首次失败/原因 |
|---|---|---:|---:|---:|---|
| A/S05 | **REJECTED** | 120 | 7/60 | 0/60 | 左绝对平移 `0.1279523 mm` |
| A/S10 | **REJECTED** | 120 | 12/120 | 0/60 | 左绝对平移 `0.1106434 mm` |
| B/S05 | SKIPPED_WITH_REASON | 0 | 0 | 0 | 完整动力学 SOURCE_BLOCKED |
| B/S10 | SKIPPED_WITH_REASON | 0 | 0 | 0 | 同上 |
| C/S05 | SKIPPED_WITH_REASON | 0 | 0 | 0 | 同上；没有 14D 绕过 |
| C/S10 | SKIPPED_WITH_REASON | 0 | 0 | 0 | 同上 |

Level B：实际 Cube/ideal D6。共享完整动力学失败阻断依赖的接触阶段；未用旧 P2-3 BLOCKED 状态冒充通过，未启动静态 D6 或移动尝试。

| Controller/profile | 状态 | 物理步 | 原因 |
|---|---|---:|---|
| D/S05 | SKIPPED_WITH_REASON | 0 | 完整动力学 SOURCE_BLOCKED；实际物体/attachment 源未建立 |
| D/S10 | SKIPPED_WITH_REASON | 0 | 同上 |
| E/S05 | SKIPPED_WITH_REASON | 0 | 同上；calibrated contact/reaction source 也未建立 |
| E/S10 | SKIPPED_WITH_REASON | 0 | 同上 |

两次 A 各用全新的 Isaac 进程初始化。每个正式 step 先读取两侧状态、名称、模式、M/c/g 和 shared cursor，检查有限性、同步、实际几何与 joint safety，再验证完整 proposed effort 和 rate；两侧通过后才共同发送，推进一步，记录实际 post-state 与下一参考，再检查原 G1。没有 position-drive fallback、zero-torque 或 hold-last-torque 安全承诺。

独立评估确认，首次 G1 失败恰好是各自最后一个归档帧。分别保留 113 / 168 个未执行注册步，未完成任何一条全程运动，也未进入末端保持。

| 首次失败指标 | A/S05 | A/S10 |
|---|---:|---:|
| 运动 post-time s | 0.0583333364 | 0.1000000052 |
| physics step index | 129 | 134 |
| 左绝对平移 mm | 0.1279523 | 0.1106434 |
| 右绝对平移 mm | 0.0635455 | 0.0508958 |
| 相对平移 mm | 0.0928743 | 0.0958332 |
| 共同平移 mm | 0.0897134 | 0.0715554 |
| 相对旋转 rad | 9.8867e-5 | 1.06417e-4 |
| 最大 arm q 误差 rad | 2.69777e-4 | 2.01059e-4 |
| 最大 arm dq 误差 rad/s | 0.00757921 | 0.00345176 |
| 最大 arm command N·m | 28.60906 | 28.60906 |
| 最大 arm effort change rate N·m/s | 0.255386 | 0.0650294 |
| command/readback 最大差 | 9.47e-7 | 9.49e-7 |

原 translation 门仍是 **0.100 mm**；rotation 门仍是 **0.001 rad**。未触发 effort 或 rate 拒绝，也没有 clipping。arm 实际上限采用研究 cap 与冻结 USD/配置界的交集；rate 和 finger effort 是研究设定，全部标记 `SIMULATION_RESEARCH_LIMITS_ONLY`，未称硬件认证界。

独立 evaluator 逐帧保存 per-DOF q/dq、command/rate、共同/差分及绝对/相对 SE(3) 误差，也保存基于实际差分 qdd 的动力学残差，后者仅用于诊断，不能把未通过的完整源倒推为 PASS。A 没有 QP，因此 QP feasibility/solve percentiles 标 `NOT_APPLICABLE/null`，不是伪造 0 ms。没有物理 Cube，物体误差明确属于 **virtual object**。所有 skipped 格的跟踪、接触分配、内载和载荷裕度均为 null，不是零误差或成功。

受控循环共执行 source 30 步 + formal 259 步 = 289 步；另有三次 World reset 各自推进 2 个初始化步，已知总 physics index 增量为 **295**。不能将 127/132 个正式受控步与其末尾 physics index 129/134 混成同一计数。

## 6. 科学解释与论文贡献边界

**物体空间协作是否改善跟踪：本次不能判断。** C 没有源资格，不能用它的 CPU 可行解代替实际对照。S10 比 S05 更晚触门仍是截断失败前缀，不能解释为整轨迹协作收益。已有直接证据仅说明这两条新平滑参考下，冻结的历史重力+PD 仍在首段提前超出严格绝对门。

**限制来自哪里：** 完整自由 18D 逆动力学执行一致性先失败，阻断模型式方法；随后两格独立 A 又出现实际跟踪门失败。实际 contact、Cube 惯性、D6 attachment 和 reaction oracle 没有达到资格，不能将其描述为耦合模型移动成功或实际内力验证。

**内部 wrench：** 只有合成模型中分配、null-space、度量正交和独立数学 oracle 的证据。没有可信双侧 contact/D6 reaction observation，没有真实 internal-force closed-loop PASS。`QP6_INTERNAL_WRENCH_MODEL_PASS` 的独立源观测要求未满足。

现实可用的论文材料是可复核的源审计、坐标/符号完整推导、严格层级数值验证、保持冻结几何的 C2 参考和失败前缀/共享门禁设计。它们可以形成方法实现与负结果研究材料；当前不足以支持“更大载荷”“协作跟踪改善”“内力闭环控制”或“新型 HQP 原理”的主要实验主张。仍需另行解决带 finger/mimic/limit 约束的源模型、满足原 G1 的实际 attachment，以及独立可信 reaction 观测，才有资格开展后续性能比较。

## 7. 验证、复现、偏差与最终状态

历史五组互不重叠：**789 + 53 + 5 + 6 + 22 = 875 passed**。QP-6 新增 **54 passed**，合计 **929 passed**，各组退出码均为 0。历史通过只证明软件回归，没有把物理拒绝改成通过。命令、环境、原始日志和 JUnit 位于 `evidence/regression_*`、`focused_final`；独立原始 evaluator 位于 `evaluation_v1`。

所有流程偏差和开发失败见 [protocol_deviations.json](../../offline/task28_p5_qp_6/evidence/protocol_deviations.json)：包含源测试事后拒绝/后续诊断帧、参考速度检查后补、FCL adapter 首轮错误、数值开发失败以及 World reset 初始化计数。它们没有被追认为成功 formal evidence。Kit `app.close()` 可让进程以 0 正常退出，因此 scientific acceptance 始终来自明确 `SOURCE_BLOCKED/REJECTED` JSON 和独立重算，而不是 shell exit 0。

| 独立状态维度 | 最终结论 |
|---|---|
| `QP6_COUPLED_MATH_PASS` | **支持，仅理想数学/合成源** |
| `QP6_NO_CONTACT_TRACKING_PASS` | 不支持；两格 A 拒绝，C 未运行 |
| `QP6_IDEAL_ATTACHMENT_TRACKING_PASS` | 不支持；SOURCE_BLOCKED，未运行 |
| `QP6_INTERNAL_WRENCH_MODEL_PASS` | 不支持；缺独立可信接触观测 |
| Overall | **QP6_PARTIAL，完整动力学与接触阶段 BLOCKED** |

复现入口和依赖见 [README](../../offline/task28_p5_qp_6/README.md)。只读独立数学、preflight 和 evaluator 输出到新目录，不覆盖历史归档；formal comparison 不自动重跑。本报告所在的 task-scoped commit 与远端验证由最终交付记录给出。

任务在此停止。没有运行 ROS、生产桥、实体机器人、有限吸附控制、NR-1、碰撞梯度 QP、RL、SVM 或 PCS/RSS，也不从理想刚性模型推断这些能力。

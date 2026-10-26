# Task28 G2-P5-QP-0｜集中式 14-DOF 速度 QP 离线起步基线

仓库：`dreamerToTOE/dual-arm-embodied-palletizing`；分支：`side-suction-palletizing`。
起点：`6eee5a01cba71c7802bbf0833dc3c89739dd62f6`。

## 1. 结论与研究权限

**PASS_G2_P5_QP_0_OFFLINE**。真实的集中式 14 维凸 QP 在 9 个已有 NR-0 状态、每状态 3 个非零物体速度参考上产生数值合格解；27 个一步预测均通过原 G1 状态和 B0 有限采样门禁。18 个正式控制按预期分类；最终 **514 项测试通过（原 466＋新增 48）**。这是 G2 的最小离线起步结果，不表示完整 G2 已完成。

全部结果为 `execution_authorized=false`。没有执行 Isaac、ROS/MoveIt、机器人或吸附命令，没有改动 position-only 命令链，没有位置积分执行适配器，没有 SVM、PCS/RSS、NR-1、力控制或 RL。这里的 Euler 预测仅计算数组，不向执行链发布或写入机器人目标。

已读取起点路线图、[NR-1 延后决定](TASK28_RESEARCH_DECISION_2026_10_26_DEFER_NR1_START_G2.md)、G0-A 第 6 节及 G1/NR-0/R1 报告。NR-1 已明确延期，**不是本次离线 G2 的前置条件**。原路线图中“G2 尚未运行”保留为该起点的历史记录；新结果由本报告及新目录证据单独说明，未改写旧报告。

独立副本为 `work/p5-qp-0`；原 `work/p4-nr-0` 未切换分支或清理。425 个原跟踪文件的字节、SHA256 全部保护，涵盖冻结 fixture、模型、ACM、全部 G1 验证器、原 NR 证据及 Task24/26/27。冻结场景仍为 `NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`，不引用历史 Task24 的 0.184/0.115 mm 作为新结果。

## 2. P5 来源与本次适配

方法名为 **P5-inspired centralized 14-DOF velocity-QP adaptation**。原论文为 *A unified framework for coordinated multi-arm motion planning*，IJRR 37(10):1205–1232，DOI `10.1177/0278364918765952`。已核对原 PDF 第 10 页（刊页 1214）的 Eq.17–18；[来源清单](../../offline/task28_p5_qp_0/evidence/p5_correspondence.json)含原 PDF 及另外 11 个本地材料的实测哈希、行号/页码与来源仓库状态。

| P5 原文组件 | Task28 的关系 |
|---|---|
| Eq.17a：最小化 `0.5*qdot' W qdot`，集中式联合决策 | 继承中央速度决策和正定二次代价；改为双 FR3 共 14 关节 |
| Eq.17b：block-diagonal J 的硬任务等式 | 本次将物体速度跟踪置于软目标，同时对双臂共享物体速度施加硬兼容等式；明确是适配 |
| Eq.17c、18：位置刹车/速度盒界交集 | 本次用显式正 dt 的位置 lookahead 与冻结速度界求交；不声称复制原 μ/η 参数 |
| Eq.17d 与 Eq.24–26：学习 Γ、SVM/FK 链式梯度与 SCA 约束 | 本任务不实现；FCL 后验几何门禁不能冒称这条碰撞避免行 |
| CVXGEN/NLopt/LVI 等数值实现 | 本次是 SciPy SLSQP＋HiGHS＋独立 KKT 核验，未复现原数值轨迹、时延或被动性结论 |
| 离散加速度界 | Eq.17–18 未给出本次形式的逐步加速度盒界；默认关闭，仅另设带上一有效数值命令合同的工程控制 |

通用复现仓库 HEAD 为 `79ad22c7fded963e0b8e9cc50caadb242bca8108`；P5 数值工作树 HEAD 为 `fe0b4900cec04b3d7ade7515d1d6cca1265a0cb0`。其 planar 模型和旧 SCA SLSQP 支路没有迁入本任务。原作者 QP 完整源码快照本轮本地未找到，历史 manifest 的上游 commit/tar hash 仅作为历史审计记录保留，因此不作作者源代码逐行复现声明。来源审计还记录了历史书目元数据不一致，采用原 PDF 身份，不修改历史材料。

## 3. 空间约定与 QP 数学

使用列向量主动变换 `T_WO`，固定抓取 `G_L/G_R` 为原 G1 nominal SE(3)（含 ±61 mm 偏置和镜像抓取旋转）。输入 twist 是**用 world 分量表达、线速度原点位于 Cube 中心**的 `[v_O, omega_O]`，单位 m/s、rad/s。它不是世界原点的 spatial screw 线速度项。

令 `r_i=R_WO*t_O_Gi`，原 FR3 几何 Jacobian 输出世界 TCP 原点速度：

```text
V_TCP_i = H_i V_O
H_i = [ I  -[r_i]x ]       v_TCP_i = v_O + omega_O × r_i
      [ 0      I   ]       omega_TCP_i = omega_O
B_i = H_i^-1 J_i
B = 0.5 [B_L, B_R]          C = [B_L, -B_R]
x = [qdot_L7; qdot_R7]
```

因此旋转共享物体时，两 TCP 的线速度一般不同。工具旋转用于完整初始 SE(3) 抓取核验及分量重表达；“同一物理点改变表达坐标系”与“从 Cube 中心移到 TCP 原点”是不同操作。测试独立验证了偏置叉乘符号和旋转对象的刚体导数。

求解单个中央 QP：

```text
min 0.5 (B x - Vref)' W (B x - Vref) + 0.5 lambda x' x
s.t. C x = 0
     lower <= x <= upper
W = diag(1,1,1,0.01,0.01,0.01), lambda=1e-6
Hessian = B' W B + lambda I14, linear term = -B' W Vref
```

硬约束是速度级双侧兼容和关节盒界；跟踪属于软目标。`lambda`、0.1 m 旋转特征长度，以及跟踪门限 `1e-4 m/s / 1e-3 rad/s` 都是**新离线工程数值参数**，未改变原 G1 的 `1e-4 m / 1e-3 rad` 位姿门限。正则化会使最优解与参考略有偏差，必须实际报告，而不能仅凭 QP 可行宣称任务完成。

位置和速度来自冻结的 `effective_joint_limits.json`，同时验证左右 URDF 中数值完全相同，重复为 14 维。位置采用一 ULP 的 `nextafter` 内侧预测界，只收紧、不扩展原界：

```text
lower = max(-vmax, (q_min_interior-q)/dt)
upper = min(+vmax, (q_max_interior-q)/dt)
optional: lower=max(lower, qdot_previous-a_example*dt)
          upper=min(upper, qdot_previous+a_example*dt)
```

精确数值和 SHA256 见 [配置来源](../../offline/task28_p5_qp_0/evidence/configuration_provenance.json)。YAML 的 `position_based_velocity_limits.deceleration_limit` 不能解释为认证的对称加速度限值。默认未启用加速度界；唯一示例使用 `0.1 rad/s²`、`ILLUSTRATIVE_OFFLINE` 标签、实际先求得的零速度有效数值回执，以及相等正合成时间间隔。缺失前命令、未接受标志、错误 dt/速度界/来源 ID 均拒绝。该回执不证明实际机器人此前的速度或几何安全。

## 4. 数值后端和独立正确性

未安装新依赖。HiGHS 先检查线性等式/盒约束可行性；SLSQP 最小化固定凸二次目标（最多 500 次迭代、`ftol=1e-12`），而不是分别求解两只手臂。若等式零空间的解析驻点已在所有盒界内，它只用作不裁剪的确定性 warm start；否则使用 HiGHS 可行点。

等式用 SVD 相对 `1e-12` 做数值行秩约简，接受时重新检查**原始完整 C**。另行构造下界乘子 `alpha>=0`、上界 `beta>=0`、自由等式乘子 y，核对：

```text
H x + f + C' y - alpha + beta ≈ 0
alpha*(x-lower) ≈ 0, beta*(upper-x) ≈ 0
dual = -0.5 (f+C'y-alpha+beta)' H^-1 (f+C'y-alpha+beta)
       + alpha'lower - beta'upper
```

原 C/盒误差不超过 `1e-8`；stationarity、互补与乘子符号门限 `1e-7`，primal-dual gap 门限 `1e-7*max(1,abs(primal objective))`，并检查有限值。保留 solver 原始 status、迭代、LP 状态及全部残差；**不是仅以 SLSQP success 判定**。严格凸问题在独立数值 KKT/gap 合格时可接受近似最优解，即使优化器 stop flag 为非成功，这一合同有独立测试。它是浮点数值核验，不是精确算术或物理证书。

输入结构/非有限值、错误 Cube/frame/抓取、初始位置限位和原 SE(3) 闭链先核验。数值后端异常、溢出、非有限候选和独立门禁失败均返回无 qdot 的 `NUMERICAL_FAILURE`，不裁剪结果或放宽约束。

新增 48 项测试中的独立参考包括：

- 生产 URDF 几何 Jacobian 对原 USD joint-local-frame FK 中央差分，双臂非对称姿态最大元素误差 **1.4930e-8**。
- 手工刚体运动的 TCP 导数误差 **6.4480e-11**；独立 USD 对 B 映射误差 **5.3745e-10**；兼容切向量对 implied-object 位姿差分误差 **2.5482e-9**。
- 固定 seed 2800 的八个四维抽象 SPD/盒/等式问题，用 `3^4` 活跃面穷举 KKT 参考核验，解差最大 **1.5876e-14**、目标值差 **5.5955e-14**、独立 Cholesky 对偶目标差 **4.4409e-16**。完整矩阵、参考解和代码已存入测试/证据，不算真实 FR3 规划成功样本。
- 近限位、空盒、等式/盒不相交、奇异/病态等式、rank 截断后原 C 拒绝、非有限及 bad-dual 故障注入。病态负例返回数值失败，不错误宣称约束不可行或输出候选。

数学原始记录见 [independent_math_evidence.json](../../offline/task28_p5_qp_0/evidence/independent_math_evidence.json)。审查者与代码作者职责分开记录，但均为同一工作环境内的数学/保存数据核验，不声称外部人员或跨平台认证。

## 5. 冻结实验与全部结果

正式配置见 [experiment_config.json](../../offline/task28_p5_qp_0/evidence/experiment_config.json)。三份 NR-0 `result.json.gz` 按 SHA256 读取，分别选每条 lift/transport/place 的 `len//2` 节点，共 9 个原有效状态；没有新运行 NR 或 IK。**保存路径只提供状态，三个 twist 是新合成数值参考，不是完整 NR 轨迹的速度回放或闭环跟踪结果。**

三个主参考为：translation `[.01,0,0,0,0,0]`、rotation `[0,0,0,0,0,.02]`、combined `[.005,-.003,.002,.01,-.005,.02]`；`dt=.01 s` 为假设的离线预测区间。每个状态分别计算三个参考，合计 27 个真实冻结模型 QP＋几何诊断。

| 主实验指标（27 例） | 实测结果 |
|---|---:|
| 非零关节速度 L2 范围 | 0.04018696–0.06367185 rad/s |
| 最大物体平移速度跟踪误差 | 3.279898733e-7 m/s |
| 最大物体角速度跟踪误差 | 1.471490784e-5 rad/s |
| 最大双侧兼容平移/旋转速度残差 | 7.334208989e-18 m/s / 2.454802281e-17 rad/s |
| 最小预测位置 / 速度限位余量 | 0.286044526 rad / 2.591364498 rad/s |
| 最大原始 C 残差 / KKT stationarity | 3.122502257e-17 / 7.753345861e-19 |
| 最大绝对 primal-dual gap | 4.743384505e-20 |
| 有效当前点 / 预测点 / B0 样本 | 27 / 27 / 81（每边 3 点） |
| 最大 sampled 闭链平移 / 旋转误差 | 5.529965422e-7 m / 1.767967582e-6 rad |
| 每状态 FCL 查询 / 原 ACM 跳过 | 414 / 40 |
| 主例数值失败 / 活跃盒界 | 0 / 0 |
| 主例 QP solve wall 最小/中位/最大 | 1.257311 / 1.275849 / 1.544152 ms |

主例的小参考全部使零空间驻点处于盒界内，SLSQP 各为 1 次迭代，**不是边界约束或复杂避碰性能测试**。活跃约束由下列控制验证。solve wall 包括线性可行性/独立 KKT 数值工作，不含全部 FCL/边诊断；批次总 wall **5.384646 s**，主进程 peak RSS **186068 KiB**。这不是 servo timing、100 Hz、实时性或硬时限承诺。

| 正式控制 | 结果与范围 |
|---|---|
| 5 m/s 平移参考 | QP 可行，7 个活跃盒界；跟踪误差 2.735933104 m/s、4.294574286 rad/s，`FEASIBLE_TRACKING_REJECTED`，不算任务成功 |
| 错 Cube/frame、缺关节、dt=0、错固定 grasp | 5 项 INPUT_REJECTED，无 qdot |
| 有限但极大的 `1e308` 参考 | 数值溢出被隔离为 NUMERICAL_FAILURE，无 qdot；故障控制不代表物理状态 |
| dt=.001/.02 s | 两项原几何门禁通过；只是合成步长敏感性，不是测得控制周期 |
| 可选 0.1 rad/s² 工程加速度 | 8 个活跃盒界、离散变化不超过该示例值；可行但跟踪拒绝，未认证为 FR3 物理限值 |
| 启用加速度但缺少上次命令 | INPUT_REJECTED |
| lambda=100 正则化权衡 | QP 可行但跟踪拒绝，不冒称任务不可达或约束不可行 |
| 原 NR-0 payload/table 负控 | 零速度 QP 可行，但当前 Cube 穿桌 20 mm，由原 G1/FCL 拒绝；不是可执行下沉路径 |
| 5 项抽象 14D 矩阵控制 | 等式/盒不相交与空盒返回 QP_INFEASIBLE；近位置边界出现活跃上界；零秩与病态等式按独立门禁处理。全部标明不代表真实 FR3 可达性 |

共 **18/18 控制符合预期**。完整输入、qdot、矩阵、目标值、活跃界、数值状态、失败原因、每个原状态/B0 回执保存在无损 [results.json.gz](../../offline/task28_p5_qp_0/evidence/experiments_v1/results.json.gz)，简表在 [summary.json](../../offline/task28_p5_qp_0/evidence/experiments_v1/summary.json)。没有删去或重复调参挑选正式批次。配置中保存了正式运行前一次显式 dual 符号核验强化的源码修订，旧配置保留；数值参数和门限未改，正式批次只执行一次。

## 6. 与 G1 几何、接触和时序的边界

`integration.py` 仅计算 `q_next=q+dt*qdot`，物体参考为 `p_next=p+dt*v`、`R_next=Exp(dt*omega)R`，然后通过原 NR-0 只读适配复用 G1-1、接触层及 B0 的完整门禁。速度兼容不能保证有限步 SE(3) 闭合，因而必须独立检查当前/下一状态和中间样本。

原双臂、侧吸工具、Cube 和桌面均在 FCL 内；没有新增 ACM、扩大支撑/吸盘例外、缩小保守几何或用最近点法向伪造梯度。QP 本身**没有碰撞避免约束**；穿桌负控中“QP 成功、原几何拒绝”的不同结论被保留。距离梯度、点 Jacobian、符号和退化边界验证仍是以后碰撞约束 QP 的必要前置工作。

所有一步输入都是合成离线参考，不是实际 Cube 观测。只作单状态 contact-mode 几何诊断，使用原 `BILATERAL_TRANSITIONING` 或严格 supported 合同；插值点不增加三帧确认数，未生成新的 R1 时序确认、attachment/release 事件或释放授权。支持 hold 单元控制只表示几何候选，`physical_support_observed=false`。

本次没有对新预测边运行 B1 连续验证器，明确记录 `NOT_CERTIFIED_FINITE_SAMPLING_ONLY`，不能将 81 点通过写为连续安全。来源 NR-0 及原 B1 的运动闭链、连续碰撞 INCONCLUSIVE 保持原样，并随引用 provenance 保存。没有对多步积分累计漂移、真实 servo 动态、物体负载或有限吸附建立结论。

## 7. 复现、测试与证据完整性

从仓库根目录运行，正式输出必须是尚不存在的新目录：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
timeout --signal=TERM --kill-after=10s 300s \
python3 -u offline/task28_p5_qp_0/run_experiments.py --output NEW_UNIQUE_DIRECTORY
```

入口核对正式配置中的六项实现源码哈希和三份 NR 引用哈希，不覆盖原 evidence。环境为 Python 3.10.12、NumPy 1.24.4、SciPy 1.8.0、FCL 0.7.0；SLSQP/HiGHS 二进制和 FCL 库哈希见 [dependency_identities.json](../../offline/task28_p5_qp_0/evidence/dependency_identities.json)。无需下载、启动或构建 ROS/Isaac；原独立 FCL helper 按原接口运行。

完整回归命令：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0:offline/task28_g1_2b_1 \
python3 -m pytest offline/task28_g1_1/tests offline/task28_g1_2a/tests \
  offline/task28_g1_2b_0/tests offline/task28_g1_2b_1/tests \
  offline/task28_p4_nr_0/tests offline/task28_p5_qp_0/tests \
  -q -s -p no:cacheprovider
```

原 466 项单独运行：**466 passed in 81.62 s**；最终：**514 passed in 83.45 s**，两者及正式批次退出码都为 **0**。开发阶段 45 项新测试的日志单独保留；最后新增显式 dual-sign 故障门禁及两个可重现独立最优值测试，总新数为 48，不把重复运行相加。

原始日志、环境变量、完整 argv、开始 UTC、实际退出码见各 `*_record.json`。最终 [artifact_manifest.json](../../offline/task28_p5_qp_0/evidence/artifact_manifest.json)记录全部新增交付的字节/SHA256；[protected_baseline.json](../../offline/task28_p5_qp_0/evidence/protected_baseline.json)保护起点全部 425 个跟踪文件。保存结果的另一路只读复算见 `saved_results_review.json`；它不再次求解 QP 或冒称外部认证。正式配置中的测试哈希保留登记时版本；后来扩展的独立最优值测试由最终回归日志和交付清单绑定，没有回写正式登记。

## 8. 剩余科学缺口与停止点

本任务只建立中央速度 QP 的离线数学、约束及单步诊断基线。后续仍需分别授权并验证：

1. 碰撞约束所需可信距离梯度、点 Jacobian 和负控；当前后验 FCL 诊断不能当成已实现的在线避碰。
2. 认证的加速度/实际速度与前命令时间合同、关节动态和真实 servo 特性；没有把原配置 deceleration 或 nominal 10 ms 当物理数据。
3. 当前 position-only bridge 的独立速度模式或位置积分适配器、同步、watchdog 和暂停/恢复；本任务没有此执行接口。
4. 多步跟踪、累计闭链漂移、连续闭链/碰撞证书、实际观测时序、有限侧吸载荷与完整 A+B 工艺；均不由本次 PASS 推出。

**到 G2-P5-QP-0 停止。NR-1 继续延期；不自动开展碰撞梯度扩展、在线 QP、G3、Isaac、ROS、物理实验或 RL。**

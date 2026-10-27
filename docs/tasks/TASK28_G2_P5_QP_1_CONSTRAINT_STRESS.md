# Task28 G2-P5-QP-1｜离线约束压力与多步双臂协调验证

仓库：`dreamerToTOE/dual-arm-embodied-palletizing`；分支：`side-suction-palletizing`。
起点：`72afea314a62cb0cf622292147e05cacc556dc85`。

## 1. 结论、保护范围与权限

**PASS_G2_P5_QP_1_OFFLINE**。全部预注册离线要求已满足：原有效 FR3 状态上真实触发冻结速度界；等式兼容无界参考确实越限；固定非零参考进行了重复积分；QP 跟踪合格但有限步几何失败、累计闭链漂移和载荷穿桌均被原门禁阻止。**这不是“全部轨迹成功”或完整 G2 成功。**

完整回归 **529 项通过（原 514＋新增 15）**，退出码 0。唯一正式批次退出码 0；进程正常结束仅表示矩阵已完成，验收另由全部登记条件与回归共同决定。未按结果补选方向、状态、步长或扩大预算。

本地与远端起点一致，在新隔离副本 `work/p5-qp-1` 工作；原 `work/p5-qp-0` 保持原分支与文件。起点全部 **461 个跟踪文件**有逐文件字节/SHA256 保护：冻结 fixture、机器人、保守碰撞形状、SRDF/ACM、G1 验证器、QP-0、NR-0 证据及 Task24/26/27 均未修改。已阅读 QP-0 报告/实现/验收、G0-A §6、G1/NR 规格及 [NR-1 延期决定](TASK28_RESEARCH_DECISION_2026_10_26_DEFER_NR1_START_G2.md)，延期继续有效。

所有新结果均为 **`execution_authorized=false`**。本任务只有 CPU 数值测试和解析；正式 QP 批次没有 NR/IK、重投影、状态纠偏或新路径搜索。既有回归按原测试执行，没有开展 NR-1。没有 Isaac、ROS、机器人、吸附、力控制、SVM、PCS/RSS 或 RL；position-only 命令链未修改，**直接速度 QP 执行接口仍为 BLOCKED**。

## 2. 正式运行前冻结的设计

[preregistration.json](../../offline/task28_p5_qp_1/evidence/preregistration.json)在正式实验前冻结所有完整 q14/SE(3)、来源哈希、请求、门限和预算；[run_registration.json](../../offline/task28_p5_qp_1/evidence/run_registration.json)进一步绑定 18 项源码/配置。原始 q 和旋转逐元素核对保存 NR-0 节点，不将 Cube 旋转置零，不人为制造近限位闭链构型。

| NR-0 seed | lift 中点索引/节点数 | transport 中点索引/节点数 | place 中点索引/节点数 |
|---|---:|---:|---:|
| 28 | 32/65 | 40/81 | 32/65 |
| 29 | 8/17 | 52/105 | 8/17 |
| 30 | 16/33 | 44/89 | 16/33 |

这些 seed 是**保存 NR 路径的来源身份**，QP 本身不增加随机采样。令合成 world-at-Cube-center twist：

```text
small = [.005,-.003,.002,.01,-.005,.02]  # m/s, rad/s
translation = [.01,0,0,0,0,0]
high = [5,0,0,0,0,0]
down = [0,0,-.05,0,0,0]
```

| 登记类别 | 固定输入/预算 | 数量 |
|---|---|---:|
| 小参考正控 | 9 个中点，small，dt=.01 s | 9 对 |
| 速度界压力 | 同 9 点，high，dt=.01 s | 9 对 |
| 大步闭链压力 | 3 个 transport 中点，small，dt=1 s | 3 对 |
| 大步下沉压力 | 3 个 place 中点，down，dt=1 s | 3 对 |
| 无效请求 | 错 Cube/frame、缺 q、twist 长度错误、dt=0、不支持 independent_arms | 6 个 |
| 多步运输 | 每 seed：translation/.01/50 步；small/.01/50 步；small/.05/100 步 | 9 条 |
| 多步下沉 | 每 seed：down/.02/40 步上限 | 3 条 |

24 个配对需求分别运行 bounded / unbounded_reference，共 48 份一步结果；另有 6 个无效请求、12 条 bounded 轨迹，总共 **66 份完整 case**。多步最多 720 次迭代；正式批次 OS wall 上限 900 s、SLSQP 最多 500 次迭代、HiGHS 10 s，沿用原采样上限和分辨率。

原跟踪门限 `1e-4 m/s / 1e-3 rad/s`、SE(3) 闭链 `1e-4 m / 1e-3 rad`、跨格式 FK `1e-6 m/rad`、原 C/盒数值门限 `1e-8`、KKT `1e-7` 均未改变。默认对称加速度界继续关闭，没有把 YAML deceleration 或 Δqdot/dt 解释为认证的物理加速度。

PASS 要求所有 9 点初态及小参考正控通过，至少一个真实活跃冻结界与配对无界越限、可行但跟踪拒绝案例、至少两步非零连续接受，以及**实际失败步当前几何有效、QP 跟踪/硬门合格而预测几何拒绝**。缺失必需类别则 PARTIAL；位置 lookahead 活跃是探索统计，未触发则显式记 NOT_TRIGGERED，不事后修改 q 或 dt。

## 3. 递推、数学比较和门禁

沿用 QP-0 的主动列向量 `T_WO` 和完整固定抓取；物体 twist 用 world 分量表达，线速度原点在 Cube 中心。工具偏置旋转项仍由 `H_i=[[I,-[R_WO t_O_Gi]x],[0,I]]` 处理，`B=.5[B_L,B_R]`、`C=[B_L,-B_R]`。相同物体角速度不意味着两 TCP 线速度相等。

原 bounded QP 未改：

```text
min 0.5 (B x - V)' W (B x - V) + 0.5 lambda x'x
s.t. C x = 0, lower <= x <= upper
W=diag(1,1,1,.01,.01,.01), lambda=1e-6
```

盒界仍为冻结位置 lookahead 与速度界交集，含 QP-0 的一 ULP 内侧位置界。新 driver 独立重算 Cx、Bx−V 和冻结盒界；保留原数值 KKT 回执。活跃界沿用原 **1e-7 rad/s 的报告判据**，按 velocity / position lookahead / 两者重合分类；该报告容差不扩展硬界。

无界数学参考保持同一 q/X/V/dt/W/lambda 和完整 `Cx=0`，仅去掉盒界。用等式零空间 N 的 Cholesky 解：

```text
x_free = N (N' H N)^-1 (-N' f)
H=B'WB+lambda I; f=-B'WV
```

另查原 C 残差和 stationarity，再接受相同冻结界及 G1/B0 诊断。新增测试用**独立完整 saddle KKT 方程**核验，与零空间实现最大解差 `5.3783e-10`；随机可行方向的目标增量为正。无界参考不是“安全控制器”；数值求解缺失/失败不能被算成真实越限。比较限于相同原状态的一步需求，不把不同 rollout 分支混为同状态对照。

每步从前一**已接受** q 推进：

```text
q_(k+1) = q_k + dt*x_k
p_ref(t) = p_initial + t*v
R_ref(t) = Exp(t*[omega]x)*R_initial, t=(k+1)*dt
```

Cube 参考由初始参考和固定 desired twist 导出，不用 achieved twist 掩盖跟踪偏差，也不重解 NR/IK。记录两臂 FK 反推的 Cube SE(3)、左右相对残差及对 desired Cube 的残差；“累计漂移”指这些完整位姿误差，**不是把每步误差简单相加**。

只有 QP 数值、冻结硬界、跟踪、当前/候选原 G1 和原 B0 全部满足才接受并递推。遇到 rejected 或 inconclusive 保留完整失败候选和最后接受状态，停止该轨迹；即使跟踪先失败，仍保留该候选的独立几何诊断。外层 `STOPPED_AT_FIRST_REJECTION` 是停止标签，具体 rejected/inconclusive 仍按原 B0/数值原因保留，未把证据不足伪称物理碰撞。

全部为假定双侧附着下的 `GENERIC / BILATERAL_TRANSITIONING` 合成几何输入，无 support-site 切换、释放事件或 R1 帧。逐步时间是合成排序/积分区间，不是实际观测、100 Hz 或实测 servo timing。原 B0 仍为关节/平移线性、旋转最短 SO(3) 插值，默认 `max_dt=.05 s, Δq=.02 rad, Δp=.001 m, ΔR=.01 rad, max_samples=10001`。

## 4. 真实活跃约束与配对结果

9/9 小参考在两方法均接受一步。9/9 high 请求的 bounded QP 数值可行，分别激活 **6–8 个冻结速度界**，共 63 条活跃界记录；全部因跟踪误差拒绝，不报告成 QP_INFEASIBLE 或任务成功。

| 高需求统计（9 个真实 FR3 状态） | bounded | 等式兼容无界参考 |
|---|---:|---:|
| 平移速度跟踪误差范围 | 2.562450–3.078343 m/s | 9.7109e-5–1.63995e-4 m/s |
| 角速度跟踪误差范围 | 3.643250–4.980346 rad/s | 7.04786e-4–1.71693e-3 rad/s |
| 冻结速度界违反 | 无；最小余量 0 | 全 9 个，最大超界量各为 7.341610–11.697636 rad/s |
| 任务跟踪合格 | 0/9 | 1/9，且仍越限 |
| 离线候选可继续递推 | 0/9 | 0/9 |

软目标允许 common-object 运动偏离参考；速度闭链兼容并不强迫物体按 requested twist 运动。无界参考虽目标误差更小，也不能宣称完美跟踪，8 个仍超原跟踪门限，且全都违反冻结速度界。

全部 696 次 bounded 数值求解：687 次 `FEASIBLE_TRACKING_OK`、9 次 `FEASIBLE_TRACKING_REJECTED`，没有数值失败或 QP 不可行。硬界最小位置余量 **0.220657650 rad**、速度余量 **0**；原 C 最大残差 **4.884981e-15**、KKT stationarity **3.789017e-9**、绝对 primal-dual gap **7.551293e-12**，均在原数值门内。SLSQP 为 1–19 次迭代，原 success flag 全为成功；仍保留独立核验，不只信该 flag。

**位置 lookahead 活跃：NOT_TRIGGERED**，统计包含全部 bounded 单步与 rollout。它不是位置约束已在真实近限位闭链运动中完成压力验证；未为触发它改造 q、场景或步长。加速度活跃性也不作认证结论。

3 个 dt=1 s 的 small 当前状态均有效，QP 跟踪/硬界也合格，但下一状态最大平移闭链误差分别为 **0.307943 / 0.332959 / 0.317886 mm**，被原 `0.1 mm` 闭链门拒绝。这是真实冻结模型的“QP 速度级通过，有限步失败”，不是无效起点或抽象碰撞替身。另 3 个大步下沉也被原闭链/FCL 门拒绝；多种原因原样保存，不声称都是单一碰撞失败。

## 5. 多步漂移、碰撞与停止证据

下表的误差取 left/right、left/object、right/object 三组完整 SE(3) 的最大值。失败步不计入接受前缀；所有逐步原数组、矩阵、限位、速度变化与接触 pair 均保留。

| seed / 请求 | dt / 步数上限 | 接受/尝试 | 最后接受点最大平移 / 旋转漂移 | 终态 |
|---|---|---:|---:|---|
| 28 translation | .01 / 50 | 50/50 | .001382 mm / .001545 mrad | 完成预算 |
| 28 combined fine | .01 / 50 | 50/50 | .001439 mm / .006619 mrad | 完成预算 |
| 28 combined coarse | .05 / 100 | 100/100 | .075715 mm / .128238 mrad | 完成预算 |
| 28 down | .02 / 40 | 30/31 | .098992 mm / .084986 mrad | 闭链失败，停止 |
| 29 translation | .01 / 50 | 50/50 | .001262 mm / .001402 mrad | 完成预算 |
| 29 combined fine | .01 / 50 | 50/50 | .001741 mm / .008211 mrad | 完成预算 |
| 29 combined coarse | .05 / 100 | 100/100 | .082622 mm / .149065 mrad | 完成预算 |
| 29 down | .02 / 40 | 13/14 | .046243 mm / .025844 mrad | Cube/table，停止 |
| 30 translation | .01 / 50 | 50/50 | .001210 mm / .001074 mrad | 完成预算 |
| 30 combined fine | .01 / 50 | 50/50 | .001582 mm / .007622 mrad | 完成预算 |
| 30 combined coarse | .05 / 100 | 100/100 | .078185 mm / .135536 mrad | 完成预算 |
| 30 down | .02 / 40 | 26/27 | .090525 mm / .052080 mrad | Cube/table，停止 |

共 **672 次多步尝试、669 步接受**。9 条运输序列完成预定上限；3 条下沉序列按设计遇到第一项失败停止，不能以 9/12 作为机器人成功率。较粗运输的数值参考区间为 5 s，不是经过实测的 5 s 搬运。初始漂移和最后接受/失败漂移均见 [quantitative_summary.json](../../offline/task28_p5_qp_1/evidence/quantitative_summary.json)；例如 seed29 combined 的初始最大平移残差 `9.67565e-8 m`，5 s 粗步后达 `8.26221e-5 m`，表明速度兼容没有消除有限步累计误差。

- seed28 下沉：已接受至合成 t=.60 s；t=.62 s 候选的左右物体平移失配 **0.102322147 mm**，超过原 0.1 mm 门。其桌面间隙仍 **38.6284 mm**、碰撞原因空，明确为累计闭链漂移见证，而非穿桌替代证据。
- seed29 下沉：t=.28 s 首拒，Cube/table 间隙 **−0.496712 mm**；闭链最大平移误差仅 **0.0498483 mm**，仍在闭链门内。
- seed30 下沉：t=.54 s 首拒，间隙 **−0.494084 mm**；最大平移误差 **0.0940814 mm**，仍在门内。两者均由原 FCL 实际记录 `Cube_01/box|task28_table/box`，同时保留严格几何 penetration 原因，没有授予支撑例外。

这三项失败发生时，当前状态、QP 跟踪和冻结硬界均合格。失败候选全部保留，递推状态仍是前一接受点；未继续积分或投影回流形。原双臂、工具、Cube、桌面持续参加几何检查，每个完整当前/预测状态仍查询 **414 个 shape pair，原 ACM 跳过 40 对**。

所有 bounded 接受点（669 rollout＋9 小参考单步）共 678：最大平移/角速度跟踪误差 **1.932237e-6 m/s / 1.968900e-5 rad/s**，最小位置/速度余量 **0.268267364 rad / 2.503963250 rad/s**。这些数字排除了失败候选，全部尝试（含失败）的统计另存，未删去困难结果。

第一次 Δqdot 记为 None，之后只与前一已接受数值速度相减。最大数值 Δqdot/dt 可查逐步记录；它不是实测 qddot，未认证 FR3 加速度、力矩或负载约束。

## 6. 回归、复现与证据完整性

新测试 15 项：driver 9 项（真实原状态递推/完整载荷、失败/未决停止、伪造可行回执不能绕过独立门、无效/不支持请求）；独立无界数学参考 3 项；runner 统计/配对/活跃来源 3 项。抽象回执注入只证明软件停止合同，不充当真实碰撞或可达性证据。

开发日志 [focused_tests.txt.gz](../../offline/task28_p5_qp_1/evidence/focused_tests.txt.gz)保留了 14 passed / 1 failed：新增汇总字段后，抽象测试桩缺少完整 bound/q 字段；仅补全测试桩，未改生产门限、实验参数或原测试。登记时原日志路径为 `focused_tests.txt`，最终按原字节 gzip 归档，原始/压缩哈希见 `raw_log_archive.json`。最终完整回归 **529 passed in 90.26 s**，exit 0。

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0:offline/task28_g1_2b_1 \
python3 -m pytest offline/task28_g1_1/tests offline/task28_g1_2a/tests \
  offline/task28_g1_2b_0/tests offline/task28_g1_2b_1/tests \
  offline/task28_p4_nr_0/tests offline/task28_p5_qp_0/tests \
  offline/task28_p5_qp_1/tests -q -s -p no:cacheprovider
```

复现正式矩阵，从根目录指定尚不存在的输出目录：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
timeout --signal=TERM --kill-after=10s 900s \
python3 -u offline/task28_p5_qp_1/run_experiments.py --output NEW_UNIQUE_DIRECTORY
```

运行前重查 18 个登记绑定、起点 461 文件、三份 NR 源哈希和 9 个节点内容；不覆盖原 evidence。正式原始 [summary.json](../../offline/task28_p5_qp_1/evidence/experiments_v1/summary.json)和 66 个 `case_*.json.gz` 含成功及失败完整记录，逐件 SHA/字节见其 artifacts。总计 720 个候选尝试（696 bounded、24 数学参考），原 B0 共评估 3456 个采样点；6 个无效请求在迭代前拒绝。原始 summary 保持 `...PENDING_REGRESSION` 的运行时状态，最终 [acceptance.json](../../offline/task28_p5_qp_1/evidence/acceptance.json)合并已完成回归与证据核验，不回写正式结果。

环境：Python 3.10.12、NumPy 1.24.4、SciPy 1.8.0、pytest 6.2.5，FCL 原系统库 0.7.0；没有安装依赖。版本及 SLSQP/HiGHS/FCL 二进制哈希见 `dependency_identities.json`。本次 CPU 离线批次 wall **122.230737 s**，主进程 peak RSS **335304 KiB**。696 次 bounded QP solve wall 最小/中位/最大 **1.226338 / 1.297015 / 6.066574 ms**；它不含完整几何/序列工作，不能当作实时或 worst-case servo 保证。

完整 argv、环境、开始 UTC、退出码在 `*_record.json`。正式运行前只读设计审查见 `preformal_design_review.json`；保存数据的独立积分/数值/前缀核验见 [saved_evidence_audit.json](../../offline/task28_p5_qp_1/evidence/saved_evidence_audit.json)，不重跑求解或实验。保存数据审计进行了 **168238 项检查、0 错误**，关节递推复算误差为 0，独立 Rodrigues 参考姿态最大元素差 `2.22e-16`。审计实际命令/exit 0 另存 `saved_evidence_audit_record.json`；精确开始时间未捕获，明确为 UNKNOWN，不将报告生成时间冒充开始时间。

只复算已有数据的审计命令（输出为新的审计 JSON）：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 offline/task28_p5_qp_1/audit_saved_evidence.py \
  --output offline/task28_p5_qp_1/evidence/experiments_v1 --audit NEW_AUDIT_JSON
```

[artifact_manifest.json](../../offline/task28_p5_qp_1/evidence/artifact_manifest.json)封存所有新增源码/测试/报告/证据；`protected_baseline.json` 与最终核验保护旧文件。审阅均在同一环境内，不声称外部或跨平台认证。

## 7. 科学边界、建议与停止点

QP 仍没有可信碰撞梯度或碰撞避免不等式，FCL 是独立后验诊断；本次穿桌负例直接说明只靠 QP 关节约束不能完成避碰。保守几何、ACM、吸附阈值与支撑微米级容差均保持原样。

`COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED` 只表示预算内有限采样合格。新边没有运行或实现 B1 连续证明；全部仍为 `NOT_CERTIFIED_FINITE_SAMPLING_ONLY`，原 NR/B1 的连续闭链/碰撞未决保持不变。没有 R1 新时序观测、实测 Cube 反馈、实际支撑确认、真实控制周期、动力学、有限吸附负载或物理执行成功。

当前结论支持：这些有效冻结状态上，速度界会改变最优跟踪，有限 Euler 积分会产生可测闭链漂移，原几何门禁能停止明确的漂移及载荷碰撞。它不能支持所有方向/近限位状态、长期稳定性、步长收敛率、一般化成功率或连续安全；本次两种步长的运行时长也不同，不冒称严格同一终点的积分阶数对照。

建议先审阅此次失败前缀与误差预算，再单独授权下一项离线闭链误差控制/长期跟踪研究。真实近位置界、可信碰撞梯度、认证动态限值及执行适配器各需独立任务和验证，不能在本次报告后自动补入。

**到 G2-P5-QP-1 停止，等待审阅。不自动开展碰撞梯度 QP、速度桥接、Isaac/ROS、G3、NR-1、PCS/RSS、SVM 或 RL。**

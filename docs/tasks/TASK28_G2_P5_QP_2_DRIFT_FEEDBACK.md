# Task28 G2-P5-QP-2｜离线闭链漂移反馈

仓库：`dreamerToTOE/dual-arm-embodied-palletizing`；分支：`side-suction-palletizing`。
起点：`0a8fc0369b72998c2f51b2a637c405b1e3ac33bd`。
结论：**`PASS_G2_P5_QP_2_OFFLINE`**。

本结论只表示冻结 FR3 模型上的 CPU 离线数学、数值和有限采样门禁通过。全部证据的 `execution_authorized` 为 `false`。没有启动 Isaac Sim、ROS、机器人、吸附或物理实验，也没有实现碰撞梯度、加速度认证、NR-1、QP 在线桥接、SVM、PCS/RSS 或 RL。

## 1. 任务边界与保护

QP-0、QP-1、G1-1、G1-2A-R1、G1-2B-0、NR-0、冻结 fixture、SRDF/ACM、保守碰撞几何和 Task24/26/27 的提交文件均保持不变。`offline/task28_p5_qp_2/evidence/protected_baseline.json` 对起点 **558 个文件**逐文件保存 SHA-256；最终核验全部匹配。`baseline_verification.json` 保存了本地 HEAD、分支和起点保护结果。

QP-2 的代码只位于 `offline/task28_p5_qp_2/`，包括：

- `feedback.py`：SE(3) 误差项、世界坐标旋转对数和反馈 QP；
- `affine_solver.py`：带完整仿射等式和盒界的独立凸 QP 求解器；
- `driver.py`：与 QP-1 相同的 Euler 数值积分、G1 状态门、B0 有限采样门和失败前缀保留；
- `run_experiments.py`：读取登记的 NR-0 节点，执行正式矩阵并保存每条完整轨迹；
- `audit_saved_evidence.py`：只读检查 32 个 gzip 轨迹、摘要、哈希、执行标志和负控；不重跑规划。

QP-1 的 `C qdot=0` 和 QP-1 实现没有被改写。反馈控制器自身使用新的 `A qdot=b`，并且把旧 `C qdot` 作为独立比较诊断，而不是错误地声称反馈轨迹仍满足旧齐次等式。

## 2. 漂移机制和 SE(3) 反馈

对每个机械臂，使用固定抓取变换 `G_i` 和冻结 FK：

```text
X_i(q_i) = T_WG_i(q_i) G_i^{-1}
```

`X_i` 是由该臂当前关节状态反推的 Cube 中心位姿，所有线速度均在 world 中、原点位于 Cube 中心。由 TCP Jacobian 和抓取偏置得到独立的 implied-object 中心映射 `B_i`，令 `x=[qdot_L;qdot_R]`。

相对位姿误差为：

```text
e_p = p_L - p_R
A_R = R_L R_R^T
phi = Log(A_R)                 # world principal SO(3) log
```

角速度相对项采用 world 表达的

```text
w_L - A_R w_R
```

因此新反馈仿射项为

```text
A = [ B_L^v        -B_R^v
      B_L^w        -A_R B_R^w ]

b = [ -k_p e_p
      -k_R phi ]
```

`A x=b` 是对当前相对位姿误差的速度级一阶稳定化约束。`Log` 在固定主值域内使用，距 `pi` 小于 `1e-5 rad` 时拒绝；没有把主值对数误差当成全局无奇异参数化。对每个臂的 object-reference 误差，代码另外构造平移和旋转修正后的 mean object-twist 目标。

反馈 QP 保留原 QP-0 的正定目标、冻结位置 lookahead 与速度盒界、原跟踪门 `1e-4 m/s` 和 `1e-3 rad/s`，只把共享闭链约束换成上述仿射等式。若等式、盒界或独立门不满足，返回失败，不裁剪 qdot，不做 NR 重投影，不隐藏 QP 不可行。

有限步漂移仍有多个来源：

1. 速度级等式只约束当前微分关系；Euler 积分在位姿非线性上有截断误差。
2. 两臂 FK/Jacobian 随 q 变化，下一步的 `B_i` 与本步不同。
3. 反馈 QP 的软 object-reference 目标、正则化和盒界会使实际平均物体速度偏离期望。
4. 求解器浮点残差和独立数值门也会贡献微小误差。

报告中的 relative drift 是两臂 implied Cube 位姿之间的完整 SE(3) 残差；mean/reference drift 是两臂平均 implied pose 与合成 reference pose 的残差。两者分别记录，未把所有漂移归结为单一原因。

## 3. 预注册设计

配置在正式运行前冻结于 [`preregistration.json`](../../offline/task28_p5_qp_2/evidence/preregistration.json)，原始版本保存为 `preregistration_initial.json`。正式运行前绑定的源码和配置哈希见 [`run_registration.json`](../../offline/task28_p5_qp_2/evidence/run_registration.json)。参数固定为：

- 反馈增益：相对平移 `2.0 s^-1`、相对旋转 `2.0 s^-1`、object-reference 平移 `0.5 s^-1`、旋转 `0.5 s^-1`；
- 主值对数排除区 `1e-5 rad`；
- 3 个保存的 NR-0 transport 中点：seed 28/node 40、seed 29/node 52、seed 30/node 44；
- 每个节点的 `primary_combined=[.005,-.003,.002,.01,-.005,.02]`，匹配 `T=2.0 s` 的 `dt=.02`（100 步）和 `dt=.05`（40 步）；
- 每个节点另有 `supplement_translation=[.01,0,0,0,0,0]`；3 个保存的 place 中点运行原 QP-1 downward negative/stop 控制；
- 另存原 NR-0 Cube/table 穿透负控，两个方法均只检查初态，不继续积分；
- 同一保存的 q14、Cube SE(3)、参考 twist、joint bounds、积分法、G1/B0 几何门用于 baseline 和 feedback；
- 预注册判据：每个 dt 的 3 个可比较 pair 中至少 2 个同时把 relative translation 和 rotation 降至 baseline 的 `0.8` 以下，并保留所有失败结果。

正式设计的预注册登记总计 15 个场景、32 条轨迹（15 场景中含两方法 pair；另外包含 payload/table 负控），最多尝试 1922 个步。正式运行没有在查看结果后调参、增删方向或替换状态。

## 4. 正式结果

正式输出位于 [`evidence/experiments_v1/`](../../offline/task28_p5_qp_2/evidence/experiments_v1/)，每条 `case_000.json.gz` 至 `case_031.json.gz` 均保存 q、Cube pose、每一步 qdot/q_next、QP 状态、独立仿射残差、joint margins、FCL/B0 结果、漂移指标和失败候选。摘要见 `summary.json`；便于复核的压缩汇总见 [`quantitative_summary.json`](../../offline/task28_p5_qp_2/evidence/quantitative_summary.json)。

| dt | pair 数 | 可比较数 | 同时改善数 | translation ratio 范围 | rotation ratio 范围 |
|---:|---:|---:|---:|---:|---:|
| 0.02 s | 3 | 3 | 3 | 0.245699–0.249926 | 0.248833–0.286084 |
| 0.05 s | 3 | 3 | 3 | 0.246247–0.247731 | 0.249178–0.262808 |

6 个 primary pair 的反馈/基线最终 relative drift 如下；单位分别为 m 和 rad：

| 来源节点 | dt | baseline 平移 | feedback 平移 | baseline 旋转 | feedback 旋转 |
|---|---:|---:|---:|---:|---:|
| seed28 transport40 | .02 | 1.202916e-5 | 3.006399e-6 | 7.505359e-6 | 2.147164e-6 |
| seed28 transport40 | .05 | 3.042109e-5 | 7.536253e-6 | 2.060048e-5 | 5.413980e-6 |
| seed29 transport52 | .02 | 1.329589e-5 | 3.266780e-6 | 1.291015e-5 | 3.212468e-6 |
| seed29 transport52 | .05 | 3.322550e-5 | 8.181689e-6 | 3.230282e-5 | 8.049156e-6 |
| seed30 transport44 | .02 | 1.258723e-5 | 3.103908e-6 | 1.018347e-5 | 2.544999e-6 |
| seed30 transport44 | .05 | 3.156712e-5 | 7.775583e-6 | 2.554281e-5 | 6.378087e-6 |

正式批次的总计为 1833 次尝试、1828 个接受步、5499 个 B0 sampled states，CPU wall `248.464357423 s`，主进程峰值 RSS `685508 KiB`。所有 primary pair 都到达同一预注册终点；反馈在两个 dt 下都是 3/3 同时改善，满足预注册 PASS 判据。

反馈轨迹的独立硬仿射等式最大残差为 `2.966377e-16`（逐步记录的正式门为 `1e-8`）；QP solver 的 KKT、primal-dual、位置/速度 margin 和原 G1/B0 结果均随每一步保存。反馈的旧 `C qdot` 仅作比较，某些步骤达到 `1.205736e-4` 的旧 C 残差并不构成失败，因为该等式已由预注册设计明确替换为 `A qdot=b`；反馈自身的独立 affine residual 仍通过硬门。

### 非主实验和负控

- `supplement_translation` 的 6 个 pair 均完成各自预定步数；它们不是主要接受判据，但完整保留。
- 原 QP-1 downward 控制保留了真实几何停止。seed28 place 的 baseline 在第 30 步因 `LEFT_RIGHT_OBJECT_TRANSLATION_MISMATCH` 停止，feedback 达到预算；seed29 和 seed30 均在 Cube/table 穿透处停止，并保留 `COLLISION:Cube_01/box|task28_table/box`、`CLEARANCE_TABLE_PENETRATION` 等原因。这些是单条离线轨迹的首失败，不是“反馈已解决放置”或物理成功。
- 归档的 `NR0_saved_20mm_payload_table_penetration` 负控两种方法都在初态停止，尝试步数为 0。FCL 记录 Cube/table 穿透深度 `0.020000000000000018 m`，airborne clearance `-0.02 m`，检查了 414 个 shape pair、跳过原 ACM 的 40 对；没有添加新的允许碰撞。

负控必须区分为单状态门失败和完整 planner 失败：上述 payload negative 是初态 G1/FCL gate 失败，不能证明一个完整路径搜索器失败；downward 项是从有效前缀开始的有限步状态/边拒绝，也没有被解释为连续 planner 证明。

## 5. 独立数学和数值证据

最终回归日志的摘要记录在 [`independent_math_evidence.json`](../../offline/task28_p5_qp_2/evidence/independent_math_evidence.json)，不是只比较函数输出与自身。结果包括：

- 冻结 USD joint-local-frame FK 的实际中心映射中央差分：最大绝对误差 `1.808558858e-10`；
- 相对 SO(3) Log 导数中央差分：`3.539541993e-10`；反馈方向差分：`3.633275903e-10`；reference feedback 差分：`4.581237612e-10`；错误符号方向的最小区分量 `2.5648451266`；
- 12 个独立 SPD/仿射盒 QP 的 `3^4` active-face 穷举：最大解误差 `2.220446049e-15`，目标误差 `1.776356049e-15`，primal-dual gap `1.776356049e-15`；
- 原有 FK/Jacobian、QP-1 saddle 参考、病态等式拒绝和其他 G1/QP 证据均包含在同一回归日志哈希中。

这些是数学和离线模型证据，不是机器人可达性、动态稳定性或连续安全证书。

## 6. 测试和复现

聚焦测试命令：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0:offline/task28_g1_2b_1 \
python3 -m pytest offline/task28_p5_qp_2/tests -q -s -p no:cacheprovider
```

聚焦测试 **38 passed**。完整回归为原有 **529** 项加本任务 **38** 项，共 **567 passed**，退出码 0；完整命令还把各历史 test helper 目录加入 `PYTHONPATH`，并使用 `--import-mode=importlib` 解决历史 `test_runner.py` 同名导入，不改变测试语义。最终输出和精确命令环境保存在 [`regression_final.txt`](../../offline/task28_p5_qp_2/evidence/regression_final.txt) 与 `regression_final_record.json`。

正式运行命令：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
timeout --signal=TERM --kill-after=10s 900s \
python3 -u offline/task28_p5_qp_2/run_experiments.py \
  --output offline/task28_p5_qp_2/evidence/experiments_v1
```

保存证据只读审计命令：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 offline/task28_p5_qp_2/audit_saved_evidence.py \
  --evidence offline/task28_p5_qp_2/evidence/experiments_v1 \
  --output offline/task28_p5_qp_2/evidence/evidence_audit.json
```

`evidence_audit.json` 的 10 项检查全部通过：32/32 artifact identity、摘要状态、gzip SHA-256、执行禁用标志、有限采样边界、拒绝前缀、15 个 pair、2 个 payload negative 以及预注册要求均一致。审计不运行 QP，不写入原始 case。

正式环境为 Python `3.10.12`、Linux `6.8.0-138-generic-x86_64`、NumPy `1.24.4`；时间是 CPU 进程 wall，不是 servo timing。`time_s` 是合成数值积分时间，不是 ROS 或实际测量时间。

## 7. 机器可读证据和文件哈希

本任务的完整成功与失败记录保留在 `experiments_v1/case_*.json.gz`，摘要和聚合分别为：

- `evidence/experiments_v1/summary.json`、`registered_config.json`；
- `evidence/quantitative_summary.json`；
- `evidence/independent_math_evidence.json`；
- `evidence/evidence_audit.json`；
- `evidence/acceptance.json`；
- `evidence/baseline_verification.json`；
- `evidence/preregistration.json`、`preregistration_initial.json`、`run_registration.json`；
- `evidence/protected_baseline.json`、`regression_final.txt`、`regression_final_record.json`。

交付后的字节和 SHA-256 见 `evidence/artifact_manifest.json`。manifest 不包含自身，避免自引用；它包含本报告、实现、测试和全部压缩轨迹文件。

## 8. 科学限制和停止点

1. 反馈改善只在 3 个保存的 NR-0 transport 节点、两个匹配总时长的 dt 和登记的合成 twist 上观察到。它不是一般化成功率、长期稳定性或步长收敛阶证明。
2. 所有输入是保存状态加合成数值积分；没有真实 Cube 观测、ROS 时间同步、传感器、接触力或吸附力数据。
3. 所有边都是有限采样：`continuous_safety=NOT_CERTIFIED_FINITE_SAMPLING_ONLY`。没有实现 swept/CCD 连续证明。
4. QP 没有碰撞梯度不等式。FCL/B0 是逐状态和采样边后验门，不能冒称在线避碰控制器。
5. 数值差分的 `qdot` 变化不是认证的物理 `qddot`；没有动力学、力矩、速度桥接或 watchdog 结论。
6. 反馈控制没有新增 phase observation、attachment/release 授权或支撑例外；失败步停止并保留，不投影回可行流形。

到 G2-P5-QP-2 停止。G1-3、碰撞梯度 QP、执行桥、NR-1、Isaac/ROS、物理实验、PCS/RSS、QP 继续扩展和 RL 均需要单独授权。

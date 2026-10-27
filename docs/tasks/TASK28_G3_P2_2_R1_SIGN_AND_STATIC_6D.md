# Task28 G3-P2-2-R1：关节 effort 符号校准与六轴静态验证

**结论：`G3_P2_2_R1_STATIC_6D_PASS`，仅限本次隔离的单 FR3、静态、Isaac Sim 4.5 条件。** 独立世界轴力臂对照支持该条件下的 measured effort 差分与作用于机器人外力矩反号；两个构型上 20 个预注册工况（含零载、六个独立正轴和选定反向控制）通过固定误差门。原 G3-P2-2 的 `G3_P2_2_PARTIAL` 结论、原始数据、门限和报告保持原状，本次结果不追溯改写。

本结果不证明硬件关节扭矩与 PhysX solver effort 等价，不证明在线力观测、力控制或双臂吸附控制可执行，不授权 ROS、生产桥、真实机器人或吸盘执行。

## 1. 研究范围和记录

- 起始分支 `side-suction-palletizing`、起始提交 `a75abde2e0a481b67827b6ee746f131f7d82d95e`。仅使用独立单臂场景和冻结 FR3 USD；未更改冻结 fixture、历史数据、ACM 或生产路径。
- 冻结 USD SHA-256 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`，来源见 fixture manifest；用于本次实验的可哈希本地文件见 `independent_review.json`。未安装额外腕部 F/T 传感器，没有 D6 反力观测管线。
- 五个范围字段始终为 `execution_authorized=false`、`physical_robot_tested=false`、`wrist_ft_sensor_required=false`、`dual_arm_force_control_implemented=false`、`production_bridge_modified=false`。
- Isaac Sim 本地 4.5.0-rc.36，CPU 物理后端，独立 headless；未启动 ROS。测得的仿真时间是观察样本时间，不是实物反馈时间或真实伺服频率。

## 2. 物理符号与独立对照

固定 q 时令外载旋量 **作用于机器人**、以世界轴表达、参考点为冻结左侧虚拟 TCP，力在前三、力矩在后三，单位分别为 N 和 N·m。对 7 关节，虚功得到 `tau_external = J_TCP(q)^T wrench_on_robot`。静止平衡条件为 `tau_actuator + tau_gravity + tau_external ≈ 0`；两次近同姿态配对，静态重力及其他未建模项近似相减，故 `delta_tau_actuator ≈ -J^T wrench_on_robot`。该推导是近似，姿态差、漂移、惯量警告和接触均可能破坏抵消。

本地 Isaac `single_articulation.py` 描述 `get_measured_joint_efforts` 为 solver 在 DOF 运动方向的关节力投影；`articulation.py` 调用 `get_dof_projected_joint_forces()`。这两处并未充分说明作用/反作用，不能仅凭旧结果选符号。独立控制不调用 G1 Jacobian 或 wrench 估计器：对 world +Z 的第一关节，直接用逐帧施力点 `p` 和已知力 `F` 计算 `((p-o)×F)·[0,0,1]`，其中 o 为该轴上的基座点。加载位移与外力矩同号，measured effort 增量与外力矩反号才通过；±Fx、±Fy 均检查且不能只靠单一极性。

固定 USD 中 `fr3_joint1` 的局部 X 轴经 localRot0 变换为父体 +Z，轴点比基座高 0.333 m，沿同一直线选基座点不改变投影力矩。USD root 和 link0 的资产坐标变换为单位阵，实例化只显式给 position、orientation 默认透传；本次与原 P2-2 的 DOF 名称与索引、运行时 position-drive 增益、重力、物理步长和 articulation solver 参数逐项一致；运行时未直接记录 base quaternion，因此世界轴依据是资产与构造源码的组合，另以 link7 的模型对比误差作交叉检查，不能声称直接测量了基座姿态。相关源和哈希见 `independent_review.json`。

以下力矩都按作用于机器人定义，位移是加载均值减空载均值。平衡残差为 `|Δeffort1 + tau_external1|`：

| 负载 | 外力矩 N·m | Δeffort1 N·m | Δq1 rad | 平衡残差 N·m |
| --- | ---: | ---: | ---: | ---: |
| positive_fx | -0.10963 | 0.109581 | -1.1535e-05 | 4.93829e-05 |
| negative_fx | 0.109636 | -0.109586 | 1.15363e-05 | 4.97891e-05 |
| positive_fy | 0.416558 | -0.416511 | 4.26353e-05 | 4.71138e-05 |
| negative_fy | -0.416566 | 0.416519 | -4.26456e-05 | 4.67406e-05 |

使用相同源码、数据和 sign preregistration 事后只读重算，结果与 `sign_result_v2.json` **逐字段相等**。此证据建立的是相同 Isaac 接口、负载路径和位置保持试验条件下的 actuator-like 差分语义，不是对所有控制模式或所有硬件扭矩传感器的普遍结论。

## 3. 显式转换与验证设置

新 `offline/task28_p2_2_r1/adapter.py` 明确要求相同数据源、独立符号结果及 provenance，先计算成对稳定窗口均值差 `Δtau_measured`，再单独标记并执行 `tau_external_on_robot = -Δtau_measured`；之后才交给原 P2-2 SVD 估计器。估计器仅接收 q、七维外部广义力矩、世界系 TCP Jacobian、关节顺序、时戳与 provenance，已施加的 oracle 不进入估计器。估计输出包含秩、奇异值、重建残差、可观测投影及最小范数 wrench。顺序是 `fr3_joint1` 至 `fr3_joint7`，通过 USD 运行时全部 DOF 名称和索引核对，另两个 finger DOF 不混入。

已知的测试外载只在估计结束后用于独立比较：TCP 点的外力和世界系纯力偶相加，力矩绕 TCP；探针对 link7 以 `RigidPrim.apply_forces_and_torques_at_pos(..., is_global=True)` 应用。纯力偶对参考点平移不变；force-plus-offset 时计算 `r×F`。记录每帧实际 API 输入的世界力、世界力矩、作用点、姿态及时间；六轴评估核对预注册量和施加点，检查无期望的偏置。

采集矩阵在运行前冻结于 `sign_preregistration.json` 和 `static_preregistration.json`：同一 USD、重力 `[0,0,-9.81] m/s²`、`dt=1/120 s`、原 USD position-drive 与 solver 设置，不修改资产。sign 控制为 home 构型下 ±Fx/±Fy；主矩阵为 home、offset 各 10 个载荷，六个正轴分别为 1 N 或 0.3 N·m，加零载与 −Fx、−Fz、−Mz。纯矩 0.3 N·m 是固定力矩门限 0.08 N·m 的 3.75 倍，为隔离仿真研究值而非硬件安全限值。

每个载荷为 unloaded/loaded 各 reset 一次，每条件 600 个 settle step、末尾连续 60 帧；分别运行 4800 和 24000 physics steps，前者约 38.38 s、后者约 100.45 s（墙钟值仅供环境参考）。沿用的预注册门包括速度 `0.01 rad/s`、配对 q 差 `0.002 rad`、单关节力矩绝对误差 `0.10 N·m`、力向量误差 `0.30 N`、力矩向量误差 `0.08 N·m`、最小奇异值 `0.05`，完整列表见注册文件。不同 reset 的时钟不被伪装成同步实物反馈。

## 4. 六轴真实结果

最终 `eval_complete_v2/evaluation.json` 为 `ALL_STATIC_GATES_PASS_PENDING_INDEPENDENT_REVIEW`，全部 20 个工况无拒绝原因。下表为逐案 `max |tau_est - Jᵀw_oracle|`、力误差范数和力矩误差范数；这些是本次离线仿真对 oracle 的误差，不是物理传感器不确定度。

| 构型 | 工况 | 结果 | 关节力矩最大误差 N·m | 力误差 N | 力矩误差 N·m |
| --- | --- | --- | ---: | ---: | ---: |
| home | zero | GATES_PASS | 0 | 0 | 0 |
| home | positive_fx | GATES_PASS | 0.000270903 | 0.000610168 | 0.000232825 |
| home | positive_fy | GATES_PASS | 5.78963e-05 | 0.000184525 | 3.54038e-05 |
| home | positive_fz | GATES_PASS | 0.000141399 | 0.000606648 | 7.6563e-05 |
| home | positive_mx | GATES_PASS | 0.000123023 | 0.000243627 | 0.000135626 |
| home | positive_my | GATES_PASS | 0.00049436 | 0.0014735 | 0.000656795 |
| home | positive_mz | GATES_PASS | 0.00031436 | 0.000341325 | 0.000179509 |
| home | negative_fx | GATES_PASS | 0.000203656 | 0.00051796 | 0.000255197 |
| home | negative_fz | GATES_PASS | 0.000302074 | 0.00138762 | 0.0002657 |
| home | negative_mz | GATES_PASS | 0.000313911 | 0.000266624 | 0.000182242 |
| offset | zero | GATES_PASS | 0 | 0 | 0 |
| offset | positive_fx | GATES_PASS | 0.00027573 | 0.00117821 | 0.000242485 |
| offset | positive_fy | GATES_PASS | 0.000170425 | 0.000509013 | 8.64708e-05 |
| offset | positive_fz | GATES_PASS | 0.000501054 | 0.001425 | 0.000275621 |
| offset | positive_mx | GATES_PASS | 0.000151063 | 0.000398502 | 7.21962e-05 |
| offset | positive_my | GATES_PASS | 0.000449575 | 0.000635151 | 0.000480383 |
| offset | positive_mz | GATES_PASS | 0.000433604 | 0.00120893 | 0.000333508 |
| offset | negative_fx | GATES_PASS | 0.000522217 | 0.00132295 | 0.00040389 |
| offset | negative_fz | GATES_PASS | 0.000289685 | 0.00112676 | 0.000138945 |
| offset | negative_mz | GATES_PASS | 0.000420304 | 0.000786972 | 0.000223934 |

全部 20 案中相应最大值分别为 `0.0005222172 N·m`、`0.001473501 N`、`0.0006567952 N·m`。非零轴合计 18 案、零载 2 案；六个独立分量均在两个构型实际施加，不能仅依据 Jacobian 满秩来声称这一点。

事后独立复核还对 40 个条件文件逐一核对完整载荷和目标元数据，并计算每个窗口 q 均值与注册目标的偏差：最大单关节偏差 `0.0007967055 rad`。这是**事后描述性检查，不是预注册的目标姿态门限**；0.002 rad 门限仅针对两窗口配对差。`independent_review.json` 保存每条件数值、完整载荷检查及 SHA-256。sign 数据的 link7 实测位姿与模型最大偏差为 `4.525e-7 m`、`4.754e-7 rad`。

## 5. 失败保留、负控和回归

在 six 原始采集仍未写 `collector_status.json` 时，一次提前运行评估得到 `INCOMPLETE / MISSING_COLLECTOR_STATUS`，记录保留在 `eval_early_incomplete`，不当作通过。采集完成后的原始 `eval_complete`、较早 `sign_result.json` 亦保留。针对只读审查指出的潜在空矩阵、缺失 sign 证据及载荷/目标元数据检查不足，随后增强评估器并用相同已保存数据重算为 `eval_complete_v2`；没有调整预注册载荷或门限，没有重新解释旧 P2-2 结果。正式证据采用 v2；更早结果用于保留审计轨迹。

合成负控用于验证接口拒绝行为，不当作 Isaac 测量：包括反号 effort、错误关节次序、非有限数据、过期/重复时间、缺失数据、错误 frame/point/source、未校准符号及静态窗口不匹配等。具体断言和日志位于 `offline/task28_p2_2_r1/tests/` 及 evidence；最终完整回归为 **667 项通过**（原 627 项及新增 40 项），退出码 0；新增 40 项聚焦测试亦单独通过。

完整 CPU 回归按以下命令执行，环境使用 Python 3.10.12、禁用 pytest 插件自动加载并固定 BLAS 线程；日志与退出码见 `evidence/regression_record.json`：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=offline:offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0:offline/task28_g1_2b_1:offline/task28_g1_1/tests:offline/task28_g1_2a/tests:offline/task28_g1_2b_0/tests:offline/task28_g1_2b_1/tests:offline/task28_p4_nr_0/tests:offline/task28_p5_qp_0/tests:offline/task28_p5_qp_1/tests:offline/task28_p5_qp_2/tests:offline/task28_p5_qp_3/tests:offline/task28_p2_1/tests:offline/task28_p2_2/tests:offline/task28_p2_2_r1/tests \
python3 -m pytest offline/task28_g1_1/tests offline/task28_g1_2a/tests offline/task28_g1_2b_0/tests offline/task28_g1_2b_1/tests offline/task28_p4_nr_0/tests offline/task28_p5_qp_0/tests offline/task28_p5_qp_1/tests offline/task28_p5_qp_2/tests offline/task28_p5_qp_3/tests offline/task28_p2_1/tests offline/task28_p2_2/tests offline/task28_p2_2_r1/tests -q -s -p no:cacheprovider --import-mode=importlib
```

## 6. 警告与科学边界

PhysX 在 `fr3_link8` 和 `fr3_hand_tcp` 报告可能无效的惯量及负质量，并采用小球近似；控制台日志已保留。资产未修改。相同姿态下的静态配对能抵消部分共同重力影响，但姿态不完全相同、残余速度和惯量/阻尼耦合使抵消仅近似；结果不能推广到动态补偿或真实力值。该模型也没有实际侧吸附工具与 D6 连接，施加载荷是已知的虚拟 TCP 作用于 link7 的外载。

静态两次 reset 的差分依赖重复基线，不是随手可用于运行中估计的在线 observer。真实 FR3 `tau_J`、模型重力/摩擦/负载、滤波延迟、传感器单位、frame、时间同步和噪声必须另行验证；Isaac solver 的 actuator-like effort 不等于已校准硬件 `tau_J`。不以本次结果授予碰撞、执行或有限吸附能力认证。

## 7. 重现与证据

下述命令只适用于经授权的隔离 Isaac 4.5 单臂研究环境；输出位置必须是不存在的新目录，`ASSET` 必须指向上述 SHA-256 的相同 USD。先完成并确认 sign 控制通过，再执行六轴采集；每条命令结果均应保存。命令不接入 ROS 或机器人。

```bash
set -e
ASSET=/path/to/fr3.usd
RUN=/tmp/task28_p2_2_r1_new
mkdir "$RUN"
OMNI_KIT_ACCEPT_EULA=YES PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh offline/task28_p2_2_r1/static_isaac_probe_r1.py \
  --mode sign --asset "$ASSET" --output "$RUN/sign_raw"
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 offline/task28_p2_2_r1/sign_control.py --input "$RUN/sign_raw" --output "$RUN/sign_result.json"
# 只有 SIGN_CONTROL_PASS 后继续
OMNI_KIT_ACCEPT_EULA=YES PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh offline/task28_p2_2_r1/static_isaac_probe_r1.py \
  --mode six --asset "$ASSET" --output "$RUN/six_raw"
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 offline/task28_p2_2_r1/evaluate_six.py --input "$RUN/six_raw" --output "$RUN/evaluation" \
  --config offline/task28_p2_2_r1/evidence/static_preregistration.json \
  --calibration "$RUN/sign_result.json" --sign-input "$RUN/sign_raw"
```

上述 `#` 是必须检查的先决条件，不能在 sign 控制失败时自动继续。最终报告、注册配置、脚本、raw 观察、逐案结果、早期失败和清单位于 `offline/task28_p2_2_r1/`；`independent_review.json` 为事后只读审查记录。临时目录仅用于重现，不作为长期存档。

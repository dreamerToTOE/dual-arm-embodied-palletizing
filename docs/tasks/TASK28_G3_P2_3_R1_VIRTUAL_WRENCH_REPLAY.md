# Task28 G3-P2-3-R1：已归档双臂虚拟 wrench 的 CPU 回放

**结果：`G3_P2_3_R1_VIRTUAL_REPLAY_PASS`，仅限数学虚拟分解。** 原 G3-P2-3 的冻结 G1 几何 `BLOCKED` 原样保留：零载左右推定物体位姿的相对误差为 **0.169917 mm、0.00133642 rad**，超过 0.100 mm、0.001 rad；稳态左右 TCP 相对名义抓取位置误差为 0.441309、0.333326 mm。这里没有把该门限用作虚拟回放的前提，也没有修改门限或改变该失败的意义。九个工况的原始机器人观测都被成功回放；本次虚拟回放没有失败工况。

Isaac 原实验没有 Cube、吸盘或共同接触，本回放不证明真实吸附内力、共同物体闭链可行性、动态外力矩补偿、在线力观测或力控制。`execution_authorized=false`、`physical_robot_tested=false`、`wrist_ft_sensor_required=false`、`dual_arm_force_control_implemented=false`、`production_bridge_modified=false`。B-only 不要求额外腕部 F/T 传感器，不等于真实机器人信号已标定。

## 数据与方法

只读使用 `offline/task28_p2_3/evidence/formal_raw/` 的 18 份 loaded/unloaded 尾窗、原始预注册、四项双侧独立正负载符号控制以及最终几何评估。原任务证据清单 **46 项的文件长度和 SHA-256 均匹配**；本次结果记录清单哈希、原输入哈希、冻结 fixture manifest 和 fixture binding、G1/P2 模型及新增源码哈希。各侧 joint1–joint7 的名称、顺序、现场 articulation 身份、静态窗步号/时间、有限性和独立现场 FK 检查均保留；左右在同一个采样窗逐帧同步，但 loaded 与 unloaded 经两次 reset，**不具物理同步性**。已归档的独立四项关节 1 符号控制逐项复算，平衡残差约 5.58e-5–7.02e-5 N·m。该符号仅用于原 Isaac 原生位置保持条件，并不是实机力矩标定。

预处理丢弃每帧 `load_oracle` 字段，仅以 `q`、`dq`、joint effort、实际 TCP 和来源身份构造两臂估计输入。独立历史符号控制是先前标定证据；每个工况完成估计后，才读取该工况的已知施加载荷进行事后 oracle 比较。施力点/输入力记录在物理步之前，q、effort 和现场 TCP 在该步之后；回放中检查前后差异并记录，不假称同刻。所有工况中前后最大关节差为 **7.153e-7 rad**、TCP 平移差 **2.977e-7 m**、旋转差 **8.159e-7 rad**，小于原注册检查限值，但仍非动态同步性能证明。

沿用 `T_A_B` 将 B 中列向量映射到 A；世界轴几何 Jacobian 在各臂加载窗口测得 q 的均值处计算。R1 符号约定给出作用于机器人外力矩 `−(effort_loaded−effort_unloaded)`；六维最小范数求解通过满秩和最小奇异值门。将估计作用于机器人、关于实际 TCP 的 wrench 反号为假设作用于 Cube 的 wrench。实际 TCP 原点 `p_a` 到冻结名义 TCP 原点 `p_n` 的力矩平移为 `(p_a−p_n)×f`，随后旋转到冻结接触轴，输入未修改的 P2-1 `GraspModel.frozen`。Cube 中心固定为 `[0.65, 0, 0.4] m`，未由数据拟合或改写名义 grasp。

负载 oracle 以每帧实际施力点相对同一个固定 Cube 中心的 `r×f` 独立求和。另对**估计**局部接触向量手工构建 G/W 并直接求解 18×18 KKT，以独立验证加权最小范数 external/internal 分解；针对实际 TCP 估计的中心力矩，还绕过 P2-1 和 KKT 直接求和，对照参考点平移。与 oracle 的 weighted external/internal 差是混合 SI 坐标的描述值，并非经物理标定的精度或容量判据。

## 九工况与冻结 G1 区分

表中误差是估计的假设 Cube 合力、合力矩与事后施加载荷 oracle 的欧氏范数误差；最后两列是本次对应加载窗口保留的冻结 G1 相对闭链误差。所有状态为虚拟回放 PASS，所有原几何判断依然失败。

| 工况 | 力误差 N | 力矩误差 N·m | G1 平移 mm | G1 旋转 rad |
|---|---:|---:|---:|---:|
| zero | 0 | 0 | 0.169917 | 0.00133642 |
| left_positive_fx | 0.000604643 | 0.000228161 | 0.163185 | 0.00132432 |
| left_negative_fx | 0.000394015 | 0.000165473 | 0.159587 | 0.00131028 |
| right_positive_fx | 0.000349719 | 0.000178617 | 0.176319 | 0.00132216 |
| right_negative_fx | 0.000434388 | 0.000155212 | 0.162424 | 0.00135563 |
| opposing_contact_line | 0.00136777 | 0.000439644 | 0.181124 | 0.00137465 |
| same_direction | 0.00265585 | 0.000861043 | 0.146842 | 0.00126603 |
| opposing_offset_forces | 0.000544689 | 0.000207207 | 0.157425 | 0.00134267 |
| asymmetric | 0.00165083 | 0.000486310 | 0.149959 | 0.00129219 |

反向接触线工况的 oracle 合力为零，但实际施力点偏移带来约 `[1.00922e-4, 0, −5.62327e-5] N·m` 的中心力矩，不能说它在该实际几何中严格零净 wrench。反向偏置力偶的合力为零、中心力矩约 `[0, 9.63878e-5, −0.122118498] N·m`，**不是纯内力**。同向载荷的 oracle 合力 `[0, 0, −2] N`；非对称载荷合力约 `[−0.3,−1.0,−0.2] N`。

全部工况中单臂关节力矩、力和力矩的最大 oracle 误差分别为 **0.000729737 N·m、0.00150912 N、0.000596757 N·m**；各自比较原预注册的 0.1 N·m、0.3 N、0.08 N·m。合力、合力矩门为 0.6 N、0.25 N·m。代数检查门为 `1e-10`：P2-1 零空间残差最大 **5.56e-16**，重构残差最大 **0**，独立 KKT 最大差 **4.58e-16**，绕过 P2-1 的实际 TCP 到中心参考点一致性最大差 **1.39e-17**。代数检查不应误解为物理传感器精度。

## 负控和复现

新增测试验证左右身份、关节顺序、frame、标定来源、重复/过期和乱序时间、姿态和资产输入，及错误作用/反作用符号、错误力臂、错误 weighted split。单侧力反号导致与独立合力 oracle 不匹配；手工移除 61 mm 力臂的相反载荷力偶损失 **0.122 N·m**，由独立参考点的 `1e-10` 代数门拒绝。该检测不依赖过宽的 0.25 N·m 物理 oracle 误差门；仅凭 null/reconstruction 也不能证明加权最优，因此使用 KKT 独立验证。观察解析测试删除 `load_oracle` 后仍可构造估计输入，之后 oracle 完整性单独验证。

代码与数据：[offline/task28_p2_3_r1](../../offline/task28_p2_3_r1/)，完整逐窗身份、逐侧估计、oracle、实际施力点、矩阵/分解、门限判定、每工况几何失败及哈希见 [virtual_replay.json](../../offline/task28_p2_3_r1/evidence/virtual_replay.json)。绑定文件加入前的首次本任务回放留作 [virtual_replay_before_binding.json](../../offline/task28_p2_3_r1/evidence/virtual_replay_before_binding.json)；两次都只读历史记录。复现时从仓库根目录使用不存在的输出文件：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 offline/task28_p2_3_r1/evaluate_replay.py --output /tmp/task28_p2_3_r1_replay_new.json

PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 -m pytest offline/task28_p2_3_r1/tests -q -p no:cacheprovider
```

原有 706 项独立回归通过；最终完整回归 **731 项通过**（含新增 25 项），退出码为 0。完整命令、环境、输出和哈希见本任务 `evidence/regression_record.json`。本次没有运行 Isaac、ROS、物理机器人或吸附执行，不改变历史证据、冻结资产、ACM 或几何门限。

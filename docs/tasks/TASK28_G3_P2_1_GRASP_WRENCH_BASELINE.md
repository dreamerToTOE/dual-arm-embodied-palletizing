# Task28 G3-P2-1｜离线抓取力旋量映射与内载分解

**结论：`G3_P2_1_MATH_PASS`。** 本结论仅覆盖冻结名义几何上的合成数学模型、独立数值对照及回归；真实接触 wrench 估计仍为 `BLOCKED`，没有实现吸附容量标定、内力反馈控制或物理执行。

仓库 `dreamerToTOE/dual-arm-embodied-palletizing`，分支 `side-suction-palletizing`，起点 `77b8f221f4f179a7a025e2df2e2c99c254fdfd8b`。本地和远端起点一致，开始前工作树干净。新代码仅在 [`offline/task28_p2_1`](../../offline/task28_p2_1/)；冻结 fixture、模型、ACM、G1/G2 验证器及历史证据未修改。

## 1. 来源及适用范围

本模型读取 G1-1 [`nominal_grasps()`](../../offline/task28_g1_1/task28_validator/closed_chain.py#L46) 的 `T_O_L`、`T_O_R`，而不是把新设的对称几何冒充源配置。Cube 半边长 0.060 m，TCP 名义位置在物体坐标下分别为 `(0,-0.061,0)` 和 `(0,+0.061,0)` m；其中 1 mm 是名义抓取目标间隙，并非实测吸附锚点。左右 TCP 旋转分别为

```text
R_OL = [ 0  1  0 ]       R_OR = [ 0 -1  0 ]
       [ 1  0  0 ]              [-1  0  0 ]
       [ 0  0 -1 ]              [ 0  0 -1 ]
```

`T_W_O` 可为任意有效 SE(3)，`T_W_TCP_i = T_W_O T_O_i`；完整冻结 Cube、TCP 安装及 world 定义见 [fixture manifest](../../fixtures/task28_g1_0_new_single_cube_v0/fixture_manifest.json)。此处受力点是名义 TCP 的**数学抽象**，不是已观测的 D6 实际锚点。直接构造其它 SE(3) 的模型标记为 `ABSTRACT_CONTROL`；`.frozen()` 才标记 `G1_1_NOMINAL_FROZEN_CONTRACT`。

已读取 [G3-P2-0](TASK28_G3_P2_0_OBSERVABILITY_AUDIT.md)、[G0-A §7](TASK28_G0A_INTERFACE_AUDIT.md)、[路线图](TASK28_PAPER_TO_MAIN_RESEARCH_ROADMAP.md) 和独立复现仓库 `79ad22c7fded963e0b8e9cc50caadb242bca8108` 的 P2 方法卡、TASK07。原论文使用接触模型、接触模式及视觉/触觉融合；这里是其抓取映射和内载分解的最小数学适配，不是原论文完整感知或控制复现。独立复现仓库的 TASK07 标为 TODO。G3-P2-0 所述 D6 reaction、物体身份、实际抓取变换和双侧标定信号仍未解决。

## 2. 坐标、符号和完整抓取矩阵

主动列向量约定 `T_A_B` 将 B 坐标映射到 A。每个输入是**作用在 Cube 上**、关于该接触 TCP 原点的局部六维 wrench：

```text
w_i^i = [f_x, f_y, f_z, tau_x, tau_y, tau_z]^T
x = [w_L^L; w_R^R]                 # N, N·m
```

设 `R_Wi = R_WO R_Oi`，`r_Wi = R_WO p_Oi` 为 Cube 中心指向 TCP 的世界向量，`[r]× f = r×f`。关于 Cube 中心、以世界坐标表达的净 wrench 为

```text
G_i = [ R_Wi          0  ]
      [ [r_Wi]× R_Wi  R_Wi ]
G = [G_L G_R]                     # 6 × 12
w_object^W = G x
```

如果输出以 object 轴表达，将合力与合力矩分别乘 `R_WO^T`；`T_WO` 的平移不影响关于 Cube 中心的力矩。`contact_to_world/world_to_contact` 只改变轴，不改变力矩参考点；`contact_to_center/center_to_contact` 才通过力臂转换参考点，并支持逆变换。动作方向统一为接触对 Cube 的作用，反作用需要另外取负。接口对非有限数、错误形状、非法旋转、错误 frame/对象/ordering/provenance 均拒绝。

这是每侧可传递 6 个独立分量的**聚合合成模型**。它不说明侧吸工具能够独立传递或命令这些分量，也不包含单侧拉压、摩擦锥、吸附力矩、D6 破断阈值或实际关节力矩限制。

## 3. 度量定义的分解与合成分配

对正定接触度量 `W`，设 `D = W^(-1/2)`（实现以 Cholesky 构造），对 `G D` 作 SVD 并记录数值秩。加权广义逆为 `G_W# = D (G D)^+`：

```text
w_net = G x
x_external = G_W# w_net
x_internal = x - x_external
```

验收分别检查 `G x_internal≈0`、`x_external+x_internal≈x`、`x_external^T W x_internal≈0`。激进秩截断若把可产生物体载荷的分量错误留在 `internal` 中，接口拒绝 `TRUNCATED_DECOMPOSITION_NOT_IN_NULL_SPACE`；无法满足指定净 wrench 的分配则拒绝 `UNSUPPORTED_REQUIRED_WRENCH`。这些是数值可行性判断，不是物理接触容量或机器人可达性判断。

分配基线求解

```text
minimize  1/2 x^T W x
subject to G x = w_required
```

默认 `W` 以力尺度 1 N、力矩尺度 0.061 N·m 归一化，每侧相同；这些仅是数学尺度，**不是**有限容量，也与现有 `1e6` D6 断裂阈值无关。非对称测试使用明确的不同左右权重。改变 wrench 轴、参考点、接触子空间、广义逆或度量会改变所定义的 external/internal 分量；一个数学分配不能由净载荷唯一识别为实际接触负载。

## 4. 独立合成验证

[synthetic_math.json](../../offline/task28_p2_1/evidence/synthetic_math.json) 保存完整 6×12 矩阵、输入、输出、残差、权重、源哈希和环境。数值来源均为确定性合成输入。

| 手算控制，物体坐标 | 独立预期净 wrench `[F; τ]` | 实际结果 |
|---|---|---|
| 零载 | `[0,0,0; 0,0,0]` | 一致 |
| 左侧 +X 1 N，右侧零 | `[1,0,0; 0,0,0.061]` | 一致；检验 `r×f` 符号 |
| 两侧各 +X 1 N | `[2,0,0; 0,0,0]` | 一致；同向力产生净载荷 |
| 两侧沿抓取轴相反的 local +X 1 N | 全零 | 一致；属于所定义的 internal 分量 |
| 左右物体坐标 +X/−X 各 1 N | `[0,0,0; 0,0,0.122]` | 一致；反向力未必是零净载荷 |
| 纯 +Z 力矩 | `[0,0,0; 0,0,1]` | 一致 |

六项手算的最大绝对误差为 **0**；各分解的最大 `||G x_internal||` 为 **5.49e-16**。此满六维模型的无权 `G` 数值秩为 **6**、零空间维数 **6**；奇异值约 `[1.416842,1.416842,1.414214,1.414214,1.414214,1.414214]`。因力与力矩量纲不同，未经尺度归一化的奇异值不应解释为物理条件数或容量。

对合成目标 `[3,-2,7.848,0.1,-0.2,0.3]`，对称和非对称权重的等式残差分别为 **1.84e-15** 和 **1.20e-15**。两个分配与独立直接 KKT 方程求解的最大解差为 **1.78e-15**。旋转后的独立虚功对照：两接触局部功率之和与净 wrench 对 Cube 中心 twist 的功率差为 **2.22e-16**；这个对照同时检查旋转和力矩臂符号。

此外测试覆盖力/力矩交换或符号反转、左右镜像、平移不变性、中心/TCP 双向变换、稠密正定度量、错误对象或 frame、缺失合成来源、非有限输入、非正定度量、秩阈值截断和不可表示的合成目标。手写解析 oracle 独立于生产矩阵构造。`request_physical_estimate()` 对任何请求返回 `BLOCKED`；合法的 `SYNTHETIC_OFFLINE` 标签不能升级为物理观测。

## 5. 复现和回归

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 -m offline.task28_p2_1.generate_evidence

PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 -m pytest offline/task28_p2_1/tests -q -p no:cacheprovider
```

生成器拒绝覆盖已有 `synthetic_math.json`；复现时应在独立检出中使用新输出位置。新增 **23 项测试通过**；完整回归 **595 项通过**（原有 572 项加新增 23 项），退出码 0。运行命令及原始结果保存在 `evidence/regression_record.json` 与 `evidence/regression.log`。

## 6. 不可由本结果推出的结论

本任务只完成“合成 wrench → 数学合力与分配”。经标定的双侧接触 wrench 估计、真实吸附体身份和锚点、时间同步、单杯或聚合接触可传力范围仍未知；未计算实际吸附裕量、关节 torque 可行性、摩擦/接触动力学或内力控制效果。

没有运行 Isaac、ROS、机器人或吸附实验，没有修改 bridge、D6 参数或执行路径。G2-QP-3 实际执行接口仍为 PARTIAL；NR-1 仍延期。到 G3-P2-1 停止，G3-P2-2 与仪表化实验需另行授权。

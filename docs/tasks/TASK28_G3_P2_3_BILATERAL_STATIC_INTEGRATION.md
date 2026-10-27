# Task28 G3-P2-3：双 FR3 B-only 静态虚拟接触验证

**结论：`BLOCKED`（G3_P2_3_BLOCKED）。** 两路 Isaac 关节 effort 的采集、左右独立符号检查和实时 FK 比较有可复核结果；但是正式工况的稳态 TCP 不满足事先登记的冻结 G1 名义抓取位置容差，因而不能在同一个名义 Cube 上完成可信的双侧 wrench 集成。9 个正式工况均被该门禁拒绝。这里的结果属于 `VIRTUAL_CONTACT_STATIC`：场景没有 Cube、吸盘、D6 或实体接触，也没有在线观测器或力控制。

本次执行授权字段均为 false：`execution_authorized`、`physical_robot_tested`、`wrist_ft_sensor_required`、`dual_arm_force_control_implemented`、`production_bridge_modified`。其中 `wrist_ft_sensor_required=false` 表示本研究路径未要求额外腕部传感器，不是对硬件测量能力的证明。

## 基线与实验来源

起点 `de00569dba616ef8be099154a6476e8896518097`；沿用冻结 `task28_g1_0_new_single_cube_v0`，输入是 `offline/task28_g1_1/tests/control_states.json` 中的 `lifted_symmetric`：名义物体中心 `[0.65, 0, 0.4] m`、零 yaw，左右基座分别 `[0.65, -0.60, 0]`、`[0.65, 0.60, 0] m`，两者均无旋转。原始保存状态的闭链平移残差约 `1.867e-9 m`，旋转残差约 `2.057e-8 rad`；双臂 Jacobian 最小奇异值约 `0.195161`、`0.200365`。这只证明所保存的初始关节向量满足离线几何检查，不代表位置保持后的状态也满足。

使用同一 Isaac Sim 4.5 独立场景中的 `/World/Probe_left` 与 `/World/Probe_right` 两个 FR3 articulation；每臂按名称确认 `fr3_joint1` 至 `fr3_joint7` 的顺序与索引，实际资产还包括两个手指 DOF。USD SHA-256 为 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`。没有修改原资产的关节、碰撞或位置驱动；本场景不加载生产 bridge，也不加载 Cube。

每个工况分别整体 reset 并采集 unloaded、loaded 两段；每段尾部 60 帧同时包含两臂同一 physics step 和仿真时间。**两次 reset 的 unloaded 与 loaded 不具物理同步性**；静态配对差分只是离线实验方法，不是可部署的在线外力矩观测器。原始 joint effort 为 Isaac `SingleArticulation.get_measured_joint_efforts` 输出，不等价于经过硬件标定的真实 Franka 力矩测量。

## 预注册与执行记录

`offline/task28_p2_3/evidence/preregistration.json` 在正式采集前冻结；注册了零载、左右单臂各正负 Fx、沿接触线相反力、同向力、接触点偏置的相反力产生物体力矩、非对称载荷，合计 9 个工况。载荷是施加在机器人上的已知模拟力/纯力矩，只能作验证 oracle；估计模块不接收 oracle。时间步 `1/120 s`，每段 `600` 步、观察尾窗 `60` 步、每段预检查 `1` 步，总预算 `11000` 步、墙钟预算 `600 s`。正式采集记录 `10818` 物理步、18 个条件文件，约 `93.92 s`。

重要预注册门限均保持原值：live link7 FK 平移 `1e-4 m`、旋转 `1e-4 rad`；名义 grasp 平移 `1e-4 m`、旋转 `1e-3 rad`；最大关节速度 `0.01 rad/s`；成对平均 q 差 `0.002 rad`；窗内 q 极差 `0.002 rad`、effort 标准差 `0.05 N·m`；Jacobian 最小奇异值 `0.05`；关节外力矩误差 `0.1 N·m`、单臂力误差 `0.3 N`、单臂力矩误差 `0.08 N·m`、零载差分 `0.05 N·m`；第一关节符号平衡残差 `0.1 N·m`；双臂对象合力误差 `0.6 N`、对象合力矩误差 `0.25 N·m`，零空间残差 `1e-10`。这些是事前研究门槛，不是经过物理认证的传感器性能指标。

存在一次被中止的应用启动：其旧预注册和控制台输出单独保存于 `evidence/aborted_start_preregistration.json`、`evidence/aborted_start_console.log`。该次启动在 scene/physics 采集之前终止，未获得正式工况数据。随后在正式采集之前明确增加集成契约并冻结 r2；正式数据未用于修改 r2 门限。正式过程和状态分别记录于 `evidence/formal_console.log` 与 `evidence/formal_raw/collector_status.json`。

复现入口（资产路径为本机已核验的缓存；在其他主机应提供相同哈希的资产，输出目录必须不存在）：

```bash
/home/ubuntu2004/isaacsim-4.5.0/python.sh offline/task28_p2_3/collector.py \
  --asset /home/ubuntu2004/Documents/Codex/2026-10-25/files-pasted-by-the-user-task/work/g1_0b_asset_audit/fr3.usd \
  --output /tmp/task28_p2_3_new_raw
python3 offline/task28_p2_3/evaluate.py \
  --input /tmp/task28_p2_3_new_raw --output /tmp/task28_p2_3_new_evaluation
```

评估命令对当前失败数据的预期退出码为 1（`PARTIAL`），而非成功；运行于项目根目录并使用独立的新输出目录。

以上是复核协议，不授权连接设备或生产场景。Isaac 的原始启动输出含 `fr3_hand_tcp`、`fr3_link8` 负质量/可能无效惯量和近似惯量警告；本结果不得外推为实机动力学标定。

## 关节符号与参考系

约定 `T_A_B` 把 B 坐标映射到 A；Jacobian 是世界轴下的冻结名义 side-suction TCP 几何 Jacobian。施加载荷是对机器人作用，在记录的虚拟 TCP 点，以世界坐标表示；如果虚构共同物体的接触成立，作用到 Cube 的对应 wrench 取反，再把力矩从机器人实际 TCP 参考点平移到名义接触点，并旋转到冻结 `object→TCP` 接触局部坐标。`GraspModel` 映射的是关于 Cube 中心的 wrench，数学函数未修改。本工况没达到共同名义接触的几何前提，故这些转换与零空间分解没有作为正式可接受输出执行。

通过独立几何公式（世界第一关节轴 `+z`，施力点相对轴的叉乘）分别验证两臂符号，没有使用 Jacobian 求逆，也没有把 oracle 输入估计器。单位为 N·m：

| 控制 | 对机器人第一关节的已知外力矩 | effort 加载差 | `|Δeffort + τ外|` |
|---|---:|---:|---:|
| 左 +Fx | -0.53894010 | +0.53886990 | 0.00007019 |
| 左 -Fx | +0.53895035 | -0.53888239 | 0.00006796 |
| 右 +Fx | +0.53894096 | -0.53888519 | 0.00005577 |
| 右 -Fx | -0.53894141 | +0.53887538 | 0.00006602 |

四项均通过事前 `0.1 N·m` 阈值，支持本场景、当前 USD 原生 position hold 条件下 `τ外 = -(effort加载 - effort卸载)`。这一结论不是实机力矩符号或动态外力矩观测器的标定。记录详见 `evidence/sign_result.json`，包含每段源文件哈希、步号、时间和替代符号残差。

## 独立 FK 与失败工况

采集器每段初始化后先以零载推进 1 步，之后在每次施力前检查左右 live link7 pose 对冻结模型 FK；末窗每帧保存 link7 pose、TCP、q、dq、effort、作用点与步骤标识。零载预检查中左、右名义 grasp 平移残差分别为 `0.03573 mm`、`0.02721 mm`，低于 `0.100 mm`；但该检查不是稳态承诺。零载稳态分别变为 **`0.441309 mm`、`0.333326 mm`**，均超过 `0.100 mm`，对应旋转偏差 `0.000818598 rad`、`0.000631200 rad`。其位置保持稳态位移可从原始 q 和 FK 重现。

这与 FK 实现一致性是两个不同判断：稳态零载尾窗中，左 live link7 对模型最大平移/旋转误差约 `2.979e-7 m`/`5.650e-7 rad`，右约 `2.888e-7 m`/`6.798e-7 rad`，满足 FK 阈值；这并不能消除稳定后 q 对名义共同物体抓取姿态的偏移。最终评估对九个工况均记录左右各自的 G1 抓取平移不符，也记录左右推定物体姿态之间的平移与旋转不符。零载时相对闭链平移残差为 `0.169917 mm`、旋转残差为 `0.00133642 rad`，超过冻结的 `0.100 mm`、`0.001 rad` 门限。G1 的物体推定平移残差还受 TCP 旋转与抓取偏置影响，因此不同于下表的 TCP 位置残差。表中单位为 mm，数值为加载窗口 q 均值相对注册物体和相应名义抓取的残差：

| 工况 | 左 | 右 | 正式集成 |
|---|---:|---:|---|
| zero | 0.4413 | 0.3333 | 拒绝 |
| left_positive_fx | 0.4358 | 0.3333 | 拒绝 |
| left_negative_fx | 0.4271 | 0.3333 | 拒绝 |
| right_positive_fx | 0.4413 | 0.3281 | 拒绝 |
| right_negative_fx | 0.4413 | 0.3378 | 拒绝 |
| opposing_contact_line | 0.4557 | 0.3486 | 拒绝 |
| same_direction | 0.4123 | 0.3100 | 拒绝 |
| opposing_offset_forces | 0.4358 | 0.3378 | 拒绝 |
| asymmetric | 0.4200 | 0.3204 | 拒绝 |

每个条件原始输入、施力 oracle、两臂 DOF、步骤和姿态保存在 `evidence/formal_raw/`；原始输出不可覆盖。因名义 grasp 先决条件失败，不报告这九个工况的接受版双臂 net/weighted external/weighted internal wrench，也不报告物理吸附内力。即使单臂外力矩信号确有数值变化，也不能把两个错位的虚拟点直接当作已建立共同物体接触的测量。

## 离线评估与测试

首次离线评估把 `precheck` 后第一帧的时间步尾窗算错一位，9 项因 `WINDOW_NOT_PREREGISTERED_SETTLED_TAIL` 拒绝；这属于评估代码的下标问题，不是 Isaac 数据被重采样或调参。修正后再次对相同原始数据评估，早期结果先报 `FROZEN_GRASP_MISMATCH:left`；最终版本进一步逐侧记录并调用完整 G1 相对闭链门，9 项均拒绝。旧版评估输出及其源码哈希已保留，但最初两版源码快照未保存，故不能仅凭哈希重跑旧版；`evaluation_before_full_closure` 源码快照和最终评估器均已归档。最终评估 JSON 的 `PARTIAL` 表示采集和符号等局部工作完成；本报告总状态 `BLOCKED` 表示冻结几何阻断了全部九项的共同物体集成。不得把这一几何失败改称验证成功。

原有 667 项完整回归：**667 passed**（`evidence/regression_baseline.log`）。本任务新增 focused 测试为 **39 passed**；最终全量回归 **706 passed**（原 667 加新增 39），退出码 0；签名、DOF、姿态、时间窗、数值有限性、frame 身份和 rank 缺陷的负控只证明软件拒绝指定输入，不证明硬件保护功能。

完整 CPU 回归在仓库根目录执行：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=offline:offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0:offline/task28_g1_2b_1:offline/task28_g1_1/tests:offline/task28_g1_2a/tests:offline/task28_g1_2b_0/tests:offline/task28_g1_2b_1/tests:offline/task28_p4_nr_0/tests:offline/task28_p5_qp_0/tests:offline/task28_p5_qp_1/tests:offline/task28_p5_qp_2/tests:offline/task28_p5_qp_3/tests:offline/task28_p2_1/tests:offline/task28_p2_2/tests:offline/task28_p2_2_r1/tests:offline/task28_p2_3/tests \
python3 -m pytest offline/task28_g1_1/tests offline/task28_g1_2a/tests offline/task28_g1_2b_0/tests offline/task28_g1_2b_1/tests offline/task28_p4_nr_0/tests offline/task28_p5_qp_0/tests offline/task28_p5_qp_1/tests offline/task28_p5_qp_2/tests offline/task28_p5_qp_3/tests offline/task28_p2_1/tests offline/task28_p2_2/tests offline/task28_p2_2_r1/tests offline/task28_p2_3/tests -q -s -p no:cacheprovider --import-mode=importlib
```

## 未解决事项

要继续研究，需要先设计并事前登记满足冻结名义 grasp 和静态收敛的隔离初始化/保持方案，证明新的左右实际 TCP 仍与冻结 frame 一致，并在新实验保留所有失败记录；这不能通过事后放宽门槛、替换物体姿态、把 oracle 喂给估计器或修改历史证据实现。即使未来通过该静态虚拟测试，也仍须独立解决实际吸附接触、动态/重力补偿、真实传感器标定、反馈时序和安全控制问题。本任务不自动执行后续工作。

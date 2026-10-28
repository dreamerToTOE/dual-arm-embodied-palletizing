# Task28 G2-P5-QP-6-R1：约束感知动力学恢复与 HQP 续作审查

**最终状态：`QP6_R1_SOURCE_PARTIAL_FORMAL_BLOCKED`。** 本任务复现了原自由九维源失败，验证了若干 mimic 机制与 arm reduction 数学结构，但原 0.04 m 条件下的完整约束/降阶源仍未通过预注册资格门。四个 B/C 正式移动单元全部 `SKIPPED_WITH_REASON`，没有重跑旧 A baseline，也没有实际 Cube/D6、ROS 或实体机器人执行。

## 1. 起点、保护与参考实现审查

开始时本地、远端 `side-suction-palletizing` 均为 `cbfea96775696defb864d48d4cef30ce689ff959`，工作树干净。起点 **1204 个已跟踪文件**受逐文件 SHA-256 保护，最终全部一致。新工作仅在 `offline/task28_p5_qp_6_r1/` 和本报告；原 QP6_PARTIAL、A/S05 第 7 步拒绝、A/S10 第 12 步拒绝、原 SOURCE_BLOCKED 和 929 测试归档均未改写。

实现前审查记录见 [literature_review.json](../../offline/task28_p5_qp_6_r1/evidence/literature_review.json)，内容包括原 G1/G2/G3/QP-6、冻结 USD、安装版 articulation API、PhysxMimicJointAPI tests/demo，以及以下第一方资料：

- [Isaac Sim 4.5 mimic/闭环教程](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/rig_closed_loop_structures.html)：直接抓取返回 404，但版本页面的索引内容可读；与本机安装测试交叉核对，没有用 latest 文档替代安装版。
- [PhysX articulations](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/ovphysx/docs/simulation_setup/articulations.md)：当前 main 包含 2026 ovphysx/codeless schema 接口，不能假定等同 Isaac 4.5；这里只借鉴约束、限位和 mimic 的概念。
- [Pinocchio reduced model](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b_examples_e_reduced_model.html) 与 [约束动力学例程](https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/simulation-closed-kinematic-chains.py)：锁定指定关节要明确参考构型，约束动力学要建立明确约束对象；不能任意截取质量矩阵。本任务审查例程，没有声称运行 Pinocchio 或以其替代 Isaac 源。
- [TSID 实现](https://github.com/stack-of-tasks/tsid/blob/devel/src/formulations/inverse-dynamics-formulation-acc-force.cpp)：在完整质量/非线性项及接触运动/力约束上构造问题，消去 effort 变量不等于消除物理反力。
- [Franka Arm/Hand 架构](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html)：Robot 的实时 arm torque 和 Gripper 的手部操作有不同接口。FCI 的重力/摩擦处理也不同于 Isaac 原始 effort，因此该架构只能支持接口区分，不能证明本次降阶或硬件等价性。
- [IROS 2023 书目](https://cris.fau.de/publications/306133864/) 与原 QP-6 摘要审查保留。未取得新的论文全文，没有声称对象空间 HQP 或内载分配本身新颖。

安装版为 `4.5.0-rc.36+release.19112.f59b3005.gl`。实际测试使用 bundled Python 3.10.15、NumPy 1.26.0；CPU 回归环境独立记录。冻结 USD SHA-256 仍为 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`。安装版源码和模型哈希见 [source_binding.json](../../offline/task28_p5_qp_6_r1/source_binding.json)。

## 2. 原失败的独立取证

[forensic_v1](../../offline/task28_p5_qp_6_r1/evidence/forensic_v1/summary.json) 不调用原 QP-6 evaluator，按实际 pre/post dq、真实时间差及 applied-command readback 重建：

```text
qdd_obs = (dq_post-dq_pre)/dt
r = M_pre*qdd_obs + c_pre + g_pre - tau_applied_readback
```

首次失败重现为 **row18、右臂、physics post-step21、t≈0.1750000091 s**。请求 joint4 约 `+0.05 rad/s²`，finger qdd 请求为零；实际两指加速度均约 `−0.00441594 m/s²`，相应残差约 `−6.68708e-5 / −6.68034e-5 N`。

该帧两指 pre-q 为 `0.0399999618530 / 0.0399999581277 m`，距实际 USD 上限约 `3.73e-8 / 4.10e-8 m`。pre-dq 约 `−2.476e-7 m/s`，post-dq 约 `−3.70471e-5 m/s`。mimic q 残差约 `−3.72529e-9 m`，dq 差约 `−2.27e-13 m/s`，加速度差约 `2.73e-11 m/s²`。

安装版测试的关系是 `q_target+gearing*q_reference+offset=0`。每次新源运行通过真实 PhysxSchema 确认 gearing `−1`、offset `0`、finger2 引用 finger1，故该处约束 Jacobian 为 `J_m=[0…0,−1,+1]`，允许两根手指同向广义坐标运动。

原残差主要沿两指共同方向，mimic-only 允许方向残差约 `−9.4522e-5 N`，因此 **mimic 单独不能解释失败**。加入上限方向后可代数拟合其共同分量；但两个上限与 mimic 行存在冗余，不能唯一识别每根手指的反力，更不能把拟合 lambda 称为传感器读数或已验证的接触互补性。

取证还保存了完整 9×9 M、arm/finger 耦合块、q/dq、applied 与 projected effort 的区别、允许方向/约束方向残差、时间差和 float32 差分敏感性。速度量化误差不足以单独解释上述加速度差；位置量化和约束位置修正仍可能影响限位附近的离散动力学，未分配未经验证的唯一因果解释。

## 3. F/K/R 三种策略及独立数学检查

完整定义见 [MODEL_AND_QUALIFICATION.md](../../offline/task28_p5_qp_6_r1/MODEL_AND_QUALIFICATION.md)。

**F：自由 18 坐标。** 保持每臂九维自由方程和原 `0.02 rad/s² / 0.002 m/s²` 误差门，原始失败不改判。

**K：完整约束模型候选。** 使用 `M*qdd+h=tau+J_c.T*lambda`、mimic 位置/速度修正和单边预测位置界。上限法向为 `−e_f`、lambda 非负；向内运动应释放，不能把限位永久锁成双边等式。采用安装版 TGS 测试的 ERP `.9` 作为明确候选假设，并与原始物理输出独立比较。对每个源状态枚举 lower/free/upper 活动面，直接解 KKT 并核对原 OSQP 结果；最大解差约 **`4.05e-12`**。这证明数值实现一致，不能代替物理源一致性。

**R：带保留手指策略的 arm subsystem。** 对 `f1=f2=w`，以 9×8 embedding S 得到 `M8=SᵀM9S`、`h8=Sᵀh9`，保留虚功和动能。实际 arm 方程仍是：

```text
Maa*qdd_a + h_a + Maf*qdd_f = tau_a
Maf*qdd_f = (Maf1+Maf2)*mean(qdd_f)
            + .5*(Maf2-Maf1)*(qdd_f2-qdd_f1)
```

冻结模型的两列 Maf 近似抵消，是可验证的结构，而非允许盲切 `M[:7,:7]` 的一般理由。C 配置逐步保存完整 M，检查 mimic q/dq/qdd、共同列和与实际耦合项，并检查手指 force-servo 状态、约束残差结构及共同安全门。

C 的完整九维命令采用：

```text
tau_arm = Maa*qdd_requested + c_arm + g_arm
tau_finger = g_finger + 20*(0.04-q_finger) - 2*dq_finger
```

手指位置仍是原 `0.04 m` 参考，单位为 N/m 和 N·s/m；每侧全部九个 effort 都发送，原 native drive gains 在临时场景置零，不写回 USD。没有删除 finger、修改 mimic、限制范围、fixture、ACM 或 G1。

测试包含两质量同向约束的独立手算、上限释放、mimic normal-force 负控、惯性抵消与非抵消反例、active-face oracle、命令单位和拒绝门。**R 的完整资格要求不只看 arm 加速度。** 它在实际源 C 中仍未通过。

## 4. 预注册、启动失败与四个源配置

[preregistration.json](../../offline/task28_p5_qp_6_r1/preregistration.json) 在新物理前冻结四配置、状态、命令公式、加速度、时长、阈值和逐步拒绝政策。`source_registration.json` 和 `source_registration_v2.json` 保存两个启动实现版本及源码哈希；v2 没有改变任何数值门、初态或控制参数。

首轮四个进程都在发送 effort 前中止：SimulationApp 将捆绑 OSQP **0.6.5** 加到路径前端，而 QP-6 数值接口使用 1.0.5 的 `polishing` 参数。受控步数均为 0，只有各 2 个 World reset 初始化步。没有把这些启动记为科学通过；日志、异常和原 probe 源码快照均保留。错误前读取的 pre-state 未写入 raw，这是启动证据的局限，没有事后补造。

随后固定在 SimulationApp 前加载任务局部 OSQP **1.0.5**，先在 bundled Python 做零载数值检查，再执行**原有同一组四配置**。没有新增物理配置或重复调参；四次初始化失败与四次实际受控运行的进程数和物理步分别披露。

| 源配置 | 身份与用途 | 受控步/预算 | 结果 |
|---|---|---:|---|
| A | 原 USD、原 0.04 m、原自由 qdd 脉冲；不是重跑正式控制器 A | 19/30 | REJECTED，row18/right 重现原 finger 自由加速度门失败 |
| B | 原 USD、**新身份 0.02 m** 内侧诊断，同一自由脉冲 | 30/30 | 完成诊断；不能推广到原 0.04 m |
| C | 原 0.04 m、arm reduction + 完整 finger 力策略 | 140/268 | REJECTED，约束残差结构门失败 |
| D | **新身份 0.02 m**，两指 ±0.001 N 反向 normal-force 负控 | 20/20 | 完成；自由模型负例、K 预测通过，仅覆盖该内侧机制 |

A 完成 12 个 gravity/c 保持步后在第 7 个脉冲步停止。C 完成 120 个保持步和 20 个 signed pulse 步，仅覆盖前五个 arm 关节的完整登记脉冲；joint6/7 的显式脉冲和 120 个末端保持均未执行，剩余 **128 步**。A 剩余 **11 步**。两者首个注册拒绝帧均是最后一个 raw 帧，没有在失败后继续控制。

B 的 finger 自由加速度误差峰值约 `0.00050003 m/s²`，虽低于 `.002` 数值门，但单指参考加速度本身不满足 mimic 微分关系，因此它不能证明完整九坐标可独立自由驱动。D 的自由模型差约 `0.0667673 m/s²`，而 K 误差最大约 `2.05e-5`；独立已知反向输入说明该内侧条件下 mimic 约束响应不能被自由模型忽略。D 没有验证上限附近的反力或原配置模型。

## 5. C 的首次失败与源资格

C 首次注册失败为 **row139、右臂、physics post-step142、t≈1.183333395 s**。当时：

| 指标 | 数值/结论 |
|---|---|
| pre finger q，m | `0.0399998724461 / 0.0399998761714` |
| 距上限 gap，m | `1.26660e-7 / 1.22935e-7` |
| 注册 near-upper 候选区间，m | `1e-7`，两指均在其外 |
| 实际 finger qdd，m/s² | 两者约 `−0.00121017` |
| 原始 finger residual，N | `+2.02827e-4 / −2.40765e-4` |
| mimic-only 剩余共同 residual，N | 两者约 **`−1.89693e-5`** |
| 剩余 residual 门，N | **`5e-6`，失败** |
| 左/右绝对 grasp 平移，mm | `0.0256957 / 0.0096693`，通过原 G1 |
| 相对/共同平移，mm | `0.0251585 / 0.0147866`，通过原 G1 |

该帧其他 R 条件全部通过，包括 arm 加速度、含 finger coupling 的 arm 方程、finger 保持、mimic q/dq/qdd、质量耦合抵消及 gap consistency。整个 C 前缀最大 arm residual 约 `1.819e-5 N·m`，共同耦合列峰值约 `4.307e-9`，实际 finger coupling 项约 `4.771e-11 N·m`。

这些是 R 候选的有利证据，但不足以越过失败的完整资格门。扩大 near-upper 区间会改变活动集和结论；本任务没有在看过失败后扩大该区间或放宽力残差门。该失败说明声明的约束活动集/残差解释尚不完整，不能单凭此帧判定 PhysX 错误或给出唯一原因。

K 候选在原 A 有 1 个右侧观测失败，在 C 前缀也有 1 个右侧观测失败：C row136 的 finger 预测差约 `−0.003778 / −0.003730 m/s²`，超过原 `.002` 门。K 在 C 只是独立诊断；C 的继续/停止由事先登记的 R 资格门决定。不能把数值 KKT 正确、B/D 内侧通过或一部分 arm 方程通过改称原完整约束模型通过。

因此最终源标签为：

- `UNCONSTRAINED_FULL_18D_SOURCE_STILL_BLOCKED`：保留。
- `FULL_18D_CONSTRAINED_DYNAMICS_VALIDATED`：**不成立**。
- `ARM14_REDUCED_DYNAMICS_VALIDATED_WITH_FINGER_POLICY`：**不成立**。
- 综合：**SOURCE_PARTIAL，formal continuation BLOCKED**。

## 6. 正式控制器矩阵与性能主张

只有原 0.04 m 源 C 完整通过、独立负控 D 通过及共享源/几何门满足，才允许正式 B/C。该条件未满足，因此未建立 actuator continuation adapter 或启动正式比较；没有必要用尚未合格的模型继续发送移动控制命令。

| 正式 controller/profile | 状态 | 运动步 | 末端保持 | 原因 |
|---|---|---:|---:|---|
| B/S05 | SKIPPED_WITH_REASON | 0 | 0 | 原配置共享源 C 资格门失败 |
| B/S10 | SKIPPED_WITH_REASON | 0 | 0 | 同上 |
| C/S05 | SKIPPED_WITH_REASON | 0 | 0 | 同上 |
| C/S10 | SKIPPED_WITH_REASON | 0 | 0 | 同上 |

旧 QP-6 A/S05、A/S10 仍分别是第 7/12 个移动步被原 `0.100 mm` 门拒绝的冻结 baseline，未重跑或改增益。S05/S10、原 q14/object pose 和 G1 均未改动。四个新正式格的 tracking、effort、dynamic/constraint residual、HQP priority 和 solver wall-time distribution 均为 **null/未运行**，不是零误差、零耗时或成功实验。

源 K QP 的可行性、求解时间和独立 active-face 数值对照已归档，但它们不是 C controller benchmark，也不支持实时能力。没有实际 B/C 对照，因此 **B tracking PASS、C tracking PASS、C 相对 A/B 的可测收益均不支持**。

## 7. Cube/D6/internal-wrench readiness：只读审查

[contact_readiness.json](../../offline/task28_p5_qp_6_r1/evidence/contact_readiness.json) 明确记录：实际 Cube stage 的 COM/密度/惯量未独立验证；0.8 kg、0.12 m 来自 fixture，均匀 Cube 的 `0.00192 kg·m²` 仍是数学假设。冻结名义 grasp 与实际 attachment identity/anchor 不是同一证据。

G3-P2-3 的历史静态几何阻断保持：零载稳态左/右约 `0.441309 / 0.333326 mm`，不能当成通过过的物理 Cube attachment。R1 没有测试 D6，没有部署或自动建立任何 Cube attachment。

contact wrench 仍须区分作用到 Cube 的 f、到机器人的 −f、各 TCP 原点与 Cube 中心的 `r×F` 平移。手指内部约束 residual/lambda 拟合不是 Cube contact wrench，QP/D6 predicted reaction 也不自动是独立观测。可信 reaction 信号、实际锚点、aggregation 和 action/reaction 验证仍缺失。

有限吸附还需要正常拉压、剪切/摩擦、力矩/压力中心、压力/有效面积、密封与 detach/break 动态标定。旧 `1e6` break 参数不是容量。**本次没有 D/E 物理接触单元，也没有内部力闭环 PASS。**

## 8. 测试、复现、计数与停止

历史组实际重跑：**789 + 53 + 5 + 6 + 22 + 54 = 929 passed**，路径互不重叠。R1 新聚焦 **30 passed**，合计 **959 passed**，各组退出码 0。新测试覆盖约束模型、独立活动面、单边释放、拒绝负控、真实源前缀停止和不能用内侧/arm-only 结果绕过资格门。软件 PASS 不等同物理控制 PASS。

实际受控源步为 `19+30+140+20=209`；八次进程（四次发送前中止、四次实际配置运行）各有 2 个 reset 初始化步，共 16 步，已知物理步总计 **225**。正式 B/C 为 0 步。没有把启动错误或完整诊断配置的数量算作额外成功实验。

启动接口失败、源码快照及 evaluator 的一次 `abs(list)` 聚合代码错误均保留；后者只修聚合实现，未改变物理数据或数值门。真正受控源运行的每次拒绝均逐步锁存，独立 evaluator 确认没有失败后的继续推进。Kit 正常退出 0 仍不代表科学通过，以明确结果 JSON 和独立重算为准。

完整复现入口、命令和依赖见 [README](../../offline/task28_p5_qp_6_r1/README.md)。所有代码、数学说明、文献、注册、raw、负控、失败/跳过、哈希及回归位于新 R1 目录，旧档案没有覆盖。提交 SHA、远端 HEAD 和干净工作树由最终交付核验给出。

本次可支持的研究材料是“约束感知源审计、可复核降阶条件、独立负控与适用边界”，不能主张协作跟踪改善、完整约束动力学恢复、真实内载观测或有限吸附控制。任务在 **QP-6-R1** 停止，不继续 ROS、实体 FR3、生产桥、D/E、RL 或无关规划。

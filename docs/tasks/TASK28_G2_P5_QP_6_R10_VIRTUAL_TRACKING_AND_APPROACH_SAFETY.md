# Task28 G2-P5-QP-6-R10：闭环虚拟物体跟踪与接近安全

## 摘要

1. 实际虚拟物体跟踪相对 R9 有改善，但峰值门没有通过：修正过渡后 V10-A 为 **0.578620 mm**，V10-B 为 **0.982210 mm**，V10-C 为 **0.983523 mm**；R9 峰值为 1.034351 mm。
2. 冻结的 0.250 mm 虚拟物体峰值要求没有通过；0.150 mm 相对 TCP 平移要求只有 V10-A 通过，B/C 没有通过。
3. 结果由独立 CPU 重算、独立 Isaac 进程评估和独立 process/cache 审计复现；三次开发单元均 210 个双臂 effort 命令、无接触完成。
4. 原始 Cube 接近路径的连续安全没有建立。区间验证在深度 0/1/2/4 均留下碰撞/支撑未决项；现有 FCL 只有单状态碰撞接口，没有关节 FK 连续 CCD 回调。
5. 没有 Cube 吸附、吸盘命令、D6 命令、ROS actuation 或物理 Cube 位移。
6. 下一技术阻塞点是：在不放宽冻结峰值门的前提下，处理 R8 torque impedance 对启动瞬态和双臂相对漂移的影响，并建立覆盖整条实际关节路径的保守连续几何证明。R10 不宣称 inverse-dynamics HQP。

## 范围和保护

R10 从 `3a81a4a83144d41b07a5bf9aeb79717e2b1c5927` 开始，只新增本任务目录和本报告。场景仍由 `fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py::build_scene` 构建，使用原始双 FR3、L 形四杯工具、`link8`/`side_suction_tcp`、Cube_01、table、原始 TGS 60 Hz、native fingers、原始 joint limits、FCL/ACM、抓取变换和 FR3 USD SHA-256 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`。R8/R9 文件和历史 evidence 没有修改；保护收据在 `offline/task28_p5_qp_6_r10/evidence/preservation.json`。

所有运行均为 CPU NumPy/Isaac headless 原始场景。R10 没有启用 suction、Cube attachment、D6、ROS sink、GPU retry、hardware、internal-wrench control 或 inverse-dynamics HQP runtime。

## R9 独立重建和误差归因

`evidence/r9_reconstruction.json` 从 R9 `V1_RECOVERY2` 和 `V2` 原始 `frames.jsonl` 逐行重建 desired virtual pose、左右 TCP-implied virtual pose、共同误差、相对误差、TCP 位置项、虚拟偏移引起的 orientation lever 项、q/qref、qdot、native dq、QP residual 和 torque components。V1/V2 的独立峰值完全一致并与归档评估一致：

| 指标 | R9 峰值 | 冻结门 |
|---|---:|---:|
| common virtual-object translation | 1.034350888 mm | 0.250 mm |
| relative implied-object translation | 0.160911338 mm | 0.150 mm |
| common object rotation | 0.002604637 rad | 仅作诊断 |
| relative rotation | 0.000022688 rad | 0.001 rad |
| TCP-to-virtual offset | 0.502485010 m | 诊断量 |

峰值位于 physics step 201（relative translation 峰值在 step 202）。step 201 的向量分解为：左 TCP 位置项 1.232456 mm、orientation lever 项 0.331384 mm，向量和为 0.901443 mm；右 TCP 位置项 0.709512 mm、orientation lever 项 0.329841 mm，向量和为 1.034351 mm。该分解使用 `inv(T_VT)[:3,3]` 的正确 TCP 坐标 lever，而不是把原始虚拟坐标平移直接当作 TCP lever。

第一控制步 step 183 的 qdot reference 为零且 QP hard residual 为零，但 measured finite-difference relative twist 的平移约为 `[+0.125,-0.228,+3.373] mm/s`；common implied-object twist 约为 `[-1.318,-0.448,+5.668] mm/s`，角速度 y 分量约 `-15.732 mrad/s`。这是 native drive ownership transition 的实测响应，不是 QP 约束不可行。R9 的 cubic handoff 还在 qref 中积分继承 dq，joint 2 在 step 201 相对 q_hold 产生约 `-0.000862 rad` 的参考漂移；R8 对照参考回到 q_hold。综合逐步数据和 R8/R9 控制对照，主要归因是 **启动 ownership velocity transient + feedforward-only/no pose feedback**，orientation lever 是二级放大因素；QP infeasibility 不是原因。

## R10 数学修正

R10 没有改 R9 源码，新增 `offline/task28_p5_qp_6_r10/qp_math.py`：

- 虚拟运动采用 `s(u)=10u^3-15u^4+6u^5`，在两端位置、速度、加速度满足 C2 rest-to-rest 条件。
- native dq transfer 使用独立五次 zero-net-area handoff：起点速度匹配观测 dq，0.15 s 后 qref 回到 q_hold、速度和加速度归零，不再累积 R9 的 cubic drift。
- measured `T_WL_actual/T_WR_actual` 先构造 implied virtual pose，再用 SE(3) body log 通过实际 pose adjoint 映射到 world spatial error；relative correction 使用 left-TCP body relative frame。
- QP 为 14 维 centralized box/equality QP：hard finite source、±0.12 rad/s、`q+dt*qdot` 在原始 joint limits 内并留 0.02 rad margin、affine relative-pose correction；common object task 和 smoothness 为 soft 项。affine equality 先求 box-feasible seed，再求 shifted homogeneous equality QP，独立 residual/KKT 记录。
- common/relative correction 限制为 0.002 m/s 与 0.01 rad/s；输出始终 `execution_authorized=false` 且 `inverse_dynamics_claim=false`。
- preflight、reference integration、physics、observation 全部明确记录 1/60 s；旧 R9 的错误 `dt_sample_s=1/600` 不在 R10 证据中。

CPU 候选 A/B/C 均 210/210 QP optimal、hard affine residual 在数值容差内、FCL sampled 无 unexpected pair；`evidence/cpu_candidates.json` 不构成 Isaac 精度或执行证明。

## V10-A/B/C 真实隔离运行

每个单元使用新 Isaac 进程、独立 portable cache、原始 scene、R8 exclusive effort adapter、原始 nine-coordinate writes 和 post-step state。三次 process isolation audit 均通过，result status 均为 `NO_CONTACT_R10_VIRTUAL_COOPERATION_COMPLETE`，每个单元 210 commands/arm、无 suction/Cube contact、Cube displacement 为 0。

| 单元 | 反馈 | virtual peak | relative TCP translation peak | relative rotation peak | q tracking peak | native dq peak | effort peak | rate peak | 结论 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| V10-A | quintic/reference only | 0.578620 mm | 0.089301 mm | 0.000011740 rad | 0.000998 rad | 0.010722 rad/s | 28.005 Nm | 4.407 Nm/s | no-contact pass, accuracy fail |
| V10-B | common SE(3) feedback | 0.982210 mm | 0.287302 mm | 0.000055989 rad | 0.000982 rad | 0.010722 rad/s | 28.043 Nm | 3.724 Nm/s | no-contact pass, accuracy fail |
| V10-C | common + relative feedback | 0.983523 mm | 0.212007 mm | 0.000082359 rad | 0.000970 rad | 0.010722 rad/s | 28.047 Nm | 3.755 Nm/s | no-contact pass, accuracy fail |

V10-A 的相对 TCP 峰值通过 0.150 mm，但 common object 峰值仍超过 0.250 mm；B/C 的 common 和 relative peak 均未通过。反馈候选改善了部分最终残差，但没有建立全样本峰值精度。没有删除 startup interval，也没有用 moving-phase 或 final-only 数据制造 PASS。

冻结 selection rule 是 A→B→C，选择第一个满足全部门的 cell，D 只对该 cell 做 fresh exact repeat，FORMAL 只在两个独立 eligible diagnostics 后运行。A/B/C 均不 eligible，因此 `V10-D = SKIPPED_NO_ELIGIBLE_CELL`，`FORMAL = SKIPPED_NO_TWO_ELIGIBLE_CELLS`；没有调参式重跑。

## 原始 Cube 接近路径

R10 重新生成并保留 R9 的 `FREE_HOME → HIGH_OUTER → OUTER_CONTACT → PRE_CONTACT_READY` 三段、51 点 FCL sampled candidate 和 R3 直接 HOME-to-Cube 负例。R10 独立运行 `continuous_safety_audit.py`，在区间 verifier 深度 0、1、2、4 都得到 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`：joint-limit interval 可验证，但 collision/support bounds 仍有未决 AABB/局部支撑规则；输入 typed approach 还出现原始 grasp/closure 和 airborne-clearance 负例。现有 `offline/task28_g1_1/task28_validator/backend/fcl_state.cpp` 只调用 single-state `fcl::collide`；安装的 rigid `continuousCollide` 不提供 articulated FK callback，不能把 API 存在误报为整条双臂关节路径 CCD 证明。`offline/task28_g1_2b_1/adversarial_controls.py` 的 off-grid negative controls 也保留在审计记录中。

因此 R10 不执行 HOME-to-Cube、pre-contact descent、suction closure、Cube attachment、lift、transport 或 D6。`continuous_safety_audit.json` 明确标记 `not_a_certificate=true`，`ORIGINAL_CUBE_PRE_CONTACT_EXECUTION_BLOCKED=PASS`。

## HQP 边界

R10 是 centralized kinematic cooperative QP 后接 R8 gravity-plus-frozen-mass impedance。它没有同一双臂运动状态的完整 mounted-tool `M/c/g` inverse-dynamics hierarchy、接触 wrench 观测、Cube effective inertia、action/reaction convention、真实闭链 torque feasibility 或连续碰撞证明。因此 `FULL_INVERSE_DYNAMICS_HQP_STATUS=NOT_ESTABLISHED_KINEMATIC_QP_PLUS_R8_IMPEDANCE_ONLY`；没有内部力控制或 suction capacity 结论。

## 测试和归档

`evidence/regressions/summary.json` 收据：既有 1051 项（历史 789、QP-4 53、QP-5 5、QP-5-R1 6、QP-5-R2 22、R6/R7/R8/R9 等聚焦集合）全部通过；R10 CPU 聚焦 16 项全部通过，合计 1067 个不重复通过项。R10 聚焦包含 C2/zero-area handoff、SE(3) frame order、relative pose order/sign、14D affine QP、lookahead margin、CPU candidate/reconstruction 和 continuous-safety fail-closed 控制。

机器证据包括：`source_contract.json`、`preregistration.json`、`registrations.json`、`literature_and_source_review.md`、`evidence/r9_reconstruction.json`、`evidence/cpu_candidates.json`、`evidence/evaluation.json`、`evidence/process_isolation_audit.json`、`evidence/continuous_safety_audit.json`、`evidence/selection.json`、`evidence/final_status.json`、`evidence/artifact_hashes.json`。

## 最终状态

- `ORIGINAL_SCENE_PRESERVED = PASS_SCENE_BUILDER_AND_FR3_HASH_VERIFIED`
- `R9_HISTORICAL_RESULTS_PRESERVED = PASS_NO_PRIOR_TRACKED_FILES_CHANGED`
- `R9_ERROR_CAUSE_STATUS = ATTRIBUTED_PRIMARY_OWNERSHIP_VELOCITY_TRANSIENT_AND_FEEDFORWARD_NO_POSE_FEEDBACK; ORIENTATION_LEVER_SECONDARY; NOT_QP_INFEASIBILITY`
- `REFERENCE_TIME_PARAMETERIZATION_VERIFIED = PASS_R10_QUINTIC_C2_AND_60HZ_METADATA_CORRECTED`
- `VIRTUAL_OBJECT_POSE_FEEDBACK_STATUS = IMPLEMENTED_COMMON_AND_RELATIVE_SE3_FEEDBACK_BUT_FROZEN_PEAK_GATE_NOT_PASSED`
- `COOPERATIVE_QP_MATH_STATUS = PASS_CPU_14D_AFFINE_RELATIVE_QP_LOOKAHEAD_BOX_AND_NEGATIVE_CONTROLS`
- `VIRTUAL_COOPERATION_DIAGNOSTIC_STATUS = V10_A_B_C_COMPLETED_NO_CONTACT_AND_PROCESS_ISOLATION_PASS_ACCURACY_GATE_FAIL`
- `FORMAL_VIRTUAL_COOPERATION_STATUS = SKIPPED_NO_TWO_INDEPENDENT_ELIGIBLE_DIAGNOSTIC_CELLS`
- `RELATIVE_TCP_ACCURACY_STATUS = V10_A_PASS; V10_B_V10_C_FAIL_FROZEN_0_150_MM_PEAK`
- `ORIGINAL_CUBE_APPROACH_CONTINUOUS_SAFETY_STATUS = CONTINUOUS_SAFETY_NOT_ESTABLISHED`
- `ORIGINAL_CUBE_PRE_CONTACT_EXECUTION_BLOCKED = PASS_NO_PHYSICAL_APPROACH_OR_CUBE_COMMAND`
- `FULL_INVERSE_DYNAMICS_HQP_STATUS = NOT_ESTABLISHED_KINEMATIC_QP_PLUS_R8_IMPEDANCE_ONLY`
- `CUBE_D6_NOT_TESTED = PASS_NOT_TESTED`

归档收尾：R10 实现与证据已提交为 `b8928154790b317c34895deb7eb36641f6b8486d`（`Add R10 virtual tracking and approach safety evidence`），并已推送到 `origin/side-suction-palletizing`；推送核对时 remote HEAD 与该 SHA 一致。该提交之后只会增加本报告的收据元数据，不改变任何 R10 运行结果；当前工作树在收据提交前保持 clean。

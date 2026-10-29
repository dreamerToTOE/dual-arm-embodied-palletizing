# Task28 G2-P5-QP-6-R9 — Cooperative QP and safe bilateral suction approach

## 中文结论

两只真实 Isaac 双 FR3 在原始 60 Hz TGS 场景中执行了一个很小的**虚拟物体公共任务空间运动**。参考由新的集中式 14 自由度运动学 QP 生成，再经原 R8 重力加阻抗控制器和原始全 9 坐标 effort 接口发送；这不是 Cube 的运动，也不是逆动力学 HQP。

FREE_HOME 的左右 TCP 相对平移峰值为 `0.1609113381 mm`，相对旋转峰值为 `2.2687763e-5 rad`。相对旋转通过冻结门限，但冻结的 `0.150 mm` 相对平移门限略微失败；虚拟物体跟踪峰值 `1.0343509 mm` 也超过预注册的 `0.250 mm`。因此 R9 记录为“无接触执行完成、精度门失败”，没有把它升级为合作操作通过。

针对实际 Cube_01，原始名义吸盘姿态的双臂端点 IK 在 CPU 上可行；三个同步 51 点边均通过有限采样 FCL。但这不是连续碰撞安全证明，且 R3 的 HOME 直插负例仍在轨迹比例 `0.92` 发生左吸盘工具 3、5 与 Cube 碰撞。因此没有把长 HOME 到抓取的路径塞进 R8 小运动控制器，实际 Cube 接近、吸附、D6、搬运均跳过。

两次成功虚拟单元没有 Cube 位移、吸盘命令、D6 命令或未预期碰撞。仍被阻塞的是冻结精度门、连续碰撞/动态接近资格、安装工具与物体的同状态完整动力学、接触 wrench 分配和真实吸附执行。

## Scope and preservation

The task started at `94a0ace265ecc60d8cb0e4935f05459323a99365` on branch `side-suction-palletizing`. All R9 implementation and evidence are new files under `offline/task28_p5_qp_6_r9/`; the exact requested report is this file. The original scene entry remains:

`fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py::build_scene`

The source contract retains the original fixture and FR3 asset hash, CPU NumPy backend, 60 Hz (`dt=1/60 s`) TGS step, native fingers, mimic constraints, original L-tools and TCP frames, joint limits, ACM, Cube pose `(0.65, 0, 0.26)`, suction parameters and the 1 mm nominal gap. No source, fixture, USD, ACM, tool shape or Cube pose was edited. R8's ownership transfer, source gate, exclusive effort adapter, process isolation and full nine-coordinate command policy were reused. `evidence/frozen_preservation_verification.json` records the read-only R8 archive check.

The three frame types are deliberately separate:

| Typed state | Meaning | R9 use |
| --- | --- | --- |
| `ATTACHED_PHYSICAL_CUBE_CANDIDATE` | frozen `T_WCube` from the original scene | endpoint/approach audit only |
| `PRE_CONTACT_ORIGINAL_CUBE` | exact G1 `T_Cube_GL`, `T_Cube_GR`, 1 mm gap | original Cube approach audit only |
| `FREE_HOME_VIRTUAL_COOPERATION` | midpoint world frame and measured FREE_HOME TCP offsets | no-contact QP and Isaac cells only |

The virtual frame is constructed from the actual initial left/right `side_suction_tcp` transforms. It never promotes to the physical Cube. Applying the original nominal grasps to FREE_HOME is retained as a negative control: the implied object centers differ by `0.945217679 m`; absolute grasp errors are `0.688190575 m` and `0.535122703 m`.

## Model and controller

The QP uses the world geometric twist convention `[v; omega]`. For a fixed virtual offset `r_i`, the object-to-TCP map is

`H_i = [[I, -skew(r_i)], [0, I]]`.

With the two source-verified 6x7 Jacobians stacked as `J` and `H = [H_L; H_R]`, the hard relative compatibility matrix is `C = (I - H H^+) J`. Each solve uses the 14-joint decision `qdot` and enforces:

`C qdot = 0`, `-0.12 <= qdot <= 0.12`.

The strict-SPD objective is

`min 1/2 || W_t (J qdot - H v_V) ||^2 + 1/2 || W_s (qdot - qdot_previous) ||^2`,

where the common virtual-object task and joint smoothness are soft. Finite source values, equality feasibility, independent primal/dual KKT reconstruction, joint limits and geometry gates are checked before a write. The solver result is marked `execution_authorized=false`; it contains no `M(q)qdd`, `C(q,dq)dq`, `g(q)`, contact wrench, suction capacity or physical attachment equality.

The runtime chain is:

`virtual common translation -> bilateral TCP goals -> centralized 14-DOF QP -> q_ref/dq_ref -> R8 g(q)+Kp(q_ref-q)+Kd(dq_ref-dq_end_step) -> original full 9-coordinate effort writes`.

The native finger policy is preserved. Isaac's applied effort is treated as the command buffer documented by the R8 audit, not as a torque sensor. Every accepted write passed the R8 absolute and rate source gates; no saturation or fail-closed rejection occurred after the successful cells began.

## CPU audits

`audit_cpu.py` independently re-evaluates archived R8 D3, D4, FORMAL and MOVING evidence. All four remain PASS; FORMAL has 60/60 control steps and MOVING has 210/210. The preserved R8 metrics are static TCP peaks `0.692523082 mm` (left) and `0.396352303 mm` (right), moving peaks `0.118970701 mm` and `0.064836384 mm`, and final errors `0.021257624 mm` and `0.019716337 mm`. R8 was a `+0.002 rad` joint-2 reference, not Cube translation.

The CPU virtual QP plan has 211 sampled states, no unexpected FCL collision, finite source/KKT checks, and Jacobian finite-difference errors of `2.0113e-9` (left) and `1.7524e-9` (right). The initial equality matrix has 12 rows and numerical rank 6; a 12x14 row representation is not treated as grasp closure. The QP feasibility and constraint evidence is in `evidence/virtual_qp_cpu.json`.

## Isaac experiment matrix

The preregistration froze a world-X virtual translation of `0.0005 m` over `3.5 s` and 210 control steps, with virtual tracking limits of `0.25 mm`, relative translation `0.15 mm`, relative rotation `0.001 rad`, joint tracking `0.02 rad`, native speed `0.3 rad/s`, zero unexpected FCL collision, unchanged Cube, zero suction and zero D6 commands.

| Cell | Result | Physics/control work | First gate or measured result |
| --- | --- | --- | --- |
| V1 | rejected before effort | 0 control writes | source-contract `state` key missing at CONTEXT; no command issued |
| V1_RECOVERY | rejected before effort | 180 native startup steps, 0 control writes | source-contract R8 research effort key missing at LEVEL1_GATE; no command issued |
| V1_RECOVERY2 | executed | 210 commands per arm, 420 full robot writes, 390 steps including startup | no-contact pass; virtual tracking `1.034350888 mm`, relative translation `0.160911338 mm` |
| V2 | exact independent repeat | 210 commands per arm, 420 full robot writes, 390 steps including startup | same no-contact result and same frozen accuracy-gate failure |

For both successful cells, the independent evaluator reports:

- Cube displacement `0.0 m`, Cube unchanged, suction `0`, D6 `0`;
- no unexpected collision;
- QP hard relative residual peak `2.6089e-19`;
- joint tracking peak `0.000892879 rad`, QP qdot peak `0.000675417 rad/s`, and measured native `dq` peak `0.010769423 rad/s`;
- virtual object tracking peak `0.001034351 m`;
- relative TCP translation peak `0.000160911 m` and relative rotation peak `2.2688e-5 rad`;
- effort peak `28.0043 Nm`; command-rate recomputation from successive 60 Hz writes `1.86015 Nm/s`;
- clean isolated process exit, no competing worker, no KVDB warning and portable cache cleanup.

The no-contact and process gates pass, but the preregistered virtual accuracy gate does not. That failure is preserved in `evidence/evaluation.json`; thresholds were not loosened after execution.

## Original Cube approach audit

The physical audit uses the original Cube pose and exact G1 nominal grasp transforms. Endpoint IK succeeds for `HIGH_OUTER`, `OUTER_CONTACT` and `PRE_CONTACT_READY` on both arms, with endpoint translation errors below `1.0e-10 m` and rotation errors below `1.0e-9 rad` in the recorded solutions. The synchronized candidate uses these standoffs:

- `HIGH_OUTER`: left/right y offsets `-0.161/+0.161 m`, z `0.54 m`;
- `OUTER_CONTACT`: left/right y offsets `-0.161/+0.161 m`, z `0.26 m`;
- `PRE_CONTACT_READY`: exact original nominal 1 mm gap.

Each of the three bilateral joint-linear edges has 51/51 sampled FCL states with no unexpected collision. This is a finite screen only; the evidence explicitly sets `continuous_safety=NOT_ESTABLISHED_FINITE_SAMPLING_ONLY`. The archived R3 direct HOME-to-nominal negative is preserved unchanged: first collision at fraction `0.92`, shapes `left_fr3_side_suction/3|Cube_01/box` and `left_fr3_side_suction/5|Cube_01/box`. No ACM pair was suppressed. Because there is no continuous certificate or qualified dynamic/torque margin, physical pre-contact execution is `SKIPPED`.

Task24's historical sequence remains the planning reference: high pre-contact, outer descent, synchronized contact microtiming, then common lift/transport. R9 does not claim that this CPU candidate proves the full sequence or authorizes suction.

## Inverse-dynamics/HQP readiness

The full object-centered inverse-dynamics controller remains blocked. A valid future formulation would require a same-state bilateral model, mounted-tool inertia, native finger coupling, object inertia, acceleration-consistent timing, fixed physical grasp transforms, and a measured or explicitly estimated contact wrench. The available R3/R8 `M/c/g` evidence comes from a different qualification scope and does not establish this R9 moving cooperative state. The command buffer is not a torque sensor. `computed_torque_readiness.json` and `cooperative_readiness.json` record these boundaries. No IDHQP runtime, internal-force control or contact-wrench equality was executed.

The literature review compares the Dio et al. IROS 2023 cooperative HQP framework, Stack-of-Tasks TSID, MoveIt 2 dual-arm planning and MoveIt Pro multi-tip Cartesian planning. They provide task-priority, multi-tip and inverse-dynamics frameworks; they do not qualify this mounted L-tool, suction mode, object inertia, contact wrench or installed FR3 runtime. No external dependency or performance transfer is claimed. The source review is in `literature_and_source_review.md`.

## Required final statuses

| Status key | R9 value |
| --- | --- |
| `ORIGINAL_SCENE_PRESERVED` | `PASS` |
| `R8_TORQUE_BASELINE_PRESERVED` | `PASS` |
| `VIRTUAL_OBJECT_FRAME_VALIDATED` | `PASS_TYPED_FREE_HOME_VIRTUAL_COOPERATION` |
| `CENTRALIZED_14DOF_QP_MATH_STATUS` | `PASS_CPU_KKT_FD_FCL` |
| `QP_TO_TORQUE_REFERENCE_INTERFACE_STATUS` | `PASS_FULL_9DOF_R8_SOURCE_GATE` |
| `NO_CONTACT_VIRTUAL_COOPERATION_STATUS` | `EXECUTED_V1_RECOVERY2_V2_NO_CONTACT_ACCURACY_GATE_FAIL` |
| `ORIGINAL_CUBE_GRASP_ENDPOINT_FEASIBILITY` | `PASS_CPU_IK_BILATERAL_ENDPOINTS` |
| `ORIGINAL_CUBE_APPROACH_PATH_STATUS` | `CPU_SAMPLED_CANDIDATE_NO_CONTINUOUS_PROOF` |
| `ORIGINAL_CUBE_PRE_CONTACT_READY_OR_BLOCKED` | `BLOCKED_PRE_CONTACT_EXECUTION_NOT_AUTHORIZED` |
| `FULL_INVERSE_DYNAMICS_MODEL_STATUS` | `BLOCKED_UNQUALIFIED_MOUNTED_TOOL_OBJECT_CONTACT_MODEL` |
| `COOPERATIVE_IDHQP_RUNTIME_STATUS` | `SKIPPED_KINEMATIC_QP_ONLY_NO_IDHQP` |
| `PHYSICAL_SUCTION_D6_NOT_TESTED` | `true` |

The machine-readable copy is `evidence/final_status.json`. R9 stops here; no Cube suction, D6 closure, object transport, production ROS, internal wrench control or hardware was started.

## Tests and delivery

The prior non-overlapping suites were rerun without physics launch: 789 historical, QP-4 53, QP-5 5, QP-6/R1 30, QP-6/R2 12, QP-6/R3 7, QP-6 54, P5-R1 6, P5-R2 22, R4 focused 12, R5 focused 13, R6 focused 12, R7 focused 4 and R8 focused 15, totaling **1034 passed**. The new R9 focused suite has **17 passed**. The total unique software checks represented by this task are therefore **1051 passed**. Logs and JSON results are under `evidence/regressions/` and `tests/test_r9.py`.

The final delivery commit SHA and remote HEAD are reported by the delivery command after the R9-only commit. The worktree is required to be clean at delivery; `evidence/delivery_metadata.json` records the starting commit and the R9-only scope. `evidence/artifact_hashes.json` is generated after all files are finalized and excludes itself from its own hash manifest.

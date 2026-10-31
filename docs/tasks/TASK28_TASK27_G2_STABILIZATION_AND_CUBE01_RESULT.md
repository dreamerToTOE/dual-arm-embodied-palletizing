# TASK28 Task27 G2 Stabilization and Cube_01 Result

## Outcome

This iteration is **PARTIAL / BLOCKED**. The live bilateral 28-variable QP now takes over the original Task27 GUI scene with a measured native-drive settle and a runtime gravity handoff. A no-contact hold ran for 270 consecutive QP solves with small measured TCP drift and no torque saturation. The requested 600-step hold was not achieved: longer runs developed bilateral relative TCP drift and stopped at the unchanged fail-closed local envelope. Common motion was physically observed, but common, relative, and combined experiments were not accepted because relative drift grew to approximately 19--20 mm.

A single Cube_01 integrated attempt was run in the original GUI scene. MoveIt started, but the torque run stopped before a valid Cube cycle. The current live-QP reference still does not consume the Task27 ROS joint trajectory, so no grasp, lift, transport, or release is claimed.

No USD, robot asset, L-shaped tool, Surface Gripper, D6, Cube geometry, feed/rail configuration, or scene physics parameter was modified.

## Baseline and scene identity

The task named `6fa4ae681ba83669b59dc5567f4c7673a1e3424a` as its baseline. The checked-out branch already contained the prior live-QP implementation at `269955e`; this report records only changes made on top of that implementation.

- Branch: `side-suction-palletizing`
- Scene entry: `isaac/scripts/task27_five_cube_center_insert_scene.py`
- Scene SHA256: `74eb11999b799681ec93b124d58b854c32018f1e23219a5ff07f7918fd201801`
- Original Task26 scene SHA256: `43aa7c2eb6a5667da5e94eaaef347229fe6178ec58d6a10d12dddf14ba2aa9e7`
- Original feed bridge SHA256: `0f7123657ea8422cf0efd468e7bcd59624e144ae8c9ca315a82af57576b68dc6`
- Runtime roots: `/World/left_fr3`, `/World/right_fr3`
- TCP paths: `/World/{left,right}_fr3/fr3_link8/side_suction_tool/side_suction_tcp`

## Changes

Only the execution bridge and GUI runner were changed:

1. `isaac/scripts/task28_task27_one_pass.py` can instantiate the original feed bridge first, let its authored position drives settle, and then pass the same bridge to the torque owner. The settle receipt requires ten consecutive samples with `max |dq| <= 0.05 rad/s`; the observed receipt was 10 samples with `max |dq| = 0.01070 rad/s`.
2. `isaac/scripts/task28_task27_torque_bridge.py` clears only the seven arm velocities at the runtime handoff and writes one measured PhysX gravity bootstrap before the first QP callback. The first callback is explicitly logged as `HANDOFF_GRAVITY_BOOTSTRAP`. Finger modes/gains and the original feed/gripper/rail owner remain unchanged.
3. Live-QP simulation effort limits are recorded and applied as a simulation-only profile: absolute `[87,87,87,87,12,12,12] N*m`; rate limits remain `[2400,600,240,2400,240,240,240] N*m/s`. The historical research profile `[35,12,8,35,4,4,4] N*m` remains recorded for comparison and is not overwritten.
4. The previous universal 10 mm local-sweep stop is replaced by a bounded 20 mm per-experiment envelope. This is still a fail-closed diagnostic stop, not an acceptance threshold.
5. Desired relative world-frame feedforward is transported through the same measured relative-frame map used by the relative Jacobian. A higher relative-gain diagnostic was run and rejected because it did not improve the measured drift; the final bridge keeps the prior R12/R11 relative gains.
6. `offline/task28_task27_live_qp/metrics.py` summarizes actual TCP feedback, task-error diagnostics, solver status, residuals, speed, saturation, readback, and writes without inferring PASS.

## P0/P1 evidence

The earlier 200-solve run failed after physical drift because the unchanged research torque box became infeasible (`TORQUE_ACCELERATION_BOX_INFEASIBLE`); measured J2 gravity had reached approximately `-37.4 N*m` left and `+15.4 N*m` right against a `12 N*m` J2 limit. The new receipt shows that the initial mode-transition velocity, rather than an initially infeasible settled posture, was the immediate startup failure mechanism.

Successful no-contact hold receipt:

`offline/task28_task27_integration/evidence/g2_velocity_reset_20261031_2310/`

- 270 `QP_OPTIMAL` solves and 270 actual torque writes;
- 0 arm position writes, 0 fail-closed events, 0 torque saturation frames;
- actual common TCP displacement max `0.0172 mm`, P95 `0.0117 mm`;
- actual relative TCP displacement max `0.6948 mm`, P95 `0.5078 mm`;
- maximum measured arm speed `0.00368 rad/s`;
- mean QP solve time `13.18 ms`, P95 `17.15 ms`;
- GUI scene and final viewport screenshots are stored with the receipt.

The longer hold attempt is preserved at `offline/task28_task27_integration/evidence/g2_final_hold_20261031_230517/`. It produced 400 `QP_OPTIMAL` solves and then stopped fail-closed at 19.54 mm relative drift. Therefore the 600-step hold target is **not met**.

## P2 physical motion results

All values below come from actual PhysX TCP feedback. The `common_error_translation_m` and `relative_error_translation_m` fields are controller diagnostics; actual displacement is reported separately.

| Experiment | QP result | Actual common displacement | Actual relative displacement | Maximum `|dq|` | Saturation | Result |
|---|---:|---:|---:|---:|---:|---|
| Hold, stable receipt | 270 optimal | 0.0172 mm | 0.6948 mm | 0.00368 rad/s | 0 | partial hold evidence |
| Common | 400 optimal, then local stop | 3.024 mm | 19.928 mm | 0.1092 rad/s | 0 | **FAIL** |
| Relative | 431 optimal plus bootstrap, then local stop | 0.528 mm | 19.303 mm | 0.1055 rad/s | 0 | **FAIL** |
| Combined | 432 optimal, then local stop | 3.061 mm | 19.871 mm | 0.1067 rad/s | 0 | **FAIL** |

Evidence:

- Common: `offline/task28_task27_integration/evidence/g2_velocity_reset_common_20261031_2320/`
- Relative (after relative-frame transport correction): `offline/task28_task27_integration/evidence/g2_velocity_reset_relative_fixed_20261031_2340/`
- Combined: `offline/task28_task27_integration/evidence/g2_final_combined_20261031_230642/`

The common task itself moved the TCPs, and its logged common translation error stayed below `0.050 mm`, but the two TCPs separated. The frame-transport correction and a higher-gain relative diagnostic did not remove that separation. The evidence supports a remaining relative-task/model/feedback inconsistency; it does not prove whether the remaining source is the PhysX Jacobian transport, relative acceleration model, or bilateral coupling. No torque saturation or QP infeasibility explains these stops.

The higher-gain diagnostic is retained at `offline/task28_task27_integration/evidence/g2_gain_relative_20261031_231411/`: 459 optimal solves and 19.49 mm relative displacement before the same fail-closed stop. Its gains were not promoted.

A separate posture-weight diagnostic at `offline/task28_task27_integration/evidence/g2_relative_posture10_20261101_0030/` used a simulation-only joint-posture weight of 10.0. It reached 481 optimal QP solves but still reached 19.73 mm relative displacement with no saturation. The final bridge therefore retains the validated posture weight of 0.2.

## P3 Cube_01 attempt

Evidence: `offline/task28_task27_integration/evidence/g2_cube_attempt_20261031_231020/`

- Original Task27 GUI scene loaded; `headless=false`.
- MoveIt exited normally (`0`); the Task27 executor was started with `first_batch:=1`, `max_batches:=1`, and scale `3.0`.
- Torque bridge recorded 1 handoff bootstrap, 397 `QP_OPTIMAL` solves, and 398 torque writes with 0 arm position writes.
- The live run stopped at the 20 mm local envelope; the executor was interrupted (`-2`) and produced no valid completion receipt.
- The live bridge currently captures ROS qref but its live-QP reference remains the internal local virtual trajectory. Consequently, the executor's HOME/PRE-CONTACT/CONTACT/LIFT/TRANSPORT/PLACE/RELEASE trajectory was not actually tracked by the torque QP.

No Cube displacement, suction engagement, lift, transport, or release is claimed. This is a technical blocker, not a successful physical execution.

## Safety and scope

All runs were Isaac Sim GUI simulations. No real FR3 command, hardware tau_J, D6 hardware, or physical robot execution was used. The original HOME-to-Cube continuous safety proof remains `CONTINUOUS_SAFETY_NOT_ESTABLISHED`; the discrete local-sweep stop is not a continuous collision proof. No V10/formal acceptance or five-Cube run is authorized by this result.

## Verification

- Focused QP/T0/R8 regression: **36 passed**.
- Python compilation for the changed Isaac scripts and metrics module: passed.
- `git diff --check`: passed.
- The previously recorded repository regression (789 historical + prior Task28 suites) remains unchanged; it was not rerun in full during this GUI iteration.

## Remaining blocker and next step

The startup handoff is now bounded and repeatable. The remaining blocker is bilateral relative TCP drift despite `QP_OPTIMAL`, no torque saturation, and small common-task error. Before another Cube attempt, the live bridge must consume the Task27 trajectory as a verified task-space reference (or establish a separately validated qref-to-TCP mapping) and resolve the relative dynamics/feedback discrepancy. Continuous safety remains unestablished, so Cube contact and transport stay blocked.

## Delivery

Final commit and remote status are recorded in the delivery commit below. The selected GUI receipts are committed with this report.


# TASK28 Task27 P2 Live Cooperative Torque QP Result

## Outcome

This iteration is **PARTIAL / BLOCKED**, not `FULL_PASS`. A real no-contact live bilateral acceleration/torque QP was implemented and connected to the original Task27 Isaac Sim scene. The GUI run solved the 28-variable QP from current PhysX feedback and wrote the resulting 14 arm torques through the existing exclusive torque sink with exact canonical readback. The controller did not achieve a stable hold: the physical state drifted until the measured dynamics and unchanged torque box became infeasible, and the runtime stopped fail-closed. No Cube contact, suction, D6, or approach execution was performed.

The original Task27 scene, FR3 assets, L-shaped tools, Surface Grippers, D6, Cube geometry, rail/feed configuration, collision geometry, and safety thresholds were not modified.

## Sources and runtime identity

- Baseline: `f68105a8d19c7127c381c33362e523f02c538274`
- Scene entry: `isaac/scripts/task27_five_cube_center_insert_scene.py`
- Scene SHA256: `74eb11999b799681ec93b124d58b854c32018f1e23219a5ff07f7918fd201801`
- Original Task26 scene SHA256: `43aa7c2eb6a5667da5e94eaaef347229fe6178ec58d6a10d12dddf14ba2aa9e7`
- Original feed bridge SHA256: `0f7123657ea8422cf0efd468e7bcd59624e144ae8c9ca315a82af57576b68dc6`
- Runtime roots: `/World/left_fr3`, `/World/right_fr3`
- TCP frames: `/World/{left,right}_fr3/fr3_link8/side_suction_tool/side_suction_tcp`

The runtime source gate uses the actual PhysX articulation: exact nine-DOF order, `fr3_link8` Jacobian row, transported TCP Jacobian, finite/symmetric full 9x9 mass, full 9-vector C/G, runtime position limits, and fresh timestamps. The historical single-Cube fixture FK is not used as the live dynamics or Jacobian source.

## Controller implementation

New source:

- `offline/task28_task27_live_qp/torque_qp.py`
- `offline/task28_task27_live_qp/tests/test_torque_qp.py`
- live integration in `isaac/scripts/task28_task27_torque_bridge.py`
- GUI options in `isaac/scripts/task28_task27_one_pass.py`

The QP decision is `[qdd_left(7), qdd_right(7), tau_left(7), tau_right(7)]`. It enforces the measured dynamics relation `tau = M qdd + h`, common and caller-verified relative TCP acceleration objectives, joint position/velocity look-ahead, torque limits, torque-rate limits, numerical interior margins, and fail-closed infeasibility/non-convergence. Finger accelerations are held at zero in the reduced arm dynamics; finger values remain under the original bridge policy and are never commanded by the QP.

The live bridge obtains PhysX Jacobians for `fr3_link8`, transports them to the measured TCP, constructs the left-TCP relative Jacobian, reads M/C/G from the same physics step, computes finite-difference `Jdot*dq`, and writes only the seven arm torques per side. Immediate and post-step applied readback remain strict canonical command-buffer checks.

## Live GUI evidence

The formal no-contact run used Isaac Sim 4.5 GUI, the original Task27 scene, and `--live-qp --experiment hold`.

### First successful live solve run

Evidence: `offline/task28_task27_integration/evidence/p2_live_qp_hold_20261031_0630/`

- `200` `QP_OPTIMAL` solves;
- `200` actual torque writes;
- `0` arm position writes;
- applied command readback matched the float32 canonical command;
- solver status distribution: `QP_OPTIMAL=200`, then `QP_INFEASIBLE=1`;
- fail-closed reason: `TORQUE_ACCELERATION_BOX_INFEASIBLE`.

The infeasibility was physical and preserved the safety box. At the stop, measured gravity included approximately `-37.4 N*m` on left J2 and `+15.4 N*m` on right J2 while the existing J2 torque bound remained `12 N*m`. Earlier in the same run, the arms developed large joint/TCP drift and task residuals; this is a tracking failure, not a successful stationary hold.

### Runtime local-sweep stop

Evidence: `offline/task28_task27_integration/evidence/p2_live_qp_hold_posture_20261031_0700/`

After adding a conservative joint-posture/nullspace term, the runtime local safety gate stopped after `4` valid QP solves when the measured local swept displacement exceeded `10 mm`. This confirms the independent fail-stop path; it does not pass the hold experiment.

Earlier conversion and numerical-boundary failures are preserved in `p2_live_qp_hold_20261031_0500/`, `0520/`, `0540/`, and `0600/`. The first three stopped before any write on invalid TCP matrix conversion. The `0600` run produced `186` QP solves before the exact torque boundary rejected a tiny over-limit value; the subsequent interior-margin change removed that numerical boundary failure.

No common, relative, or combined motion run was promoted to acceptance after the hold experiment showed physical instability. Running those trajectories would not establish a safe milestone while the same dynamics/torque feasibility issue remains.

## P2-D pre-contact gate

New independent entry:

- `ros_ws/src/fr3_dual_palletize/src/task27_pre_contact_gate.cpp`
- CMake target: `task27_pre_contact_gate`

The gate reuses the original Task27 executor and scene-facing geometry, restores Cube collision geometry for the check, validates synchronized HIGH_PRE_CONTACT trajectories with the Cube present, executes only bilateral HIGH_PRE_CONTACT, waits for joint settling, checks TCP residuals, Cube drift, measured TCP/Cube clearance, effort bounds, stop speed, and suction-open state, then exits before OUTER_DESCENT, CONTACT, or Surface Gripper activation. Failure is fail-closed. The source and wrapper were syntax-checked and manually linked against the existing ROS target contract. Continuous swept safety remains `CONTINUOUS_SAFETY_NOT_ESTABLISHED`; sampled FCL is not a continuous proof.

The gate was not used to authorize Cube contact because the live QP hold gate is not yet stable.

## Acceptance status

| Requirement | Result |
|---|---|
| Actual bilateral 14-DOF torque QP | **Implemented and physically invoked** |
| Live solve count | **200 valid solves in the main receipt** |
| Actual M/C/G/J feedback | **Recorded from Task27 PhysX runtime** |
| Common/relative objective | **Implemented and logged; residuals became large during drift** |
| Torque/rate/position/velocity constraints | **Enforced; infeasibility stops the run** |
| Stable stationary hold | **FAIL** |
| Common/relative/combined accepted motion | **NOT ACCEPTED** |
| Pre-contact-only gate | **Implemented; continuous safety still unestablished** |
| Cube contact or transport | **NOT RUN** |

## Regression

- Live QP tests: **9 passed**.
- T0/R8 torque interface and adapter tests: **25 passed**.
- Combined focused run: **34 passed**.
- Modified Python sources: `py_compile` passed.
- `git diff --check`: passed before evidence staging.

## Remaining blocker

The unchanged Task27 torque box is not dynamically feasible after the live controller leaves the settled local region. The next safe step is to resolve the measured dynamics/torque feasibility and nullspace stabilization without changing limits, then repeat hold/common/relative/combined no-contact runs. Cube contact remains prohibited until those runs pass and the pre-contact gate has an execution receipt.

## Delivery

- Final commit: the commit returned by `git rev-parse HEAD` at delivery
- Remote: `origin/side-suction-palletizing`
- Worktree: clean after push

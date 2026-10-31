# TASK28 Task27 G2 Recovery and First-Cube Result

## Result

This iteration is **PARTIAL / BLOCKED**, not `FULL_PASS`. The applied-effort gate was repaired with an evidence-backed Isaac float32 command-buffer contract. The same torque bridge then passed stationary hold and a bounded 5 mrad bilateral joint motion in the original Task27 GUI scene. A stale reference-velocity bug was fixed and the return residual disappeared. The live Task27 bridge still uses the existing R8 impedance-plus-gravity torque law and records `solve_count=0`; the R12 replay is explicitly diagnostic-only. The representative pre-contact gate, live cooperative torque QP, and Cube_01 contact/lift/transport/release were therefore not authorized or run.

No scene builder, USD asset, FR3 model, L-shaped tool, Surface Gripper, D6, Cube, table, truck, rail, collision geometry, ACM, suction event, or release event was modified.

## Baseline and original sources

- Repository: `dreamerToTOE/dual-arm-embodied-palletizing`
- Branch: `side-suction-palletizing`
- Baseline: `b561c9d46175a781fdfad74d1114af01cd97b41e`
- Scene entry: `isaac/scripts/task27_five_cube_center_insert_scene.py`
- Scene root: `/World/Task27`
- Arm roots: `/World/left_fr3`, `/World/right_fr3`
- Task27 scene wrapper SHA256: `74eb11999b799681ec93b124d58b854c32018f1e23219a5ff07f7918fd201801`
- Original Task26 scene SHA256: `43aa7c2eb6a5667da5e94eaaef347229fe6178ec58d6a10d12dddf14ba2aa9e7`
- Original feed/Surface Gripper bridge SHA256: `0f7123657ea8422cf0efd468e7bcd59624e144ae8c9ca315a82af57576b68dc6`

The torque bridge owns only the seven arm DOFs per side. The two finger values are carried forward from the existing articulation buffer and remain under the original bridge's finger policy; no finger torque controller was added.

## Gate status

| Gate | Result | Evidence and limitation |
|---|---|---|
| P0 readback semantics | **PASS** | Full 9-DOF immediate and post-step logs, raw/canonical/applied values, projected effort separately labeled, exact canonical comparison, and negative ULP rejection. |
| P1-A stationary hold | **PASS** | `g2_recovery_hold_20261031_0330`: 926 GUI physics writes, no latch, 0 arm position writes, exact canonical readback. |
| P1-B bounded motion | **PASS with startup transient** | `g2_recovery_small_motion_decay_20261031_0420`: bilateral ±5 mrad J1 target and return, no latch, 0 position writes, stable return after the reference-velocity fix. |
| P1-C representative pre-contact | **BLOCKED** | No isolated stop-before-contact entry exists in the current Task27 executor, and the original HOME→Cube path remains `CONTINUOUS_SAFETY_NOT_ESTABLISHED`. The integrated executor cannot be used as a pre-contact-only proof. |
| P2 live cooperative torque QP | **BLOCKED** | Bridge still records `NOT_IMPLEMENTED_TASK27_LIVE_QP` and `solve_count=0`. The non-actuating replay below is R12 velocity QP only and remains unauthorized. |
| P3 bilateral attachment/lift/transport/release | **NOT RUN** | No suction or Cube motion was authorized after P1-C/P2 failed. Completed physical Cube count is `0/1`. |
| Safety | **PASS for executed diagnostics; P3 not admitted** | Existing torque/rate/position ownership gates were retained. No scene or safety threshold was relaxed. |

## P0: applied-readback diagnosis and repair

The original failure was reproducible at physics step `1960`. The left J4 request was `33.87004262397471 N*m`; Isaac returned `33.87004089355469 N*m`, a raw difference of `-1.730420024e-6 N*m`. `float(np.float32(requested))` is exactly the returned value, so this was float32 storage quantization rather than a stale, reordered, or rejected command. The previous `1e-6` comparison therefore had the wrong representation semantics.

The repaired boundary now:

1. checks finite, absolute, rate, cursor, frame, and unit constraints on raw float64 controller values;
2. canonicalizes the seven arm values to the Isaac float32 command-buffer representation;
3. writes a complete nine-DOF buffer, preserving the two existing finger entries;
4. records raw requested, canonical requested, immediate applied, post-step applied, projected effort, full DOF order, timestamps, step, and per-joint differences; and
5. accepts only exact canonical applied readback. A stale, reordered, or one-ULP-different value fails closed.

The focused interface and adapter tests contain canonical round-trip and over-ULP negative cases. The original failing run is retained at `offline/task28_task27_integration/evidence/g2_recovery_first_cube_20261031_0300/`; the successful hold receipt is at `offline/task28_task27_integration/evidence/g2_recovery_hold_20261031_0330/`.

## P1: torque tracking

The bridge now computes `dq_ref` from the ROS trajectory timestamp difference when that difference is valid instead of assuming one Isaac step. A second issue was found in the bounded-motion run: the last finite-difference velocity remained latched after the target was captured. The bridge now decays an old `dq_ref` on the Isaac timeline with a 50 ms time constant until a new target arrives. This is a reference-lifecycle correction; no torque, rate, velocity, or safety limit was changed.

The successful stationary run (`g2_recovery_hold_20261031_0330`) recorded:

- 926 torque writes and 925 post-step checks;
- 0 arm position writes and no latched error;
- peak takeover transient `|q-q_ref|`: left `0.0587006 rad`, right `0.0584259 rad`;
- peak `|dq|`: left `0.635469 rad/s`, right `0.632419 rad/s`;
- final `|q-q_ref|`: `5.3406e-5 rad` on both arms;
- no torque-rate-limited frames; and
- zero canonical immediate-readback difference.

The bounded-motion run (`g2_recovery_small_motion_decay_20261031_0420`) injected a bilateral 5 mrad J1 target and returned to the captured home target through the same patched sink, without ROS, suction, or Cube commands:

- 516 torque writes, target captures `2/2`, 0 arm position writes, no latch;
- outward segment maximum `|q-q_ref|`: left `0.0050125 rad`, right `0.0049882 rad`;
- return segment maximum `|q-q_ref|`: left `0.0049994 rad`, right `0.0049997 rad`;
- return-segment maximum `|dq|`: about `0.00785 rad/s`; and
- final `|q-q_ref|`: `5.3406e-5 rad` on both arms.

The pre-fix bounded-motion receipt is retained as a failed diagnostic only in the local run history; it showed a repeatable ±2.5 mrad J1 residual caused by the stale `dq_ref`, which the new run removes. These tests establish no-contact torque ownership and basic tracking, not loaded or collision-qualified Task27 motion.

## P2: QP status

The Task27 torque bridge deliberately still reports `NOT_IMPLEMENTED_TASK27_LIVE_QP` and does not relabel the R8 impedance controller. The R12 solver is a 14-DOF kinematic velocity QP; its API returns `execution_authorized=false` and `inverse_dynamics_claim=false`. It does not produce a torque decision variable and is not bound to the Task27 runtime's synchronized bilateral target, full TCP orientation/Jacobian, or same-state `M/c/g`.

For traceability, `offline/task28_task27_integration/qp_replay_diagnostic.py` replays settled Task27 joint feedback without connecting to Isaac, ROS, suction, or a torque sink. The current receipt is `offline/task28_task27_integration/evidence/g2_qp_replay_decay_20261031_0430/summary.json`:

- 416 `QP_OPTIMAL` solves;
- mean/max solve time `9.21/13.25 ms`;
- maximum hard-relative residual `2.21e-16`;
- maximum common-task residual `1.99e-2` in the bounded-motion replay;
- model-FK to logged TCP-translation residual max/P95 `0.1128/0.0537 mm`; and
- `execution_authorized=false`, `inverse_dynamics_claim=false` for every solve.

This is useful mathematical replay evidence only. It is not live Task27 QP execution and does not authorize P3.

The earlier integrated attempt, retained at `offline/task28_task27_integration/evidence/g2_recovery_first_cube_20261031_0300/`, reached 1960 torque frames but stopped fail-closed before contact. At failure the left arm had peak `|q-q_ref|=1.734 rad`, peak `|dq|=5.260 rad/s`, 85 rate-limited frames, and target captures `left=491/right=0`. The unilateral target stream and large tracking divergence independently fail P1-C/P2; the Cube was not attached.

## P3 and continuous safety

No Surface Gripper activation, bilateral attachment, lift, transport, placement, release, D6 action, or Cube displacement was run in this iteration. The original HOME→Cube approach remains `CONTINUOUS_SAFETY_NOT_ESTABLISHED`; sampled planning/FCL evidence is not a continuous collision proof. A path-specific pre-contact stop gate and a validated bilateral live QP/dynamics binding are required before any Cube contact attempt.

## Reproduction

The GUI runs used Isaac Sim 4.5 and the original Task27 entry. The environment must source ROS Humble and expose both system ROS libraries and Isaac's ROS bridge libraries:

```bash
source /opt/ros/humble/setup.bash
export DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES
export ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib"

/home/ubuntu2004/isaacsim-4.5.0/python.sh \
  isaac/scripts/task28_task27_one_pass.py \
  --mode smoke --frames 600 --run-id g2_recovery_hold_20261031_0330

/home/ubuntu2004/isaacsim-4.5.0/python.sh \
  isaac/scripts/task28_task27_one_pass.py \
  --mode smoke --small-motion --frames 240 \
  --run-id g2_recovery_small_motion_decay_20261031_0420
```

The failed first-Cube receipt is preserved as a diagnostic; it is not a Cube execution result and is not reported as a pass.

## Regression

- Historical repository suite rerun with its recorded import-mode/environment: **789 passed in 104.36 s**.
- Current Task28 focused suite (`offline/task28_t0/tests offline/task28_p5_qp_6_r8/tests`): **25 passed**.
- Modified Isaac scripts and replay diagnostic: `py_compile` passed.
- `git diff --check`: passed.

An unscoped `python3 -m pytest -q` is not a valid repository regression command here because the repository intentionally contains repeated test-module names and package-local imports; it fails during collection before testing. The recorded historical launcher with `--import-mode=importlib` was used instead.

## Remaining blocker and next safe work

The next implementation must add a path-specific pre-contact stop gate and bind a real Task27 14-DOF torque-level cooperative QP to synchronized bilateral feedback, including verified TCP orientation/Jacobians and same-state dynamics/constraint data. Until those gates pass, Cube_01 contact and transport remain prohibited. No real robot, hardware FR3 interface, Sim2Real, ROS hardware command, or G3 internal-force claim is made here.

## Delivery

- Final commit: the commit returned by `git rev-parse HEAD` at delivery
- Remote: `origin/side-suction-palletizing` at the same commit
- Final worktree: clean

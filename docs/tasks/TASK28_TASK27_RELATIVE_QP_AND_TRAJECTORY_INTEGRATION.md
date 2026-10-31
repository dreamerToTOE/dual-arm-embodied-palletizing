# TASK28 — Task27 Relative QP and Trajectory Integration

## Result

This task is **PARTIAL**. The bilateral relative TCP drift was corrected in the
original Task27 GUI scene and the live QP now consumes timestamped Task27 joint
targets. A full bilateral Task27 approach or Cube_01 manipulation was not
demonstrated: the executor produced a unilateral left-arm stream first, and the
live QP subsequently stopped fail-closed at its certified acceleration/torque
box. No suction, Cube contact, lift, transport, release, or real-robot command
was run.

The original Task27 scene and bridge remain the source of the run. No scene
builder, USD asset, robot model, tool, gripper, D6, collision geometry, or
physical parameter was modified.

## Root cause and correction

The old relative task combined a PhysX link Jacobian with a world-origin
spatial relative map even though the measured TCP rows were being used as
TCP-origin geometric velocities. This applied the TCP lever arm twice. A
second issue was that the authored USD child TCP transform lagged the live
articulation pose during effort control.

The torque bridge now:

1. Builds the runtime TCP pose from the live PhysX `fr3_link8` pose and the
   authored, fixed link8-to-TCP transform.
2. Uses the direct TCP-origin geometric relative body map
   `pair_tcp_geometric_relative_matrix(T_left, T_right)`.
3. Keeps the transported PhysX Jacobian as an audit signal, but selects the
   frozen Task27 URDF geometric Jacobian for the QP after per-arm pose
   qualification and finite-difference replay.
4. Computes `Jdot*dq` from the selected model Jacobian and logs the PhysX/model
   mismatch instead of hiding it.

The offline and runtime evidence supports this choice. On the same replay,
the model-J TCP twist residual was at most about `0.384 mm/s` per arm and
`0.501 mm/s` for the relative body twist, while the transported PhysX audit
Jacobian reached about `3.325 mm/s` per arm and `8.972 mm/s` relatively.

## Runtime measurements

All four runs below used the original Task27 GUI scene, seven arm DOFs per
arm, synchronized effort writes, the existing QP hard limits, and no torque
saturation or rate limiting unless stated.

| Run | QP writes / status | Common result | Relative result | Outcome |
| --- | ---: | ---: | ---: | --- |
| `r14_modelJ_hold600_002246` | 1,350 writes; 1,349 `QP_OPTIMAL` | max displacement `0.00122 mm` | max displacement `0.00588 mm` | completed |
| `r14_modelJ_common_002434` | 630 writes; 629 `QP_OPTIMAL` | 3 mm commanded, max tracking error `0.0319 mm` | max error `0.0202 mm` | completed |
| `r14_modelJ_relative_002117` | 405 writes; 404 `QP_OPTIMAL` | max error `0.1711 mm` | 1 mm commanded, max error `0.0987 mm` | stopped by 20 mm local envelope |
| `r14_modelJ_combined_002549` | 387 writes; 386 `QP_OPTIMAL` | 3 mm commanded, max error `0.1709 mm` | 1 mm commanded, max error `0.0742 mm` | stopped by 20 mm local envelope |

The local-envelope stops were fail-closed diagnostic bounds, not acceptance
thresholds. The relative and combined motion errors are reported separately
from anchor displacement; the approximately 1 mm relative displacement was the
commanded exploratory motion, while the tracking residual stayed below the
historical `0.150 mm` relative research threshold in these runs.

Evidence directories:

- `offline/task28_task27_integration/evidence/r14_modelJ_hold600_002246/`
- `offline/task28_task27_integration/evidence/r14_modelJ_common_002434/`
- `offline/task28_task27_integration/evidence/r14_modelJ_relative_002117/`
- `offline/task28_task27_integration/evidence/r14_modelJ_combined_002549/`

## Task27 trajectory adapter

`offline/task28_task27_live_qp/trajectory_adapter.py` is a timestamp-ordered
adapter with cubic Hermite interpolation. It accepts repeated timestamps by
replacement, rejects non-monotonic input, estimates velocity and acceleration
when the original Task27 stream supplies only positions, and supports a
missing/unilateral arm without inventing a closed-chain target.

The live bridge captures the original Task27 bridge's timestamped arm targets,
validates frozen FK TCP pose against the live PhysX TCP before use, and records
the reference source, age, active arms, q/dq/ddq, TCP target pose, and QP
status. Task modes are explicit: `FREE_APPROACH`, `DUAL_ATTACHED`,
`SINGLE_ARM_PUSH`, and `RELEASE_RETREAT`. Suction command state drives runtime
mode transitions; elapsed time is not used as a grasp event.

The GUI integrated replay `r14_qref_earliest_003704` contains the first direct
proof that a real Task27 target reaches the live QP: the left adapter reports
`source=timestamped_task27_adapter`, `tcp_targets_active=true`, and increasing
left target counts while the right stream remains absent (`target_count=0`).
The run produced 529 optimal QP solves before the 20 mm local envelope stopped
the bridge. A subsequent extended diagnostic (`r14_qref_posture2_004459`,
using a logged `0.50 m` exploratory envelope) captured 295 left targets and no
right targets, then stopped on `QP_INFEASIBLE:EMPTY_ACCELERATION_OR_TORQUE_BOX`.
The latter is not a pass: the one-sided stream caused the left arm to leave the
certified joint/torque intersection while the right arm still used its handoff
posture.

The executor startup was also corrected. ROS prefix variables are cleared for
the subprocess, the historical MoveIt overlay is scoped to MoveIt only, and
the Task27 executor is started while MoveIt action servers are coming up. This
prevents the previous runner-side wait from stopping the original feed bridge
before the executor could observe its rail state.

## Cube_01 progress and limits

MoveIt planning candidates and synchronized FCL checks were reached in the
integrated attempts, but no complete physical approach was accepted by the
torque bridge. The furthest demonstrated milestone is unilateral left-arm
qref binding to live QP. There is no evidence of a bilateral qref, contact,
Surface Gripper closure, D6 attachment, Cube motion, lift, transport, release,
or retreat.

The original Task27 rail/feed handshake remains authoritative. A missing or
stale rail state is rejected; the runner never substitutes the authored `0.650`
m rail position. The QP also remains fail-closed on infeasible dynamics/torque
boxes, measured velocity limits, invalid models, and local exploratory
envelopes.

## Reproduction

GUI smoke/diagnostic run:

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib" \
/home/ubuntu2004/isaacsim-4.5.0/python.sh \
isaac/scripts/task28_task27_one_pass.py \
  --mode smoke --live-qp --experiment hold --frames 600
```

Integrated Task27 attempt (diagnostic envelope only):

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib" \
/home/ubuntu2004/isaacsim-4.5.0/python.sh \
isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 \
  --local-sweep-limit-m 0.50
```

The `0.50 m` value is recorded as an exploratory run envelope. It does not
change QP torque, rate, joint, velocity, collision, suction, or acceptance
limits and must not be interpreted as a safety certification.

Regression after the adapter and relative-map changes:

```text
PYTHONPATH=. python3 -m pytest -q \
  offline/task28_task27_live_qp/tests \
  offline/task28_p5_qp_6_r12/tests \
  offline/task28_t0/tests \
  offline/task28_p5_qp_6_r8/tests
59 passed
```

No hardware or ROS command output was used for these tests.

## Remaining work

The next task should make the Task27 executor publish synchronized left and
right trajectories before allowing approach execution, then re-run the live
QP with the default `0.020 m` diagnostic envelope first. Only after a fresh
bilateral qref stream is stable should the original suction/contact phases be
attempted. Cube contact and transport remain unvalidated, and no continuous
safety claim is made for the original HOME-to-Cube path.

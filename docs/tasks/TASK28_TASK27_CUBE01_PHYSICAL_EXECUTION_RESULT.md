# TASK28 — Task27 Execution Scheduling and Cube_01 Result

## Result

This run is **PARTIAL**. The Task27 scheduling defect and missing FR3 collision
mesh resolution were fixed. Both left and right Task27 trajectory streams now
reach the live QP together, and FREE_APPROACH uses independent joint tracking.
The executor reaches the original HIGH_PRE_CONTACT planning/FCL stage, but the
torque controlled terminal joint state does not settle within the existing
executor gate. No suction closure, Cube contact, lift, transport, release, or
Cube displacement was confirmed.

The original Task27 scene, FR3 assets, L-shaped tools, Surface Grippers, D6s,
Cube geometry, and collision parameters were not modified.

## P0 scheduling correction

The Task27 HIGH_PRE_CONTACT path previously called blocking `left.executeAt()`
and then `right.executeAt()`. A left-side failure or long wait could therefore
prevent the right trajectory from being published. The Task27-specific branch
now uses the existing synchronized scheduler:

```text
executeSync(left, left_pre, right, right_pre)
left.waitAtTarget(...)
right.waitAtTarget(...)
```

The same correction was applied to the Task27 early-handoff move. Task26 keeps
its historical staged `executeAt()` behavior; explicitly unilateral pusher and
helper phases remain staged.

The runner now starts the executor while MoveIt is starting rather than waiting
for the full MoveIt startup window. This keeps the original feed/rail state
publisher alive while the executor subscribes.

## Collision resource correction

The executor previously sourced only this checkout's isolated install, which
does not contain the external `franka_description` package. MoveIt then logged
missing `package://franka_description/...` collision meshes and created empty
collision geometry for the FR3 links.

The runner now injects the historical isolated
`franka_description` install prefix after sourcing the local workspace. It does
not copy meshes, suppress warnings, or source the historical executor package.
Fresh MoveIt and executor logs contain no `Error retrieving file
[package://franka_description]` or “collision geometry will be left empty”
messages.

## FREE_APPROACH and trajectory handling

When a real Task27 stream is active, FREE_APPROACH, SINGLE_ARM_PUSH, and
RELEASE_RETREAT use an explicit independent-joint QP objective:

- common and relative soft task weights are zero;
- joint posture weight is 10;
- the measured mass, bias, torque, rate, acceleration, velocity, and position
  constraints remain active;
- the verified relative Jacobian remains logged for diagnostics.

If only one arm stream is present, the missing arm's TCP reference is latched
at the first measured phase-handoff pose instead of being reset to the drifting
current pose each cycle. DUAL_ATTACHED retains the cooperative relative task.

The runner exposes `--execution-time-scale` so trajectory retiming is explicit
and reproducible. The existing source timestamps remain mapped to the Isaac
simulation clock from the first received point.

## GUI execution evidence

All runs used the original Task27 GUI scene and the real Task27 executor. The
key evidence is:

| Run | Retiming | QP result | Task27 targets | Furthest result |
| --- | ---: | ---: | ---: | --- |
| `task28_cube01_sched_010815` | 3.0 | 496 optimal | left/right 23/23 | synchronized qref and FCL preflight; stopped by 0.50 m exploratory envelope |
| `task28_cube01_independent_011309` | 3.0 | 2,095 optimal | left/right 546/546 | synchronized qref, FCL preflight, terminal settle failure `27.83°` |
| `task28_cube01_retime8_011640` | 8.0 | 2,049 optimal | left/right 846/846 | terminal settle failure `9.20°` |

The `execution_time_scale=8` retry was the best terminal result, but it still
did not satisfy the executor's joint-state settle gate. The QP itself did not
become infeasible; the remaining failure is trajectory tracking error under
the current torque-controlled plant and reference path.

The executor reached and logged synchronized FCL passes for
`HIGH_PRE_CONTACT`, `OUTER_DESCENT`, `CONTACT` preflight, and the loaded lift/X
probes. These are planning and collision-check results. They are not evidence
that the Cube was physically contacted or attached during the failed execution.

Evidence directories:

- `offline/task28_task27_integration/evidence/task28_cube01_sched_010815/`
- `offline/task28_task27_integration/evidence/task28_cube01_independent_011309/`
- `offline/task28_task27_integration/evidence/task28_cube01_retime8_011640/`

## Cube_01 physical stages

| Stage | Result | Evidence |
| --- | --- | --- |
| MoveIt planning and synchronized FCL | reached | executor logs |
| Bilateral qref delivery | passed | both target counts increase; `active_arms=[left,right]` |
| Torque QP execution | passed for bounded run | all recorded solves `QP_OPTIMAL` |
| HIGH_PRE_CONTACT terminal settle | failed | executor residual 27.83° at scale 3; 9.20° at scale 8 |
| Surface Gripper activation | not reached/confirmed | no suction-close receipt |
| Cube attachment/lift/transport/release | not demonstrated | no Cube pose change or attachment GT |

No teleport or synthetic attachment was used.

## Reproduction

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install

DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib" \
/home/ubuntu2004/isaacsim-4.5.0/python.sh \
isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 \
  --local-sweep-limit-m 2.0 \
  --execution-time-scale 8.0 \
  --timeout-s 120
```

Focused regression after the scheduling, independent-objective, and resource
changes:

```text
PYTHONPATH=. python3 -m pytest -q \
  offline/task28_task27_live_qp/tests \
  offline/task28_p5_qp_6_r12/tests \
  offline/task28_t0/tests \
  offline/task28_p5_qp_6_r8/tests
```

The final count is recorded with the commit validation output.

Final local result: **61 passed**; Python compilation and `git diff --check`
also passed.

## Remaining blocker

The original Task27 approach reference remains dynamically difficult for the
torque plant: even after retiming to 8×, the terminal joint error exceeded the
executor's existing settle tolerance. The next bounded experiment should keep
the synchronized schedule and independent QP, then either retime the actual
MoveIt trajectory further or generate a dynamically feasible approach segment.
Cube contact and all later physical stages remain unvalidated.

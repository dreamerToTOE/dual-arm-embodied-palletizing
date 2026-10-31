# TASK28 — Task27 Terminal Tracking and Cube_01 Result

## Outcome

This iteration fixed the pre-contact terminal phase error in the live torque QP and completed a real GUI Isaac Sim HIGH_PRE_CONTACT run in the original Task27 scene. Cube_01 planning, contact geometry, both original Surface Gripper close commands, POST_CLOSE attachment validation, and the COMMON_LIFT FCL preflight were reached.

The first loaded cooperative motion still fails closed. No Cube lift, transport, placement, release, or retreat is claimed. The remaining blocker is loaded-contact dynamics: after the two Surface Grippers report `CLOSED`, the measured joint acceleration diverges from the unloaded-arm inverse-dynamics model, the left arm approaches its joint-7 velocity/position boundary, and the next acceleration/torque intersection becomes empty. Hard limits were retained.

Execution remained GUI Isaac Sim 4.5 (`headless=false`), with no real robot, hardware FR3 interface, ROS hardware command, or Sim2Real claim.

## P0 diagnosis and correction

The old `execution_time_scale=8` receipt (`task28_cube01_retime8_011640`) had 2,049 `QP_OPTIMAL` solves, no acceleration/torque/rate constraint activation, and no torque saturation. The QP followed its timestamped adapter sample, but that sample was behind the latest Task27 target. At the terminal, the raw latest MoveIt target differed from the sampled target by approximately 7.72° on left joint 1 and 7.59° on right joint 7. ROS source stamps advanced several times faster than Isaac physics; the previous source/physics ratio acceptance upper bound of 4 rejected the observed ratio near 6 and left the adapter phase lagging.

The bridge now:

- maps the buffered source trajectory to Isaac time with a measured source/simulation clock ratio (accepted finite range `0.25..20`);
- samples the adapter's actual `q/dq/ddq` target used by the QP;
- adds the adapter `ddq_ref` to the existing feedback acceleration and keeps the bounded desired acceleration command at ±8 rad/s²;
- holds terminal `dq_ref=0` and `ddq_ref=0`;
- records sampled target, latest captured target, position/velocity error, desired acceleration, clock ratio, measured acceleration and tracking error;
- gates `DUAL_ATTACHED` on both original suction commands and both original Surface Grippers reporting `CLOSED`;
- reads Cube_01's authored scene mass and applies a runtime-only equal half-load gravity term after the close gate. This does not alter USD, create a D6, or claim a validated contact model.

The optional DUAL_ATTACHED posture-weight increase was tested once and reverted because it failed earlier. The cooperative common/relative objective and hard QP limits remain the baseline configuration.

## GUI receipts

| Run | Result | Evidence |
|---|---|---|
| `task28_precontact_clock20_014713` | **PASS**: executor exit 0; 2,215 QP solves, all optimal; left/right terminal joint error 0.003°/0.004°; TCP error 0.058/0.068 mm; Cube drift 0; clearances 100.971/40.937 mm; suction remained open | `offline/task28_task27_integration/evidence/task28_precontact_clock20_014713/` |
| `task28_cube01_ddq_013329` | reached POST_CLOSE and COMMON_LIFT FCL preflight; failed at step 3362, `t=56.03 s`, `EMPTY_ACCELERATION_OR_TORQUE_BOX` | `offline/task28_task27_integration/evidence/task28_cube01_ddq_013329/` |
| `task28_cube01_payload_scale16_015114` | slower scale 16; reached POST_CLOSE and COMMON_LIFT FCL preflight; failed at step 5428, `t=90.47 s`, after 171 DUAL_ATTACHED frames | `offline/task28_task27_integration/evidence/task28_cube01_payload_scale16_015114/` |
| `task28_cube01_payload_posture10_015739` | DUAL_ATTACHED posture diagnostic; failed earlier at step 5319, `t=88.65 s`; change reverted | `offline/task28_task27_integration/evidence/task28_cube01_payload_posture10_015739/` |

Each evidence directory contains `summary.json`, `metrics.json`, GUI `scene_loaded.png`/`final.png`, MoveIt and executor logs, and the compressed torque trace `task27_torque_bridge.jsonl.gz`.

The scale-16 executor log records the actual original scene sequence:

- `PRE_CLOSE_GEOMETRY ATTACHMENT`: x/z error 0.011/0.020 mm, gap delta 0.004 mm;
- both original suction calls and `waitSuction(true)` completed;
- `POST_CLOSE ATTACHMENT`: x/z error 0.040/0.025 mm, gap delta 0.007 mm;
- `COMMON_LIFT` left/right Cartesian and synchronized FCL preflight passed;
- no `trace_held("COMMON_LIFT")` receipt was produced before the torque bridge failed, so Cube movement is unverified;
- executor later reported that emergency suction-off confirmation was not obtained after the bridge had failed closed.

## Loaded failure analysis

The scale-16 failure was not a solver-status-only failure. Immediately before the fail-closed step, the measured left joint state was approximately:

- `q_left = [1.022, 0.072, 1.154, -2.246, -0.160, 2.228, 2.960] rad`;
- left joint-7 speed about `2.62 rad/s`;
- sampled left target joint 7 about `1.68 rad`, so the position error was about `-1.28 rad`;
- the next hard position/velocity/acceleration/torque intersection was empty.

Requested torques immediately before failure remained within the configured effort box and the run did not report torque-rate saturation. Offline finite differences from the saved trace give a maximum measured acceleration near `19.88 rad/s²` and a maximum requested-versus-measured acceleration discrepancy near `14.26 rad/s²` in the loaded segment. The candidate 0.8 kg equal-split gravity term was active, but it did not account for the complete Cube/Surface-Gripper contact wrench and did not prevent drift.

This identifies the remaining cause as an unqualified loaded-contact dynamics/model mismatch, with a fail-closed limit event. It does not justify relaxing joint, velocity, acceleration, torque, or torque-rate limits.

## Code and scene scope

Changed files:

- `isaac/scripts/task28_task27_torque_bridge.py`: timestamped source-clock mapping, `ddq_ref` feedforward, sampled-reference telemetry, measured-acceleration telemetry, strict suction-state mode transition, and runtime-only diagnostic payload gravity term;
- `isaac/scripts/task28_task27_one_pass.py`: `--pre-contact-only` entry for the existing Task27 pre-contact gate;
- `offline/task28_task27_live_qp/tests/test_trajectory_adapter.py`: explicit acceleration and terminal zero-velocity/zero-acceleration contract test.

The original Task27 scene entry, Task26 scene builder, FR3 assets, L-shaped side-suction tool, Surface Gripper configuration, Cube geometry, table/truck collision geometry, D6 lifecycle and release logic were not edited. Scene and bridge source hashes in every run remain:

- scene source: `43aa7c2b...` (`task26_truck_box_scene.py`);
- original feed bridge: `0f712365...` (`task26_truck_box_bridge.py`);
- Task27 entry hash: `74eb1199...`.

## Regression and reproduction

Focused regression after the final code state:

```text
PYTHONPATH=. python3 -m pytest -q \
  offline/task28_task27_live_qp/tests \
  offline/task28_p5_qp_6_r12/tests \
  offline/task28_t0/tests \
  offline/task28_p5_qp_6_r8/tests

62 passed
```

Python compilation and `git diff --check` also pass.

Pre-contact GUI reproduction:

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib" \
/home/ubuntu2004/isaacsim-4.5.0/python.sh \
isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --pre-contact-only --max-batches 1 \
  --local-sweep-limit-m 2.0 --execution-time-scale 8.0 --timeout-s 160
```

Full Cube_01 attempt used the same GUI entry and original scene:

```bash
.../python.sh isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 \
  --local-sweep-limit-m 2.0 --execution-time-scale 16.0 --timeout-s 300
```

## Final status

`HIGH_PRE_CONTACT` is physically reached with the corrected source-clock mapping. Cube_01 has not completed a verified lift/transport/place/release cycle. The next implementation must qualify the loaded Cube/Surface-Gripper contact dynamics and a bounded loaded-segment entry/stop policy before another physical lift attempt. The fail-closed behavior and the unconfirmed emergency suction-off state are retained in the evidence; this result must not be reported as Cube manipulation success.

## Git delivery

- branch: `side-suction-palletizing`
- final local HEAD: `df98b25`
- remote branch: `origin/side-suction-palletizing` at `df98b25`
- working tree: clean after the delivery commit

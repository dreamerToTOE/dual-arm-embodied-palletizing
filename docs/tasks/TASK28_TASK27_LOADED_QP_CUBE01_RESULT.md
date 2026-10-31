# TASK28 — Loaded Cooperative QP and Cube_01 Result

## Result

The loaded controller completed a real bilateral Cube_01 lift, cooperative transport, placement, release and retreat in GUI Isaac Sim using the original Task27 scene, both original Surface Grippers and the live torque QP.

The strongest run is `task28_loaded_posture1e4_full_023503`:

- 14,851 live QP solves, all `QP_OPTIMAL`; no torque-QP fail-closed event;
- measured POST_CLOSE reference hold completed for 60 physics frames;
- Cube_01 lifted from z≈0.260 m to z≈0.5404 m;
- Cube_01 transported to approximately x=0.770 m, y=0.060 m;
- controlled descent and side push completed with suction still CLOSED;
- Cube_01 reached the table, both suction commands reported OPEN, and the original preplanned bilateral retreat passed FCL.

After this completed cycle, the original Task27 executor continued an optional REGRASP/PUSH sequence. The 600 s runner timeout stopped that continuation; PUSH slice 8 then reported a right-arm settle failure (3.738° residual) and emergency suction-off confirmation was not obtained. This post-release continuation failure does not invalidate the completed lift, placement, release and retreat.

The Isaac runner summary is `headless=false`, `moveit_exit=0`, bridge `status=COMPLETED`, with 14,851 QP solves and no latched error. The runner reached its 600 s bound while the optional post-release continuation was still running (`executor_exit=-15`, `timeout_reached=true`); therefore this result is a Cube_01 cycle result, not a claim that the full five-Cube continuation completed.

## Loaded failure mechanism

The previous scale-16 failure was caused by a loaded objective conflict after `FREE_APPROACH → DUAL_ATTACHED`, rather than a simple torque shortage:

1. The queued COMMON_LIFT target was consumed immediately after both grippers closed.
2. The unloaded arm model plus equal-split gravity term did not represent the bilateral constraint state.
3. Common/relative task weights dominated the small posture term and drove the right wrist toward its lower joint limit.
4. The QP remained optimal until the hard position/velocity/acceleration intersection became empty.

The saved baseline reached Cube z≈0.4506 m, then failed at right J7≈−2.99 rad. No hard joint, velocity, torque or rate limit was relaxed.

## Controller changes

`isaac/scripts/task28_task27_torque_bridge.py` now includes:

- a measured 60-step POST_CLOSE hold initialized from the actual left/right q and TCP transforms at the mode transition;
- common and relative TCP targets held at the measured attachment transforms during that interval, preventing a queued trajectory jump;
- support-aware Cube gravity compensation: the runtime Cube mass is read from PhysX, but the load term is zero while Cube z remains at the table support plane and ramps only after the Cube rises;
- loaded posture weight `1e4`, matching the existing common/relative objective scale, while FREE_APPROACH retains its validated posture weight `0.2`;
- Cube runtime diagnostics: Ground Truth pose, linear/angular velocity and acceleration, runtime mass, runtime inertia, and D6 path availability;
- attachment-mode telemetry and measured qdd/QP qdd tracking diagnostics.

The offline G3 grasp matrix remains available for analysis, but its nominal ±61 mm grasp transforms are not used as measured D6 anchors. Isaac Sim’s high-level Surface Gripper wrapper exposes closed/open state but no direct D6 reaction wrench; the report therefore distinguishes measured Cube motion and runtime inertial data from model-based contact estimates.

## Physical stage receipts

From `task28_loaded_posture1e4_full_023503/task27_executor.log`:

- `POST_CLOSE ATTACHMENT`: x/z error 0.012/0.036 mm, gap delta 0.014 mm;
- `COMMON_LIFT GT=(0.500, 0.000, 0.540), suction=(left:CLOSED right:CLOSED)`;
- `COMMON_LIFT ATTACHMENT`: x/z error 0.013/0.036 mm, gap delta 0.013 mm, tilt 0.040°;
- `COMMON_X_TRAVEL GT=(0.770, 0.000, 0.540), suction=(left:CLOSED right:CLOSED)`;
- `COMMON_X_TRAVEL ATTACHMENT`: x/z error 0.012/0.037 mm, gap delta 0.013 mm, tilt 0.069°;
- `COMMON_Y_ALIGN GT=(0.770, 0.060, 0.540), suction=(left:CLOSED right:CLOSED)`;
- `COMMON_Y_ALIGN ATTACHMENT`: x/z error 0.012/0.069 mm, gap delta 0.013 mm, tilt 0.069°.
- `COMMON_DESCENT_TO_ENTRY GT=(0.770, 0.060, 0.280)`, suction CLOSED;
- `COMMON_DESCENT_TO_ENTRY ATTACHMENT`: x/z error 0.016/0.042 mm;
- `COMMON_SIDE_SHORT_PUSH GT=(0.790, 0.060, 0.280)`, suction CLOSED, attachment x/z error 0.012/0.036 mm;
- `COMMON_DROP_TO_TABLE GT=(0.773, 0.060, 0.271)`, attachment x/z error 0.185/0.037 mm, followed by the original log receipt that both side suctions were OPEN;
- `PREPLANNED_COMMON_RETREAT` synchronized FCL PASS, then both arms settled at approximately 0.004° residual.

The full-run Cube trace independently bounds the loaded motion at approximately:

```text
x: 0.49941 .. 0.82979 m
y: -0.00357 .. 0.06410 m
z: 0.26000 .. 0.54042 m
```

The post-release optional regrasp/push accounts for the later x≈0.830 m
maximum. Before release, the completed transport and placement path reached
x≈0.770 m, y≈0.060 m, z≈0.540 m and returned to the table at z≈0.271 m.

Runtime PhysX Cube data were approximately mass 0.800 kg and diagonal inertia 0.00192 kg·m². These are runtime simulation values, not hardware parameters.

The compressed trace contains 14,852 `TORQUE_FRAME` records: one gravity-handoff frame and 14,851 `QP_OPTIMAL` solves. No torque saturation, torque-rate limiting, QP box violation or fail-closed stop occurred in this run. The largest all-trace joint error was 13.50° during a transient; loaded-phase telemetry also contains transient model/task-space residuals, so those values are not substituted for the executor's segment-end attachment receipts above. The runtime Surface Gripper/D6 path is not exposed as a valid reaction-wrench source (`valid=false`); Cube motion and suction state are the physical simulation evidence used here.

Evidence is stored under `offline/task28_task27_integration/evidence/task28_loaded_posture1e4_full_023503/` (`summary.json`, `metrics.json`, `task27_executor.log`, `moveit.log`, `final.png`, and the compressed torque trace). The original scene entry and source hashes are recorded in `summary.json`; no USD, robot, tool, Cube or suction asset was changed.

## Comparator runs

- `task28_loaded_hold_021750`: transition hold stabilized the grasp and produced a real Cube lift to z≈0.45057 m, then right J7 reached its lower bound and the QP failed closed.
- `task28_loaded_posture10_hold_022321`: reproduced a real lift to z≈0.45210 m, then failed at the same loaded wrist-limit mechanism.
- `task28_loaded_posture1e4_hold_022848`: removed the objective conflict sufficiently to complete lift, X transport and Y alignment with all QP solves optimal.
- `task28_loaded_posture1e4_full_023503`: completed the same loaded sequence through drop, suction release and bilateral retreat; the later optional regrasp push failed at slice 8 during right-arm settling.

Each evidence directory contains GUI screenshots, executor/MoveIt logs, `summary.json`, `metrics.json`, and compressed `task27_torque_bridge.jsonl.gz` traces.

## Validation and reproduction

Focused regression remains:

```text
PYTHONPATH=. python3 -m pytest -q \
  offline/task28_task27_live_qp/tests \
  offline/task28_p5_qp_6_r12/tests \
  offline/task28_t0/tests \
  offline/task28_p5_qp_6_r8/tests

62 passed
```

GUI reproduction:

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib" \
/home/ubuntu2004/isaacsim-4.5.0/python.sh \
isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 \
  --local-sweep-limit-m 2.0 --execution-time-scale 16.0 --timeout-s 600
```

## Remaining work

The required Cube_01 loaded cycle through placement, release and bilateral retreat is demonstrated. The later optional Task27 regrasp/push continuation remains incomplete because the right arm exceeded its post-release settle residual at PUSH slice 8; it is a separate post-cycle failure. No further unloaded tracking audit is needed.

No real robot, hardware FR3 interface, Sim2Real claim, synthetic Cube teleport or fake attachment is used.

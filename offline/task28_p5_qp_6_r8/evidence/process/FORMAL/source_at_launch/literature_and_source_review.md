# R8 source review, before new Isaac control comparisons

The installed Isaac Sim 4.5 runtime and frozen scene are authoritative. The only
builder is `fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py::build_scene`.
The builder itself enables the ROS extension to construct its graph; R8 disables
the anonymous graph and extension before timeline play, without editing the builder.

## Primary references checked on 2026-10-29

- [Franka joint impedance and FR3 Duo examples](https://frankarobotics.github.io/docs/doc/franka_ros2_jazzy/franka_example_controllers/doc/index.html): torque interfaces use q/dq feedback; Duo includes collision supervision. Hardware gravity-only uses zero external torque because the robot compensates internally. This convention is NOT copied into Isaac.
- [libfranka overview](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html): external torque loops have hardware timing and model semantics. R8 uses 60 Hz TGS, explicitly supplied g, and command-buffer readback, not a physical torque sensor.
- [Pinocchio dynamics](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_a-features_g-dynamic.html): CRBA supplies inertia, RNEA inverse dynamics, ABA unconstrained forward dynamics. A positive mass matrix alone is not a qualification of a constrained simulator's entire inverse dynamics.
- [TSID](https://github.com/stack-of-tasks/tsid): optimization-based inverse dynamics builds on a rigid-body model. Its availability does not certify this mounted-tool model or imply executed cooperative control.
- [PhysX rigid-body dynamics](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/RigidBodyDynamics.html): velocities reflect forces and constraints. R5's local paired experiment and installed external-force-every-iteration flag are the specific evidence for this TGS end-step/whole-step distinction. R8 leaves that flag false and never substitutes finite-difference velocity into feedback.
- The requested [Isaac 4.5 API URL](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.prims/docs/index.html) returned 404. Installed `exts/isaacsim.core.prims/isaacsim/core/prims/impl/articulation.py` is used instead: `set_joint_efforts` builds a complete buffer and requires disabled drives for effort control; q/dq and M/c/g are copied; `set_gains(...,save_to_usd=False)` is runtime-only.

## R6/R7, geometry, and source decisions

R6 traces preserve full nine-coordinate actuation, native finger drives/mimic,
positive/negative perturbations, and restricted M/c/g observations. R7 S1 is
**damping-only**, S3 is PD: the report's later 'two PD' sentence is not adopted.
The frozen G1 `RobotModel.jacobian` is world-frame TCP linear/angular Jacobian;
its link8-to-TCP transform includes the mirrored L tools. FCL uses original
ACM and detached Cube policy; Cube/table contact is the existing allowed contact.
No collision threshold, effective joint limit, mass, tool geometry, or fixture
identity is changed. Joint limits are explicitly checked in R8 live gates.

R7 S2 has SIGSEGV and mutex non-owner SIGABRT startup evidence, no valid serialized
frame and no effort. KVDB contention appears in its logs and a previous successful
run. That supports a process/resource-isolation fault hypothesis, not a proven
single cause for the crash. `SimulationApp.close()` contains an unbounded
`while is_stage_loading()` loop after its shutdown message. A shutdown message or
saved result is not proof of process exit.

R8 launches installed Python with the same setup script as python.sh but uses
`exec` to preserve the actual exit code/signal. Each run has its own session and
portable root. Parent ownership/lifecycle journals, a serialization lock, bounded
owned-group cleanup, runtime cache-token checks, and no-residual-process checks
must succeed. No unrelated PID is signaled. Initial failures count as failed cells;
there is no unregistered retry. Per-run logs and source snapshots are archived
before deleting only that run's temporary portable cache.

## Controller derivation and uncertainty

`analyze_r7.py` replays all recorded control intervals and recomputes torque
components, TCP drift, frozen Jacobian contributions, generalized eigenmodes and
a coupled discrete approximation. R7's low-frequency modes have very low damping;
its correction is small compared with the initial momentum and gravity terms.
The fitted behavior is evidence against assuming unknown tool inertia is the sole
cause. The source matrix is used as a **frozen gain metric**, not as a certified
inverse-dynamics model. Set Kp=16 M0 and Kd=8 M0 (wn=4 rad/s, zeta=1). The
semi-implicit rigid-body prediction has spectral radius approximately .94847 at
60 Hz; it predicts torque and rate feasibility for the observed initial state.
This stability prediction is explicitly not a TGS stability proof.

D2 captures actual q0 and damps native dq toward zero. D3 starts q_ref=q0 and
its derivative at native dq0, then uses a C2 polynomial returning to q0 in .15 s.
Both use the same fixed gains. D4 repeats the preregistered strongest measured
eligible strategy; no gains are fitted to diagnostic results. Performance uses
all samples, not only the final frame. 1 mm is separate from the 10 mm safety
abort envelope and the unchanged 0.100 mm G1 grasp-closure threshold.

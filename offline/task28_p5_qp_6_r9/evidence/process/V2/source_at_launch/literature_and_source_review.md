# R9 literature and source review

The frozen G1 validator, Task24 staged side approach, QP0/QP1/QP2 kinematic
formulations, QP6 mathematical HQP model, and R6-R8 evidence were reviewed
before implementation.  The only scene entry is the original G1 builder; the
fixture manifest, USD hash, ACM, collision geometry, Cube pose, and nominal
1 mm gap remain source contracts.

The R9 solver uses the QP0 world geometric twist convention.  For a fixed
virtual-object TCP offset `r_i`, `H_i = [[I,-skew(r_i)],[0,I]]` maps a world
object twist to the TCP twist.  A 14-joint decision `qdot` uses the two frozen
source Jacobians.  The hard constraint is the six-dimensional null-space
condition `(I-H H^+) J qdot = 0`, which preserves the measured TCP pair's
relative rigid twist.  Common virtual-object motion and posture smoothness are
soft quadratic objectives.  The solver checks strict SPD, feasibility, primal
and dual KKT residuals, and box margins.  It contains no `M qdd`, contact
wrench, suction capacity, or physical attachment equality.

Task24's established sequence is retained as a planning reference: high
pre-contact, outer descent, synchronized contact micro-timing, then common
lift/transport.  The R3 direct HOME-to-nominal interpolation is retained as a
negative: the archived 51-sample screen first collided at fraction 0.92 with
left suction shapes 3 and 5.  The new candidate uses the original Cube and
fixed grasp transforms but is only a finite synchronized FCL screen.

TSID and the Dio et al. IROS 2023 cooperative HQP paper are frameworks for
hierarchical task-space inverse dynamics.  MoveIt 2 dual-arm and MoveIt Pro
multi-tip Cartesian planning document namespaced multi-arm planning and
multi-tip trajectory generation.  None of these references qualifies the
mounted L-tool inertia, current object inertia, contact wrench, suction mode,
or the installed FR3 runtime.  No external dependency is added and no
performance transfers are claimed.

Isaac Sim 4.5 installed articulation APIs remain authoritative: efforts are
queued for the next step, `get_applied_joint_efforts` is a command buffer, and
solver/projected efforts are not torque sensors.  R9 preserves R8's full
9-coordinate effort adapter, native fingers, CPU NumPy backend, 60 Hz TGS,
process isolation, source gate, and fail-closed ownership transfer.

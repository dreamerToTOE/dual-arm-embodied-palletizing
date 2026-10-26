# Task28 single-Cube fixture

`task28_g1_0_new_single_cube_v0`
`NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`
Acceptance: **`G1_0B_CONDITIONALLY_READY`** (static source/configuration readiness).

This independent fixture lives outside `ros_ws/src`. It includes an Isaac scene
builder, an opt-in bridge, a controller task-input module, a MoveIt source package,
a generated frozen URDF, and a machine-readable manifest. Existing Task24/26/27
sources, targets, launches, collision allowances and suction thresholds are unchanged.

Exactly one physical object is specified: `Cube_01`, task index 0, Cube index 0,
PoseArray order `[Cube_01]`, and MoveIt CollisionObject ID `Cube_01`. Its initial
position is `(0.650, 0, 0.260) m`, target `(0.820, -0.075, 0.260) m`, size
`0.120 m` per edge and mass `0.8 kg`. All positions are in metre/Z-up `world`.

The scene and bridge require the explicitly identified FR3 primary USD with the
frozen SHA256; they reject a missing or different model. All authored prims live
under `/World/Task28G10SingleCube`. ROS endpoints use `/task28_g1_0`. The bridge
defaults to telemetry only, and MoveIt trajectory execution is disabled. The
controller module supplies an immutable single-task specification and emits no
robot commands. Motion/closed-chain algorithms are outside G1-0B.

Offline USD inspection found matching joint kinematics but different raw joint
limits and collision meshes in Isaac and `franka_description` 2.8.1. Both raw
models are preserved. The selected frozen planning URDF uses the intersection
of position limits, minimum velocity/effort limits, and conservative link meshes
containing both original source geometries. Tool geometry and SRDF ACM are copied
unchanged. The Cube remains a world collision object; a future validator must
include it as carried payload without adding collision exceptions.

Run the bounded standard-library checks from the repository root:

```bash
python3 fixtures/task28_g1_0_new_single_cube_v0/tests/static_check.py
```

The checker verifies source/configuration/asset bindings, local and world TCP
semantics, HOME, bases, limits, mesh containment and source hashes. It rejects
17 bundle mutations, 4 task-input errors, 5 model-renderer errors and 8 mesh
outliers. It imports no Isaac/ROS runtime and sends no commands. Optional
`--asset-file` hashes an already acquired USD; `--xacro` checks exact offline
URDF regeneration. See [launch_protocol.md](launch_protocol.md) for dependencies,
commands and the future launch sequence, which was not executed here.

Remaining unknown: `OmniPBR.mdl` visual material identity/hash. The primary USD
contains its physics/geometry, so this does not block static kinematics/collision
readiness. Runtime build/ABI, state freshness, PhysX cooking, suction and physical
success remain unverified. G1-1 requires separate authorization. Historical
Task24-H `0.184/0.115 mm` values are historical report evidence only.

# Task28 isolated MoveIt source package

`NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`

This package belongs only to `task28_g1_0_new_single_cube_v0`. It is outside
`ros_ws/src` and does not modify or replace any Task24/26/27 package. Its launch
source uses `/task28_g1_0`, defaults RViz off, and disables MoveIt trajectory
execution. The two helper scripts publish state/geometry only. No controller,
joint-command publisher, suction command, or robot driver is launched.

The copied tool, SRDF collision allowances, and kinematics files are byte-identical
to the recorded legacy sources. The robot xacro changes only the tool include to
a relative local filename. The OMPL configuration is copied locally, avoiding a
runtime dependency on `franka_fr3_moveit_config`. Provenance and source hashes
are recorded in `source_provenance.json`.

The launch reads `urdf/task28_single_cube_frozen.urdf`, generated offline from
the unchanged raw robot xacro through `scripts/render_robot_description.py`.
This source-only transformer narrows
the 14 revolute arm joints to `config/effective_joint_limits.json`; it rejects
missing/duplicate joints, invalid intervals, and any expansion of an original
URDF position interval or velocity/effort maximum. Its separate collision
transform replaces only the 16 arm collision mesh filenames with the local
conservative hulls containing both original STL and authored USD source points.
It requires the expected source filenames and identity collision origins/scales;
all origins and ACM entries remain unchanged. The frozen intersection
configuration must be present; there is no fallback model or default interval.

On a fresh, isolated MoveIt scene, the environment source adds exactly
`task28_table` and `Cube_01`. Valid `world` PoseArrays with exactly one entry update
the Cube as a world collision object throughout carrying. It neither removes the
payload nor adds an ACM exception. An attached payload and closed-chain/FCL
validator remain G1-1 work. No other scene writer may share this namespace.

AST/XML parsing, offline xacro regeneration, source comparisons, limit checks and
independent conservative-mesh containment checks were performed in G1-0B. This package
was not built, installed, imported into ROS, or launched. Future runs require the
top-level fixture protocol, verified external asset/dependency closure, and
separate execution authorization. Runtime namespace behavior, message timing,
and MoveIt scene insertion remain unexecuted checks.

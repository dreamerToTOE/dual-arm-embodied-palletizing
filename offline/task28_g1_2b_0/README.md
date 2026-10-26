# Task28 G1-2B-0 offline sampled trajectory-edge validation

This isolated module reuses the unchanged frozen fixture, G1-1, G1-2A-R1 and
FCL. It checks prescribed trajectories as supplied; it does not plan, project
joint states onto a constraint manifold, run a simulator or command a robot.

`Trajectory` / `Waypoint` hold synthetic time, both joint configurations, shared
Cube SE(3), contact mode, independent intent/feedback metadata and explicit input
provenance. Joint interpolation is linear or explicitly specified cubic Hermite;
object translation is linear/Hermite, rotation follows the shortest SO(3)
geodesic. Independent arm interpolation is deliberately not assumed closed-chain.

`SampledEdgeValidator.validate` reports `endpoint_valid` (endpoint-only request),
`sampled_edge_valid`, `sampled_edge_rejected` or `edge_inconclusive`.
`continuous_safety` is always `NOT_ESTABLISHED_FINITE_SAMPLING_ONLY`.
The `StrongerVerifier` protocol is only a future extension seam, with no swept
or CCD implementation.

Every reconstructed sample retains the full Cube geometry, independent USD vs
URDF FK checks, limits, closure and full phase-specific FCL checks. Lift/place
samples use strict support geometry at the frozen site; separated samples use
`BILATERAL_TRANSITIONING` without support exceptions. No ACM changes occur.
Only explicit waypoint/prelude observations go to R1; interpolated samples never
contribute to its three-frame confirmation. A requested terminal confirmation
must be reached, or the edge is inconclusive. A placement endpoint can be
geometrically supported while still awaiting the explicit stationary support
observations that authorize release.

Run at the repository root:

```bash
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0 \
  python3 -u offline/task28_g1_2b_0/run_validation.py
```

The runner preserves and executes all 296 accepted tests and the new edge tests,
then writes `evidence/summary.json` and full per-trajectory `.jsonl` traces.
Each trace starts with the typed trajectory, records every evaluated sample and
explicit temporal observation, and ends with the result. Computation times are
local offline wall-clock measurements, not real ROS timing or execution latency.

The A-stage input is `examples/a_stage_waypoints.json`. Its separate bounded
single-state IK builder is optional and takes about 13 minutes on the recorded
host; ordinary reproduction uses the saved joint vectors. A quick audit is:

```bash
python3 offline/task28_g1_2b_0/examples/build_a_stage.py --revalidate-waypoints
```

This retains the solved q and original solve receipts and reruns frozen gates.
`examples/collision_control.json` is a real frozen-kinematics negative control:
valid endpoints, valid-closure interior Cube/table penetration of 20 mm. It is
an intentionally invalid candidate, not an executable trajectory.

Details: [G1-2B-0 report](../../docs/tasks/TASK28_G1_2B_0_EDGE_VALIDATION.md).

# Task28 G1-1 offline validator

Status: `G1_1_PASS` means **offline single-state scope only**.
The fixture remains `NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`.

This directory is independent of existing Task24/26/27 executables. It imports no
ROS or Isaac modules, publishes no messages and contains no trajectory planner,
NR projection, edge/swept-volume checker, QP, force control or training code.
The frozen G1-0B fixture is read-only and checked by its manifest hash plus every
listed file hash before model/collision construction.

From the repository root:

```bash
python3 offline/task28_g1_1/run_validation.py --output /tmp/task28_g1_1_results.json
```

This runs all tests, bounded deterministic IK controls and real standalone FCL
queries, then emits machine-readable quantitative residuals, rejection reasons,
source hashes and dependency identities. The delivered run is
[evidence/validation_results.json](evidence/validation_results.json). To run tests
alone:

```bash
PYTHONPATH=offline/task28_g1_1 python3 -m pytest offline/task28_g1_1/tests -q -p no:cacheprovider
```

Dependencies: Python 3.10+, the pinned NumPy/SciPy/pytest in `requirements.txt`,
g++/pkg-config, FCL 0.7.0 development libraries and their Eigen/CCD/OctoMap
requirements. Nothing is downloaded or installed by these commands. Only the
standalone FCL helper is compiled, in `/tmp/task28_g1_1_fcl/`; no robot workspace
is built. See [backend documentation](task28_validator/backend/README.md).

`T_A_B` maps column coordinates in B into A. Poses are full SE(3), translations
metres, rotations radians and input quaternions xyzw. `nominal_grasps()` defines
constant object-to-TCP transforms at the +/-Y Cube faces, with a 1 mm nominal
capture gap and the frozen side-grasp rotations. This is an explicit G1-1 derived
interface; no fixture value was changed. Exact face contact (`gap=0`) is a
separate mathematical/collision diagnostic. Neither profile certifies suction.

Core interfaces:

- `model.RobotModel`: frozen URDF tree FK, effective limit checks, geometric
  Jacobian and bounded trust-region-reflective IK. FK alone can evaluate an
  out-of-limit algebra control; `within_limits` and the integrated validator are
  the acceptance gates. IK success proves pose residual and limits, not collision
  safety. A local solve failure is a budget failure; only the loose geometric
  reach bound produces `IK_OUTSIDE_REACH_BOUND`.
- `closed_chain.evaluate_closure`: compares both TCP-implied object poses with
  each other and with the supplied object pose, using translation norm and full
  SO(3) log angle. A common object-pose bias is also rejected.
- `state.validate_state`: typed, immutable joint/TCP/Cube observations with
  synthetic timestamps, exact identities/frames and coherent attachment flags.
  Default maximum age/skew/future lead are 0.1/0.02/0.001 seconds; policy changes
  may only tighten these limits. This is a new offline contract, not ROS timing
  evidence. All observation frames are explicitly `world`, including the offline
  joint observation header; future ROS adapters must map their inputs to this
  contract deliberately.
- `collision.CollisionValidator`: all 30 robot subgeometries, carried Cube and
  table are solids. It checks 414 shape pairs and preserves the original SRDF
  exemptions. Intentional contact is a bounded state-local cup-cap policy;
  manifold, deep, wrong-face, detached and payload/environment collisions reject.
  It never adds ACM entries or ignores an entire tool/Cube pair.
- `validator.ClosedChainValidator`: requires the state gate, bilateral attachment,
  limits, FK observation consistency, full closure and collision to all pass.
  Structurally invalid states stop before FK; otherwise collision is evaluated
  even if a closure check already failed. A valid result covers one state only.

Independent checks use the frozen USD joint-frame audit rather than the URDF FK
implementation, closed-form pose controls, central-difference Jacobians, a
necessary reach bound and deliberately colliding solid geometry. Source-state
controls and all test inputs are version controlled. No conservative mesh was
shrunk to make a control pass.

The frozen initial/target Cube poses are also IK-reachable, but their exact
Cube/table support contact is rejected by the carried-state policy. Support and
release transitions need explicit future rules; there is no global table waiver.

Remaining G1-2 work includes trajectory-edge interpolation and swept validation,
contact-mode transitions, object/robot state coherence along every edge, required
sampling/error bounds and negative path controls. Future runtime observations
need verified clock-domain conversion, timing behavior, namespace routing and
physical contact validation. G1-1 grants none of that execution authorization.
Historical Task24-H numerical results remain historical report evidence only.

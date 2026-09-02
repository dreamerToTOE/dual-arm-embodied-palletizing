# Task 01 — Single-Arm Pick & Place

## Status

🟡 **In progress — stable grasp/lift validated**

The 30 mm cube grasp has now been retried multiple times and is stable. The FR3 reaches the aligned top-down pose, descends vertically, closes around the cube, and lifts it without losing the object.

Validated chain:

```text
HOME
  ↓
ALIGNED_APPROACH
  ↓
OPEN_GRIPPER
  ↓
CARTESIAN_Z_DESCENT
  ↓
CLOSE_GRIPPER
  ↓
MOVEIT_ATTACH
  ↓
CARTESIAN_Z_LIFT
```

Next chain to implement:

```text
CARTESIAN_Z_LIFT
  ↓
TRANSFER / PRE_PLACE
  ↓
CARTESIAN_Z_PLACE
  ↓
MOVEIT_DETACH
  ↓
OPEN_GRIPPER
  ↓
CARTESIAN_Z_RETREAT
```

---

## Scene

### FR3 base

```text
/World/fr3
Translate = (0, 0, 0)
Rotate    = (0, 0, 0)
```

Isaac `World` and the MoveIt `base` model frame are treated as aligned in the current baseline scene.

### Table

```text
Center = (0.55, 0.00, 0.025) m
Size   = (1.20, 0.80, 0.05) m
Top Z  = 0.05 m
```

### PickCube — current baseline

```text
Center = (0.45, 0.15, 0.065) m
Size   = (0.03, 0.03, 0.03) m
Yaw    = 0 rad
```

The 30 mm cube is intentionally easy to grasp. Task 01 is intended to validate the complete manipulation pipeline first; larger boxes will be reintroduced later for palletizing and placement-executability experiments.

---

## Coordinate-frame fix

A Cartesian-path failure was diagnosed as:

```text
Robot model frame        = base
Pose reference frame     = fr3_link0
Cartesian fraction       = -1.0000
MoveIt error_code        = -21 (FRAME_TRANSFORM_FAILURE)
```

The fix is to use the MoveIt model frame consistently:

```text
MoveIt pose reference frame = base
Table CollisionObject frame  = base
Cube CollisionObject frame   = base
```

After this change the Cartesian descent and lift became operational and repeatably successful.

---

## Grasp-orientation rule

A valid grasp is treated as a full 6D pose, not only a TCP position.

For the current axis-aligned cube:

```text
TCP +Z axis        -> vertically downward
Finger motion axis -> parallel to cube Y axis
Finger faces       -> aligned with the two cube side faces being grasped
```

The current top-down grasp rule is:

```text
R_grasp = Rz(cube_yaw) * Rx(pi)
```

For `cube_yaw = 0`:

```text
Quaternion (x,y,z,w) = (1,0,0,0)
```

The code parameterizes the quaternion from `CUBE_YAW`, so a later rotated box can reuse the same grasp-frame rule.

---

## Validated motion strategy

```text
HOME
  ↓
RRTConnect -> ALIGNED_APPROACH
  ↓
OPEN_GRIPPER
  ↓
remove pick_cube from MoveIt world for intentional contact
  ↓
Cartesian path: only Z decreases
  ↓
CLOSE_GRIPPER
  ↓
MoveIt AttachedCollisionObject
  ↓
Cartesian path: only Z increases
```

Current TCP targets:

```text
APPROACH z = 0.140 m
GRASP    z = 0.075 m
LIFT     z = 0.175 m
```

During Cartesian descent and lift:

```text
X           fixed
Y           fixed
orientation fixed
only Z      changes
```

Cartesian interpolation step:

```text
2 mm
```

The implementation requires an essentially complete Cartesian path (`fraction >= 0.999`) before execution.

---

## Gripper commands

```text
OPEN  = 0.040 m per finger joint  (~80 mm total opening)
CLOSE = 0.014 m per finger joint  (~28 mm total target opening)
```

The 28 mm command provides a small preload against the 30 mm cube. During lift, the closed command is continuously held while the arm trajectory is published.

---

## MoveIt attached object

After physical closure, `pick_cube` is represented as an attached collision object for carried-object collision checking.

```text
attached link = fr3_hand_tcp
object size   = 0.03 x 0.03 x 0.03 m
touch links   = fr3_hand_tcp, fr3_hand, fr3_leftfinger, fr3_rightfinger
```

Isaac Sim currently relies on real finger/cube contact. The repeated successful lift means an Isaac FixedJoint is not required for this baseline Task 01 grasp.

---

## Next: complete Pick & Place

The next implementation should finish Task 01 in one pass rather than adding more micro-stages.

Recommended baseline placement target on the same table:

```text
PLACE cube center = (0.65, -0.15, 0.065) m
```

Planned sequence:

```text
LIFT
  ↓
RRTConnect transfer while carrying the attached cube
  ↓
PRE_PLACE with the same top-down box-aligned orientation
  ↓
Cartesian Z-only descent to placement height
  ↓
MoveIt detach / restore cube as a world collision object at the target
  ↓
OPEN_GRIPPER
  ↓
Cartesian Z-only retreat
```

This completes the first deterministic single-arm Pick & Place demonstration.

---

## Current source

```text
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp
```

The next source revision should include the validated `base` frame fix plus the complete transfer/place/release/retreat chain.

---

## Validation status

```text
Task 00 MoveIt -> Isaac baseline      ✅
60 mm cube grasp                      ✅ when well aligned
60 mm robustness issue identified     ✅
30 mm Isaac cube scene                ✅
Explicit grasp orientation            ✅
Coordinate-frame fix (base)           ✅
ALIGNED_APPROACH                       ✅
Cartesian Z-only descent              ✅ repeated success
CLOSE_GRIPPER on 30 mm cube           ✅ repeated success
MOVEIT ATTACH                          ✅
Cartesian Z-only LIFT                 ✅ repeated success
TRANSFER                               Next
PRE_PLACE                              Next
PLACE                                  Next
DETACH                                 Next
OPEN / RELEASE                         Next
RETREAT                                Next
HOME / DONE                            Pending
```

Acceptance criterion for the next revision: after a stable grasp/lift, the FR3 carries the cube to a second table location, descends vertically without changing the grasp orientation, releases the cube, and retreats without collision.

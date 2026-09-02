# Task 01 — Single-Arm Pick & Place

## Status

🟡 **In progress**

The baseline manipulation strategy has been revised after the 60 mm cube lift test exposed sensitivity to small end-effector yaw/alignment errors.

Current test chain:

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

---

## Scene

### FR3 base

```text
/World/fr3
Translate = (0, 0, 0)
Rotate    = (0, 0, 0)
```

Isaac `World` and MoveIt `fr3_link0` are treated as aligned in the current baseline scene.

### Table

```text
Center = (0.55, 0.00, 0.025) m
Size   = (1.20, 0.80, 0.05) m
Top Z  = 0.05 m
```

### PickCube — current baseline

The user has now changed the Isaac Sim cube to the 30 mm baseline geometry.

```text
Center = (0.45, 0.15, 0.065) m
Size   = (0.03, 0.03, 0.03) m
Yaw    = 0 rad
```

The 30 mm cube is intentionally easy to grasp. Task 01 is intended to validate the complete manipulation pipeline first; larger boxes will be reintroduced later for palletizing and placement-executability experiments.

---

## Grasp-orientation rule

A valid grasp is treated as a full 6D pose, not only a TCP position.

For the current axis-aligned cube:

```text
TCP +Z axis        -> vertically downward
Finger motion axis -> parallel to cube Y axis
Finger faces       -> aligned with the two cube side faces being grasped
```

The Franka hand description defines both prismatic finger joints along the hand local Y axis. Therefore the current top-down orientation uses:

```text
R_grasp = Rz(cube_yaw) * Rx(pi)
```

For `cube_yaw = 0`:

```text
Quaternion (x,y,z,w) = (1,0,0,0)
```

The code parameterizes the quaternion from `CUBE_YAW`, so a later rotated box can reuse the same grasp-frame rule instead of hard-coding an arbitrary yaw.

---

## Revised motion strategy

The old final grasp step used a new unconstrained pose plan. It has now been replaced by a Cartesian straight-line motion.

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

The code requires an essentially complete Cartesian path (`fraction >= 0.999`) before execution.

---

## Gripper commands

```text
OPEN  = 0.040 m per finger joint  (~80 mm total opening)
CLOSE = 0.014 m per finger joint  (~28 mm total target opening)
```

The 28 mm target gives a small preload against the 30 mm test cube.

During the Cartesian descent the code continuously holds the gripper open; during the Cartesian lift it continuously holds the closed command while publishing the arm trajectory.

---

## MoveIt attached object

After the physical fingers close, `pick_cube` is represented as an attached collision object for carried-object collision checking.

```text
attached link = fr3_hand_tcp
object size   = 0.03 x 0.03 x 0.03 m
touch links   = fr3_hand_tcp, fr3_hand, fr3_leftfinger, fr3_rightfinger
```

At the current grasp pose, the physical cube center is 10 mm below the TCP. The MoveIt attached model uses a 9 mm TCP-frame offset so its lower face begins approximately 1 mm above the table and does not start the lift in an exact table-contact collision state.

Isaac Sim still uses physical finger/cube contact for this test. No FixedJoint is added yet. If the aligned 30 mm grasp still slips during lift, the first deterministic Pick & Place demo will use an Isaac logical attachment / Physics FixedJoint rather than spending time tuning contact indefinitely.

---

## Current source

```text
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp
```

Current implementation includes:

```text
30 mm cube parameters
+ grasp orientation derived from cube yaw
+ RRTConnect to aligned approach
+ Cartesian Z-only descent
+ close gripper
+ MoveIt attach
+ Cartesian Z-only lift
```

---

## Validation status

```text
Task 00 MoveIt -> Isaac baseline      ✅
60 mm cube grasp                      ✅ when well aligned
60 mm robustness issue identified     ✅
30 mm Isaac cube scene                ✅
Explicit grasp-orientation design      ✅ implemented
Cartesian Z-only descent               Testing
CLOSE_GRIPPER on 30 mm cube            Testing
MOVEIT ATTACH                           Testing
Cartesian Z-only LIFT                   Testing
TRANSFER                                Pending
PRE_PLACE                               Pending
PLACE                                   Pending
DETACH                                  Pending
RETREAT                                 Pending
HOME / DONE                             Pending
```

Immediate acceptance criterion: the FR3 reaches the aligned approach, descends vertically without changing yaw/orientation, closes around the 30 mm cube, and lifts it approximately 100 mm without losing the object.

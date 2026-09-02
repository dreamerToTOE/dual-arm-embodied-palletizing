# Task 01 — Single-Arm Pick & Place

## Status

🟡 **In progress**

Validated so far:

```text
HOME
  ↓
PRE_GRASP
  ↓
APPROACH
  ↓
OPEN_GRIPPER
  ↓
GRASP
  ↓
CLOSE_GRIPPER
```

The 60 mm cube can be grasped when alignment is good, but the lift test exposed insufficient robustness: small end-effector yaw/alignment errors can cause the fingers to miss or lose the cube.

---

## Scene

### FR3 base

```text
/World/fr3
Translate = (0, 0, 0)
Rotate    = (0, 0, 0)
```

Isaac `World` and MoveIt `fr3_link0` are treated as aligned in this scene.

### Table

```text
Center = (0.55, 0.00, 0.025) m
Size   = (1.20, 0.80, 0.05) m
```

---

## Revised grasp design decision

Two changes are adopted before continuing the lift/transfer sequence.

### 1. Reduce the test cube size

The current 60 mm cube is still too sensitive to small orientation errors for the baseline Pick & Place task.

For Task 01, the cube may be reduced to approximately half the current width:

```text
Recommended baseline cube size = 0.03 x 0.03 x 0.03 m
```

With the current table top at `z = 0.05 m`, the nominal cube center becomes:

```text
Cube center z = 0.05 + 0.03 / 2 = 0.065 m
```

This is a deliberate baseline simplification. The purpose of Task 01 is to validate the complete manipulation pipeline first; larger and more demanding box sizes will be reintroduced in later palletizing/executability experiments.

### 2. Treat grasp orientation as a first-class constraint

Reaching the target TCP position is not sufficient for a reliable two-finger grasp.

The grasp target must specify the full end-effector pose:

```text
position + orientation
```

For the axis-aligned cube, the desired grasp geometry is:

```text
1. TCP/tool Z axis points vertically downward.
2. The gripper finger opening/closing axis is parallel to a pair of cube side-face normals.
3. The finger faces are parallel to the two cube side faces being grasped.
4. X/Y and orientation remain fixed during the final descent.
```

The grasp orientation should be derived from the cube orientation plus a fixed gripper grasp-frame offset rather than treating yaw as arbitrary.

---

## Revised motion strategy

The preferred grasp approach becomes:

```text
HOME
  ↓
PRE_GRASP
  ↓
APPROACH WITH FULL GRASP ORIENTATION
  ↓
OPEN_GRIPPER
  ↓
CARTESIAN STRAIGHT-LINE DESCENT
  ↓
CLOSE_GRIPPER
  ↓
ATTACH / LIFT
```

The final descent should no longer be a new unconstrained RRTConnect pose-planning problem. Instead, use a Cartesian path from the aligned approach pose to the grasp pose while keeping orientation and X/Y fixed.

Conceptually:

```text
Approach pose:
(xg, yg, z_pre, R_grasp)

Final grasp pose:
(xg, yg, z_grasp, R_grasp)
```

Only `z` changes.

This directly enforces the intended behavior: once the gripper is correctly aligned above the object, it only moves vertically downward before closure.

---

## Why this change matters

The previous implementation guaranteed a target TCP pose numerically, but grasp success was still sensitive to the relative orientation between the two finger faces and the cube faces.

For a parallel-jaw gripper, robust grasping depends on both:

```text
TCP position accuracy
+
end-effector orientation / finger-face alignment
```

This will also be important later for palletizing placement executability, where the box and gripper swept volume must remain aligned with insertion clearances.

---

## MoveIt grasp-contact handling

Before intentional finger/object contact, the cube may still be removed from the MoveIt world collision objects or handled with an appropriate attached/touch-link representation.

After closure, the object should be represented as an attached collision object for subsequent lift/transfer collision checking.

---

## Current source

```text
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp
```

The next code revision should implement:

```text
30 mm baseline cube parameters
+ explicit grasp orientation
+ aligned approach pose
+ Cartesian final Z descent
+ close gripper
+ lift test
```

---

## Validation status

```text
HOME                         ✅
PRE_GRASP                    ✅
APPROACH                     ✅
OPEN_GRIPPER                 ✅
60 mm cube grasp             ✅ when alignment is favorable
Robustness issue identified  ✅
30 mm baseline cube          Next
Explicit grasp orientation   Next
Cartesian Z-only descent     Next
CLOSE_GRIPPER                Retest after revision
MOVEIT ATTACH                Pending after revision
LIFT                         Pending after revision
TRANSFER                     Pending
PRE_PLACE                    Pending
PLACE                        Pending
DETACH                       Pending
RETREAT                      Pending
HOME / DONE                  Pending
```

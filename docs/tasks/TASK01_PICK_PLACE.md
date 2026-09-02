# Task 01 — Single-Arm Pick & Place

## Status

🟡 **In progress**

Current validated chain:

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

The 60 mm cube is now physically grasped in Isaac Sim without being visibly pushed away or ejected.

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

### PickCube

The initial 80 mm cube was replaced with a 60 mm cube for practical Franka gripper clearance.

```text
Center = (0.45, 0.15, 0.08) m
Size   = (0.06, 0.06, 0.06) m
Mass   = 0.2 kg
```

Cube top surface:

```text
z = 0.11 m
```

---

## Current grasp sequence

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
  ↓
MOVEIT ATTACH
  ↓
LIFT
```

Current TCP targets:

```text
PRE_GRASP = 0.23 m
APPROACH  = 0.16 m
GRASP     = 0.10 m
LIFT      = 0.20 m
```

Current gripper commands:

```text
OPEN  = 0.040 m per finger joint
CLOSE = 0.030 m per finger joint
```

The grasp at `TCP z = 0.10 m` has been visually validated in Isaac Sim. The fingers reach the sides of the 60 mm cube, the gripper closes around it, and the cube is not visibly ejected.

---

## MoveIt grasp-contact handling

Before the final grasp descent, `pick_cube` is removed from the MoveIt world collision objects. This allows intentional finger/object contact without causing the final grasp plan to be rejected.

After the physical gripper closes, the current implementation creates an explicit MoveIt `AttachedCollisionObject`:

```text
attached link = fr3_hand_tcp
object id     = pick_cube
size          = 0.06 x 0.06 x 0.06 m
```

Touch links:

```text
fr3_hand_tcp
fr3_hand
fr3_leftfinger
fr3_rightfinger
```

The attached cube is expressed directly in the `fr3_hand_tcp` frame, avoiding dependence on the fake-hardware current joint state when determining the attachment transform.

The nominal physical cube center is 20 mm below the TCP at the grasp pose. The MoveIt attached model uses a 19 mm relative offset so that its lower face starts approximately 1 mm above the table, preventing an exact table-contact state from being interpreted as a collision at the start of lift planning.

---

## Isaac grasp strategy

No Isaac FixedJoint is used for the current lift test.

The physical cube remains a rigid body and is held by the actual simulated finger/cube contact. This is deliberately tested first because the current physical grasp is stable at rest.

If the cube slips or falls during lift, a logical Isaac attachment / Physics FixedJoint will be added as a deterministic fallback for the first Pick & Place demo.

---

## Current source

```text
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp
```

The source now implements:

```text
HOME
→ PRE_GRASP
→ APPROACH
→ OPEN
→ GRASP
→ CLOSE
→ MoveIt AttachedCollisionObject
→ LIFT 100 mm
```

---

## Validation status

```text
HOME                         ✅
PRE_GRASP                    ✅
APPROACH                     ✅
OPEN_GRIPPER                 ✅
60 mm cube geometry          ✅
GRASP z=0.10                 ✅
CLOSE_GRIPPER                ✅
Cube not visibly ejected     ✅
MOVEIT ATTACH                Testing
LIFT 100 mm                  Testing
TRANSFER                     Pending
PRE_PLACE                    Pending
PLACE                        Pending
DETACH                       Pending
OPEN_GRIPPER                 Pending
RETREAT                      Pending
HOME / DONE                  Pending
```

The immediate acceptance criterion is that MoveIt accepts the attached object, successfully plans the lift, and the physical Isaac cube remains held while the TCP rises from `z = 0.10 m` to `z = 0.20 m`.

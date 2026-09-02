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

Both MoveIt/RViz and Isaac Sim reach the expected grasp pose. The 60 mm cube is now visibly held between the Franka fingers without being pushed away or ejected.

---

## Scene

### FR3 base

```text
/World/fr3
Translate = (0, 0, 0)
Rotate    = (0, 0, 0)
```

The current scene therefore treats Isaac `World` and MoveIt `fr3_link0` as aligned.

### Table

```text
Center = (0.55, 0.00, 0.025) m
Size   = (1.20, 0.80, 0.05) m
```

### PickCube — revised and validated

The original 80 mm cube was unnecessarily close to the full Franka gripper opening and was replaced with a 60 mm test cube.

```text
Center = (0.45, 0.15, 0.08) m
Size   = (0.06, 0.06, 0.06) m
Mass   = 0.2 kg
```

The top surface is therefore at `z = 0.11 m`.

---

## Current motion sequence

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

No redundant intermediate grasp stages are used.

Current target TCP heights are:

```text
PRE_GRASP = 0.23 m
APPROACH  = 0.16 m
GRASP     = 0.10 m
```

The current gripper commands are:

```text
OPEN  = 0.040 m per finger joint
CLOSE = 0.030 m per finger joint
```

The 0.030 m close target is retained because the latest Isaac Sim test shows the 60 mm cube is stably positioned between the fingers and is not visibly pushed away.

---

## MoveIt grasp-contact handling

Before the intentional final descent into the grasp region, `pick_cube` is removed from the MoveIt world collision objects. This prevents intentional finger/object contact from being rejected as a collision during the final grasp plan.

The physical cube remains present in Isaac Sim, and the table remains active as a MoveIt collision object.

After successful closure, the cube will be restored to the MoveIt Planning Scene and attached to the Franka hand as an `AttachedCollisionObject` / `attachObject` state before planning the lift. This ensures later planning accounts for the carried box geometry.

Because the revised cube is now physically held by finger contact in Isaac Sim, the next test will first use the real simulated contact/friction during lift. An Isaac `PhysicsFixedJoint` will be kept only as a fallback if repeated lift tests show slipping or unreliable transport.

---

## Latest validation

Observed result in Isaac Sim:

- FR3 descended to the revised grasp pose at TCP `z = 0.10 m`.
- The open fingers entered the sides of the 60 mm cube correctly.
- The close command completed normally.
- The cube remained centered between the fingers.
- The cube was not knocked over, pushed away, or ejected.

This validates the first physical grasp geometry for Task 01.

---

## Validated items

```text
HOME                         ✅
PRE_GRASP                    ✅
APPROACH                     ✅
OPEN_GRIPPER                 ✅
60 mm cube scene update      ✅
GRASP z=0.10                 ✅
CLOSE_GRIPPER                ✅
MOVEIT ATTACH                Next
LIFT                         Next
TRANSFER                     Pending
PRE_PLACE                    Pending
PLACE                        Pending
DETACH                       Pending
OPEN_GRIPPER                 Pending
RETREAT                      Pending
HOME / DONE                  Pending
```

## Current source

```text
ros_ws/src/fr3_moveit_test/src/single_arm_pick_place.cpp
```

The current source is configured for the validated 60 mm cube and single-step final grasp descent.

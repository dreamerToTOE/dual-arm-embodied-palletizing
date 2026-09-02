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
```

Both MoveIt/RViz and the Isaac Sim FR3 reach the expected pose above the cube. The Franka gripper opening command has also been validated through `/joint_command`.

---

## 1. Scene

Isaac Sim scene:

```text
/World
├── fr3
├── Table
├── PickCube
└── ActionGraph
```

### FR3 base

```text
/World/fr3
Translate = (0, 0, 0)
Rotate    = (0, 0, 0)
```

Therefore the first implementation treats Isaac `World` and MoveIt `fr3_link0` as aligned for the current scene.

### Table

```text
Center = (0.55, 0.00, 0.025) m
Size   = (1.20, 0.80, 0.05) m
```

### PickCube — revised deterministic test geometry

The original 0.08 m cube is unnecessarily close to the Franka gripper maximum opening. For the first deterministic Pick & Place baseline, reduce the cube to 0.06 m.

```text
Center = (0.45, 0.15, 0.08) m
Size   = (0.06, 0.06, 0.06) m
Mass   = 0.2 kg
```

With the table top at `z = 0.05 m`, a 0.06 m cube resting on the table has center height:

```text
z = 0.05 + 0.06 / 2 = 0.08 m
```

If desired, initialize Isaac slightly above this value (for example `z = 0.082 m`) and let PhysX settle it onto the table; MoveIt should use the settled nominal center `z = 0.08 m`.

---

## 2. Program

Current ROS 2 executable:

```text
fr3_moveit_test single_arm_pick_place
```

Trajectory execution uses the validated 100 Hz interpolation adapter.

---

## 3. Validated poses

### HOME

```text
[0.0, -0.7, 0.0, -2.2, 0.0, 2.0, 0.8]
```

### Current validated near-object motion

The existing PRE_GRASP and APPROACH motions have been validated in RViz and Isaac Sim. The next implementation should not add multiple extra descent waypoints.

---

## 4. Latest validation

Validated:

- Planning Scene accepted Table + PickCube.
- RRTConnect successfully planned the near-object motion.
- RViz and Isaac Sim reached the expected pose above the cube.
- Gripper opening was manually validated using the two finger joint names.
- Both finger joints opened correctly.

---

## 5. Simplified grasp-sequence decision

Avoid over-segmenting the first Pick & Place baseline. Use a simple sequence:

```text
HOME
  ↓
NEAR_OBJECT
  ↓
OPEN_GRIPPER
  ↓
ONE DESCENT TO GRASP HEIGHT
  ↓
CLOSE_GRIPPER
  ↓
ATTACH
  ↓
LIFT
```

The earlier 0.08 m test cube is replaced with a 0.06 m cube to leave meaningful finger clearance.

For the revised cube:

```text
CUBE_SIZE = 0.06 m
CUBE_Z    = 0.08 m
```

A practical first final-grasp TCP target is approximately:

```text
z_grasp ≈ 0.10 m
```

This corresponds to:

```text
GRASP_Z_OFFSET = 0.02 m
```

The exact value can be adjusted once if the visual finger depth is slightly high or low, but the project should not introduce additional intermediate stages merely for tuning.

For a 0.06 m cube, the symmetric finger displacement corresponding to roughly a 0.06 m opening is about 0.03 m per finger. The first close command can therefore target approximately `0.028–0.030 m` per finger rather than the previous 0.039 m value.

Logical attach/detach remains the authoritative grasp mechanism for this first deterministic simulation baseline.

---

## 6. Remaining Task 01

```text
HOME                         ✅
NEAR_OBJECT                  ✅ baseline motion validated
OPEN_GRIPPER                 ✅ manually validated
DESCEND_TO_GRASP             Next — use one descent only
CLOSE_GRIPPER                Pending
ISAAC + MOVEIT ATTACH        Pending
LIFT                         Pending
TRANSFER                     Pending
PRE_PLACE                    Pending
PLACE                        Pending
MOVEIT + ISAAC DETACH        Pending
OPEN_GRIPPER                 Pending
RETREAT                      Pending
HOME / DONE                  Pending
```

Final Task 01 acceptance will require a complete deterministic pick/place cycle followed by repeated-cycle testing.

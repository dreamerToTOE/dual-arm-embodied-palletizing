# Task 01 — Single-Arm Pick & Place

## Status

🟡 **In progress**

Current validated chain:

```text
HOME
  ↓
PRE_GRASP
  ↓
OPEN_GRIPPER
  ↓
APPROACH
```

Both MoveIt/RViz and the Isaac Sim FR3 reach the expected pose above the cube, and the Franka gripper opening command has been validated through `/joint_command`.

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

Current nominal configuration:

```text
Center = (0.55, 0.00, 0.025) m
Size   = (1.20, 0.80, 0.05) m
```

The table is represented as a static collider in Isaac and as a `CollisionObject` in MoveIt.

### PickCube

Current nominal configuration:

```text
Center = (0.45, 0.15, 0.09) m
Size   = (0.08, 0.08, 0.08) m
Mass   = 0.2 kg
```

The cube has collision + rigid-body physics in Isaac and is also added to the MoveIt Planning Scene.

---

## 2. Program

Current ROS 2 executable:

```text
fr3_moveit_test single_arm_pick_place
```

Current implementation responsibilities:

1. connect to `/move_group`;
2. copy `robot_description` and `robot_description_semantic`;
3. create `MoveGroupInterface("fr3_arm")`;
4. select `RRTConnectkConfigDefault`;
5. add Table and PickCube to the MoveIt Planning Scene;
6. plan HOME → PRE_GRASP;
7. execute the trajectory on Isaac through `/joint_command`;
8. explicitly open the gripper;
9. use the PRE_GRASP trajectory endpoint as the next planning start state;
10. plan PRE_GRASP → APPROACH;
11. execute APPROACH on Isaac.

Trajectory execution continues to use the validated 100 Hz interpolation adapter.

---

## 3. Validated poses

### HOME

```text
[0.0, -0.7, 0.0, -2.2, 0.0, 2.0, 0.8]
```

### PRE_GRASP

Cube center:

```text
(0.45, 0.15, 0.09) m
```

Pre-grasp offset:

```text
+0.15 m in Z
```

Target TCP position:

```text
(0.45, 0.15, 0.24) m
```

### APPROACH

Approach offset:

```text
+0.08 m in Z from cube center
```

Target TCP position:

```text
(0.45, 0.15, 0.17) m
```

The TCP remains approximately 0.04 m above the top surface of the current 0.08 m cube.

---

## 4. Latest validation result

Observed program output includes:

```text
Planning HOME -> PRE_GRASP...
PRE_GRASP planning SUCCESS.
PRE_GRASP execution COMPLETE.

Planning PRE_GRASP -> APPROACH...
APPROACH planning SUCCESS.
APPROACH execution COMPLETE.

Current Pick & Place stage COMPLETE:
HOME -> PRE_GRASP -> APPROACH
```

Validation:

- Planning Scene accepted Table + PickCube.
- No relevant collision failure occurred.
- RRTConnect successfully planned both stages.
- RViz robot reached the expected cube-above pose.
- Isaac Sim FR3 followed the planned motion and reached the same stage successfully.
- Gripper opening was manually validated using:

```bash
ros2 topic pub --once \
/joint_command sensor_msgs/msg/JointState \
"{name: ['fr3_finger_joint1','fr3_finger_joint2'], position: [0.04,0.04]}"
```

- Both finger joints opened correctly in Isaac Sim.

---

## 5. Grasp-sequence decision

The grasp sequence is intentionally kept simple. There is no need to introduce multiple redundant descent stages.

The intended logic is:

```text
PRE_GRASP / near-object pose
  ↓
OPEN_GRIPPER
  ↓
FINAL_DESCENT_TO_GRASP_HEIGHT
  ↓
CLOSE_GRIPPER
  ↓
ATTACH
  ↓
LIFT
```

The already validated `APPROACH` pose can be retained as the near-object pose if desired. From that pose, only one final descent to the calibrated grasp height is needed before closing the fingers.

The purpose of the final descent is not to add an unnecessary motion stage; it is simply to place the open fingers alongside the cube before closure. If the chosen near-object pose already places the fingers at the correct grasp depth, this extra descent can be removed entirely.

### Important geometry note

The current cube width is `0.08 m`, which is approximately the full opening scale of the Franka parallel gripper. That leaves essentially no insertion clearance in an idealized top-down grasp. The first project version therefore continues to use logical attach/detach for reliable transport, while still commanding the gripper open/closed for visually and logically consistent state transitions.

---

## 6. Remaining Task 01 sequence

```text
HOME                         ✅
PRE_GRASP                    ✅
OPEN_GRIPPER                 ✅
APPROACH                     ✅
FINAL_DESCENT_TO_GRASP       Next
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

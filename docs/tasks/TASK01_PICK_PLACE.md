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

### PickCube

```text
Center = (0.45, 0.15, 0.09) m
Size   = (0.08, 0.08, 0.08) m
Mass   = 0.2 kg
```

---

## 2. Program

Current ROS 2 executable:

```text
fr3_moveit_test single_arm_pick_place
```

Current implementation:

1. connect to `/move_group`;
2. copy `robot_description` and `robot_description_semantic`;
3. create `MoveGroupInterface("fr3_arm")`;
4. select `RRTConnectkConfigDefault`;
5. add Table and PickCube to the MoveIt Planning Scene;
6. plan HOME → PRE_GRASP;
7. execute through `/joint_command`;
8. use the previous trajectory endpoint as the next MoveIt start state;
9. plan PRE_GRASP → APPROACH;
10. execute APPROACH on Isaac.

Trajectory execution uses the validated 100 Hz interpolation adapter.

---

## 3. Validated poses

### HOME

```text
[0.0, -0.7, 0.0, -2.2, 0.0, 2.0, 0.8]
```

### PRE_GRASP

```text
TCP = (0.45, 0.15, 0.24) m
```

### APPROACH / open-gripper pose

```text
TCP = (0.45, 0.15, 0.17) m
```

This pose is now treated as the practical near-object pose at which the gripper is explicitly opened.

---

## 4. Latest validation

```text
Planning HOME -> PRE_GRASP...
PRE_GRASP planning SUCCESS.
PRE_GRASP execution COMPLETE.

Planning PRE_GRASP -> APPROACH...
APPROACH planning SUCCESS.
APPROACH execution COMPLETE.
```

Validated:

- Planning Scene accepted Table + PickCube.
- RRTConnect successfully planned both motion stages.
- RViz and Isaac Sim reached the same expected pose above the cube.
- Gripper opening was manually validated with:

```bash
ros2 topic pub --once \
/ joint_command sensor_msgs/msg/JointState \
"{name: ['fr3_finger_joint1','fr3_finger_joint2'], position: [0.04,0.04]}"
```

Both finger joints opened correctly.

---

## 5. Final grasp-sequence decision

The grasp sequence is simplified to one near-object pose plus one final descent. No redundant intermediate descent stages are required.

```text
HOME
  ↓
PRE_GRASP
  ↓
APPROACH / near-object pose
  ↓
OPEN_GRIPPER
  ↓
DESCEND_TO_GRASP
  ↓
CLOSE_GRIPPER
  ↓
ATTACH
  ↓
LIFT
```

The current `APPROACH` pose at `z = 0.17 m` is used as the gripper-opening pose. After opening, the robot performs exactly one final vertical descent to the calibrated grasp height, then closes the fingers.

The logical attach operation will provide reliable object transport in the first deterministic demo; finger closure is still commanded for visually and logically consistent manipulation.

---

## 6. Remaining Task 01

```text
HOME                         ✅
PRE_GRASP                    ✅
APPROACH                     ✅
OPEN_GRIPPER                 ✅ manually validated
DESCEND_TO_GRASP             Next
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

# Dual-Arm Embodied Palletizing

面向混合码垛任务的双机械臂具身智能协调规划与控制项目。

当前工程采用 **Isaac Sim 4.5 + ROS 2 Humble + MoveIt 2 / OMPL + Franka FR3** 作为主要仿真与运动规划平台。

## Project documents

- [Master project roadmap](docs/PROJECT_PLAN.md)
- [Task 00 — FR3 / Isaac / ROS 2 / MoveIt baseline](docs/tasks/TASK00_BASELINE.md)
- [Task 01 — Single-arm Pick & Place](docs/tasks/TASK01_PICK_PLACE.md)

`docs/PROJECT_PLAN.md` is the source-of-truth roadmap for later development and thesis mapping.

## Current progress

| Task | Description | Status |
|---|---|---|
| Task 00 | FR3 + Isaac Sim + ROS 2 + MoveIt 2 baseline | ✅ Complete |
| Task 01 | Single-arm Pick & Place | 🟡 In progress: HOME → PRE_GRASP → APPROACH validated |
| Task 02 | Placement Skill / robot-executable placement | Planned |
| Task 03 | B-A-C placement executability | Planned |
| Task 04 | Placement sequence adjustment | Planned |
| Task 05+ | Dual-arm loose/tight coordination and embodied skill routing | Planned |

## Validated baseline

```text
MoveIt 2 / OMPL
        ↓
RRTConnect
        ↓
RobotTrajectory / JointTrajectory
        ↓
moveit_to_isaac / pick-place executor
        ↓
100 Hz interpolation
        ↓
/joint_command
        ↓
Isaac Sim ROS 2 Bridge
        ↓
Articulation Controller
        ↓
FR3 simulation
```

Isaac Sim publishes robot state through ROS 2:

```text
/clock
/joint_states
/tf
```

## Environment baseline

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Isaac Sim 4.5.0
- NVIDIA GeForce RTX 3070 8 GB
- NVIDIA 580-series driver
- Python 3.10.12 (`/usr/bin/python3`)
- Franka ROS 2 `humble`
- `franka_description` 2.8.1
- `libfranka` 0.20.4
- MoveIt 2 / OMPL

## Current Task 01 state

Validated so far:

```text
HOME
  ↓
PRE_GRASP   TCP ≈ (0.45, 0.15, 0.24)
  ↓
APPROACH    TCP ≈ (0.45, 0.15, 0.17)
```

Both MoveIt/RViz and Isaac Sim execute the stages successfully.

Next implementation sequence:

```text
OPEN_GRIPPER
↓
GRASP_DESCENT
↓
CLOSE_GRIPPER
↓
ATTACH
↓
LIFT
↓
TRANSFER
↓
PRE_PLACE
↓
PLACE
↓
DETACH
↓
OPEN_GRIPPER
↓
RETREAT
```

The first version intentionally uses known object pose + deterministic attach/detach. Real contact grasping and RGB-D perception are deferred until the motion-planning and placement-executability pipeline is stable.

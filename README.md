# Dual-Arm Embodied Palletizing

面向混合码垛任务的双机械臂具身智能协调规划与控制项目。

当前工程采用 **Isaac Sim 4.5 + ROS 2 Humble + MoveIt 2 / OMPL + Franka FR3** 作为主要仿真与运动规划平台。

## Current progress

| Task | Description | Status |
|---|---|---|
| Task 00 | FR3 + Isaac Sim + ROS 2 + MoveIt 2 baseline | ✅ Complete |
| Task 01 | Single-arm Pick & Place | ⏭️ Next |
| Task 02 | Placement Skill / robot-executable placement | Planned |
| Task 03 | B-A-C placement executability and sequence adjustment | Planned |
| Task 04+ | Dual-arm loose/tight coordination and embodied skill routing | Planned |

## Validated baseline

```text
MoveIt 2 / OMPL
        ↓
RRTConnect
        ↓
RobotTrajectory / JointTrajectory
        ↓
moveit_to_isaac
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

Isaac Sim also publishes the robot state through ROS 2:

```text
/clock
/joint_states
/tf
```

The complete Task 00 procedure, dependency setup, troubleshooting notes, and acceptance results are recorded in:

[`docs/tasks/TASK00_BASELINE.md`](docs/tasks/TASK00_BASELINE.md)

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

## Next milestone

Task 01 implements a deterministic single-arm Pick & Place pipeline using a known cube pose:

```text
HOME
↓
Pre-Grasp
↓
Approach
↓
Close Gripper
↓
Attach
↓
Lift
↓
Move
↓
Pre-Place
↓
Place
↓
Detach
↓
Open Gripper
↓
Retreat
```

The first version intentionally uses known object pose + attach/detach. Real contact grasping and RGB-D perception are deferred until the motion-planning and placement-executability pipeline is stable.

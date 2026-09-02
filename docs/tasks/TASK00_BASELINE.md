# Task 00 — FR3 + Isaac Sim + ROS 2 + MoveIt 2 Baseline

## Status

✅ **Complete**

This task establishes the minimum verified simulation and motion-planning pipeline for the thesis project before starting palletizing skills.

---

## 1. Environment

Validated environment:

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Python 3.10.12 (`/usr/bin/python3`)
- Isaac Sim 4.5.0
- NVIDIA GeForce RTX 3070 8 GB
- NVIDIA 580-series driver
- MoveIt 2 / OMPL
- Franka ROS 2: `humble` branch
- `franka_description`: 2.8.1
- `libfranka`: 0.20.4

### Python environment isolation

ROS 2 is run outside Conda. The validated ROS terminal is:

```bash
source /opt/ros/humble/setup.bash
which python3
python3 --version
which ros2
```

Expected output:

```text
/usr/bin/python3
Python 3.10.12
/opt/ros/humble/bin/ros2
```

This avoids mixing Miniconda Python with ROS 2 Humble system packages.

---

## 2. Isaac Sim stability

Isaac Sim 5.1 previously crashed during RTX renderer initialization. After switching the NVIDIA driver and using Isaac Sim 4.5.0, the simulator starts normally and remains stable.

The project therefore freezes the current simulation baseline at:

```text
Ubuntu 22.04.5
+ ROS 2 Humble
+ Isaac Sim 4.5.0
+ NVIDIA 580-series driver
```

No further driver or Isaac Sim upgrades should be made unless required by a later task.

---

## 3. FR3 model in Isaac Sim

The built-in Isaac Sim FR3 asset is used directly instead of re-importing URDF:

```text
Robots/Franka/FR3/fr3.usd
```

Validated properties:

- Articulation Root exists.
- `fr3_joint1` through `fr3_joint7` exist.
- Franka hand and finger joints exist.
- `fr3_hand_tcp` exists.
- Fixed base behaves correctly.
- Play/Stop is stable.
- Joint-position controller can independently move all seven arm joints.
- Gripper open/close motion works.

The project scene uses the Isaac asset as a reference rather than modifying the original asset.

---

## 4. ROS 2 Bridge

The Isaac ROS 2 Action Graph was configured and validated.

Validated topics:

```text
/clock
/joint_states
/tf
/joint_command
```

### `/joint_states`

The actual joint order published by Isaac Sim is:

```text
fr3_joint1
fr3_joint2
fr3_joint3
fr3_joint4
fr3_joint5
fr3_joint6
fr3_joint7
fr3_finger_joint1
fr3_finger_joint2
```

This naming is used as the reference when connecting MoveIt 2 to Isaac Sim.

### Joint command path

Isaac subscribes to:

```text
/joint_command
```

with message type:

```text
sensor_msgs/msg/JointState
```

The command is passed to the Isaac Articulation Controller.

Validated command path:

```text
ROS 2 /joint_command
        ↓
ROS2 Subscribe Joint State
        ↓
Isaac Articulation Controller
        ↓
FR3 articulation
```

---

## 5. Franka ROS 2 and MoveIt installation

The official Franka ROS 2 Humble repository is used:

```bash
git clone -b humble https://github.com/frankarobotics/franka_ros2.git
```

The official dependency file is used to keep compatible versions:

```bash
vcs import src < src/franka_ros2/dependency.repos --recursive --skip-existing
```

Relevant pinned dependencies:

```text
franka_description 2.8.1
libfranka          0.20.4
```

`libfranka` requires its `common` Git submodule. If the build reports that `libfranka/common` has no `CMakeLists.txt`, initialize it with:

```bash
cd src/libfranka
git submodule sync --recursive
git submodule update --init --recursive
```

Workspace dependencies are installed with:

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y
```

The workspace is then built using:

```bash
colcon build \
  --symlink-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTS=OFF
```

The complete workspace compiled successfully.

---

## 6. Official FR3 MoveIt configuration

No custom MoveIt Setup Assistant configuration is required.

The official package is used directly:

```text
franka_fr3_moveit_config
```

Validated launch command:

```bash
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

Validated results:

- RViz starts normally.
- `/move_group` is available.
- Planning group `fr3_arm` is available.
- FR3 robot model loads correctly.
- OMPL planning succeeds.
- `RRTConnectkConfigDefault` is present.
- RViz `Plan` succeeds.

The command:

```bash
ros2 param get /move_group planning_pipelines
```

returns `Parameter not set` in this configuration. This is not treated as an error because `/move_group` contains the OMPL planner configuration and RRTConnect is explicitly present:

```text
RRTConnectkConfigDefault:
  type: geometric::RRTConnect
```

and actual planning succeeds.

---

## 7. MoveIt → Isaac trajectory adapter

The project reuses the already validated `moveit_to_isaac` architecture from the previous FR3 integration prototype.

The node performs the following steps:

1. Wait for `/move_group` parameter service.
2. Copy `robot_description` and `robot_description_semantic` from `/move_group` into the adapter node.
3. Create `MoveGroupInterface("fr3_arm")`.
4. Select `RRTConnectkConfigDefault`.
5. Plan a collision-aware `RobotTrajectory` / `JointTrajectory`.
6. Interpolate trajectory points according to `time_from_start` at 100 Hz.
7. Publish the interpolated joint positions to `/joint_command`.
8. Isaac Sim receives the commands through the Articulation Controller.

Validated pipeline:

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
Isaac ROS 2 Bridge
        ↓
Articulation Controller
        ↓
FR3 physical simulation
```

### Baseline test configuration

Start configuration:

```text
HOME = [0.0, -0.7, 0.0, -2.2, 0.0, 2.0, 0.8]
```

Goal configuration:

```text
Pose A = [0.3, -0.8, 0.2, -2.3, 0.2, 2.1, 0.6]
```

### Result

✅ MoveIt planning succeeded.

✅ `/joint_command` was received by Isaac Sim.

✅ The FR3 in Isaac Sim moved smoothly from HOME toward Pose A.

✅ Behavior matched the previously validated FR3 + Isaac + MoveIt prototype.

Therefore the motion-planning-to-simulation execution chain is considered restored and validated on the new workstation.

---

## 8. Task 00 acceptance

| Item | Result |
|---|---|
| Isaac Sim 4.5 starts stably | ✅ |
| Built-in FR3 asset loads correctly | ✅ |
| FR3 Articulation Root | ✅ |
| Seven arm joints controllable | ✅ |
| Gripper controllable | ✅ |
| ROS 2 `/clock` | ✅ |
| ROS 2 `/joint_states` | ✅ |
| ROS 2 `/tf` | ✅ |
| ROS 2 `/joint_command` | ✅ |
| Franka ROS 2 Humble builds | ✅ |
| Official `franka_fr3_moveit_config` launches | ✅ |
| `fr3_arm` planning group | ✅ |
| OMPL / RRTConnect planning | ✅ |
| MoveIt trajectory converted to `/joint_command` | ✅ |
| Isaac FR3 executes planned trajectory | ✅ |

## Conclusion

**Task 00 is complete.**

The project now has a validated baseline for the next engineering stage:

```text
Task 01 — Single-Arm Pick & Place
```

The first Pick & Place version will use a known cube pose and deterministic attach/detach behavior before introducing real contact grasping or RGB-D perception.

# Master Project Plan — Dual-Arm Embodied Palletizing

> **Source of truth for the project roadmap.**
>
> This document records the intended research architecture, implementation sequence, task boundaries, and thesis mapping. When the project direction changes, update this file first so later work can recover the plan directly from GitHub.

## 1. Research objective

Working thesis direction:

**面向混合码垛任务的双机械臂具身智能协调规划与控制方法研究**

The project targets mixed-size palletizing with two Franka FR3 manipulators and combines:

- robot-executable palletizing / placement planning;
- single-arm and dual-arm motion planning;
- loose dual-arm coordination and collision avoidance;
- tight dual-arm cooperative manipulation with force control;
- a hierarchical embodied decision layer that selects and executes reusable skills;
- closed-loop feedback and recovery in Isaac Sim.

The project is **simulation-first**. Physical experiments and learning-based policies are optional rather than prerequisites for completing the core thesis pipeline.

---

## 2. System architecture

```text
Environment / task state
        ↓
Embodied state representation
        ↓
High-level decision / skill router
        ↓
Skill library
        ↓
Motion / force execution
        ↓
Isaac Sim physical interaction
        ↓
State + contact + task feedback
        └──────────────→ re-decision / recovery
```

State representation:

```text
S_t = {
  S_env,
  S_robot,
  S_contact,
  S_task
}
```

The first implementation can use Isaac ground-truth state. RGB-D perception can be added later without changing the high-level architecture.

### Planned skill library

- `E_S`: SingleArmPickPlace
- `E_L`: LooseCoordination
- `E_T`: TightCoordinationForceControl
- `E_P`: PlacementSkill
- `E_R`: RecoverySkill

High-level routing can initially be a deterministic rule/FSM controller:

```text
E_t = pi_R(S_t, G)
```

No reinforcement learning is required for the first complete system.

---

## 3. Core engineering principles

### 3.1 Planning and execution are different layers

MoveIt / OMPL performs collision-aware planning. Isaac Sim performs physical execution.

Validated baseline:

```text
MoveIt 2 / OMPL
        ↓
RobotTrajectory / JointTrajectory
        ↓
ROS 2 trajectory adapter
        ↓
100 Hz /joint_command
        ↓
Isaac Articulation Controller
        ↓
FR3 simulation
```

### 3.2 Geometric feasibility is not enough

A palletizing target pose is only valid if the robot can actually execute the placement while considering:

- box geometry;
- gripper geometry;
- safety clearance;
- insertion direction;
- swept-volume collision;
- release and retreat feasibility.

For placement between existing boxes B and C, a simplified clearance condition can be written as:

```text
d_BC >= w_A + 2 t_g + 2 delta
```

A complete placement motion is treated as:

```text
T_pre → T_insert → T_place → Release → T_retreat
```

and placement executability as:

```text
C_place = C_reach * C_insert * C_release * C_retreat
```

If a candidate cannot be executed, the system may:

1. change placement sequence;
2. change grasp direction;
3. change coordination mode;
4. reject the candidate placement.

### 3.3 Loose and tight coordination are distinct modes

Loose coordination focuses on independent / parallel arm execution with collision and timing constraints.

Tight coordination is cooperative manipulation of one object and is treated as a **force-control problem**, not only a trajectory synchronization problem.

Planned tight-control structure:

```text
Manipulator dynamics
        ↓
Grasp matrix G
        ↓
Object-motion wrench + internal grasp wrench decomposition
        ↓
Desired internal gripping force
        ↓
Object impedance + internal-force feedback
        ↓
J^T wrench-to-joint-torque mapping
```

A first implementation can use hybrid position/force control: force control along the gripping normal and motion control along the remaining directions.

---

## 4. Implementation roadmap

| Task | Engineering milestone | Main acceptance target | Status |
|---|---|---|---|
| 00 | FR3 + Isaac + ROS 2 + MoveIt baseline | MoveIt trajectory executes on Isaac FR3 | ✅ Complete |
| 01 | Single-arm Pick & Place | Full deterministic pick/place cycle | 🟡 In progress |
| 02 | Placement Skill | Pre-insert / insert / place / release / retreat | Planned |
| 03 | B-A-C placement executability | Detect gripper/box insertion infeasibility | Planned |
| 04 | Sequence adjustment | Reorder placements when later insertion is blocked | Planned |
| 05 | Dual-FR3 baseline | Two FR3 models, namespaces, planning groups and commands | Planned |
| 06 | Loose parallel execution | Two arms execute independent tasks concurrently | Planned |
| 07 | Spatiotemporal conflict detection | Detect arm-arm / carried-object conflicts over time | Planned |
| 08 | Master-slave local replanning | Resolve detected conflicts without full global replanning | Planned |
| 09 | Loose-coordination experiments | Repeatable simulation metrics and comparison cases | Planned |
| 10 | Tight shared-object baseline | Two arms contact / constrain one common box | Planned |
| 11 | Wrench sensing | Obtain / validate contact or joint-derived wrench signals | Planned |
| 12 | Grasp matrix + internal force | Compute object wrench and internal force components | Planned |
| 13 | Internal-force control | Track desired gripping force without changing object wrench | Planned |
| 14 | Object motion + force control | Cooperative transport with motion + internal-force regulation | Planned |
| 15 | Skill library | Wrap validated behaviors as reusable skills | Planned |
| 16 | High-level skill router | Rule/FSM mode selection from task state | Planned |
| 17 | Recovery behavior | Handle planning failure, blocked placement, failed grasp, etc. | Planned |
| 18 | Full embodied demo | Closed-loop mixed palletizing sequence | Planned |
| 19 | Optional RGB-D perception | Replace selected ground-truth state with perception output | Optional |
| 20 | Batch experiments | Quantitative simulation evaluation for thesis | Planned |

---

## 5. Current Task 01 plan

Single-arm deterministic Pick & Place:

```text
HOME
  ↓
PRE_GRASP
  ↓
OPEN_GRIPPER
  ↓
APPROACH
  ↓
GRASP_DESCENT
  ↓
CLOSE_GRIPPER
  ↓
ATTACH (Isaac + MoveIt)
  ↓
LIFT
  ↓
TRANSFER
  ↓
PRE_PLACE
  ↓
PLACE
  ↓
DETACH (MoveIt + Isaac)
  ↓
OPEN_GRIPPER
  ↓
RETREAT
  ↓
HOME / DONE
```

The first version deliberately uses:

- known object pose;
- MoveIt collision objects;
- logical attach/detach for reliable transport;
- physical gripper opening/closing for visible state consistency;
- no vision requirement;
- no learned grasping requirement.

Task 01 should eventually be repeat-tested for 10 consecutive cycles. The target is a test criterion, not a pre-declared thesis result.

---

## 6. Planned thesis mapping

A working chapter structure is:

### Chapter 2 — Robot-executable mixed palletizing planning

- palletizing representation / candidate placement generation;
- support and stability constraints;
- gripper-aware placement executability;
- insertion / release / retreat feasibility;
- sequence adjustment.

### Chapter 3 — Dual-arm kinematics and loose collision-free motion planning

- kinematics and Jacobian foundations;
- collision models;
- RRT / RRT-Connect and improved planning;
- temporal conflict detection;
- master-slave collision avoidance and local replanning.

### Chapter 4 — Tight coordinated manipulation and force control

- dual-arm/object dynamics;
- grasp matrix;
- external/object-motion wrench and internal-force decomposition;
- desired gripping-force generation;
- object impedance and internal-force feedback;
- Jacobian-transpose torque mapping.

### Chapter 5 — Hierarchical embodied decision and skill execution

- embodied state;
- skill library;
- loose/tight mode selection;
- placement/recovery skills;
- closed-loop feedback and re-decision.

### Chapter 6 — Simulation experiments

- placement executability experiments;
- loose-coordination experiments;
- tight force-control experiments;
- integrated embodied palletizing demonstrations.

---

## 7. Project workspace conventions

Main project root:

```text
~/lmy/dual-arm-embodied-palletizing
```

ROS workspace:

```text
~/lmy/dual-arm-embodied-palletizing/ros_ws
```

Expected repository structure:

```text
dual-arm-embodied-palletizing/
├── README.md
├── docs/
│   ├── PROJECT_PLAN.md
│   └── tasks/
├── configs/
├── assets/
├── isaac/
│   ├── scenes/
│   └── scripts/
├── ros_ws/
│   └── src/
├── experiments/
├── results/
└── scripts/
```

Generated build products (`build/`, `install/`, `log/`) and large raw experiment outputs should not be committed.

---

## 8. Documentation rule

After each validated milestone:

1. update the corresponding `docs/tasks/TASKxx_*.md` file;
2. update the status table in this roadmap if task status changes;
3. update `README.md` for major milestone changes;
4. keep failed attempts / important debugging notes when they are useful for reproducibility.

This GitHub roadmap is intended to make the project recoverable even if conversational context is lost.

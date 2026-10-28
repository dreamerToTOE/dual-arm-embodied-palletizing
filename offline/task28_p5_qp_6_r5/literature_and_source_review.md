# R5 literature and installed-source review

## Published references

1. [PhysX 5.7 Simulation](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/Simulation.html): TGS divides a frame into position substeps; the documented steady-state example explains why a driven joint can retain a nonzero final velocity when gravity is applied once per full frame, and identifies `eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS` as the comparison flag. The same page states that reported velocities need not match position differences when split-impulse position/velocity constraints are involved.
2. [NVIDIA forum: DOF Velocity Offset at Rest](https://forums.developer.nvidia.com/t/dof-velocity-offset-at-rest/205799): a 2022 discussion reports the same position/velocity symptom and an NVIDIA explanation that TGS substep/final-velocity semantics can produce it; this is historical forum evidence, not a proof for the installed FR3 runtime.
3. [PhysX 5.1.1 Rigid Body Dynamics](https://nvidia-omniverse.github.io/PhysX/physx/5.1.1/docs/RigidBodyDynamics.html): velocities are affected by gravity, forces and constraints; applied forces affect acceleration and are accumulated per simulation frame.
4. [Isaac Sim 4.5 Core Prims API](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.prims/docs/index.html?highlight=UsdGeom%2520Xformable): articulation positions/velocities default to cloned buffers.

## Installed Isaac Sim 4.5 observations

- `isaacsim.core.prims` `ArticulationView.get_joint_positions/velocities` calls `_physics_view.get_dof_positions/get_dof_velocities`, with `clone=True` by default; numpy `clone_tensor` uses `np.copy`.
- `SimulationContext.step(render=False)` runs `simulate` then `fetch_results`.
- `PhysxSchema.PhysxSceneAPI` exposes `Get/ CreateEnableExternalForcesEveryIterationAttr`, plus solver type and min/max position/velocity iteration attributes. This means R5 C is supported as a documented temporary stage variant.
- `omni.physx` interface does not expose a Python method to toggle the flag after scene creation; C sets the authored anonymous-stage attribute before context creation.
- `SimulationContext` CPU construction selects MBP and disables GPU dynamics from its CPU device branch; R5 records this adaptation.
- Dynamic Control remains deprecated and is treated only as same-backend corroboration.

## Evidence limits

The published TGS example is a simple prismatic system and PhysX 5.7, while this host uses Isaac Sim 4.5/PhysX extension 106.5.7 and a coupled 9-DOF FR3 scene. R5 therefore tests the hypothesis in the installed runtime and does not promote the literature explanation into a control certificate by itself. R4 remains a valid negative result under its original finite-difference source contract.

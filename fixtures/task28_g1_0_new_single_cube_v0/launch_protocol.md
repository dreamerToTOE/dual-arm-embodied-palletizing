# Reproducibility and future launch protocol

Fixture: `task28_g1_0_new_single_cube_v0`
Status: `NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`
**No build, Isaac launch, ROS node, MoveIt launch or robot command in this protocol
was executed during G1-0B. Only the offline checks in step 2 were performed.**

1. Check out the delivered commit of `side-suction-palletizing` in an isolated
   directory. Set `TASK28_FIXTURE` to its absolute
   `fixtures/task28_g1_0_new_single_cube_v0` directory. Verify the manifest's
   repository source hashes and fixture hashes; the manifest itself is bound by
   the Git commit, avoiding a circular self-hash. Preserve the existing workspace.

2. Run the static checks first. Dependencies for the default checker are Python
   3.10+ standard library and the recorded external `franka_description` tree.
   A relocated tree can be supplied with `--external-model-root`.

   ```bash
   python3 "$TASK28_FIXTURE/tests/static_check.py" \
     --external-model-root /absolute/path/to/franka_description \
     --asset-file /absolute/path/to/verified/fr3.usd
   ```

   The exact primary USD is
   `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Franka/FR3/fr3.usd`,
   17,462,793 bytes, SHA256
   `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`.
   Acquire this exact file before a future run; no fallback or substitute is allowed.
   Physics/geometry are embedded. `OmniPBR.mdl` remains `UNKNOWN` visual material.

   Offline regeneration additionally requires ROS Humble's `xacro` 2.1.1 and
   `ament_index_python`, with the frozen external model installed/discoverable in
   `AMENT_PREFIX_PATH` and ROS Python libraries in `PYTHONPATH`. Add
   `--xacro /opt/ros/humble/bin/xacro` to the checker. This expands XML and compares
   the complete transformed output with `urdf/task28_single_cube_frozen.urdf`;
   it starts no ROS nodes. Collision hull replay is separate:

   ```bash
   python3 "$TASK28_FIXTURE/assets/generate_conservative_envelopes.py" \
     --source "$TASK28_FIXTURE/moveit/task28_single_cube_description/meshes/source_collision_points.json" --check
   ```

   Hull replay requires NumPy 1.24.4 and SciPy 1.8.0. It uses only the frozen
   source point snapshot and a temporary directory. Existing outputs are never
   overwritten. Default static checks independently verify delivered float32
   meshes with the standard library; NumPy/SciPy are not required for those checks.

3. Before any future runtime work, complete the G1-1 payload/FK/IK/closed-chain/FCL
   gate and obtain separate execution authorization. Record the fixture commit,
   complete environment/overlay, ROS domain, DDS implementation, seed/time-budget
   policy and exact commands in a new run manifest. ROS domain and DDS selection
   are currently `UNKNOWN` per-run settings; choose an unused isolated domain and
   record it rather than inheriting an unrecorded shell. Geometry units/world
   shift are fixed: metre, Z-up, identity planning world shift.

4. For a separately authorized future runtime session, build only
   `moveit/task28_single_cube_description` into a new `TASK28_RUN` build/install/log
   directory using explicit `colcon --base-paths`, without adding this package to
   the user's original workspace. Source the required ROS Humble and verified
   external model install, then the new package install. Never source an
   unrecorded alternative model. The selected launch reads the frozen URDF,
   local OMPL/kinematics, and unchanged SRDF. It does not launch an executor.

   ```bash
   colcon --log-base "$TASK28_RUN/log" build \
     --base-paths "$TASK28_FIXTURE/moveit/task28_single_cube_description" \
     --build-base "$TASK28_RUN/build" --install-base "$TASK28_RUN/install"
   ```

5. In Isaac Sim 4.5.0, start a fresh metre/Z-up stage containing `/World`, with
   Timeline stopped and no other task/physics/collision/ActionGraph roots. Set
   `TASK28_FR3_USD` to the verified absolute local file and
   `TASK28_FR3_USD_SHA256` to the frozen SHA above. Load
   `scene/task28_single_cube_scene.py` as a module and call `build_scene()`.
   It creates only `/World/Task28G10SingleCube`: two FR3 references, table, tools,
   one dynamic Cube and one nonphysical target marker. It rejects foreign physics
   roots instead of deleting them. NEW explicit construction defaults are gravity
   `(0,0,-9.81) m/s²`, 60 physics steps/s, TGS, CCD and stabilization.

6. After separate authorization to Play, load
   `bridge/task28_single_cube_bridge.py` and call `start_bridge()` with its default
   `enable_robot_commands=False`. It verifies scene identity/count/asset and
   publishes one world PoseArray entry, TCP poses and suction telemetry. All
   endpoints, including clock/joint states, are under `/task28_g1_0`. It creates
   no grippers or command subscriptions in this mode. Command-enabled mode is
   outside G1-0B and must not be selected without further authorization.

7. The future state-only MoveIt launch command is:

   ```bash
   ros2 launch task28_single_cube_description task28_single_cube_moveit.launch.py \
     use_rviz:=false use_sim_time:=true
   ```

   Nodes/services/topics are scoped under `/task28_g1_0`, with namespaced
   TF/clock. Only `task28_table` and `Cube_01` are fixture collision objects.
   Valid single-entry PoseArrays update the Cube world object; no payload removal
   or ACM exception is provided. **These adapters are not freshness gates:** the
   environment retains the initial/last valid Cube pose and the merger retains
   last arm states. A future validator must reject absent, stale, invalid or
   unsynchronized input before planning. No other scene writer may share the
   namespace. Confirm actual routing and scene insertion in that later authorized
   session; source inspection does not prove runtime behavior.

8. The controller input is
   `controller/task28_single_cube_task.py:load_task()`. Only task/index 0 and
   `[Cube_01]` are accepted. No transport algorithm or motion executor is supplied
   by G1-0B. Future planners must use the frozen effective joint limits and
   conservative meshes and keep the carried Cube in collision state. Preserve all
   suction thresholds and SRDF allowances. Record new measurements in a new run
   artifact; do not copy historical Task24-H errors into new fixture result fields.

Stop after G1-0B. This protocol specifies future actions and grants no runtime or
G1-1 development authorization.

# TASK28 Stage A / G3：双臂内部力控制结果

## 结论

Stage A 已在原 Task27 GUI Isaac Sim 场景中完成三次独立回放：双吸附、抬升、X/Y 协同运输、下降、放置、释放和双臂撤退。三个执行器都以 `executor_exit=0` 正常结束，并记录 `STAGE_A_ONLY PASS`；没有进入 REGRASP/PUSH 或五 Cube continuation。

G3 当前完成的是一个沿实测双 TCP 抓取轴的一维、仿真候选内部力通道。它确实进入 live torque QP，并改变了左右关节力矩分配；Cube 的独立 Ground Truth 运动学净 wrench 用于交叉比较。Isaac Surface Gripper/D6 没有提供可用的约束反力接口，因此本报告不把候选估计写成接触传感器读数，也不宣称六维物理接触力已验证。

## P0：Stage-A-only 入口

新增 `--stage-a-only`，向 Task27 执行器传递 `stage_a_only:=true`。在共同撤退执行并完成后，执行器记录：

```text
STAGE_A_ONLY PASS: attach -> lift -> transport -> place -> release -> retreat complete; optional REGRASP/PUSH skipped.
```

Task26 默认路径未改变。场景入口仍为：

```text
isaac/scripts/task27_five_cube_center_insert_scene.py
```

其 SHA-256、原始 Task26 场景/Bridge SHA-256、`headless=false` 和 GUI 截图均保存在每个运行目录的 `summary.json` 中。

## G3 观测与几何

在线残差使用同步 Isaac 状态和上一物理步实际应用的关节 effort：

```text
tau_ext_candidate = M(q) qdd + c(q,dq) + g(q) - tau_applied
```

输入来源分别保留为：

- `get_applied_joint_efforts`：仿真应用命令回读，不是传感器；
- 运行时 `M/c/g`、关节速度差分和冻结 Task27 TCP Jacobian；
- `Cube_01` Ground Truth 位姿、速度、运行时质量 `0.8 kg` 和惯量对角约 `0.00192 kg·m²`。

残差随后投影到当前实测 TCP 几何构成的双臂 `Jᵀ` 接触候选，并复用 `offline/task28_p2_1/grasp.py` 的 `GraspModel`、加权分解和 nullspace。运行时几何由 `offline/task28_g3_stage_a/bilateral_load.py` 的 `RuntimeGeometry` 构造，来源标记为 `RUNTIME_MEASURED_SIMULATION`；历史名义 ±61 mm 抓取 fixture 不进入控制路径。

独立参考是 Cube Ground Truth 的质量/惯量运动学净 wrench：

```text
F_net = m (a - g)
M_net = I_world alpha + omega × (I_world omega)
```

该参考只验证物体净运动响应，不能唯一分辨左右接触负载。D6 路径在运行时仍为 `valid=false`，所以左右六维反力状态保持 `CANDIDATE_ESTIMATE`。

## QP 内部力通道

QP 新增默认关闭的 `internal_torque_weight` 和 `torque_target14`。非零目标时，控制器在 `DUAL_ATTACHED` 中构造一个沿实测左右 TCP 轴的反向接触力，并使用：

```text
tau_robot_internal = -Jᵀ w_contact_on_cube
```

目标限制为 `|F_internal| <= 5 N`，实验使用 `2 N`，反馈增益 `kp=0.5`，QP 内部力权重 `1000`。原有共同/相对任务、关节位置/速度/加速度、力矩和变化率硬约束均保留。若 QP 无法同时满足约束，仍按原 fail-closed 路径停止。

## 实验 A/B/C

证据目录：`offline/task28_task27_integration/evidence/`

| 实验 | 运行 | 内部力设置 | QP | Cube z 范围 | 结果 |
|---|---|---:|---:|---:|---|
| A 基线 | `task28_stagea_g3_A_0003` | 0 N，候选观测开启 | 10,047 `QP_OPTIMAL` | `0.2600 → 0.5409 m` | Stage-A-only PASS |
| B 非零目标 | `task28_stagea_g3_B_2N` | 2 N，全双臂 loaded 段 | 11,057 `QP_OPTIMAL` | `0.2600 → 0.5409 m` | Stage-A-only PASS |
| C 运输中切换 | `task28_stagea_g3_C_switch120` | `t=120 s` 从 0 N 切换到 2 N | 9,859 `QP_OPTIMAL` | `0.2600 → 0.5408 m` | Stage-A-only PASS |

三次均有真实 Cube Ground Truth 轨迹、GUI 截图和原始 executor 日志。共同阶段的典型回执为：

- `COMMON_LIFT`：Cube 约 `(0.500, 0.000, 0.540)`，左右吸盘 `CLOSED`；
- `COMMON_X_TRAVEL`：约 `(0.770, 0.000, 0.540)`；
- `COMMON_Y_ALIGN`：约 `(0.770, 0.060, 0.540)`；
- `COMMON_DROP_TO_TABLE`：约 `(0.772, 0.060, 0.270–0.273)`，随后两侧吸盘 `OPEN`；
- `PREPLANNED_COMMON_RETREAT`：同步 FCL PASS；
- `STAGE_A_ONLY PASS`：正常退出，未执行 Stage B。

`task28_stagea_g3_A_0002` 保留了一次未纳入验收的启动失败：MoveIt `/move_group` RobotModel 参数超时，执行器退出码为 1，桥接器随后按原有局部位移门禁停止；该次没有进入接触。之后的 A_0003 重试完成并作为基线。

B 的 loaded 段共有 2,344 个 `ACTIVE_CANDIDATE` 帧。模型候选的有效目标平均约 `3.05 N`（比例反馈后的 effective target），内部目标关节力矩范数平均约 `1.11 N·m`；相对于 A，loaded 段平均应用力矩的显著变化包括左 J2 `+2.58 N·m`、左 J3 `−2.06 N·m`、右 J2 `+0.90 N·m`、右 J4 `−11.75 N·m`。这证明非零目标进入了约束 QP 并改变了双臂力矩分配，但这些变化仍是仿真控制量，不是已校准接触传感值。

C 的目标更新事件为 `previous_target_N=0.0 → target_N=2.0`。更新后有 177 个 `ACTIVE_CANDIDATE` 帧；更新前后平均应用力矩差包括左 J2 `−10.65 N·m`、左 J3 `+5.36 N·m`、右 J4 `−3.03 N·m`，Cube 仍完成放置、释放和撤退。Cube 的净 wrench 统计、逐帧候选内部/外部分解、目标残差、模式切换和原始轨迹见各运行的 `metrics.json`。

## 物理边界与未完成项

已经展示：

1. 真实 PhysX Cube 位姿变化；
2. live torque QP 执行；
3. 一维非零内部力目标导致的双臂力矩重分配；
4. 目标切换期间的稳定 Stage-A-only 周期；
5. 运行时实测 TCP 几何和独立 Cube 净运动学参考。

尚未展示：

- 可独立读取的 Surface Gripper/D6 左右约束反力；
- 左右接触力的唯一物理真值标定；
- 任意六维内部力闭环；
- 真实 FR3 `tau_J`、真实机器人、ROS 控制或 Sim2Real 等价性。

因此 G3 的当前状态是：**Stage A 仿真一维内部力候选通道已接入并完成 A/B/C 运行；六维物理接触力控制仍未建立。** 不启动 Stage B。

## 复现与验证

基线 A：

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib" \
/home/ubuntu2004/isaacsim-4.5.0/python.sh \
isaac/scripts/task28_task27_one_pass.py --mode integrated --live-qp \
  --max-batches 1 --stage-a-only --internal-force-target-n 0 \
  --local-sweep-limit-m 2.0 --execution-time-scale 16.0
```

B 使用 `--internal-force-target-n 2 --internal-force-weight 1000 --internal-force-kp 0.5`；C 使用 `--internal-force-target-n 0 --internal-force-change-s 120 --internal-force-after-n 2`，其余参数相同。

验证结果：

```text
PYTHONPATH=. python3 -m pytest -q \
  offline/task28_g3_stage_a/tests \
  offline/task28_p2_1/tests \
  offline/task28_p2_2/tests \
  offline/task28_p2_4/tests \
  offline/task28_p5_qp_6_r8/tests \
  offline/task28_task27_live_qp/tests \
  offline/task28_t0/tests
```

ROS workspace `fr3_dual_palletize` 已用 `colcon build --packages-select fr3_dual_palletize --symlink-install` 构建。使用 `PYTHONPATH=offline/task28_g1_1:.` 运行上述回归命令，结果为 **150 passed**；Python 编译检查通过。

所有成功运行目录均含 `summary.json`、`metrics.json`、`task27_executor.log`、`moveit.log`、`scene_loaded.png`、`final.png` 和压缩的 `task27_torque_bridge.jsonl.gz`。原始 Cube 接近路径的连续安全状态未被改变；本任务没有执行 Stage B、REGRASP/PUSH、五 Cube continuation、真实机器人或 ROS 控制命令。

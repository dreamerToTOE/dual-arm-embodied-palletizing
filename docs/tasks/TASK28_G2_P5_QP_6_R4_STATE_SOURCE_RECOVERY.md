# Task28 G2-P5-QP-6-R4 — Isaac joint-state consistency recovery

## 简明结论

R4 已完成：在原始 Task28 双 FR3、L 形 side-suction、Cube 和 table 构建器上，使用六个独立进程完成了 A–F 状态源诊断，并保存了每一帧九维 `q/dq`、时间、物理步、链路/TCP 位姿、FK 误差、碰撞结果、M/c/g 和源缓冲元数据。关节状态问题没有解决：六个单元都在第 128 个显式物理步因“位置冻结而速度仍非零”触发 fail-closed 门。没有发送真实力矩、吸盘、Cube/D6、ROS、机器人或硬件命令。剩余阻塞是可信的 q/dq 源和随后依赖它的动力学/力矩资格。下一步只有在建立独立且可验证的时间参数化与状态源后，才能重新申请力矩实验；R4 在此停止。

## 范围和不可变对象

- 分支：`side-suction-palletizing`。
- 原始 builder：`fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py`。
- 每个单元都在新进程、新匿名 stage 中调用原始 builder；没有创建替代 FR3/TCP，没有改 Cube pose、质量、工具、手指、mimic、碰撞、ACM 或历史文件。
- ROSGraph 只在匿名诊断 stage 中停用；没有启动 command-enabled ROS bridge、命令图或机器人端点。
- R4 运行时使用安装版 Isaac Sim 4.5 CPU `SimulationContext`。安装实现的 CPU 分支会把有效 PhysX broadphase 从 builder 的 GPU 值切换为 MBP 并关闭 GPU dynamics，即使传入 `set_defaults=False`。这不是 builder 或冻结 USD 的写入；每次诊断都记录了该运行时适配，并将其列为 CPU harness 限制，不能把 CPU 运行结果当作原始 GPU backend 的等价证明。

## R3 历史重建

`offline/task28_p5_qp_6_r4/evidence/historical/dev_02.json` 和 `dev_03.json` 由 R3 原始 `raw.json` 逐帧重算，没有覆盖 R3 证据。重建内容包括相邻 `Δq/Δt`、报告 `dq`、梯形积分、单步积分误差、link8 和实际 side-suction TCP 位姿变化、Dynamic Control 与 wrapper 差值、回调时间和 drive 增益。R3 档案没有保存源指针或 dtype，所以历史数组是否别名保持 `UNKNOWN`；不能从“两个 wrapper”推出独立测量。

R3 事实保持原结论：真实回调 `dt` 约 `0.01666666753590107 s`，R3 dev02/dev03 的后段位置差分与 `dq` 的最大关节差约 `0.0107 rad/s`，显式 wake 没有消除它，未执行静态力矩或合作跟踪。R3 的直线关节插值仍保留为负例：到 nominal suction grasp 的 `0.92` fraction 处发生 `left_fr3_side_suction/3 | Cube_01/box` 碰撞；R4 没有扩大 ACM 或把工具接触全部豁免。

## 安装版 API 路径和证据边界

源代码审计保存在 `offline/task28_p5_qp_6_r4/evidence/source_api_audit.json`。安装版 `SingleArticulation.get_joint_positions/velocities` 委托 `ArticulationView`；`ArticulationView` 从 `_physics_view.get_dof_positions/get_dof_velocities` 取值，默认 `clone=True`。numpy backend 的 `clone_tensor` 调用 `np.copy`，R4 又立即转为新的 `float64` 数值快照。相反，R4 D 单元直接保留 `_physics_view` 的 tensor 引用，下一物理步后上一引用相对即时副本最大改变为 `0.08762110769748688`，因此借用 buffer 从未用于资格判定。

`SimulationContext.step(render=False)` 走 `PhysicsContext._step` 的 `simulate` 后 `fetch_results`；F 单元在显式 fetch 后调用已安装的 `SimulationContext.render()`，回调计数没有增加，也没有额外物理步。`omni.physics.tensors.flush` 没有作为修复手段使用；`update_articulations_kinematic` 是运动学更新路径，不是 velocity source validator。Dynamic Control 在 B/E 只作为同一 PhysX 后端的并行访问，不能当作独立传感器。

官方接口背景：[Isaac Sim 4.5 Core Prims API](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.prims/docs/index.html)、[Isaac Sim physics-based sensors](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_physics.html)、[PhysX articulations](https://nvidia-omniverse.github.io/PhysX/physx/5.1.1/docs/Articulations.html)。这些资料用于解释 API 和 sleep/awake 的可能边界；它们不被用来替代本地观测，也没有把 published sleep 机制当作非零速度的唯一原因。

## 六个独立诊断单元

每个单元均保存 `frames.jsonl`，在 `SimulationApp.close()` 前写出 `result.json`；每个单元的 `robot_command_writes` 和 `suction_commands` 都为 0。

| 单元 | 唯一变化 | 结果 |
|---|---|---|
| A | 原生 `SimulationContext.play`，物理步后立即复制 float32/float64 | 第 128 步 `FROZEN_Q_NONZERO_DQ` |
| B | R3 原始读取顺序，附加即时副本和 Dynamic Control | 同一失败；Dynamic Control 与 wrapper `q/dq` 最大差为 0 |
| C | post-step callback 与显式 step 后立即读取对照 | 128 个 post-step 回调；q/dq 对照最大差为 0；同一失败 |
| D | 跨步保留借用 tensor，与不可变快照对照 | 借用 q 引用最大改变 `0.0876211`；同一失败 |
| E | 原生 tensor、Dynamic Control、link7/link8/TCP 同帧记录 | 同一失败；Dynamic Control 与 wrapper 最大差为 0 |
| F | fetch 后调用已安装 `SimulationContext.render` | 无额外物理步；同一失败 |

六个结果均为 `REJECTED`，每个有 129 帧（初始帧加 128 个显式步）。末帧左臂最大 `|fd(q)-dq|` 为 `0.010724863037467003 rad/s`，右臂为 `0.010585268028080463 rad/s`；末段积分误差约 `0.0001787477 rad`。TCP/FK 仍与链路观测一致（最大平移误差 `6.9847e-7 m`、旋转误差 `1.0069e-6 rad`），所以这不是一个可只用 TCP 读取错误解释的结果。所有帧的离散碰撞检查只保留原始 Cube/table 支撑接触，没有新的 unexpected collision。

## 冻结的资格契约

`offline/task28_p5_qp_6_r4/source_contract.json` 冻结了以下门：九维 DOF/robot identity、单调实际 physics step、正且一致的时间戳、不可变 q/dq、q/dq 独立一致性、link7/link8/TCP 与 FK 一致、无未解释 buffer alias、无 unexpected collision、无竞争的 position sink。settle 速度门仍是 R3 的 `0.01 rad/s`；没有通过放宽速度容差来制造 PASS，并且 settled 与 moving 两种资格分开记录。

六个单元在同一个冻结门上失败，因此结论是 `JOINT_STATE_SOURCE_BLOCKED`，不是“用有限差分修正后通过”。链路和 TCP 可证明姿态仍在被正确读出，但不能单独证明报告的 joint velocity 可信。R4 没有声称唯一根因；证据排除了 callback/read-after-fetch 时序、render 多步和两个 wrapper 读数不一致，但同一 PhysX 后端的缓存/状态语义仍需独立解决。

## 动力学、力矩和合作 QP 状态

每帧读取了原始两臂 M/c/g，并保存了时间戳；由于 q/dq source 未资格化，不能把这些量标成同状态逆动力学可行性证明。因而：

- `ORIGINAL_SCENE_UNCHANGED`：通过 builder/资产/prim/质量/碰撞身份检查；CPU backend 适配单独披露。
- `JOINT_STATE_SOURCE_BLOCKED`：六单元一致失败。
- `NATIVE_SETTLE_UNRESOLVED`：没有可信 settled state。
- `ARM_DYNAMICS_SOURCE_PARTIAL`：有读数和元数据，但没有 source-valid state transition 认证。
- `EXCLUSIVE_TORQUE_COMMAND_BLOCKED`：pre-command gate 从未打开，真实 Isaac effort writes 为 0。
- `STATIC_TORQUE_HOLD_SKIPPED`、`SHORT_TORQUE_TRACKING_SKIPPED`：没有静态或运动力矩实验。
- `VIRTUAL_COOPERATIVE_QP_STATUS = BLOCKED_SOURCE_PREREQUISITE`：没有虚拟合作 QP 执行。
- `CUBE_SUCTION_UNTESTED`：没有 suction closure、D6、Cube 搬运、ROS 或硬件操作。

`offline/task28_p5_qp_6_r4/evidence/negative_controls.json` 运行了 13 个 fault-injection 控制：stale q、stale dq、shared mutable array、错误 step/time、错误 robot identity、physics pause、position sink、effort mode、非法质量源、错误 TCP、unexpected collision，以及 rejection 后再尝试 torque；全部按预期拒绝，命令计数为 0。`source_gate.py` 的 rejection 是 latched，不能在失败后偷偷写入力矩。

## 回归与归档

历史九个非重叠套件合计 978 项通过：历史 789、QP-4 53、QP-5 5、QP-6 retry 54、QP-6-R1 retry 30、QP-6-R2 12、QP-6-R3 7、R1 6、R2 22。R4 新增 focused tests 12 项，均通过；总计 `990 passed`。原始日志、每帧记录、注册矩阵、源码审计、资格评估、负例和跳过单元均在 `offline/task28_p5_qp_6_r4/`。

R4 只加入本任务目录和本报告；没有改 QP-4、QP-5、QP-5-R1、G1、NR、fixture、ACM、R3 或历史证据。任务在 R4 停止。下一次若获明确授权，应先建立独立平滑时间参数化和同一双臂状态的 M/c/g、真实应用时刻、pre-command fail-closed gate 与独立几何验证，再重新评估力矩实验；不能跳过本报告的 source block。

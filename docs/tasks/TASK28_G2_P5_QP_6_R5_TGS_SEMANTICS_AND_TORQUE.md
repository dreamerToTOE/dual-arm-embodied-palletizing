# Task28 G2-P5-QP-6-R5 — PhysX TGS velocity semantics, state qualification and original-scene torque validation

## 简明结论

R5 完成了文献、安装版 API 和五个独立诊断单元的核验。PhysX TGS 解释得到支持：原始 CPU TGS 基线的末端 `dq` 与整步 `Δq/Δt` 不一致，而在安装版支持的 `EnableExternalForcesEveryIteration=True` 临时对照中，这个差异显著下降。R4 的失败因此主要是把“末端 solver 速度必须等于整步平均速度”写成了过严的源契约；它仍是该契约下有效的负结果，不能被改写为“R4 误报”。

R5 没有把语义解释直接升级成力矩控制资格。原始场景的 q/dq 源完整性和运动观测为 `PARTIAL`，M/c/g 尚未通过独立激励和实际动力学响应验证，因此没有发送真实力矩，没有静态 hold、短轨迹或合作 QP。GPU 对照在首帧前发生 CUDA illegal memory access，已作为非基线诊断记录并停止。没有执行吸盘、Cube/D6、ROS、硬件或物理机器人操作。

## 冻结范围

- 仅调用原始 builder：`fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py`。
- 两个原始 FR3、镜像 L 形 side-suction、四个杯、原始 link8/TCP、Cube_01、table、关节/碰撞/ACM/mimic/finger drives 保持不变。
- 每个 Isaac 单元使用独立进程和匿名 stage；ROSGraph 只在匿名 stage 中停用，不启动 command bridge。
- 原始 60 Hz、TGS、gravity、drive、Cube 和碰撞数据都记录；C 的外力 flag 是临时匿名 stage 变体，不是冻结原始配置。
- R4 原始代码、source gate、历史结果和结论未修改。R4 的 `JOINT_STATE_SOURCE_BLOCKED` 仍在其原始有限差分契约下成立。

## 文献和安装版源码

预注册在 `offline/task28_p5_qp_6_r5/preregistration.json`，文献和安装版审计在 `offline/task28_p5_qp_6_r5/literature_and_source_review.md`。主要参考为 [PhysX 5.7 Simulation](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/Simulation.html)、[NVIDIA DOF Velocity Offset at Rest 讨论](https://forums.developer.nvidia.com/t/dof-velocity-offset-at-rest/205799)、[PhysX 5.1.1 Rigid Body Dynamics](https://nvidia-omniverse.github.io/PhysX/physx/5.1.1/docs/RigidBodyDynamics.html) 和 [Isaac Sim 4.5 Core Prims API](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.prims/docs/index.html?highlight=UsdGeom%2520Xformable)。

PhysX 文档说明，TGS 把完整步拆成多个 position substep，最后 velocity iteration 的速度字段不是完整步的平均速度；在 gravity/drive 只按整步施加的情况下，驱动关节稳态位置和报告速度可能出现差异，并提供 `eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS` 作为修正对照。该公开例子是简单 prismatic system、较新的 SDK，不足以单独证明 FR3；R5 使用安装版行为做验证。

安装版 Isaac Sim 4.5/PhysX 106.5.7 的 `PhysxSchema.PhysxSceneAPI` 暴露 `Get/CreateEnableExternalForcesEveryIterationAttr`、solver type 和 min/max position/velocity iteration 属性。`omni.physx` Python 接口没有后置 toggle 方法，所以 C 在 `SimulationContext` 创建前只对匿名 stage 写入该属性。`ArticulationView` 的 joint getter 默认 clone，numpy backend 使用 `np.copy`；`SimulationContext.step(render=False)` 走 `simulate` 后 `fetch_results`。完整探针在 `evidence/installed_api_probe.json`。

## 预注册诊断和结果

所有 A/B/C/E 都完成 150 个显式 physics steps、151 帧；每帧保存九维 q/dq、时间/step、drive target/gain/mode、M/c/g、link8/TCP、FK、碰撞、整步差分、末端 solver dq、梯形积分误差和 raw copies。所有单元命令计数均为 0。

| 单元 | 实际物理差异 | 结果 |
|---|---|---|
| A `ORIGINAL_CPU_TGS_BASELINE` | 原始 builder，CPU numpy，TGS，原生 drives | 完成；末段平均 `|fd(q)-dq|` 左/右约 `0.00253/0.00249 rad/s`，最大约 `0.01072/0.01059` |
| B `TGS_SETTINGS_AND_DRIVES` | 与 A 相同，仅增加 solver/drive/target/effort 元数据和 Dynamic Control 对照 | 完成；Dynamic Control 与 wrapper q/dq 最大差为 0；设置与 A 一致 |
| C `NON_BASELINE_EXTERNAL_FORCES_EVERY_TGS_ITERATION` | 匿名 stage 将 flag 从 `False` 设为 `True`；其他几何、dt、drive、solver type 不改 | 完成；末段平均差降到约 `0.000110/0.000294 rad/s`，相对 A 平均下降约 `12.43×`；末段积分误差降到约 `9.5e-6/3.1e-5 rad` |
| D `GPU_BACKEND_DIAGNOSTIC` | 请求 torch/cuda；只作 backend 诊断，不等价于原始 CPU 证据 | 在首帧前 `SingleRigidPrim` 初始化时发生 CUDA illegal memory access；0 帧、0 命令，安全停止 |
| E `ORIGINAL_MOTION_SETTLE` | 原始 CPU TGS 启动运动到 settle 的独立重复 | 与 A 的 98 个 moving samples、末段差异和 Dynamic Control 对照一致 |
| F `SOURCE_FAULT_CONTROL` | 纯确定性负例，不启动 Isaac | 13 个真实 stale/delayed/time/identity/mass/TCP/contact/sink/mode/failure-latch 负例全部拒绝 |

A/E 的 startup 运动有双臂约 98 个样本的明显 q 变化，link8 位姿也同步变化；TCP/FK 最大误差约 `6.98e-7 m` 和 `1.01e-6 rad`，unexpected collision 为 0。末端 q 的小位移与非零 solver dq 在原始 TGS 中可重复；C 的同一外力施加语义对照显著降低差异。C 是非基线解释性诊断，不能替代原始场景资格。

D 的 GPU 失败不被解释为场景或 CPU TGS 结论；它只说明该实际 host 上的 torch/cuda backend 在这套原始 stage 初始化路径不可用。原始 CPU 的 `SimulationContext` 仍会把 authored GPU/GPUDynamics 属性适配为 MBP/CPU；该运行时适配已在每次结果中记录，builder/USD 身份没有改写。

## 状态契约和科学判断

R5 将状态分成四层：

1. **Source integrity**：A/B/E 具备正确 prim/DOF、有限 q/dq、单调 step/time、即时复制、link/TCP 和无竞争 sink；D 不通过 backend 初始化。
2. **Kinematic consistency**：FK、link8/TCP 和 moving q 变化一致，且 E 可重复。
3. **Solver semantics**：原始 TGS 的 `dq` 明确标为 end-of-step solver velocity，不能再当作整步平均速度；有限差分只能作为 estimator，不能静默替换控制源。
4. **Control qualification**：原始 CPU TGS end-step dq 的物理语义得到支持，但尚没有独立电机/编码器式观测，也没有经过 source-qualified transition 的实测 M/c/g 残差和 finger-coupling 动力学响应，因此控制用途为 `PARTIAL`。

这解释了 R4：它不是单纯的 stale buffer 证据，也不能继续把 `fd(q)/dt=dq` 当作唯一正确关系。更准确的结论是“R4 契约过严 + TGS solver 语义真实存在 + 原始场景控制资格仍未完成”，而不是三者择一的简单修正。

## 力矩、动态和抓取状态

- `ORIGINAL_SCENE_PRESERVED`: PASS。
- `TGS_VELOCITY_SEMANTICS_SUPPORTED`: PASS（安装版 flag 对照支持，且基线/重复运行一致）。
- `STATE_SOURCE_VALID_FOR_CONTROL`: PARTIAL；解释了末端 solver dq，但没有独立动力学激励资格。
- `ARM_DYNAMICS_QUALIFIED`: PARTIAL；读取了原始 M/c/g 和九维状态，未把 solver-projected effort 当作电机传感器，也未发明 L-tool 惯量。
- `EXCLUSIVE_TORQUE_ACTUATION_VERIFIED`: BLOCKED。
- `STATIC_TORQUE_HOLD`: SKIPPED；真实 effort writes 为 0。
- `SHORT_TRACKING`: SKIPPED；没有执行四个运动 cell。
- `COOPERATIVE_QP_STATUS`: `BLOCKED_STATE_AND_DYNAMICS_PREREQUISITE`。
- `CUBE_D6_NOT_TESTED`: true。

没有调用 R3 effort adapter 的真实写入路径，没有关闭 arm drives，也没有做任何 torque/readback 假实验。这样保留了“语义解释不等于控制通过”的边界。

R3 的直线抓取路径负例保留：约 `0.92` fraction 发生 `left_fr3_side_suction/3 | Cube_01/box` 碰撞；R5 没有扩 ACM、忽略工具碰撞或传送 Cube，也没有执行 Cube/D6 attachment。

## 测试和证据

R4 已验证的 978 个非重叠回归和 12 个 R4 focused tests 作为前置 990 项保留；R5 新增 focused tests 13 项，合计 `1003 passed`。R5 负例 13 项全部拒绝、命令计数为 0。所有 raw traces、first failures、solver differences、source binding、文献、预注册和跳过 cell 在 `offline/task28_p5_qp_6_r5/`。

R5 在此停止。下一阶段若要真实力矩，必须先补充独立 source-qualified excitation、同状态 M/c/g 和 finger coupling 残差，再按新的预注册单元打开 fail-closed pre-command gate；不能把 C 的临时 flag 诊断直接当作原始场景控制 PASS。

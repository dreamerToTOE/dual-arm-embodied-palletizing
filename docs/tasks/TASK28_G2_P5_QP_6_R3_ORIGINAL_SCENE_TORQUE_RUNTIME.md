# Task28 G2-P5-QP-6-R3：原 L 型侧吸场景力矩运行研究

**本轮实际启动并推进了原 Task28 单 Cube 场景，测得物理步长约 16.667 ms。** 两台 FR3、L 型吸具和 Cube 沿用冻结场景，初始姿态由原生 position drive 收敛。源状态未通过静止和一致性门：一段时间内关节位置读数不再变化，速度读数仍约 `0.0107 rad/s`；两种安装版读取接口给出相同数据，显式唤醒也没有解决该现象。故本轮没有对 Isaac 发送力矩，静态力矩、A/B 短运动和物体空间 HQP 均按依赖关系跳过。

总体为 **`QP6_R3_ORIGINAL_SCENE_RUNTIME_PARTIAL_SOURCE_BLOCKED`**。这不是 Cube 搬运或吸附控制通过，也不声称状态不一致的根因已经确定。

## 原场景、物理和输入所有权

起点为 `33e12b488209e0ffd865740bfae692ca5e7435ed`，`side-suction-palletizing`。四次开发运行均在新进程、新匿名 stage 中直接调用冻结的 `task28_single_cube_scene.py::build_scene()`，没有手工搭建替代机器人或工具，也没有把 Cube 或机器人瞬移到参考姿态。只存在 `/World/Task28G10SingleCube/physicsScene` 这一个 physics scene。未改旧 fixture、bridge、USD、MoveIt、ACM 或历史报告；起点的 **1330 个 Git 跟踪文件**与 HEAD 一致，外部 FR3 USD 的 SHA-256 仍为 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`。[核验](../../offline/task28_p5_qp_6_r3/evidence/protected_files.json)

实际物理回调共 **496 次**：四次初始化各 2 次，显式请求 `180+127+180+1=488` 步。回调 dt 均为 **`0.01666666753590107 s`**，与 60 Hz 一致。实际场景开始前读取到 TGS、重力和 60 Hz；原 builder 的源码指定 CCD。当前证据没有在 SimulationContext 初始化后独立回读 CCD、Cube 的数值质量和全部 physics 属性，所以“完整运行配置逐项验证”仍为部分结论。安装版 `PhysicsContext` 对 CPU broadphase/GPU dynamics 以及匿名 stage 时间码有初始化写入，后值未单独归档，不能把它们声称为未受影响的实测事实。

原 builder 自己加载 `isaacsim.ros2.bridge` 扩展并加载其内部 rclpy；为了不执行图或接收外部命令，本任务在每个**新匿名 stage**将原本只发布状态的 `ROSGraph` 临时设为 inactive，再开始 physics，未保存 stage。没有调用原位置命令 bridge，也没有创建 suction bridge 或发送 ROS 命令。该扩展加载事实与“没有运行 ROSGraph/命令桥”应分开理解。

原场景每侧的 L 工具和四个 cup 均位于 `fr3_link8/side_suction_tool`，TCP 使用该工具的 `side_suction_tcp`，其局部位姿为左右 `[0,∓0.155,0.080] m`、`Rz(∓90°)`。运行中读取真实 link8 刚体位姿，并乘以从原 TCP prim 核验的固定局部变换；没有使用 link7 近似。每帧与冻结 G1 FK 对照，运行清单记录最大误差。[运行清单](../../offline/task28_p5_qp_6_r3/evidence/run_inventory.json)

## 状态、碰撞和四次开发运行

起始远离 Cube 的姿态分类为 `FREE_HOME_OR_SETTLING`。原 G1 的 **0.100 mm 平移与 0.001 rad 旋转**抓取门只用于需要名义共同抓取的状态；没有把远离 Cube 的 HOME 判成附着抓取失败。每个采样帧用冻结 URDF/SRDF 与 FCL `detached` 检查 414 个形状对。FCL 的 Cube/table 支撑接触仅在结果解释中单独分类，未修改 ACM；各已采样帧没有其他 FCL 碰撞。FCL 是离散、保守模型检查，没有独立 PhysX 接触传感器或连续净空证明，不能替代真实接触观测。

| 开发运行 | 作用和首次失败 | 显式步 | 结果 |
|---|---|---:|---|
| 01 | 原生启动驱动；physics step `182`、`3.03333349 s`，未满足预注册连续静止门 | 180 | `NATIVE_DRIVE_SETTLE_TIMEOUT` |
| 02 | 同一原生策略，额外对照 Dynamic Control API；step `129`、`2.15000011 s`，连续位置不变且速度仍超门 | 127 | `FROZEN_POSITION_NONZERO_VELOCITY_SOURCE_MISMATCH` |
| 03 | 每个显式步前唤醒两臂，不改 drive、状态或参数；step `182`、`3.03333349 s` | 180 | `NATIVE_DRIVE_SETTLE_TIMEOUT` |
| 04 | 读取挂载工具与 link8 的质量属性；保持原生驱动 | 1 | 质量属性读取完成；不构成力矩控制通过 |

四次都没有调用显式机器人 position/effort 写入，也没有 suction 闭合。原生 drive 自身确实使两臂从初始姿态向 HOME 运动。各进程的科学 JSON 在 `SimulationApp.close()` 前保存；关闭阶段等待 USD 资源，随后终止本任务自己的关闭等待。步数、首失败、状态和原始记录可在 [run_inventory.json](../../offline/task28_p5_qp_6_r3/evidence/run_inventory.json) 与各 `dev_*/raw.json` 复查。

限制：每次 `SimulationContext.play()` 有两次初始化回调，第一帧采样发生在这两步之后；这两步没有逐步 FCL 采样，不能声称它们经过逐步 fail-closed 碰撞验证。第一轮只注册了静止超时门；后补的“连续 q 不变而 dq 非零”诊断在 row `127` 已出现，但第一轮继续到 row `180` 的注册超时，后续 53 帧不能称为该后补门的 fail-closed 结果。第二轮在新增、事前登记的同一源一致性门首次失败时停止；第三轮的唤醒诊断保留独立身份和相同停止门。

## 机械臂源和原生手指

真实启动驱动后的两个原生 finger 均位于约 `0 m` 下限附近，保留原始 mimic 和 drive，不是侧吸开合执行器。没有强制旧 R1 的 `0.04 m` 或诊断 `0.02 m` 状态。每帧记录实际 `q9/dq9`、`M9×9`、`c9/g9`、applied command readback 和 solver-projected joint effort；后两者分别是命令来源和求解器投影量，不是独立标定的电机力矩传感器。

在最后样本，矩阵为有限、对称正定，分别记录 `Maa`、`Maf`、差分 `qdd_finger` 与 `Maf*qdd_finger`。例如 dev-03 最后一步的手指耦合项最大约 `9.14e-12 Nm`（左）和 `1.75e-14 Nm`（右），但这是单步差分观察，不是全轨迹耦合上界。使用同一 post-state 的 solver-projected effort 所计算的臂方程 residual 只是带时间对齐和力源不确定性的诊断；不能证明无外力或源合格。记录见 [dev-03 audit](../../offline/task28_p5_qp_6_r3/evidence/dev_03_audit.json)。

两种状态 API 在 dev-02 给出完全相同 q/dq，它们可能共享同一 PhysX 后端，不能称为独立物理测量。最后两帧左臂关节位置差分速度为零，而其返回速度最大约 `0.0107249 rad/s`；右侧约 `0.0105853 rad/s`。dev-03 的显式唤醒仍未消除该差异。当前既没有能解释或排除该时间/状态语义问题的独立源，也没有同状态接触外力证明，故 `ARM_SUBSYSTEM_SOURCE_PARTIAL`，未验证完整九维受约束动力学，也未将 finger reaction 当成 suction 力。

工具的七个物理几何（两段支架、manifold、四个 cup）是 link8 下的 collider，没有各自独立的 RigidBodyAPI 或 MassAPI。dev-04 读到左右 link8 有效质量均约 `0.18600638 kg`，有效惯量矩阵也已归档；单一挂载状态不能从中分离 L 工具对惯量的增量，所以工具惯量是否、以何数值进入 articulation 动力学仍为 **UNKNOWN**。原始结果中名为 `link8_authored_mass_kg` 和 `link8_authored_diagonal_inertia` 的字段只是 USD 属性 `Get()` 的返回值，未检查是否为 authored 值，可能是 fallback，不能据此声称源资产明确写入零质量或零惯量。没有添加假定 payload。原 USD 的 `fr3_hand_tcp` 还触发了安装版 PhysX 对负质量/近似惯量的警告，不能把仿真动力学当成硬件标定。

## 力矩适配器、静态和短运动

[effort_adapter.py](../../offline/task28_p5_qp_6_r3/effort_adapter.py) 实现了候选 Isaac sink 的成对身份和预命令门、独占模式、研究 effort/rate 门及 post-step 游标检查。它只在外部提供绑定源证据、碰撞门和持久日志后才能启用；未满足时默认关闭。零 stiffness/damping 仅计划用于 arm 的运行会话，finger drive 保留。安装版代码显示，使用七个 `joint_indices` 的 `set_joint_efforts` 会创建九维零缓冲；因此候选 sink 先读取完整 applied-command 向量，显式保留原两指输入，再提交九维向量，而不是盲目零填充。即使如此，实际 finger policy、日志、碰撞门及整个 sink 仍需在合格源上验证。模拟 handle 的 7 个 CPU 聚焦测试只检查实现逻辑，不是 Isaac 力矩实测。

原 4.5 文档说明 effort 控制需要零 drive stiffness/damping，并将 applied effort 描述为 `set_joint_efforts` 的命令读回；本机实现和文档核对记录在 [API 审计](../../offline/task28_p5_qp_6_r3/evidence/api_source_audit.json)。[Isaac Sim 4.5 Core Prims API](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.prims/docs/index.html)

正式原场景 gravity+PD 静态保持因静止、状态一致性和 arm source 前置条件未满足而跳过。A/S05、A/S10 随静态门跳过；B/S05、B/S10 还缺受约束 arm dynamics 资格，均有独立 skip JSON。没有正式控制器 A/B 的结果、没有 effort 写入或 tracking PASS；未以停止循环冒称硬件急停。

## 物体空间就绪与后续条件

使用 dev-03 最后记录的 Cube 姿态和冻结 G1 名义抓取做离线 IK，两臂各找到端点解。但从该记录姿态到端点的直线关节插值筛查在第 `46` 号采样（比例 `0.92`）发现左侧吸具与 Cube 碰撞；只检查了 47 个样本，不能证明不存在其他路径，也不证明连续安全。[离线筛查](../../offline/task28_p5_qp_6_r3/evidence/approach_audit.json) 不代表物理接近或 `PRE_CONTACT_READY`。实际状态源、静态力矩、完整 approach、独立接触源均未满足，虚拟物体空间 HQP 也跳过。没有 D6 闭合、真实 Cube 抬升/运输或内部力闭环。

## 结论与验证

| 维度 | 结果 |
|---|---|
| `ORIGINAL_SCENE_RUNTIME_PHYSICS_VERIFIED` | 部分：原 scene 实际推进、60 Hz 回调及单 physics scene 已核对；CCD/完整 physics 属性的运行后读数未归档 |
| `ORIGINAL_L_SUCTION_TOOL_PRESERVED` | 原始工具 prim、几何及 link8→TCP 已核对 |
| `ORIGINAL_SCENE_ARM_DYNAMICS_SOURCE_STATUS` | `ARM_SUBSYSTEM_SOURCE_PARTIAL`，未合格 |
| `NATIVE_FINGER_CONSTRAINT_MODEL_UNVALIDATED` | 是；完整受约束九维动力学未验证 |
| `MOUNTED_TOOL_INERTIA_UNKNOWN` | 是；已读 link8 有效惯量，工具增量未分离 |
| `EXCLUSIVE_ARM_TORQUE_ACTUATION_VERIFIED` | 否；实际 effort 写入 0 |
| `ORIGINAL_SCENE_STATIC_TORQUE_PASS` | 否，跳过 |
| `ORIGINAL_SCENE_SHORT_TRACKING_PASS` | 否，A/B 全部跳过 |
| `VIRTUAL_COOPERATIVE_HQP_READY_OR_BLOCKED` | BLOCKED：状态源和接近路径均未资格化 |
| `ACTUAL_CUBE_SUCTION_UNTESTED` | 是 |

前次 **971 项**软件测试按互不重叠的原分组重跑通过；R3 聚焦 **7 项**通过，共 **978 passed**。额外运行 G1-0B 静态 fixture 检查；这些测试不表示物理控制通过。最终提交、远端 HEAD 和工作树状态由提交后的交付核验提供。

要继续，应先辨明 PhysX 在原启动驱动下的 q/dq 一致性及时间语义，取得同状态、可信的 arm 动力学与接触排除证据；随后在新注册的原场景试验中验证独占 effort sink 和静态保持，再进行合格的短运动与安全接近。真实 Cube/D6 闭合需独立前置条件和另行授权。本任务在 R3 停止。

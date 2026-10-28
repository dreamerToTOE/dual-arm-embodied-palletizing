# Task28 G2-P5-QP-6-R2：保留原 L 型侧吸场景的力矩控制接入审计

**状态：`QP6_R2_SCENE_VERIFIED_CONTROL_BLOCKED`。** 原 Task28 单 Cube fixture 的静态来源与停止时间线的场景构建得以核对。当前没有原场景的同步动态源、无碰撞起始状态或合格的 14 关节动力学；本次力矩适配器是默认禁用的 CPU 接口原型，还不是可运行的协作 HQP 控制器。原场景静态力矩和无接触跟踪未执行，不能声称通过。

## 1. 实际场景和来源

使用 `fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py` 的原 `build_scene()`，而不是手工重建场景。它引用用户指定的本地 FR3 USD，SHA-256 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`；该文件本身与原 fixture 不被改写。原场景根、两台机器人和供料 Cube 为：

- `/World/Task28G10SingleCube`
- `/World/Task28G10SingleCube/left_fr3`、`/World/Task28G10SingleCube/right_fr3`
- `/World/Task28G10SingleCube/Supply/Cube_01`

Task24/26/27 是历史工作及其物理记录；当前 Task28 G1-0B 是独立的新 fixture，不能从历史单 Cube 成功转移吸附通过结论。G0A、G1、QP-4、QP-5、QP-6、QP-6-R1 和 G3 的来源及边界见 [静态来源地图](../../offline/task28_p5_qp_6_r2/evidence/static_identity.json) 和既有任务报告。QP-6 的 `/World/QP6_left`、`/World/QP6_right` 是没有本场景工具、桌面和 Cube 的隔离 FR3 实验，不是原 Task28 场景。

在新建的匿名 stage 中，明确设置其 metre/Z-up 元数据，时间线保持停止，然后调用原 builder。stage 仅存在于独立进程；没有改写用户已有 stage，构建的 ROSGraph 未运行，没有启动 ROS bridge。该次 [只读场景证据](../../offline/task28_p5_qp_6_r2/evidence/runtime_scene_v2.json) 记录根、工具、两侧 mimic、资产哈希和 PhysX scene 属性。实际构建属性为 `60 steps/s`、TGS、重力 `9.81 m/s²`；没有推进物理，所以 **实际运行 dt 未测量**，不能将旧 QP-6 的 `120 Hz` 用在本场景。

## 2. L 工具、TCP 和 Cube 的变换

每侧固定工具实际位于 `fr3_link8/side_suction_tool`，物理 TCP 位于其 `side_suction_tcp`。停止 stage 的 USD 实际读数与冻结合同一致：竖直段 `0.080 m`，横段 `0.130 m`，manifold `0.070 × 0.070 m`，每臂四个 cup；左/右 link8→TCP 平移分别 `[0,-0.155,0.080]` / `[0,+0.155,0.080] m`，局部 `Rz(-90°)` / `Rz(+90°)`。`runtime_scene_v2.json` 保留 link8 和 TCP 的世界矩阵及 Cube 的 authored 变换。工具局部关系与冻结 MoveIt URDF、xacro 和 fixture 合同一致；这里没有用 link7 的固定偏置替代真实 prim。

link8→TCP 是安装变换；G1 的 Cube→TCP 是另一组名义抓取：相对 Cube 中心左右 Y 为 `-0.061/+0.061 m`，包含原有 `1 mm` 名义接近间隙，旋转也使用原 G1。二者没有混用。原 G1 门限仍为平移 `0.100 mm`、旋转 `0.001 rad`。

停止时间线时，USD authored Cube 中心为 `[0.65,0,0.26] m`，左右 TCP 平移为约 `[0.738,-0.445,0.846]` / `[0.738,0.445,0.846] m`。用原 G1 闭链函数计算，左右绝对平移误差约 `706.113/706.113 mm`，相对平移误差 `768 mm`。这只是 **物理启动及 drive 收敛之前的 authored 几何**，不是动态运行的姿态、接触距离或成功接近。不能瞬移 Cube 或把 QP-4 中点套到此 scene；要先建立并验证从真实状态到接触位的路径。[计算记录](../../offline/task28_p5_qp_6_r2/evidence/initial_usd_closure.json)。

## 3. 原生手指、动力学与之前结果

原 USD 两侧每个 articulation 均有七个 arm 关节和两个原生 finger 关节。新 fixture 隐藏 hand/finger 的可视与碰撞部分，不等于删除其关节、物体或惯量。停止 stage 读取到 finger2 的 mimic gearing `-1`、offset `0`、引用 finger1；两指上限约 `0.0399999991 m`。这些关节不是 L 吸具的主动开合装置，也没有被删除、强设 `0.02 m` 或改造。正式协作控制目标应是 14 个 arm 关节，同时保留全部 finger 状态、约束及必要的九维 effort 接口。

本任务没有得到 60 Hz 原场景中同状态、同时间的 q/dq/M/c/g；因此不能认定当前装有 L 工具、手指约束的 14 维子系统合格，也不能以任意 `M[:7,:7]` 代替验证。场景代码把几何挂在 link8，但工具质量与惯量是否进入安装版 Isaac articulation 数值动力学仍为 **UNKNOWN**；没有添加 payload 或额外惯量。

旧 QP-6 与 R1 同一 FR3 USD 的 DOF、mimic、limit 风险和裸臂源拒绝值得保留为背景：R1 原条件 C 在 residual 门失败，内侧 B/D 不能推广。它们使用不同 stage、工具条件和 120 Hz，不能证明本次 60 Hz 原场景的 M/c/g 或控制器通过，也不能单独证明本场景 source 必然失败。下一步需在原场景实际状态、已核验无碰撞条件下单独采集和验证含手指耦合的 M/c/g、时间和约束残差。

## 4. 力矩适配和 suction 状态接口

新增 [torque_adapter.py](../../offline/task28_p5_qp_6_r2/torque_adapter.py) 中，双臂 payload 使用明确 `effort` 和原九关节名称、共同 stamp；拒绝位置字段及与位置 sink 共同持有。默认关闭；由于原场景动力学和在线碰撞/闭链门尚未接通，`activate_torque()` 和 effort 计算均明确拒绝，伪造 owner 也不能绕过。它没有连接 Isaac actuator，也没有发送 torque；消息设计通过 CPU 负例不构成 `EXCLUSIVE_TORQUE_ADAPTER_VALIDATED`。旧 bridge 使用 `ArticulationAction(joint_positions=...)`，本任务没有启动或修改它。

[targets.py](../../offline/task28_p5_qp_6_r2/targets.py) 使用冻结 G1 的 Cube→双 TCP；旧 QP-6 理想耦合 HQP 及其 source 限制原样保留。本任务没有完成原场景的 14D 实际 task-space HQP、完整硬约束、在线碰撞接入或 live actuator sink，不能声称它已产生协作力矩。计划链条仍为 Cube 期望轨迹 → G1 左右 TCP → 已资格化的协作逆动力学 HQP → 双臂 effort；没有把预测 wrench 或 finger mimic reaction 作为 Cube 实测接触力。

[suction_interface.py](../../offline/task28_p5_qp_6_r2/suction_interface.py) 只做 CPU 状态合同设计，拒绝闭合。原合同为每臂一个 Surface Gripper、`3 mm` capture、`bendAngle=0`、`stiffness=1e4`、`damping=1e3`、force/torque break 均 `1e6`、不禁用重力及 retry close。没有调用 `start_bridge(enable_robot_commands=True)`，没有初始化或闭合 D6。break 阈值不等于已标定吸附容量。

## 5. 结果与停止条件

| 状态项 | 结果和实际范围 |
|---|---|
| `SCENE_IDENTITY_VERIFIED` | 停止时间线的实际新 stage 与核心 prim、资产、60 Hz 属性一致；不是物理运行资格 |
| `L_SUCTION_GEOMETRY_AND_TCP_VERIFIED` | 静态合同与停止 stage 的几何、局部 TCP 及 prim 读取一致；物理运行时 TCP/碰撞尚未验证 |
| `EXCLUSIVE_TORQUE_ADAPTER_VALIDATED` | 未通过；仅 CPU 原型和拒绝负例，无 live actuator sink |
| `ARM_DYNAMICS_SOURCE_VALIDATED` | 未通过；原场景的同步 q/dq/M/c/g 与约束资格未建立 |
| `ORIGINAL_SCENE_STATIC_TORQUE_PASS` | 未通过；静态 effort 试验跳过，当前状态、碰撞及 source 前置条件尚未建立 |
| `NO_CONTACT_COOPERATIVE_TRACKING_PASS` | 未通过；依赖动力学、实时几何/碰撞与完整 controller，故跳过 |
| `CUBE_D6_RUNTIME_NOT_YET_AUTHORIZED` | 是；未建立独立 attachment 与 reaction 观测先决条件，也无该任务外的闭合授权 |

有两次具备结果记录的只读场景构建，均为独立进程、停止时间线、0 physics steps、0 robot commands。此前一次系统 Python 启动因缺 `isaacsim` 失败；另外三次 Isaac 启动因在 `app.close()` 之后才写结果而没有产生机器证据，不能独立确定其执行范围或追认为通过；其中一份可用控制台日志保留为 `unrecorded_attempt.console.log`。已将独立只读探针改为先写结果再关闭，未将早期无证据尝试计入有效验证。

R1 之前各互不重叠测试重新执行：`789+53+5+6+22+54+30=959 passed`；R2 聚焦 `12 passed`，总计 **971 passed**。首次 QP-6 两组使用系统环境收集失败，原因是缺失 OSQP；使用 R1 记录的 `../qp6_deps` 重跑后分别 `54/30 passed`，失败与重跑日志均保留。CPU PASS 不等于物理 PASS。

起点 `069c03ed39001e107d1ee48ef7a8a74d33fd8787` 的 **1288 个 Git 跟踪文件**与 HEAD 一致；单独校验 FR3 USD SHA-256 相同。新增内容仅为 R2 目录与本报告；旧场景、bridge、MoveIt、USD、几何、ACM 及历史证据没有编辑。最终提交 SHA、远端 HEAD、工作树由提交后的交付核验提供。

要实现真实 Cube 搬运，首先需要在保留原 fixture 与独占执行器的前提下验证实际状态、原碰撞/闭链门和 60 Hz 同状态受约束动力学，完成可运行 HQP 与静态、无接触验证；再分别建立真实接近、独立 attachment 与接触反力观测，并取得另行明确的 Cube/D6 运行授权。R2 在此停止，没有 ROS、实体 FR3 或生产桥替换。

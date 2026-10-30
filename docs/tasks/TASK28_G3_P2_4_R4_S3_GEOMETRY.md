# TASK28 G3-P2-4-R4：S3 双臂几何前置条件审计

基线：`bfbbab1606e3afe7f06a48aeee4437c3d6cd1aba`。本文件只审计既有 P2-3/P2-3-R1、G1 和 R14 证据，没有启动 Isaac、接触、Cube、吸附、ROS 或真实机器人。机器记录见 [s3_geometry_preconditions.json](../../offline/task28_g3_p2_4_r4/evidence/s3_geometry_preconditions.json)。

结论是 **S3_NOT_READY / BLOCKED**。原始 Cube 接近路径继续保持 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`。P2-3 的静态双臂数据在力矩符号层面有局部证据，但稳态 TCP 没有满足冻结 G1 的双侧共同抓取几何，因此不能把双臂 wrench 或内部力分配当作同一物体的接触测量。P2-3-R1 的 `VIRTUAL_REPLAY_PASS` 只表示离线矩阵、KKT 和虚拟 oracle 分解可复算，不能覆盖该几何失败。

## 冻结几何合同

冻结对象标识为 `Cube_01`，名义物体中心为 `[0.65, 0, 0.4] m`、yaw 为 0。两侧 TCP 标识为 `left_fr3_side_suction_tcp` 和 `right_fr3_side_suction_tcp`。G1 名义 object-to-TCP 平移分别为左 `[0,-0.061,0] m`、右 `[0,0.061,0] m`；其中 61 mm 包含 60 mm 半边长与 1 mm 名义间隙。该 transform 是运动学抽象，不是已识别吸盘贴合点或承载能力。

每个 S3 观测帧必须同时通过：

- 左、右各自 live TCP 与冻结 FK 的平移误差不超过 `1e-4 m`、旋转误差不超过 `1e-3 rad`；
- 左、右由实际 TCP 反推的物体位姿分别与冻结物体一致；
- 左右反推物体位姿彼此一致，平移不超过 `1e-4 m`、旋转不超过 `1e-3 rad`；
- q、dq、TCP、时间戳和源身份在同一 physics step 对齐，残差覆盖整个执行局部包络，而不是只检查一个端点。

坐标约定为 `T_A_B` 把 B 坐标列向量映射到 A；六维 wrench 必须明确 force-then-torque 顺序、作用对象、参考点、frame、符号和单位。

## P2-3 失败的直接证据

P2-3 正式的 9 个条件全部在 frozen G1 closure gate 被拒绝。零载尾窗最能直接显示问题：

| 项目 | 左 | 右 | 左右相对 |
| --- | ---: | ---: | ---: |
| 抓取/物体平移误差 | 0.441309 mm | 0.333326 mm | 0.169917 mm |
| 抓取/物体旋转误差 | 0.000818598 rad | 0.000631200 rad | 0.001336417 rad |

其中左右相对平移和旋转都超过 `0.100 mm` 与 `0.001 rad` 的 G1 门限。9 个条件的相对平移误差范围为约 **0.146842–0.181124 mm**，相对旋转误差范围为 **0.00126603–0.00137465 rad**，所以加载方向变化没有恢复共同几何。每个条件的评估状态均为 `REJECTED`，且拒绝发生在 wrench 集成前。

这与离线 G1 初始状态是两个层次。离线 `lifted_symmetric` 的 IK 误差约为左 `1.87e-9 m`、右 `5.43e-10 m`，左右闭链约 `1.87e-9 m`、`2.06e-8 rad`，并通过单状态碰撞检查；这只能证明保存的种子构型，不能证明 Isaac position hold 后的稳态 TCP 仍在冻结抓取 frame 上。P2-3 报告明确记录了该稳态偏差，不能通过替换物体姿态、放宽 G1 门限或用虚拟回放覆盖。

## S3 必须补齐的前置条件

| 条件 | 当前状态 | 进入 S3 所需证据 |
| --- | --- | --- |
| 双臂共同抓取几何 | **BLOCKED** | 新隔离运行逐帧保存两臂 TCP、冻结 FK、左右反推物体 pose 和完整 G1 residual；全局 max 在门限内。 |
| 相对位姿/闭链残差 | **BLOCKED** | 覆盖全部 S3 局部运动边的残差记录、同 step cursor 绑定、q/dq 与 pre/post 时间关系；端点检查不能代替全过程。 |
| Cube body/anchor/contact identity | **MISSING** | 独立记录 Cube body、左右 anchor/contact frame、contact pair、attachment mode 及 attach/break/release/recapture 事件。 |
| 双侧 wrench source | **MISSING** | 两臂同 step 的来源适配输出，分别声明 effort 语义、关节顺序、符号、单位、参考 frame/point、时间戳和失效 latch；不能使用 synthetic oracle 作为 estimator 输入。 |
| 接触模型和独立参考 | **MISSING** | 有限刚度/阻尼/摩擦和可传 wrench 子空间、稳定 timestep、独立 PhysX 接触参考、穿透/力/功率/求解失败停止规则。 |
| 局部连续安全 | **BLOCKED** | 新接触夹具的初态、碰撞对、局部 swept envelope 和支持状态均需保守连续证据；原始接近路径仍禁止。 |
| 力分配与 torque budget | **UNREADY** | 独立 object net/internal oracle，`G/W/nullspace` 重构，任务与力项合并后的逐关节 effort/rate/QP 可行性。P2-1/P2-3-R1 数学结果只能复用为离线检查。 |

## 最短可执行路线

1. 建立新的隔离接触夹具并冻结 scene、asset、body、anchor、contact-pair 和配置哈希；直接构造的安全初态不得冒充原始 Cube 接近路径证明。
2. 先在无接触/受控单臂条件下完成严格 observer source adapter 和时间/失败门；两臂都具备合格同 step 来源后才组合。
3. 在新构型重新验证稳态双侧 G1 closure，并保存所有失败帧；任何一侧或相对 closure 超门限即停止进入 S3。
4. 用独立接触参考做零载、同净 wrench/非零内部 wrench、释放/abort 对照；控制器不得读取 oracle。
5. 只有几何、source、contact、局部安全和 torque/rate 门全部通过，才进入双臂内力调节；S4 Cube 吸附搬运另行准入。

历史 P2-3/P2-3-R1/G1 文件保持不变；本审计没有修改 R12/R13 控制参数，也没有把 `CONTINUOUS_SAFETY_NOT_ESTABLISHED` 改成通过。

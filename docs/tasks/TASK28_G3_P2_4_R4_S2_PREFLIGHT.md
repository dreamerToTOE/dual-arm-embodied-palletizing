# TASK28 G3-P2-4-R4：S2 严格 Observer Source Adapter 与受控接触预检

本报告记录基于 R14 的 S2 受控单臂接触准入预检。预检基线为 `bfbbab1606e3afe7f06a48aeee4437c3d6cd1aba`。本次没有启动 Isaac 接触、Cube 运动、吸盘/D6、ROS 控制或真实机器人命令；机器证据见 [`s2_preflight.json`](../../offline/task28_g3_p2_4_r4/evidence/s2_preflight.json)。

结论是 **`S2_NOT_READY / PREFLIGHT_ONLY_NO_CONTACT_RUN`**。现有证据足以继续实现严格 source adapter 和离线拒绝型测试，但不足以安全启动接触仿真。R14 已冻结的 G2 控制器和门限没有修改；原始 Cube 接近路径继续保持 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`。

## 观测器来源边界

P2-4 的候选观测器使用完整九维模型

```text
M(q) qdd + c(q,dq) + g(q) = tau_actuator_on_robot + tau_external_on_robot
```

并以因果后向差分、完整关节顺序、SI 广义单位、线性后角速度的 Jacobian 顺序计算 TCP 等效拟合。这个 API 的 `source_status=SOURCE_UNVALIDATED_CANDIDATE` 时，输出只能标为 `CANDIDATE_ESTIMATE`；它不提供 FR3 `tau_J` 传感器语义，也不能把未知模型误差、多点接触或约束反力唯一识别成外力。

Isaac 的 `get_applied_joint_efforts()` 是显式命令/输入缓冲的回读，`get_measured_joint_efforts()` 是求解器投影 DOF 力。两者都没有被本仓库证据校准为真实 FR3 `tau_J`。真实 `tau_J`、`tau_ext_hat_filtered` 和 `O_F_ext_hat_K` 在当前 Isaac 证据中不存在。仿真接触真值也必须走独立通道，不能由同一条动力学残差方程或控制器命令回读充当真值。

P2-4-R3 的 collector 在采集后才由 evaluator 重算 observer；非零外力的 application point 是最后一个点重复到整段，且没有独立 contact pair、frame、sign、step/time 记录。因此 R3 的 `CANDIDATE_ESTIMATE` 只能作为候选模型回放，不能升级为 S2 接触力观测。

## 既有场景和静态证据

现有 `task28_g1_0_new_single_cube_v0` 不是合格的 S2 单臂接触夹具：scene contract 明确状态为 `NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`，包含双臂、动态 `Cube_01`、L 型工具和 table，并且场景构建包含 ROS graph；bridge 还包含 SurfaceGripper/D6 路径。其 controller contract 仍标记 planning/commands 未实现。该 fixture 可作为几何和来源审查对象，不能直接作为 S2 实验入口。

两个只读静态检查通过：

- 8 个保守碰撞包络闭合、包含源点云并拒绝负例；
- G1-0B bundle/任务输入/渲染器/包络负例检查通过。

这些结果只证明离线几何文件和 fixture 合同一致，不能证明 PhysX 初始无穿透、运行时接触对、连续局部运动安全或接触力时序。历史 G1-2a validator 也明确其 transition gate 只检查端点和模式元数据，不计算插值边、扫掠体积或连续安全。

## S2 门禁结果

| 门 | 结果 | 依据和影响 |
|---|---|---|
| applied/projected effort 与 FR3 `tau_J` 分离 | `PASS_FOR_PREFLIGHT` | P2-4 observer 合同和 R14 审计已明确来源，不能做硬件等价声明。 |
| 同场景严格 source qualification | `MISSING` | 只有裸单 FR3 候选；没有本次接触夹具的已验证 source、完整时序和实际工具动力学。 |
| 独立接触力真值 | `MISSING` | R3 没有逐步独立接触 oracle；后处理 oracle 不能替代 PhysX contact truth。 |
| 安全初态、无初始穿透 | `MISSING` | 没有新的 S2 scene receipt 或初始重叠测量；现有 fixture 仍未物理验证。 |
| 局部连续运动包络 | `MISSING` | 保守静态包络不是接触夹具的窄相/扫掠证书。 |
| 接触对、frame、符号、step/time | `MISSING` | 没有独立 contact-source adapter 和同步记录。 |
| 力/能量/运动/时间预算和停止合同 | `MISSING` | R14 明确这些数值尚未由来源和不确定性预注册。 |
| pre-command fail latch 与独立 live cursor | `MISSING` | R14 S2 admission 的明确前置条件尚未建立。 |
| 接触/吸附模型限制 | `PARTIAL` | 现有 SurfaceGripper 使用聚合 fixed/D6 近似和 `1e6` 力矩限值；不能解释为真实吸附容量。 |
| 静态几何包络 | `PASS_STATIC_ONLY` | 仅支持离线几何审查，不解除任何运行时接触门。 |

因此不执行 S2。把现有双臂 Cube fixture 直接用作接触实验，会混入未经验证的 Cube 初态、L-tool 物理接触、ROS graph 和 SurfaceGripper/D6 语义，无法形成独立单臂力估计对照。

## 继续工作的最短路线

1. 在新 R4 身份中完成 source adapter：逐条记录 source kind、物理定义、关节顺序、符号、单位、frame、timestamp、dt、有效性，并拒绝未知/不兼容来源。拒绝后不推进 observer history，也不写控制命令。
2. 先做 CPU 负控：空样本、NaN、过期、错序、非 SPD、低秩 Jacobian、错误作用点、缺失逐步 oracle、重复 evidence 和 cursor 不匹配均应 fail closed。
3. 另建不含 Cube/吸盘/D6/ROS command 的单臂接触夹具，固定一个安全初态；在任何接触前记录 scene/asset/config hash、无穿透检查、所有接触对、独立 PhysX 真值 frame/sign/time、接触点和法向。
4. 运行前预注册小力参考、接触刚度/阻尼、力矩/速率预算、能量和超时停止条件；每步保留 pre-command cursor 与 post-step 状态，并将接触真值和 observer 输入分开保存。
5. 通过 S2 的开环/闭环单轴对照后，再审查 S3 的双臂几何、闭链、净 wrench/内力独立 oracle。P2-3 的几何失败和 QP-6 `contact_readiness=NOT_READY` 保持不变。

在上述门具备机器证据之前，任何 `CANDIDATE_ESTIMATE` 只能用于离线诊断；不能称为已验证外力观测，也不能用于双臂 Cube 吸附搬运。

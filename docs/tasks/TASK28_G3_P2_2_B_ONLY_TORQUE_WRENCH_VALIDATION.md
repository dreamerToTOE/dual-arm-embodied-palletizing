# Task28 G3-P2-2：B-only FR3 关节力矩到 TCP 外力旋量验证

**最终状态：`G3_P2_2_PARTIAL`**

本任务完成了一个隔离的单 FR3 静态 Isaac Sim 4.5 试验、独立的 CPU 数学估计器和拒绝型数据评估。数学链路和运行时关节名/索引可复核；非零已知载荷的仿真关节 effort 与预先声明的“作用于机器人”符号相反，因此当前不能宣称已验证的外力观测链，也不能进入双臂内力控制。

本结论不是物理机器人、ROS、生产桥、吸盘或动态 observer 的结论。

## 1. 起点和范围

- 仓库：`dreamerToTOE/dual-arm-embodied-palletizing`
- 分支：`side-suction-palletizing`
- 起始及验证 HEAD：`f6023f454df5c82d3aeab1ab2027ebd5964db769`
- Isaac Sim：本机 `/home/ubuntu2004/isaacsim-4.5.0`，4.5.0-rc.36；GPU 为 RTX 3070。仅启动了本任务的 headless 独立场景。
- 未启动 Task24/26/27/28 生产场景、ROS、bridge、吸盘或机器人命令。
- 下列五个字段在每个采集状态中均为 `false`：

```json
{
  "execution_authorized": false,
  "physical_robot_tested": false,
  "wrist_ft_sensor_required": false,
  "dual_arm_force_control_implemented": false,
  "production_bridge_modified": false
}
```

第一轮运行写出了 20 个条件文件但因 Isaac 关闭顺序没有写出状态文件，作为 `run_00_missing_status` 保留，未当作正式结果。修复证据落盘顺序后，在相同冻结配置下完成 `run_01`；加入严格 `source_kind` 记录后完成 `run_02`。`run_01` 没有 source_kind，其原始评估来自早期评估器；当前评估器不会把它视作来源验证通过。三轮原始文件均不覆盖；三轮对应的 20 个条件文件逐一哈希相同，符合该设置下的确定性，但不能作为独立人员或跨环境复现。run_00 和 run_01 的采集器源码快照根据改动重建，其哈希分别与当时记录的源码哈希一致。

## 2. 来源、模型和 API 语义

冻结 FR3 USD 为 `work/g1_0b_asset_audit/fr3.usd`，SHA-256：
`3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`。
冻结 fixture 中唯一外部视觉材质仍为 UNKNOWN；物理、关节和碰撞数据内嵌。

独立探针记录的实际 articulation DOF 为：

```text
0..6  fr3_joint1 ... fr3_joint7
7     fr3_finger_joint1
8     fr3_finger_joint2
```

估计器只取按名称解析的 `fr3_joint1..7`，没有盲切前七个数组。运行时 `get_measured_joint_efforts(joint_indices=[0..6])` 是沿 articulation DOF 运动方向的 solver projected joint force；它不是 libfranka 的 `tau_J` 传感器信号。探针没有调用 `get_measured_joint_forces()`，也没有把它作为 fallback。

本地官方 libfranka clone 的 commit 为 `95b406f47c97cedfbe426c04d8fb7f83a9885e1c`；`robot_state.h` SHA-256 为 `1fe6786683c6c1813b5e585d872c48431243976a1f7f265c7e8c15c0ff503243`。其中 `tau_J` 是 link-side 传感器扭矩，`tau_ext_hat_filtered` 是模型预期扭矩与 `tau_J` 的差并排除已配置负荷，`O_F_ext_hat_K` 的正号是机器人作用于环境；它们都不能由本次 Isaac solver effort 自动冒充。

`run_02` 记录了：`physics_dt=1/120 s`、重力 `[0,0,-9.81] m/s²`、position-drive 增益、solver position/velocity iterations `32/16`、self-collision 属性，以及 `SingleArticulation.get_measured_joint_efforts` 的来源。原始资产中的 `fr3_link8` 和 `fr3_hand_tcp` 触发了 PhysX “possibly invalid inertia tensor/negative mass”警告；该限制被保留，没有修改资产。

## 3. CPU 估计器

`offline/task28_p2_2/estimator.py` 接受显式的 q7、七维**外部**关节扭矩、6×7 几何 Jacobian、关节名顺序、frame、TCP reference point、单位、符号、时间和 provenance，并求解

```text
τ_ext = J_TCP(q)^T w_ext
```

输出最小范数 wrench、奇异值、数值秩、条件数、可观测投影、不可观测基和重建残差。它拒绝 raw total effort、错误顺序、错误单位、错误符号、失配 q/J、过期/异步/非有限输入；已知施加的力不会传给估计器。

使用冻结 G1 Jacobian 的两个构型，独立中心差分最大误差为 `6.74e-11` 和 `1.81e-10`；合成测试中两构型的数值秩均为 6，条件数约为 `8.6622` 与 `8.8875`（米/牛与弧度/牛米混合坐标中的数值诊断，不能解释为物理置信度）。极大但有限输入导致 SVD 派生值非有限的负控也会被拒绝。

这些是数学和接口检查，不是传感器校准。

## 4. 预注册的实际矩阵

配置文件 `offline/task28_p2_2/evidence/static_probe_preregistration.json` 在首次仿真物理步运行前冻结，SHA-256：`c4aad0a00b82baf2dbd597950a9fb75fef813d5c52e294f37f473392937261f2`。

- 构型：`home` 和 `offset` 两个满足冻结关节限位的 q7。
- 负载：零载、世界 +X、世界 −X、世界 +Y、世界 +Y 且相对虚拟 TCP 偏置 `[0.04,0,0] m`。
- 每项均有独立 unloaded/loaded reset；每个条件 600 个 settle step，末 60 帧逐物理步记录；物理步上限 12,000，墙钟上限 300 s。
- 作用点由 G1 冻结模型的名义 side-suction TCP 计算，力由 `RigidPrim.apply_forces_and_torques_at_pos(is_global=True)` 每步施加到 `fr3_link7`。这是隔离的虚拟 TCP 负载，资产中没有真实工具或 D6 吸附锚点。
- 预注册门限包括速度、窗口姿态/力矩稳定性、q 配对、FK、Jacobian rank、直接关节力矩误差 `0.10 N·m`、力误差 `0.30 N`、力矩误差 `0.08 N·m` 和零载差分 `0.05 N·m`。失败后未调整门限。

评估器先只把 `q`、loaded-minus-unloaded effort、Jacobian 和元数据传入估计器，估计返回后才读取已知负载作为独立 oracle；不自动翻转作用/反作用符号。

## 5. 实际 Isaac 结果

`run_02` 共写出 20 个条件文件、每个 60 帧，`actual_physics_steps=12000`，状态为 `COLLECTION_COMPLETE_NOT_EVALUATED`。DOF 名称和索引、配置哈希、资产哈希和 source kind 均通过独立记录核对；FK/观察位姿残差约为 `2.4e-7–6.6e-7 m` 和 `3.5e-7–6.6e-7 rad`，速度均低于预注册的 `0.01 rad/s` 门限。

零载的两个构型均通过几何/稳定性门禁，差分 effort 为零。8 个非零负载在评估器中全部拒绝，原因为：

- `JOINT_TORQUE_ORACLE_MISMATCH`
- `FORCE_ORACLE_MISMATCH`
- `POSSIBLE_ACTION_REACTION_SIGN_MISMATCH_NO_FLIP`

直接按预声明“wrench on robot”比较，逐案 `||Δτ − Jᵀw||₂` 为 `0.6553–1.4120 N·m`；相反号比较 `||Δτ + Jᵀw||₂` 仅为 `0.000115–0.000599 N·m`。估计的力分量约为注册 oracle 的反号，力误差约 `2.00 N`。带偏置的 `Mz` 也呈反号（约 `−0.0399` 对 `+0.04 N·m`）。这证明了可重复的符号不一致，但没有证明其唯一原因是某一个 PhysX 内部 convention；可能涉及 solver effort、外力 action/reaction 或点/刚体语义，需后续专门校准。

当前非零矩阵只激励 `Fx`、`Fy` 和偏置产生的 `Mz`。虽然 Jacobian 对 q 是六阶满秩，不能据此宣称 Fz、Mx、My 已由实际仿真力矩测量验证。offset_y 的 `0.04 N·m` 小于 `0.08 N·m` 的预注册力矩误差门，也不能作为强力矩校准证据。

## 6. 运行、证据和回归

完整 CPU 回归命令为：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=offline:offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0:offline/task28_g1_2b_1:offline/task28_g1_1/tests:offline/task28_g1_2a/tests:offline/task28_g1_2b_0/tests:offline/task28_g1_2b_1/tests:offline/task28_p4_nr_0/tests:offline/task28_p5_qp_0/tests:offline/task28_p5_qp_1/tests:offline/task28_p5_qp_2/tests:offline/task28_p5_qp_3/tests:offline/task28_p2_1/tests:offline/task28_p2_2/tests \
python3 -m pytest offline/task28_g1_1/tests offline/task28_g1_2a/tests offline/task28_g1_2b_0/tests offline/task28_g1_2b_1/tests offline/task28_p4_nr_0/tests offline/task28_p5_qp_0/tests offline/task28_p5_qp_1/tests offline/task28_p5_qp_2/tests offline/task28_p5_qp_3/tests offline/task28_p2_1/tests offline/task28_p2_2/tests -q -s -p no:cacheprovider --import-mode=importlib
```

最终完整回归 **627 passed**（原有 595 项及新增 32 项），退出码 0；最终修复后聚焦测试为 **32 passed**。先前完整回归在 source-kind 负控修补后有一次 `626 passed, 1 failed` 的旧测试预期冲突，已修复并重新执行；最终回归结果写入 `offline/task28_p2_2/evidence/regression_record.json` 和 `regression_final.log`。

实际复现需在安装相同 Isaac 及匹配哈希的资产环境执行，下述命令使用新输出目录，且只执行单臂独立场景；记录中的路径可替换为匹配哈希的本地资产路径：

```bash
OMNI_KIT_ACCEPT_EULA=YES PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh offline/task28_p2_2/static_isaac_probe.py \
  --asset /home/ubuntu2004/Documents/Codex/2026-10-25/files-pasted-by-the-user-task/work/g1_0b_asset_audit/fr3.usd \
  --output /tmp/task28_new_raw
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=offline:offline/task28_g1_1 \
python3 offline/task28_p2_2/evaluate_probe.py --input /tmp/task28_new_raw --output /tmp/task28_new_evaluation
```

当前结果下评估命令预期退出码为 1（`REJECTED`）；该退出码不是通过。采集命令和分析命令的输出目录均必须事先不存在。

所有原始观测、控制台、评估和哈希清单均在：

- `offline/task28_p2_2/evidence/isaac_run_00_missing_status_raw/`
- `offline/task28_p2_2/evidence/isaac_run_01_raw/` 与 `isaac_run_01_evaluation/`
- `offline/task28_p2_2/evidence/isaac_run_02_raw/` 与 `isaac_run_02_evaluation_v2/`
- `offline/task28_p2_2/evidence/static_probe_preregistration.json`
- `offline/task28_p2_2/evidence/preflight_source_audit.json`
- `offline/task28_p2_2/evidence/artifact_manifest.json`

第一轮的缺失状态文件、两轮完整但失败的原始记录、每个 case 的逐关节误差、估计 wrench、rank、FK/稳态统计和拒绝原因都保留。

## 7. B-only 决策和边界

Task28 继续采用 B-only 路线：不新增腕部六轴 F/T 传感器；已知外力只作为事后验证 oracle，不是估计器输入。本任务没有实现双臂外力观测、G3-P2-1 内力分解接入、有限吸附力分配、动态外力 observer、在线模型补偿、力控制或执行桥。

静态 loaded-minus-unloaded 差分不是可部署的在线观测器。真实 FR3 `tau_J`、模型期望扭矩、重力/惯性/摩擦、工具负载、控制器贡献、时钟和反馈新鲜度仍需单独建立；`O_F_ext_hat_K` 的机器人作用于环境正号在接入 Cube wrench 前必须取正确 action/reaction 变换。

## 8. 下一步前置条件

在重新声称静态 B-channel 通过之前，必须：

1. 用独立 action/reaction 控制和已知零载/正负载校准 Isaac solver effort 的符号与 application-point 语义，不能在 estimator 中隐藏翻号。
2. 加入预注册的 Fz、Mx、My 独立载荷，并使力矩激励显著高于固定误差门。
3. 明确原始 FR3 资产无效惯量警告的处理边界，确认它不会污染静态力矩差。
4. 仅在上述控制通过后，再设计双臂、时间同步和 G3-P2-1 wrench-on-Cube 的 frame/sign 转换。

因此本任务是数学/API 通过、实际静态测量关系未解决的 `G3_P2_2_PARTIAL`，不授予任何 ROS、Isaac 生产执行或物理实验授权。

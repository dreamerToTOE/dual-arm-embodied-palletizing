# TASK28 Stage A / G3：独立关节反力诊断

基线：`8d1d936d6f2c9fdc5a34e262fce809653ab3b3d2`。范围仅为原 Task27 场景中的第一件 Cube 抓取、共同抬升和固定悬停 `0 → 1 → 2 → 0 N`；不进入 Stage B。

## 来源与物理含义

安装版 Isaac Sim 4.5 的 `SingleArticulation.get_measured_joint_forces()` 经 `ArticulationView` 调用 `get_link_incoming_joint_force()`。本次运行每臂返回 `(13,6)`：base 是第 0 行，其余行按 `metadata.joint_indices[name]+1` 映射，包括 fixed joints；每臂 7 个 arm DOF 不能直接作为该数组行号。实际 `fr3_joint7/8` 位于第 7/8 行，完整 body/joint 顺序保存于 trace 的 `REACTION_SOURCE_METADATA`。

| 信号 | 语义 | 本任务用途 |
|---|---|---|
| applied actuator effort | 显式 effort command buffer 回读，单位 N·m | 已完成物理区间的控制输入与后续区间的新命令分开记录 |
| projected generalized effort | solver 在 DOF 运动方向的投影 | 保留原日志，不能当作单独的接触传感器 |
| incoming joint wrench | 父 link 经 incoming joint 传给 child 的六维力/矩 | 新的只读诊断通道，不接入控制器 |
| Surface Gripper/D6 reaction | 外部闭链约束的反作用力 | 安装公开接口中未获得反力 getter |
| model external residual/TCP-equivalent wrench | `M qdd+c+g−tau` 经 `Jᵀ` 反算 | 保留为模型候选估计 |

反力分量顺序为 `[Fx,Fy,Fz,Mx,My,Mz]`，单位为 N/N·m，坐标为 child incoming-joint frame，作用点为 child anchor。离线转换用 `R_world_child R_localRot1`，并保留 joint anchor；报告中的 joint8 moment 始终在该 anchor，不把它直接称为 TCP/Cube 接触力矩。安装版 tensor tests 对旋转 joint frame 的加载给出了对应验证。本地实现与文件 SHA 见 [source_api_audit.json](../../offline/task28_g3_stage_a/evidence/reaction/source_api_audit.json)。

[PhysX 官方文档](https://nvidia-omniverse.github.io/PhysX/physx/5.3.0/docs/Articulations.html)将该信号定义为 incoming transmission，并说明其含重力、惯性、驱动及外部约束等贡献，且是物理步平均值。[tensor 文档](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html)说明它使用 child joint frame；新版本文档仅作语义交叉参考，以本机 4.5 实现和运行记录为准。

Surface Gripper 安装 C++ 实现私有保存 `mJointHandle` 并调用 DynamicControl 创建外部 D6。公开 C++ 接口提供 handle、properties 与 broken-state，但未见反力 getter；Python binding 暴露的接口更少。`forceLimit/torqueLimit` 是破断阈值。USD 路径不存在不等于没有 runtime D6，因此本次补查 runtime handle，不再只靠旧的 `valid=false` 作判断。本任务没有改写这些接口、创建替代连接或更改场景。

## 仪表化与时间

新 recorder 逐物理步保存左右 q/dq、完整 incoming wrench 数组、link transform/velocity/acceleration、运行时质量/COM/惯量、arm-only M/c/g、TCP Jacobian/transform、Cube 位姿、吸盘状态、QP 目标与当前/上一物理区间 effort。metadata 单独保存，所有新测量仅用于日志。

记录发生在新 torque write 之前，反力和状态对应已完成区间；`reaction_diagnostic.applied_effort_completed_interval_Nm` 对应该区间，`canonical_command_effort_Nm` 是接下来区间的新命令。没有用未来 torque 去反算当前反力。

旧 fixed-sequence runner 用 GUI update 次数计时；一次 update 在本机可推进两步物理时间，旧名义 2/5 s 实际约 4/10 s。本任务改为按 Isaac physics clock 控制 2 s 线性过渡和 5 s hold，日志中包含各段实际起止时间。统计窗口固定为每段过渡结束后 1 s 至该段结束前 0.25 s，不从结果挑选窗口。

## 明确复现的实现问题

对于定义在 Cube 上的接触 wrench `f_Cube`：

```text
M qdd + h_arm = tau_actuator - J.T f_Cube
tau_external_on_robot = -J.T f_Cube
tau_actuator_target = M qdd + h_arm + J.T f_Cube
```

原 observer 用 `lstsq(J.T, -(M qdd+h−tau))`，其 action/reaction 反算符号与上式一致；原 actuator target 却写为 `h−J.T f_Cube`，把外部扰动的符号用于了补偿命令。本次修正 target 为正号，没有任意翻转估计读数。

原 nullspace basis 把整个 12 维向量归一化，使对称两侧的每侧力约为标量的 `1/sqrt(2)`；反馈估计使用两侧轴向力差的一半，单位是每侧 N。本次只在 live bridge 中将目标 basis 按同一半差定义缩放，保留已有 grasp 数学模块的单位范数语义。

原 observer 还使用了包含 payload gravity compensation 的 `h`。本次把实际 arm-only `c+g` 单独传入 observer；payload 补偿继续用于已有 G2 控制律。首帧候选 wrench 显式初始化，避免数据不可用时隐式走异常 fallback。

两个代数回归测试在原实现上失败（[failure receipt](../../offline/task28_g3_stage_a/evidence/reaction/baseline_mapping_failure.log)），修正后通过；另测了 payload 输入隔离、base/fixed-joint 索引、旋转 frame、GUI 多物理步计时和异常停止。

## 实验比较

主运行是两个独立的 GUI 回放：v2 保留原 bridge 的符号/单位实现，v4 使用本次修正后的 actuator 补偿符号、per-contact force 归一化和 arm-only observer bias。两次都从原 Task27 场景启动，未修改 USD 或场景构建；v2 有窗口/进程证明，v4 的 `headless=false`、截图、summary 和完整压缩 trace 保存在对应目录。

| 项目 | v2（修正前基线） | v4（修正后） |
|---|---:|---:|
| GUI/执行状态 | `COMPLETED` | `COMPLETED` |
| QP 状态 | 7103 `QP_OPTIMAL` | 7331 `QP_OPTIMAL` |
| 有效 reaction 帧 | 7104/7104 | 7332/7332 |
| 力矩饱和 / 变化率激活帧（固定段） | 0 / 0 | 0 / 0 |
| fixed Cube 峰值漂移 | 2.094 mm | 2.790 mm |
| fixed 双臂相对 TCP 峰值 | 0.0591 mm | 0.0535 mm |
| 最大固定段关节变化 | 0.00564 rad | 0.00766 rad |

两个运行的固定段统计窗口均从每段过渡结束后 1 s 开始，至该段结束前 0.25 s，实际约 3.72 s。v2 的候选轴向量为 `+0.0273, -0.0495, -0.1243, +0.0274 N`；v4 为 `+0.0232, +0.1408, +0.2615, +0.0232 N`（目标段依次为 0、1、2、0 N）。v4 的 joint8 incoming 世界系 `Fy` 左/右均呈反号且与候选值接近：1 N 段 `+0.14084/-0.14083 N`，2 N 段 `+0.26149/-0.26148 N`。零目标段仍有约 `0.0232 N` 基线。v4 的两臂 J2 已应用力矩从首个零目标段的约 `-10.9978/+0.4466 N·m` 变化至 2 N 段的 `-11.1052/+0.3078 N·m`，回到零目标时为 `-10.9976/+0.4469 N·m`。各段完整 14 关节 effort 和位姿统计见 metrics。v2 与 v4 的关节姿态也不同，估计符号不能作为匹配姿态下的前后对照；v4 同时修正了驱动补偿代数。它们用于检查读数链路和回归，而不是宣称性能提升。

启动未加载窗口的 joint8 `Fz` 约为 `19.093 N`，对应末端子树质量 `1.94628 kg × 9.81 m/s²`；这验证了 incoming wrench 包含重力传输。固定段条件惯性上界约 `0.0010–0.0017 N`，远小于 1/2 N 段读数变化，但不能排除驱动、闭链和其他约束贡献。所有统计窗口 `all_suctions_closed=true`，但 recorder 没有得到 D6 reaction getter；Cube 位移与 incoming wrench 只证明仿真链路有可观测变化，不证明 suction 接触力已被独立测量。

证据文件：v2 的 [summary](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v2/summary.json)、[metrics](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v2/reaction_metrics.json)、[GUI process proof](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v2/process_proof.txt)、[comparison plot](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v2/reaction_comparison.png) 和 [compressed trace](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v2/task27_torque_bridge.jsonl.xz)；v4 的 [summary](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v4_corrected/summary.json)、[metrics](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v4_corrected/reaction_metrics.json)、[comparison plot](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v4_corrected/reaction_comparison.png) 和 [compressed trace](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v4_corrected/task27_torque_bridge.jsonl.xz)。

## 归因与结论

证据支持的归因如下：

1. **可读性和映射：强支持。** `get_measured_joint_forces()` 返回的 `(13,6)` 行按 `joint_index+1` 映射；base 与 fixed-joint 行没有被误当作 7 个 arm DOF。v2/v4 每一帧均通过有限值、形状、关节顺序检查，且 joint8 世界系 `Fy` 随固定序列改变并在两臂反号。
2. **重力/动力学基线：强支持。** 未加载时 `Fz≈19.093 N` 与记录的末端子树质量重力一致；固定段条件惯性界限只有毫牛级到千分之一牛级。incoming wrench 不能因此被解释为纯接触力。
3. **候选估计与 incoming transmission 的一致性：中等支持。** v4 的 1 N 和 2 N 段 joint8 `Fy` 与 arm-only 模型候选分别约 `0.1408 N` 和 `0.2615 N`，两臂符号相反；新关节反力没有被接入控制器（`new_reaction_used_by_controller=false`），已有模型候选仍用于原反馈。这支持“独立仿真关节反力候选通道可用”，不支持“已测得 Cube/suction wrench”。
4. **QP 或限幅导致的读数失真：当前证据较弱。** 两次完整回放的 QP 都为 `QP_OPTIMAL`，力矩饱和和变化率激活均为 0；因此本次没有看到约束饱和主导反力读数的证据。v3 的失败是启动时 MoveIt 参数服务超时后触发 `LOCAL_SWEEP_LIMIT`，不是观测器数据拒绝；v4 在隔离 DDS 后完成。
5. **D6/suction 接触反力：未建立。** 安装版 Python DynamicControl 只报告 `create/destroy/get/set properties`，v4 recorder 的 runtime 记录为 `handle=0`、`reaction_getter=null`；Surface Gripper 的 joint handle 是私有成员。没有独立 D6 反力或接触 wrench 参考，不能将 incoming transmission、模型 residual 或 Cube 位移直接命名为接触力。

最终分类为 **`G3_JOINT_REACTION_CANDIDATE`**。本次交付了独立、可审计的 Isaac joint-reaction 诊断输入和明确的时间/符号/单位边界；它没有完成独立 suction 接触力验证或闭环力控验证。新 reaction 数据只写日志，不改变 QP 控制输出。

## 失败记录与范围边界

v1 在沙箱内因 GPU 不可用而未建场景，保留 [launch receipt](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v1/launch_receipt.json)。v3 在默认 ROS 域中完成窗口启动和 546 次最优 QP 后因 `LOCAL_SWEEP_LIMIT` 停止，保留 [summary](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v3_corrected/summary.json) 和 [compressed trace](../../offline/task28_task27_integration/evidence/task28_g3_reaction_v3_corrected/task27_torque_bridge.jsonl.gz)；它没有被当作通过运行。v4 是修正后完成的独立 GUI 回放。

本次没有 Stage B、REGRASP/PUSH、五 Cube continuation、真实机器人命令、Sim2Real 或吸盘参数改动；吸附沿用原 Surface Gripper 的运行时连接流程，没有手工创建或修改 D6。原始 HOME→Cube 接近路径仍为 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`。

## 复现与检查

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=73 \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib \
/home/ubuntu2004/isaacsim-4.5.0/python.sh \
isaac/scripts/task28_task27_one_pass.py --mode integrated --live-qp \
  --max-batches 1 --fixed-suspended-force-sequence --record-joint-reactions \
  --fixed-force-transition-s 2 --fixed-force-hold-s 5 \
  --internal-force-weight 1000 --internal-force-kp 0.5 \
  --local-sweep-limit-m 2.0 --execution-time-scale 16 --timeout-s 900 \
  --run-id <fresh_directory_name>

PYTHONPATH=offline/task28_g1_1:. python3 -m pytest -q \
  offline/task28_g3_stage_a/tests offline/task28_p2_1/tests \
  offline/task28_p2_2/tests offline/task28_p2_4/tests \
  offline/task28_p5_qp_6_r8/tests offline/task28_task27_live_qp/tests \
  offline/task28_t0/tests

PYTHONPATH=. python3 -m offline.task28_g3_stage_a.analyze_reaction \
  offline/task28_task27_integration/evidence/<run_id>
```

回归：156 passed，Python 编译检查通过。没有修改 G2 gains、QP 速度/力矩/变化率硬约束、USD/机器人/工具/Cube/原吸盘及场景物理配置。ROS 仅用于既有仿真规划与命令通道；没有真实机器人命令、Sim2Real、Stage B、REGRASP/PUSH 或五 Cube continuation。原始接近路径仍为 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`。

最终交付 commit 与远端状态见本次交付回执。

# Task28 G2-P5-QP-6-R6 主动 effort 识别与受控响应审计

## 简短结论

R6 **确实向原始双 FR3 场景发送了受限诊断 effort**：A/B/C/D 四个独立 fresh Isaac 进程各执行 18 个双臂命令，共 72 个双臂 command steps、144 次 articulation 写入。每次都使用完整九维 effort buffer，立即 readback 与 post-step 状态均匹配。

这不是 formal torque tracking。B 的 `+0.25 N·m` 和 C 的 `-0.25 N·m` 在注册的 joint index 3 上产生相反的真实关节、link8 和 L-tool TCP 响应；D 重复 B，结果逐数值一致。因而 `BOUNDED_TORQUE_RESPONSE_VALIDATED=PASS`，但只达到诊断级 Level 1/主动 I/O 资格。

正式 static torque hold、computed-torque tracking 和 cooperative QP **均未通过前置条件而跳过**。原始 mounted-tool 的 M/c/g 仍包含手指耦合，L-tool 惯量未知，`get_applied_joint_efforts()` 只是 command buffer，`dq` 是 TGS 末步 solver velocity；独立 residual 只能标为估计值。因此 `ARM_DYNAMICS_STATUS=PARTIAL_MEASURED_RESPONSE_MODEL_RESIDUAL_NOT_CERTIFIED`。

下一步只能先做一项独立、注册的模型资格实验，覆盖 L-tool 惯量、手指耦合、残差和独立符号检查；在它通过前不要启动正式 static hold、短轨迹、cooperative QP、Cube/D6、吸盘闭合或硬件。

## 审计身份与范围

* 仓库：`dreamerToTOE/dual-arm-embodied-palletizing`
* 分支：`side-suction-palletizing`
* 起始 HEAD：`ddddc2f7311372545c9c0af9bd04d6aed7c577b5`
* 场景入口：`fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py::build_scene()`
* 新增范围：`offline/task28_p5_qp_6_r6/` 与本报告。
* 原 FR3 USD SHA-256：`3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`。

R6 没有修改 fixture、USD、ACM、G1/FCL、ROS bridge、历史 R3/R4/R5 证据、Cube 位姿、L-tool 几何、物理 scene 或 R5 的非基线 `enableExternalForcesEveryIteration` 选项。每个 cell 都新建 anonymous stage 和独立 Isaac 进程；ROSGraph 在该 anonymous stage 中关闭，没有启动 command sink。GPU 没有重试。

## 为什么替换 R4/R5 的前置门

PhysX 的固定步循环在 `simulate()` 完成并 `fetchResults()` 后才提供该步结果，TGS 子步的末步速度可以不同于整步位移差分；因此 R4 的 `FROZEN_Q_NONZERO_DQ`/`fd(q)/dt == dq` 硬门会把合法的 solver 语义误判为 source failure。R6 保留 q/dq copy、同步时钟、FK、碰撞、身份和 ownership 门，把整步差分降为 estimator，并新增两个层级：

1. `DIAGNOSTIC_ACTUATION_SOURCE_READY`：允许通过身份、时钟、fresh state、FK/TCP、FCL、无竞争 sink、手指保持、effort mode、限值和速率门后执行很小的预注册输入，不要求完整逆动力学。
2. `MODEL_BASED_CONTROL_SOURCE_READY`：还需实际响应、模型 residual、手指耦合和独立 sign/repeat 证据。R6 的 measured response 通过，但模型资格未完成。

Isaac 4.5 的 effort setter 会建立全 DOF 零 buffer 后再写索引；因此 R6 沿用 R3 adapter 的 full-buffer 规则，发送九维向量并保留手指 command buffer。`get_applied_joint_efforts()` 没有被称为 torque sensor，`get_measured_joint_efforts()` 仅记录为 solver-projected effort。

资料审查原文见 [`literature_and_source_review.md`](../../offline/task28_p5_qp_6_r6/literature_and_source_review.md)。参考资料：[PhysX Simulation](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/Simulation.html)、[Isaac Sim 4.5 Core Prims API](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.prims/docs/index.html)、[TSID](https://github.com/stack-of-tasks/tsid)。

## 预注册诊断

`preregistration.json` 在新的 Isaac 运行前创建，规定 native drive 先把双臂带到真实 FREE_HOME；之后仅运行时把七个 arm 的 Kp/Kd 置零 (`save_to_usd=False`)，保留两个 finger 的 gain/mode/约束。重力 holding effort 从该 cell 的实际姿态重新读取，扰动只在 joint index 3 上施加 ±0.25 N·m，限值 45 N·m、速率 2400 N·m/s、每 cell 18 步。

| Cell | 输入 | 结果 |
|---|---|---|
| A | `g_arm`，零扰动 | 18/18 命令完成，无意外碰撞 |
| B | `g_arm + 0.25`，左右同号 | 18/18 命令完成，joint 3 正向响应 |
| C | `g_arm - 0.25`，fresh reset | 18/18 命令完成，joint 3 反向响应 |
| D | fresh reset 重复 B | 18/18 命令完成，逐数值复现 B |

每个 command 在两臂写入前持久化 q/dq、M/c/g、gain/mode、link7/link8/TCP、FCL、finger state 和完整九维向量；写入后立即读回，再执行一个真实 60 Hz physics step。任何失败会 latch 并停止 cell；本次没有触发失败。

## 实际响应

独立 evaluation 保存在 [`evaluation.json`](../../offline/task28_p5_qp_6_r6/evidence/evaluation.json)。joint 3 的累计位移如下：

| Cell | 左臂 joint 3 Δq (rad) | 右臂 joint 3 Δq (rad) | TCP 位移量 |
|---|---:|---:|---:|
| A | +0.00101471 | +0.00103760 | 2.43 mm / 1.72 mm |
| B | +0.02703857 | +0.02660370 | 4.94 mm / 9.55 mm |
| C | −0.02488708 | −0.02433014 | 0.71 mm / 6.56 mm |
| D | +0.02703857 | +0.02660370 | 4.94 mm / 9.55 mm |

B/C 符号检查通过，B/D 最大重复误差为 0 rad。四个 cell 的每次 applied readback 都匹配完整请求；每个 cell `robot_command_writes=36`，总计 144。FCL 只保留原始 Cube/table 支撑接触，`unexpected=[]`；没有 Cube 接触、吸盘命令或 ROS command。

## 动力学边界

每步保留完整九维状态，而不是把两个 constrained mimic fingers 当成独立自由加速度。独立估计

```text
r_est = M qdd_est + c + g - tau_command
qdd_est = (dq_end[k] - dq_end[k-1]) / dt
```

最大 arm residual estimator 为 `0.0549245 N·m`，但这是用 TGS 末步 `dq` 做的数值估计，不是测得 torque，也没有识别 L-tool 惯量、接触反力或 finger coupling。因此只记录为 `PARTIAL_MEASURED_RESPONSE_MODEL_RESIDUAL_NOT_CERTIFIED`，不能把它升级成完整 inverse-dynamics/HQP 资格。

R3 适配器的历史 mock-only 结果、R4 source rejection 和 R5 TGS 诊断都保留在 [`historical_preservation.json`](../../offline/task28_p5_qp_6_r6/evidence/historical_preservation.json)；原先约 0.92 path fraction 的 direct-joint-interpolation 碰撞负例也保留在 [`approach_preservation.json`](../../offline/task28_p5_qp_6_r6/evidence/approach_preservation.json)。

## 跳过项与边界

* `STATIC_TORQUE_HOLD_STATUS=SKIPPED_MODEL_BASED_PREREQUISITE`。A 是诊断 holding baseline，不是 formal static hold。
* `SHORT_TRACKING_STATUS=SKIPPED_STATIC_HOLD_PREREQUISITE`。
* `VIRTUAL_COOPERATIVE_QP_STATUS=SKIPPED_R6_SCOPE`。
* `CUBE_D6_NOT_TESTED=true`。没有 suction closure、Cube attachment、lifting、transport 或 hardware/ROS。
* 没有自动发 unverified zero torque 或宣称 hold-last safety；simulation termination 不是 hardware emergency stop。

## 负例与回归

R6 负例覆盖 incorrect torque sign、wrong arm identity、invalid units、duplicate ownership、position drive active、wrong dt、stale/aliased state、finger buffer corruption、effort/rate overflow、unexpected collision、invalid mass/model、未声明 TGS 语义和 failure-latch 后 command。共 15 个，状态为 `ALL_R6_NEGATIVE_CONTROLS_REJECTED`，写入 0。

回归计数：

* 旧 1003：历史 789 + QP-4 53 + QP-5 5 + QP-6/R1 30 + R2 12 + R3 7 + QP-6 54 + P5-R1 6 + P5-R2 22 + R4 focused 12 + R5 focused 13，全部通过。
* R6 focused：12 passed。
* R4+R5+R6 focused：37 passed；累计可复现实验测试总数 1015 passed。

日志、命令 journal、逐帧物理响应和 negative controls 都在 [`offline/task28_p5_qp_6_r6/`](../../offline/task28_p5_qp_6_r6/)；测试汇总见 [`test_summary.json`](../../offline/task28_p5_qp_6_r6/evidence/test_summary.json)。

## 最终 Git 验证

R6 只提交了本目录和本报告。最终提交、远端 HEAD、工作树清洁状态、归档哈希在交付时写入同目录的 `final_git_verification.json`。

R6 在此停止。未经新的独立授权和模型资格证据，不继续 QP-6-R7、Cube/D6、内部力、ROS、SVM/PCS/RSS、RL 或真实机器人。


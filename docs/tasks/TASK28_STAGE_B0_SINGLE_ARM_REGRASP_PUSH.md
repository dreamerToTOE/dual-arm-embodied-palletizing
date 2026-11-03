# TASK28 Stage B0：单臂重抓与 +X 推送

## 状态：PARTIAL

在原 Task27 Isaac Sim GUI 场景中，三次完整尝试均完成冻结 Stage A、右臂在 Cube_01 的 −X 面单侧重新吸附，并用 live torque QP 物理执行原有 16 段 +X 推送。**三次均未通过执行器的深墙 3 mm 支撑间隙门限，因此 B0 未完成**：没有执行后续侧压、最终单侧释放和最终退避，也没有证据证明释放后稳定。不能把 QP 求解成功或实际推动误报为完整 B0 PASS。

冻结基线为 `966fda6195723b45c63c04a182d4c800085d9a89`。它记录三次 Stage A GUI 物理周期复现，0.25 mm XY 精度指标仍未满足；本任务不重新优化它。原 Task27 场景几何和夹爪保留。原场景抓取前仍通过直接设置位姿将休眠 Cube_01 唤醒到供料槽；该供料行为与其后的物理抓取、推送应分开理解，本轮没有在物理推送期间直接写 Cube 位姿。机械臂力矩接管后的 position-arm 写入计数为 0。

| 尝试 | 单臂反馈 Kp/Kd | 推送后的执行器深墙 gap | 结果 |
|---|---:|---:|---|
| v1 | 12/5，既有自由臂反馈 | +9.722 mm | 16 段实推后门限失败；executor 1 |
| v2 | 48/10，仅已确认吸附的 B0 推送臂 | +3.714 mm | 16 段实推后门限失败；executor 1 |
| v3 | 48/10；末端等待从 1.5 s 暂延至 4 s | +3.679 mm | 16 段实推后门限失败；executor 1 |
| v4 | 96/14 预定配置 | 未进入 B0 | GUI 进程收到 SIGTERM，退出码 143；原因未知 |
| v5 | 96/14 预定配置 | 未进入 B0 | GUI 进程收到 SIGTERM，退出码 143；原因未知 |

v4 在执行器启动前中断，v5 在 Stage A 左臂接近期间中断；都没有 summary 或 B0 结果。中断后仅停止了对应 ROS domain 的孤儿进程。96/14 未在 B0 中验证，未保留；4 s 等待未使 v3 通过，亦已撤回。当前代码保留 v2/v3 测试过的 48/10，以及原始 1.5 s 等待。该控制配置只是部分物理验证，不能视为最终方案。v2/v3 后进一步加入仅 B0 模式、且已先观察到双侧吸附和释放时才允许该增益的范围防护；这一防护经单元测试，尚未重新完成物理运行。

## 原工艺与本次接入

- `task27_five_cube_center_insert.cpp` 编译包含 Task26 执行器；复用 `PREPLANNED_COMMON_RETREAT`、`REGRASP_SWING`、`preplanPush()`、原 RRTConnect 候选、IK 和同步 FCL 检查、`executePushWithSupervision()` 的 16 段 +X 轨迹、物体 GT 与吸附反馈。Cube_01 为第一批唯一物体，执行器和 runner 均限制 `first_batch=1, max_batches=1`，不会到 Cube_02。原 Task27 外侧件需要后续 +Y 侧压，其原有完整链已做预检，但由于深墙门限未通过，本轮没有物理执行侧压。
- `isaac/scripts/task28_task27_one_pass.py` 新增 `--stage-b0-only`，要求集成 live QP、唯一批次、冻结的对象参考与 G3 载荷配置；向执行器传递 B0 标志并把失败退出记为失败。Stage A 的对象中心参考、世界系旋转修正、G3 和 QP 配置未改变。
- `isaac/scripts/task28_task27_torque_bridge.py` 离开 `DUAL_ATTACHED` 时清除对象参考、双臂保持状态及末端积分；原模式切换以吸盘**命令**触发单臂模式，物理关闭由独立 Surface Gripper 反馈逐帧确认。仅 `--stage-b0-only`、已经完成双侧释放且推送臂的吸盘实际 CLOSED 时，将该臂的独立关节反馈从 12/5 改为 48/10；辅助臂不改。独立模式的共同/相对 QP 软权重为零，不施加双臂闭链目标；无双臂 payload 或 G3 内力项。
- `ros_ws/src/fr3_dual_palletize/src/task26_truck_box_push_in.cpp` 增加 B0 单件边界、落桌／双侧打开检查、实际重抓几何和辅助臂吸盘状态记录，以及完整结束时的 B0 标记。该路径没有改变原 Task26 默认工艺；执行器每次代码变更后重新编译，v1/v2 的 build 与 install 二进制 SHA-256 均为 `6e38ab822f6cae9fa444609bcf1af9e0aebaaf893a475a4f77e4b0fd70a6e879`；v3 的 4 s 版本为 `8341bc30a8156e2bad9e2301799c96498c74e792e1fa6c128c03a85918da595b`。该等待修改已撤回并重新编译为前者。
- 新增 `offline/task28_task27_integration/analyze_b0.py`，保留完整 60 Hz 原始数据在本地，将状态、数值和 10 Hz 时序保存为紧凑证据；`offline/task28_g3_stage_a/tests/test_full_cycle_integration.py` 增加模式切换、单件范围和证据检查。

## 实测转换、重抓和推送

三个运行均记录 `DUAL_ATTACHED → RELEASE_RETREAT → SINGLE_ARM_PUSH`。Stage A 双侧 OPEN 首帧的 Cube/Table 竖向力分别为 **7.863 / 7.858 / 7.816 N**；随后执行器记录实际桌面支撑、双吸盘 OPEN 和共同退避完成。原有释放后静默互锁有效，两个滑轨从 **0.650 → 0.750 m** 重新站位，规划世界按原工艺偏移重建。随后根据实际 Cube 位姿规划重抓及整条推送／退出链。

| 实测指标 | v1 | v2 | v3 |
|---|---:|---:|---:|
| 吸附前右 TCP 至 Cube −X 面 gap (mm) | 0.970 | 0.862 | 0.990 |
| 吸附前 TCP 相对 Cube 的 Y / Z 偏差 (mm) | −0.012 / −0.127 | +0.049 / −0.070 | +0.133 / −0.162 |
| 实际关闭反馈：左 / 右 | OPEN / CLOSED | OPEN / CLOSED | OPEN / CLOSED |
| 首次单侧 CLOSED 到最后单侧 CLOSED 的实际 Cube ΔX / ΔY / ΔZ (mm) | +320.866 / +3.188 / +0.331 | +328.220 / +0.001 / +0.107 | +327.523 / −0.039 / +0.078 |
| 16 段末：命令 X / 执行器记录实际 X (m) | 1.1000 / 1.0903 | 1.1000 / 1.0963 | 1.1000 / 1.0963 |
| 执行器验收时深墙 gap / 高度误差 (mm) | +9.722 / 0.332 | +3.714 / 0.106 | +3.679 / 0.076 |
| 最后轨迹帧 Cube XYZ (m) | `(1.090670, 0.063382, 0.260331)` | `(1.097201, 0.059769, 0.260107)` | `(1.097177, 0.060013, 0.260078)` |
| 最后轨迹帧相对目标 `(1.100, 0.243, 0.260)` XYZ (mm) | `(-9.330, -179.618, +0.331)` | `(-2.799, -183.231, +0.107)` | `(-2.823, -182.987, +0.078)` |
| 最后轨迹帧姿态 XYZ 欧拉角 (°) | `(-0.00004, 0.316, 1.753)` | `(-0.00000, 0.102, 0.117)` | `(-0.00003, 0.074, 0.154)` |
| 最后轨迹帧 Cube/Table Fz (N) | 1.337 | 1.328 | 1.311 |
| 最后轨迹帧吸盘左 / 右 | OPEN / CLOSED | OPEN / CLOSED | OPEN / CLOSED |

相对目标 Y 偏差较大是因为原工艺要求先在中央安全行 +X 推入，之后再侧压至 +Y 目标；本轮侧压未开始。最后帧 Fz 是吸附状态下的瞬时桌面接触力，不证明释放后稳定，也不证明深墙接触。

执行器的 gap 使用原 Task27 ROS `/cube_poses`（原桥由 USD world transform 发布），紧凑轨迹使用 `SingleRigidPrim.get_world_pose()`；两者分别采样于不同数据路径及时间。v2/v3 最后一帧按中心几何计算分别为 2.799 / 2.823 mm，但执行器原验收时读到 3.714 / 3.679 mm，且已经失败。**不能用末帧数值追认原门限通过**；这两条状态路径的偏差及时间对应仍需专项核对。中心几何 gap 也不是独立的深墙接触力测量。

v1/v2/v3 单侧实际 CLOSED 期间有 1857 / 1931 / 2177 个 QP 帧，均记录独立关节跟踪、相对目标关闭、对象参考不活跃、双臂 G3 与 payload 关闭。16 段的 Cube 实测逐段位置和偏差见执行器日志；右臂实际吸附、左臂 OPEN，与规划结果分开核验。执行器对三次均在深墙门限停止，没有将单侧吸盘执行最终 OPEN，因此仅能报告当时保持吸附；仿真停止不等于完成受控释放。

## QP、约束与诊断

| 指标 | v1 | v2 | v3 |
|---|---:|---:|---:|
| QP optimal / infeasible | 14277 / 0 | 13391 / 0 | 13873 / 0 |
| 最大施加力矩：左 / 右 (N·m) | 35.824 / 36.197 | 33.666 / 33.588 | 36.315 / 31.669 |
| 最大逐关节力矩变化率 (N·m/s) | 942.239 | 1581.264 | 1462.525 |
| 最大实测关节速度 (rad/s) | 0.301 | 0.224 | 0.295 |
| 最大 QP 关节加速度 (rad/s²) | 10.751 | 10.376 | 10.436 |
| 单侧吸附期间全关节跟踪峰值 / RMS (rad) | 0.06568 / 0.01427 | 0.01946 / 0.00423 | 0.01849 / 0.00400 |
| 力矩锁存错误 / 接管后 position-arm 写入 | 无 / 0 | 无 / 0 | 无 / 0 |

每臂关节力矩限制 `(87,87,87,87,12,12,12) N·m`，变化率限制 `(2400,600,240,2400,240,240,240) N·m/s`，各关节记录峰值均未超出各自限制。QP 的位置和速度约束、20 rad/s² 加速度上限均未放宽。紧凑 JSON 保留逐关节峰值；力矩为仿真施加命令而非真实关节力传感。FCL 为已有采样检查，不是连续碰撞证明；采样的接触诊断未构成全场景碰撞力认证。

v1 最后一帧右臂第 7 关节参考误差约 0.0481 rad，QP 预测的关节加速度约 0.567 rad/s²，有限差分实测约 −0.006 rad/s²，且该帧无力矩／变化率饱和指标。单臂反馈提高后，v2/v3 的跟踪峰值和深墙验收间隙均下降，但数据不足以将残差完全归因于摩擦、模型误差或任何单一因素；提高增益也未达成完整任务。v3 延长等待后仍未通过原验收，故不继续参数扫描。下一步应首先同步核对 USD／PhysX 的 Cube 位姿和执行器验收时刻，再针对剩余间隙制定单一受控修正；不得放宽原 3 mm 标准。

## 证据与回归

- v1：[指标](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v1/compact/b0_metrics.json)、[10 Hz 时序](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v1/compact/b0_series.jsonl)、[执行器日志](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v1/task27_executor.log)、[summary](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v1/summary.json)。
- v2：[指标](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v2/compact/b0_metrics.json)、[10 Hz 时序](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v2/compact/b0_series.jsonl)、[执行器日志](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v2/task27_executor.log)、[summary](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v2/summary.json)。
- v3：[指标](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v3/compact/b0_metrics.json)、[10 Hz 时序](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v3/compact/b0_series.jsonl)、[执行器日志](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v3/task27_executor.log)、[summary](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v3/summary.json)、[GUI 场景截图](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v3/scene_loaded.png)。截图为场景初始化，不是最终释放截图；各次原始大型 60 Hz trace 留在本地。
- 中断记录：[v4](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v4/interruption.json)、[v5](../../offline/task28_task27_integration/evidence/task28_stage_b0_torque_v5/interruption.json)；v5 的执行器日志记录中断前进度。信号来源未确定。
- 聚焦回归命令：`PYTHONPATH=offline/task28_g1_1:. python3 -m pytest -q offline/task28_task27_live_qp/tests offline/task28_g3_stage_a/tests`，本轮最近一次结果 **51 passed**。保留原 Stage A 基线证据，未开始 B1、五 Cube 连续码垛或 Sim2Real。

复现须使用不存在的独立 run ID 与 ROS domain。v2 命令示例：

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=104 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 --stage-b0-only \
  --consistent-object-reference --assume-internal-contact-load \
  --internal-force-target-n 1 --record-joint-reactions \
  --internal-force-weight 1000 --internal-force-kp 0.5 \
  --local-sweep-limit-m 2.0 --execution-time-scale 16 \
  --timeout-s 2400 --run-id NEW_UNIQUE_RUN_ID
```

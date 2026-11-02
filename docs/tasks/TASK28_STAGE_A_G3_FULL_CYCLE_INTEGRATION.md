# Task28 Stage A：G3 载荷前馈完整周期集成

基线 `2b233c8d9820f6317a8003763cfd5e385a3ed174`，分支 `side-suction-palletizing`。已完成匹配姿态 5 mm OFF/ON 对照，以及 Cube_01 原 Stage-A-only GUI 物理周期。未进入 Stage B；完整运动仍存在参考跟踪与落桌残差，未宣称全程高精度验收通过。

## 最小集成与基线保护

场景入口仍为 `isaac/scripts/task27_five_cube_center_insert_scene.py`，SHA-256 为 `74eb11999b799681ec93b124d58b854c32018f1e23219a5ff07f7918fd201801`。原 Task27 场景、机器人、L 型工具、Surface Gripper、Cube、物理设置和 QP 硬限制没有改动；原 G2-only 与无前馈 G3 入口保留。

本次没有重写控制律。runner 允许已有 `--assume-internal-contact-load` 在 `--mode integrated --live-qp --stage-a-only` 中显式启用；默认仍关闭，并继续拒绝无 Stage-A-only 范围的普通执行。已有 bridge 只在 `DUAL_ATTACHED` 且非零内部力目标有效时，把假定的 `Jᵀ f_target` 加入 QP 动力学偏置；反馈标量仍使用运行时实测抓取轴上的每侧 N，原 action/reaction 符号和归一化保持不变。

```text
M qdd + h_arm = tau_actuator - Jᵀ f_on_Cube
QP bias = h_arm + payload_gravity + Jᵀ f_internal_target
```

内部目标和载荷项不是测得的接触力。原吸盘事件控制任务模式；释放后进入 `RELEASE_RETREAT`，内部目标力矩与 payload gravity 均返回无效/零，不修改原 executor 的运动或吸盘流程。新增回归覆盖实际模式切换与两项禁用；runner 还会将 Stage-A-only executor 非零退出/超时明确记为失败，避免只凭 QP 状态宣告完成。

## 匹配姿态 5 mm 对照

运行 `task28_g3_matched5mm_v1` 在同一次真实仿真抓取、共同抬升后，依次运行 OFF 和 ON。每组复用已有 34 s 探针：2 s 内升至 1 N，原 ±0.1/±0.2 N 悬停注入力序列，随后以五次多项式插值完成一次世界 +X 5 mm 往返。两组之间将内部目标归零，使用原 torque QP 在同一关节参考保持 6 s；没有写机器人状态或 Cube 位姿进行复位。

两组期望 q_start、IK 求解种子和 Cube 参考完全相同；实际初始 q 最大差 `9.5367e-7 rad`，Cube 位置差 `0.0002779 mm`。初始 dq 也完整保存，最大差约 `1.85e-5 rad/s`。这消除了先前两次 MoveIt 选择不同抓取姿态的主要混杂，但仍是单次、固定 OFF→ON 顺序的对照，不是随机化或重复性研究。

| 指标 | OFF | ON |
|---|---:|---:|
| Cube 理想轨迹误差峰值 | 1.38087 mm | 0.06887 mm |
| Cube 理想轨迹误差 RMS | 1.35688 mm | 0.04435 mm |
| 回程末端误差 | 1.37839 mm | 0.03649 mm |
| 相对 TCP 平移参考误差峰值 | 0.01538 mm | 0.06435 mm |
| 外向运动目标力矩残差 L∞ 均值 | 0.25814 N·m | 0.007815 N·m |
| 外向运动内部力模型候选均值 | 0.16264 N | 1.00370 N |
| 零注入力窗口 joint8 世界 Fy，左/右 | +0.16489 / −0.16489 N | +1.00407 / −1.00407 N |
| 外向运动已应用左 J2 / 右 J2 均值 | −14.50591 / −19.57766 N·m | −14.31158 / −19.41266 N·m |
| QP 最优帧 | 2040/2040 | 2040/2040 |
| 力矩饱和 / 变化率激活帧 | 0 / 0 | 0 / 0 |

完整 14 关节 effort 均值/极值、扰动窗口 joint8 六维反力和各阶段残差保存在 metrics。ON 的两项误差小于现有 0.250/0.150 mm 比较门限；OFF 的 Cube 误差不通过。理想目标按完成状态的 physics time 评价，反力/已应用 effort 对应已完成区间；同帧 q_ref 属于下一控制区间，FK 对照不冒充过去指令。提取器按物理时间截取每个探针，避免混入后续复位。

相较先前非匹配的 OFF/ON 峰值 `1.4981/0.2269 mm`，本次 `1.3809/0.0689 mm` 进一步支持前馈改善所测试工况的内部目标力矩实现和 Cube 跟踪。joint8 incoming transmission 对目标和已知扰动有响应，但仍混合子树动力学/约束贡献。

[匹配对照 metrics](../../offline/task28_task27_integration/evidence/task28_g3_matched5mm_v1/matched_metrics.json)、[对比图](../../offline/task28_task27_integration/evidence/task28_g3_matched5mm_v1/matched_comparison.png)、[summary](../../offline/task28_task27_integration/evidence/task28_g3_matched5mm_v1/summary.json)、[GUI 窗口/进程](../../offline/task28_task27_integration/evidence/task28_g3_matched5mm_v1/process_proof.txt)、[抬升后实际 viewport](../../offline/task28_task27_integration/evidence/task28_g3_matched5mm_v1/final.png)。

## 完整 GUI 周期结果

运行 `task28_g3_full_cycle_ff_v1` 使用原 executor，`--stage-a-only --assume-internal-contact-load --internal-force-target-n 1`。`executor_exit=0`，summary 为 `COMPLETED`，日志明确记录：

```text
STAGE_A_ONLY PASS: attach -> lift -> transport -> place -> release -> retreat complete;
optional REGRASP/PUSH skipped.
```

| 实际阶段 | Cube 世界 XYZ，m | 吸盘 |
|---|---|---|
| 初始静止 | (0.5000, 0.0000, 0.2600) | OPEN |
| COMMON_LIFT | (0.500, 0.000, 0.540) | 双 CLOSED |
| COMMON_X_TRAVEL | (0.770, 0.000, 0.540) | 双 CLOSED |
| COMMON_Y_ALIGN | (0.770, 0.060, 0.541) | 双 CLOSED |
| COMMON_DESCENT_TO_ENTRY | (0.770, 0.060, 0.280) | 双 CLOSED |
| 原 Stage A 放置前短程对齐 | (0.791, 0.060, 0.280) | 双 CLOSED |
| COMMON_DROP_TO_TABLE 命令结束 | (0.773, 0.059, 0.273) | 双 CLOSED |
| 首个双 OPEN 物理帧 | (0.7730613, 0.0594550, 0.2701288) | 双 OPEN |
| 撤退完成后落稳 | **(0.7740695, 0.0594035, 0.2600000)** | 双 OPEN |

中间阶段坐标来自 executor 的三位小数 GT 回执，完整逐步轨迹另存 CSV/JSONL。原 Stage A 包含的 `COMMON_SIDE_SHORT_PUSH` 是已有双吸附放置前对齐段，本次未新增推入动作；释放后只执行已有共同撤退，未执行 Stage B 的换抓/推入或下一件 Cube。

最终姿态四元数 **wxyz** 为 `[1.0, 8.0060e-7, -1.9956e-7, -2.9225e-5]`；原 raw trace 的字段名 `orientation_xyzw` 是历史命名，实际 API 数值为 Isaac 的 wxyz，本次派生结果明确标为 `orientation_wxyz`。整个周期 Cube z 范围 `0.2599997–0.5405359 m`，最大 x 为 `0.7910671 m`，不是通过写 Cube 位姿制造位移。

| 完整运行指标 | 实测 |
|---|---:|
| QP | 9051 `QP_OPTIMAL`，另 1 帧重力接管 |
| 双臂附着 / 前馈有效帧 | 2315 / 2315 |
| FREE_APPROACH / RELEASE_RETREAT 帧 | 4598 / 2138 |
| 非双臂附着模式中前馈有效帧 | **0** |
| 力矩饱和 / 变化率激活帧 | 0 / 0 |
| 关节反力有效帧 | 9052 / 9052 |
| 已应用 14 关节最大绝对力矩（附着段） | 31.686 N·m |
| 内部力目标 | 每侧轴向 1.000 N |
| 模型内部力候选，均值 / P95 / 最大 | 1.0367 / 1.1301 / 2.6149 N |
| joint8 世界 Fy 均值，左 / 右 | +1.0305 / −1.0295 N |
| 内部目标力矩残差 L∞，均值 / 最大 | 0.09730 / 0.73025 N·m |
| 实际相对 TCP 平移变化峰值（相对首个附着帧） | 0.02641 mm |
| 实际相对 TCP 旋转变化峰值 | 0.0003958 rad |
| Cube 在左右 TCP 坐标中位置变化峰值 | 0.3481 / 0.3498 mm |
| Cube 在左右 TCP 坐标中位置变化 P95 | 0.00605 / 0.00716 mm |
| **对参考的共同 / 相对 TCP 平移误差峰值** | **45.0966 / 84.4640 mm** |

以上“实际相对几何变化”和“对期望参考的误差”是不同指标，不能相互替代。最大相对参考误差位于 physics step `5432`、`t=90.533333 s`；由记录的 q_ref 经冻结 FK 和 SE(3) 误差公式独立重算得到同样的 `84.464029 mm`。该帧左右期望 TCP 的世界 x 分别为 `0.625554/0.541178 m`，实际为 `0.575142/0.556644 m`。这证明该值是实际与双臂期望参考之间的偏差，不是 84 mm 抓取滑移；本次没有进一步隔离其轨迹/时钟原因，也没有修改原 G1/G2 逻辑消除它。完整周期**不能按 0.250/0.150 mm 的高精度跟踪标准记为通过**。

落桌命令末端左/右关节残差为 `1.955°/2.217°`，原执行器使用既有 `3.438°` 静止有界残差条件继续，没有放宽门限。此时 Cube 高度约 0.273 m；首次双 OPEN 帧仍比最终落稳高约 10.13 mm，随后回落至桌面。最终落稳 XY 相对名义入口 `(0.770,0.060)` 偏约 4.11 mm。因此这次完成的是原 Stage A 的抓取、运输、释放落稳与撤退物理集成，**不是无落差精密放置**。上述残差均作为遗留问题保留，不通过继续扩展实验或调高容差掩盖。

### GUI 与原始证据

- [完整 summary](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/summary.json)、[metrics](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/cycle_metrics.json)、[逐时刻 CSV](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/cycle_timeseries.csv)、[完整周期图](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/cycle_overview.png)。
- [最终真实 viewport](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/final.png)、[初始 viewport](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/scene_loaded.png)、[Isaac GUI 窗口/进程](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/process_proof.txt)。两次运行均 `headless=false`。
- [原始压缩物理 trace](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/task27_torque_bridge.jsonl.xz)、[executor 日志](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/task27_executor.log)、[参考误差峰值状态](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/reference_error_peak.json)。

本轮两个仿真尝试均首次完成，没有碰撞停机、QP 失败或吸附中断导致的重试。匹配对照的 Kit 已记录 `Simulation App Shutting Down` 后，外层进程退出码为 143；其 executor=0、完整双探针 trace、最终截图与 `COMPLETED` summary 均已先写入。该关闭阶段非零退出被保存在 [process_exit.json](../../offline/task28_task27_integration/evidence/task28_g3_matched5mm_v1/process_exit.json)，没有改写成进程正常退出。完整周期外层进程与 executor 都返回 0。上述问题和落桌/参考残差均保留原始回执。

## 复现

先设置仿真 ROS 环境；ROS 只连接原 Isaac/MoveIt 仿真链路，无真实机器人命令。

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-}
# 每次使用新 run-id 和独立 ROS_DOMAIN_ID；必须使用 GUI。
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=86 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 --matched-suspended-comparison \
  --record-joint-reactions --internal-force-weight 1000 --internal-force-kp 0.5 \
  --local-sweep-limit-m 2.0 --execution-time-scale 16 --timeout-s 900 --run-id <matched_id>

DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=87 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 --stage-a-only \
  --assume-internal-contact-load --internal-force-target-n 1 --record-joint-reactions \
  --internal-force-weight 1000 --internal-force-kp 0.5 --local-sweep-limit-m 2.0 \
  --execution-time-scale 16 --timeout-s 1200 --run-id <cycle_id>
# 删除 --assume-internal-contact-load 保留无前馈 G3；再把 target 设为 0 即原 G2 路径。

PYTHONPATH=offline/task28_g1_1:. python3 -m offline.task28_g3_stage_a.analyze_full_cycle \
  offline/task28_task27_integration/evidence/<matched_id> --matched
PYTHONPATH=offline/task28_g1_1:. python3 -m offline.task28_g3_stage_a.analyze_full_cycle \
  offline/task28_task27_integration/evidence/<cycle_id>
```

## 范围、回归与遗留限制

分类保持 **`G3_JOINT_REACTION_CANDIDATE`**。本次没有新增独立 D6/吸盘接触 wrench 真值，1 N 是每侧轴向目标，不是已认证的实际接触力。原 HOME→Cube 路径仍为 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`；沿用原规划/FCL 与实际抓取检查，不把离散检查等同于连续证明。

相关回归 **167 passed**：G3 Stage A、P2-1/P2-2/P2-4、R8、Task27 live QP 和 T0；未重跑无关历史大规模扰动或五 Cube 规划。新增测试覆盖释放禁用、物理复位偏差拒绝，以及“释放后前馈仍激活”不能得到周期完成判定。没有新增依赖、场景资产、硬件接口或 Sim2Real 工作。[回归原始输出](../../offline/task28_task27_integration/evidence/task28_g3_full_cycle_ff_v1/regression.log)；runner 和四个分析/探针脚本通过 `py_compile`，两份压缩 trace 通过 `xz -t`。复测命令：

```bash
PYTHONPATH=offline/task28_g1_1:. python3 -m pytest -q \
  offline/task28_g3_stage_a/tests offline/task28_p2_1/tests \
  offline/task28_p2_2/tests offline/task28_p2_4/tests \
  offline/task28_p5_qp_6_r8/tests offline/task28_task27_live_qp/tests offline/task28_t0/tests
```

本次按任务要求停在 Stage A。最终 HEAD 与远端同步状态随交付消息记录。

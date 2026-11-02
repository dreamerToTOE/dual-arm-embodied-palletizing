# Task28 Stage A / G3：悬停扰动与短程搬运

基线：`bf4189ddb0adc393db5181f09be32e6ed00f9530`，分支 `side-suction-palletizing`。

本次在原 Task27 GUI Isaac Sim 4.5 场景执行了三次独立重置的真实仿真抓取、共同抬升、悬停外力扰动和往返运动。5 mm 对照中，实验性接触载荷前馈开启后 Cube 峰值误差为 0.227 mm，关闭时为 1.498 mm；随后 10 mm 开启试验的峰值误差为 0.237 mm。没有改动场景、机器人、L 型工具、Cube、吸盘模型或 QP 硬限制。

## 实验方法与实际执行

复用 `task28_task27_one_pass.py --mode integrated --live-qp`，原 Task27 executor 经 `stage_a_g3_hold:=true` 在 Cube_01 共同抬升到约 z=0.540 m 后退出；随后新增的 `--suspended-short-probe` 在同一 GUI/原场景中执行 34 s 探针。没有通过写 Cube 位姿或替代连接制造运动。

1. 前 2 s 将内部力目标从 0 平滑升至 1 N。控制仍采用已有模型候选反馈；`--assume-internal-contact-load` 仅决定是否把假定目标接触载荷加入 QP bias，默认关闭。
2. 从第 4 s 起，对原 Cube 刚体施加世界 Y 外力 `0, +0.1, +0.2, 0, -0.1, -0.2, 0 N`，每段 2 s。使用 `IPhysxSimulation.apply_force_at_pos(..., 'Force')`，作用点是逐步读取的 Cube 刚体原点，日志明确保存作用点，不假定其必为质心。每段后 1 s 为统计窗口。
3. 第 20 s 读取实际悬停 q 和 TCP，分别以实际 q 为唯一种子求世界 +X 的 5/10 mm IK。每臂最大关节变化须小于 0.10 rad，FK 与实际 TCP 平移一致性须小于 0.2 mm。用原有双臂轨迹适配入口，将 q 起终点作同步五次插值，向外运动 5 s、保持 2 s、回程 5 s、保持 2 s。

这不是严格的任务空间直线插补：关节插值的 FK 参考和理想 Cube 直线略有差异，因此两者分别评价。探针开始前检查两臂 adapter 非空且初始附着保持已结束；日志进一步确认每次探针 2040 个配对帧均有两个活动 TCP 目标。

每个物理步检查双吸盘闭合、Cube z≥0.40 m、距探针起点不超过 30 mm，以及 Cube 在每个 TCP 坐标中的位置变化不超过 5 mm；原 QP 的速度、加速度、关节位置、力矩和变化率限制继续生效。这些是本次有界仿真的停止检查，不是 D6 力测量或全场景碰撞证明。

## 运动结果

三个运行的 summary 均为 `COMPLETED`，没有 latched error；各有 2040 个配对探针帧，均为 `QP_OPTIMAL`，力矩饱和和变化率激活帧数均为 0，两侧 `is_closed()` 在所有探针样本中为 true。Cube 实际离开支撑面并产生位移；闭合标志本身不被当作附着力真值。

| 运行 | 5 mm 前馈关闭 | 5 mm 前馈开启 | 10 mm 前馈开启 |
|---|---:|---:|---:|
| 理想 Cube 轨迹误差峰值 | 1.4981 mm | 0.2269 mm | 0.2366 mm |
| 理想 Cube 轨迹误差 RMS | 1.3588 mm | 0.2030 mm | 0.2072 mm |
| 回程末端误差 | 1.4056 mm | 0.2022 mm | 0.1990 mm |
| 实际最大位移（距运动起点） | 6.3661 mm | 4.9839 mm | 10.0285 mm |
| 相对 TCP 平移参考误差峰值 | 0.0551 mm | 0.0433 mm | 0.0691 mm |
| 相对 TCP 旋转参考误差峰值 | 0.000801 rad | 0.000807 rad | 0.000500 rad |
| 相对 TCP 平移相对探针起点变化峰值 | 0.00253 mm | 0.00161 mm | 0.00150 mm |
| 生效 FK 参考与理想 Cube 直线偏离峰值 | 0.00233 mm | 0.00426 mm | 0.01662 mm |
| 外向运动模型内部力候选均值 | 0.1575 N | 1.0153 N | 1.0158 N |
| 外向运动目标力矩残差 L∞ 均值 | 0.2613 N·m | 0.00788 N·m | 0.00925 N·m |

两次开启运行的 Cube 峰值和相对 TCP 平移指标低于既有 0.250/0.150 mm 比较门限；5 mm 关闭运行的 Cube 跟踪不达该比较门限，未将 `COMPLETED` 或 QP 最优解释为跟踪通过。每组仅一次，且 MoveIt 选择的初始关节姿态不同；上述比较支持在这些运行中前馈改善目标力矩实现和跟踪，但不能单独排除姿态差异，也不是重复性验证。10 mm Cube 峰值距 0.250 mm 仅约 0.0134 mm，不继续扩大运动。

误差计算使用已完成状态的 `TORQUE_FRAME.time_s` 评价理想轨迹。该帧记录的 q_ref 是下一控制区间参考，不能冒充上一段已施加指令；FK 参考仅作为独立的几何对照。各运行有 840 个运动状态样本，其中 839 个仍来自探针参考；结束后截图更新时已有 1 帧恢复原 adapter，该帧保留在实际 Cube 末端误差中，但不计入探针 q_ref 的 FK 对照。

回程末端的世界 XYZ 误差分别为关闭 `[+1.3778, -0.0776, +0.2670] mm`、5 mm 开启 `[-0.0513, +0.0424, +0.1910] mm`、10 mm 开启 `[-0.0404, +0.0309, +0.1924] mm`。两次开启运行的残差主要沿 Z，不能由理想直线与 FK 参考的微小差异解释；本次尚未隔离出重力补偿、约束柔顺性或模型偏差各自的贡献，不直接认定根因。

## 悬停已知扰动与力观测

施力命令 k 与已完成物理步 k+1 的反力/状态配对；注入力没有供给模型观测器。外力是真实仿真注入信号，不能当作任一吸盘的独立接触力。以 5 mm 开启运行为例，每段后 1 s 的世界 Fy：

| Cube 注入力 | 左 joint8 Fy | 右 joint8 Fy | 两臂 joint8 Fy 之和 | 内部力模型候选 |
|---:|---:|---:|---:|---:|
| 初始 0 N | +1.01506 N | -1.01506 N | +0.00000 N | 1.01505 N |
| +0.1 N | +0.96883 N | -1.06991 N | -0.10108 N | 1.01942 N |
| +0.2 N | +0.92295 N | -1.12406 N | -0.20111 N | 1.02369 N |
| -0.1 N | +1.06132 N | -0.96018 N | +0.10113 N | 1.01076 N |
| -0.2 N | +1.10698 N | -0.90588 N | +0.20110 N | 1.00657 N |
| 最后 0 N | +1.01422 N | -1.01637 N | -0.00215 N | 1.01533 N |

两侧传递力对正负扰动有可逆响应，其和在世界 Y 上与注入力反号且量级相符；这支持悬停状态下该关节传递力通道是有信息的。内部力候选是差动力分量，不能拿它直接对标 Cube 上的合外力。关节传递力仍包含子树动力学和约束贡献；本次没有独立 D6 反力读取，因此维持 **`G3_JOINT_REACTION_CANDIDATE`**，不宣称真实吸盘内部接触力已验证。

## 安全证据边界与遗留问题

每次实际运动起点记录的 q_start/q_end、同物理步 Cube 位姿，经冻结 Task28 单 Cube/桌面 FCL 模型作 11 点同步插值复查。它不是当前 Task27 完整场景，也不覆盖采样点之间；报告实际检查结果，不将离散无碰撞升级为连续安全。原 HOME→Cube 路径保持 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`。

| 离线离散复查 | 5 mm 关闭 | 5 mm 开启 | 10 mm 开启 |
|---|---:|---:|---:|
| 关节限位 / 冻结模型碰撞检查 | 11/11 通过 | 11/11 通过 | 11/11 通过 |
| 单侧 TCP 参考与理想平移最大差异 | 8.005 µm | 7.857 µm | 31.219 µm |
| 参考双臂相对 TCP 平移变化峰值 | 15.323 µm | 13.913 µm | 54.339 µm |

这些是运行后对已记录参考的复查，不能追认成运行前的完整场景准入证据，也不是实际运动的测量误差。

本次新增探针没有提供完整实时碰撞接触分类。因此只能报告局部包络未触发、吸附几何保持有界和离散模型结果，不能声称整个搬运过程已经连续无碰撞认证。未运行 Stage B、REGRASP/PUSH、五 Cube continuation、放置释放、真实机器人或 Sim2Real。前馈保持实验选项默认关闭。

下一步优先在 5–10 mm 范围重复已通过的开启工况，检查小裕量和姿态差异；必要时用实际 Cube 位姿反馈降低静态残差。扩大路径、放置和释放应在本次短程结果稳定后逐项增加，接触力验证仍需独立约束反力或物理归因充分的参考。

## 证据与复现

所有目录位于 `offline/task28_task27_integration/evidence/`：

- [5 mm 关闭 summary](../../offline/task28_task27_integration/evidence/task28_suspended_probe_off_v1/summary.json)、[metrics](../../offline/task28_task27_integration/evidence/task28_suspended_probe_off_v1/suspended_probe_metrics.json)、[离散复查](../../offline/task28_task27_integration/evidence/task28_suspended_probe_off_v1/discrete_run_precheck.json)。
- [5 mm 开启 summary](../../offline/task28_task27_integration/evidence/task28_suspended_probe_on_v1/summary.json)、[metrics](../../offline/task28_task27_integration/evidence/task28_suspended_probe_on_v1/suspended_probe_metrics.json)、[离散复查](../../offline/task28_task27_integration/evidence/task28_suspended_probe_on_v1/discrete_run_precheck.json)、[对比图](../../offline/task28_task27_integration/evidence/task28_suspended_probe_on_v1/off_on_comparison.png)、[运行窗口与进程](../../offline/task28_task27_integration/evidence/task28_suspended_probe_on_v1/process_proof.txt)。
- [10 mm 开启 summary](../../offline/task28_task27_integration/evidence/task28_suspended_probe_on_10mm_v1/summary.json)、[metrics](../../offline/task28_task27_integration/evidence/task28_suspended_probe_on_10mm_v1/suspended_probe_metrics.json)、[离散复查](../../offline/task28_task27_integration/evidence/task28_suspended_probe_on_10mm_v1/discrete_run_precheck.json)、[最终 GUI 截图](../../offline/task28_task27_integration/evidence/task28_suspended_probe_on_10mm_v1/final.png)。该次没有额外采集外部窗口进程快照，GUI 证据为 `headless=false` summary、启动日志和实际 viewport 截图。

各目录保存原始压缩 JSONL、executor/MoveIt 日志和 viewport 截图。新增代码为 `suspended_probe.py`、分析器、离散复查和绘图脚本；原 runner 新增 opt-in 参数，不修改 G1/G2 主算法。

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-}
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=85 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 --suspended-short-probe \
  --record-joint-reactions --assume-internal-contact-load --short-probe-amplitude-m 0.005 \
  --internal-force-weight 1000 --internal-force-kp 0.5 --local-sweep-limit-m 2.0 \
  --execution-time-scale 16 --timeout-s 900 --run-id <fresh_id>
# 关闭前馈时删去 --assume-internal-contact-load；10 mm 时幅度为 0.01。
PYTHONPATH=offline/task28_g1_1:. python3 -m offline.task28_g3_stage_a.analyze_suspended_probe \
  offline/task28_task27_integration/evidence/<fresh_id>
PYTHONPATH=offline/task28_g1_1:. python3 offline/task28_g3_stage_a/discrete_run_precheck.py \
  offline/task28_task27_integration/evidence/<fresh_id>
```

## 回归与交付

相关完整回归 **164 passed**，覆盖 G3 Stage A、P2-1/P2-2/P2-4、R8、Task27 live QP 和 T0；新增测试检查五次往返端点、因果帧配对、截断日志、状态时间与 FK 参考区分，以及恢复 adapter 后的参考排除。runner 与四个新增脚本通过 `py_compile`。未重跑全部历史 G2 档案测试。

```bash
PYTHONPATH=offline/task28_g1_1:. python3 -m pytest -q \
  offline/task28_g3_stage_a/tests offline/task28_p2_1/tests \
  offline/task28_p2_2/tests offline/task28_p2_4/tests \
  offline/task28_p5_qp_6_r8/tests offline/task28_task27_live_qp/tests offline/task28_t0/tests
```

[回归原始输出](../../offline/task28_task27_integration/evidence/task28_suspended_probe_on_10mm_v1/regression.log)。本次提交仅新增探针、分析工具及三次实验归档，并为原 runner 增加 opt-in 入口；原 Task27 场景脚本哈希仍为 `74eb11999b799681ec93b124d58b854c32018f1e23219a5ff07f7918fd201801`。最终提交 HEAD 和远端同步状态随交付消息记录，避免在提交本身内自引用 HEAD。

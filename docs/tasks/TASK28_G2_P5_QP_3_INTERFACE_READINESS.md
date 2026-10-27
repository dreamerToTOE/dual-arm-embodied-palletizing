# Task28 G2-P5-QP-3｜A-stage 离线参考交接与执行器就绪性

仓库：`dreamerToTOE/dual-arm-embodied-palletizing`；分支：`side-suction-palletizing`。
起点和远端已核对为 `b093f2bd28422a9d3b597334602766dc9268e022`。
结论：**`PARTIAL_G2_P5_QP_3_OFFLINE`**。

这个结论表示离线的 NR-0→QP-2→position-target handoff 已经在冻结模型上形成可复核证据，并通过了静态接口和原 G1/B0 门禁。它不表示 ROS、Isaac、伺服周期、真实反馈、执行器跟踪、碰撞安全或物理实验已经验证。

## 1. 审计范围和保护

本次工作树在起点无修改；QP-0/1/2、G1 验证器、冻结 fixture、SRDF/ACM、Task24/26/27 和历史证据均保持原样。QP-3 新文件只在 `offline/task28_p5_qp_3/`，报告位于本文件。

审计读取了：

- [`TASK28_G0A_INTERFACE_AUDIT.md`](TASK28_G0A_INTERFACE_AUDIT.md)，特别是 §4.2、§6；
- 最新 [`TASK28_PAPER_TO_MAIN_RESEARCH_ROADMAP.md`](TASK28_PAPER_TO_MAIN_RESEARCH_ROADMAP.md)；
- QP-0、QP-1、QP-2 报告与 QP-2 的反馈实现、驱动和正式证据；
- `isaac/scripts/task24_side_suction_tight_bridge.py`、`ros_ws/src/fr3_dual_palletize/src/task25_batched_side_suction.cpp`；
- 冻结 fixture 的 `task28_single_cube_bridge.py` 和 bridge contract；
- 保存的 NR-0 seed 28 transport 中点和其 SHA-256。

源文件路径、字节身份和哈希保存在 [`static_interface_contract.json`](../../offline/task28_p5_qp_3/evidence/experiments_v1/static_interface_contract.json)。未启动任何 ROS 节点、Isaac stage、MoveIt 执行器或机器人命令。

## 2. 实际接口链路

当前可审计的离线链路是：

```text
NR-0 保存的 Cube SE(3) 节点
    ↓  (固定 q14、无 NR 重投影、无 waypoint 替换)
QP-2 FeedbackDriver
    ↓  (q_next = q + dt*qdot；G1 current/predicted + B0 sampled edge)
14 维候选 joint position target
    ↓  (分成左右各 7 个位置数组，只写入证据)
历史 position-only JointState sink 的静态合同
```

NR-0 `result.json.gz` 的节点只有物体位姿和 q14，未提供可执行时间参数化。QP-3 因此显式登记 `dt=0.02 s` 的合成数值时间，并把 `t+dt` 作为候选 stamp；这不是从 NR-0 恢复出的真实时钟，也不是 nominal 10 ms 的实测替代。物体几何路径和时间参数化在证据中分别记录。

QP-2 每一步先独立检查 QP 状态、跟踪残差、反馈仿射等式、位置/速度界、当前/预测状态和 B0 sampled edge。只有被 QP-2 接受的 `q_next` 才能形成 position target。出现拒绝或 `INCONCLUSIVE` 时保留前一接受状态和失败候选，停止后续生成；没有 NR re-projection、投影回流形、约束放宽或 unsafe fallback。

### 命令字段和关节顺序

静态审计确认历史 A-stage sink 使用：

- `/left/joint_command` 和 `/right/joint_command`；
- `sensor_msgs/JointState.position`，velocity/effort 字段不进入当前 Isaac bridge 的动作写入；
- 每侧顺序 `left_fr3_joint1..7`、`right_fr3_joint1..7`，QP-3 按相同顺序从 14 维向量拆分；
- `world` 用于 Cube/TCP 几何，command pair 使用 fixture bridge 已定义的 `task28_g1_0_dual_sync` frame ID；
- 左右 target 使用完全相同的合成 stamp 和 command sequence，作为原子双臂 pair 的离线表示。

fixture bridge 的 `_apply_pending_joint_commands()` 会把同一 stamp 的左右 dual commands 配成一对，并在同一个 physics callback 中分别调用 `apply_action`；不同 stamp 不会拼接。QP-3 没有调用该函数，也没有发布消息，所以这里只是静态字段兼容性，不能称运行时兼容已验证。

### 时间、反馈和失败合同

历史实现有 `kDualCommandPeriod=10ms`、`sleep_for(10ms)` 和最终 hold 常量。这些是代码中的 nominal 调度意图。G0-A 已明确实际 ROS/physics 频率、RTF、丢包、抖动、暂停恢复和驱动应用延迟为 UNKNOWN；QP-3 不把 10 ms 当成 verified servo period。

历史 joint-state callback 按名称缓存位置，但没有完整的 source timestamp、左右采样差、反馈年龄和同步快照门。QP-3 仅使用保存 q14/合成 QP 状态，并明确写入 `physical_feedback_measured=false`、`real_time_verified=false`。真实 handoff 仍需要 source stamp、receive monotonic stamp、command/applied sequence、左右 skew、age、joint completeness 和 stale-state fail-closed 门禁。

## 3. 隔离 handoff 原型

实现文件：

- [`handoff.py`](../../offline/task28_p5_qp_3/handoff.py)：无 ROS import、无 publisher 的 typed contract；检查 Cube ID、frame、关节顺序、同步 stamp、单调 sequence、反馈年龄和 `q_next=q+dt*qdot` 重构关系；
- [`run_handoff.py`](../../offline/task28_p5_qp_3/run_handoff.py)：从已保存 NR-0 节点读取输入，运行 QP-2 feedback，生成内存/JSON position targets；
- [`audit_handoff.py`](../../offline/task28_p5_qp_3/audit_handoff.py)：只读核对证据、源哈希、正控和负控；
- [`tests/test_handoff.py`](../../offline/task28_p5_qp_3/tests/test_handoff.py)：确定性正控和错误 Cube、错误关节顺序、重复/异步 stamp、过期 feedback、错误 Euler 重构等负控。

`PositionTarget` 只携带 candidate data：`sequence`、synthetic `time_s/stamp_s`、fixture frame、side、七个 joint names、七个 position、source QP step、source qdot 和 provenance。它没有 ROS message publisher，也没有执行授权字段（固定为 `false`）。

## 4. 正负证据

正式 handoff 证据在 [`experiments_v1/`](../../offline/task28_p5_qp_3/evidence/experiments_v1/)：

- 正控：保存 NR-0 `seed28:transport:midpoint40`，combined object twist `[.005,-.003,.002,.01,-.005,.02]`，`dt=.02 s`，5 个 accepted QP-2 steps；产生 5 个同步左右 target pairs（10 个 position records）。每个 predicted state 有效、每条 sampled edge 为 `sampled_edge_valid`，且每个 contract check 通过。
- 负控：归档的 NR-0 `NR0_saved_20mm_payload_table_penetration`，在 initial G1/FCL gate 处停止，未生成 position target，保留 `INITIAL_STATE_GEOMETRY_REJECTED`。这是单状态几何 gate 负控，不被解释为完整 planner 失败。
- 每个正控 target 记录 position discontinuity、qdot change（首帧为 `null`）、closure drift 和 source step。正控最大相邻 position change 为 `7.337178e-4 rad`；五帧 closure translation residual 记录在 `handoff_records.json`。
- 证据摘要显示 `no_commands_published=true`、`physical_feedback_measured=false`、`real_time_verified=false`，连续安全保持 `NOT_CERTIFIED_FINITE_SAMPLING_ONLY`。

机器可读文件：

- [`handoff_records.json`](../../offline/task28_p5_qp_3/evidence/experiments_v1/handoff_records.json)；
- [`summary.json`](../../offline/task28_p5_qp_3/evidence/experiments_v1/summary.json)；
- [`static_interface_contract.json`](../../offline/task28_p5_qp_3/evidence/experiments_v1/static_interface_contract.json)；
- [`evidence_audit.json`](../../offline/task28_p5_qp_3/evidence/experiments_v1/evidence_audit.json)。

## 5. 就绪性判定

| 项目 | 判定 | 依据 |
|---|---|---|
| 离线 reference handoff demonstrated | **是** | 5 步 QP-2 accepted `q_next` 被逐步拆成同步左右 position targets |
| 离线几何 acceptance demonstrated | **是（有限采样）** | 原 G1 current/predicted 与 B0 sampled edge 全通过；payload penetration 负控首状态拒绝 |
| velocity-to-position adapter feasibility | **离线可行** | 明确的 `q_next=q+dt*qdot`、14→7+7 拆分和位置界检查 |
| 现有 position-only bridge 静态兼容 | **部分确认** | topic、position 字段、joint order、dual stamp/frame 与代码/fixture contract 对齐 |
| 实际 ROS/Isaac bridge 运行兼容 | **未验证** | 未运行节点、未发送消息、未启动 stage |
| 实时周期、反馈年龄、执行器跟踪 | **未验证** | 代码有 nominal 10 ms，但实测 servo/physics timing 和 applied feedback 缺失 |
| 在线碰撞避免 | **未具备** | QP-2 仍无 collision-gradient constraint，FCL 是后验门禁 |

因此本任务为 **PARTIAL**。离线原型足以证明接口形状和 fail-closed handoff 逻辑，但缺少实际 bridge/runtime 证据，不能给出 PASS 级系统集成结论。该缺口来自未授权且未执行的 ROS/Isaac/物理验证，不通过伪造时间或反馈数据填补。

## 6. 测试与复现

聚焦测试命令：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=offline:offline/task28_g1_1:offline/task28_p5_qp_1:offline/task28_p5_qp_2 \
python3 -m pytest offline/task28_p5_qp_3/tests -q -s -p no:cacheprovider
```

结果：**5 passed**。运行 handoff（只能写入新的离线目录）和证据审计：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=offline:offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0:offline/task28_g1_2b_1:offline/task28_p5_qp_0:offline/task28_p5_qp_1:offline/task28_p5_qp_2 \
python3 -m task28_p5_qp_3.run_handoff \
  --output offline/task28_p5_qp_3/evidence/experiments_v1

PYTHONDONTWRITEBYTECODE=1 python3 offline/task28_p5_qp_3/audit_handoff.py \
  --evidence offline/task28_p5_qp_3/evidence/experiments_v1 \
  --output offline/task28_p5_qp_3/evidence/experiments_v1/evidence_audit.json
```

审计 8 项均通过：执行禁用、正控 target 数量、正控 G1/B0、负控初态停止、无发布、时间边界、源哈希和固定关节顺序。完整回归运行了原有测试和本任务 5 项测试，共 **572 passed**，退出码 0；其中原有 567 项全部保持通过。

## 7. 剩余前置条件

在任何真实 command integration 前仍需单独授权并完成：

1. ROS message schema 和实际 fixture bridge 的静态/运行时双端检查，包括 `frame_id`、joint names、source stamp、command sequence 和左右同步超时；
2. 不启动机器人情况下的 bridge dry-run 或仿真接收端验证，确认目标不会被旧 Task24/26/27 sink 接收；
3. 仿真中记录 applied joint state、源/接收时间、左右 skew、反馈 age、physics dt/RTF、目标跟踪误差和 fail-stop 行为；
4. 真实/仿真反馈进入 QP 前的 stale、missing、out-of-order 和 attachment/contact 证据门禁；
5. 连续碰撞/闭链安全和碰撞梯度能力，不能用本次有限采样或 position sink 静态兼容替代；
6. position adapter 的物理 servo 带宽、离散加速度/jerk、暂停恢复和最终 hold 合同。

到 G2-P5-QP-3 停止。未自动开始 ROS/Isaac、物理实验、碰撞梯度 QP、G3、NR-1、SVM、PCS/RSS 或 RL。

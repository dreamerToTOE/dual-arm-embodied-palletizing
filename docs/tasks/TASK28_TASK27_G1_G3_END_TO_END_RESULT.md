# TASK28 / Task27 G1→G2→G3 一次集成结果

## 结论

本次 Issue #1 的连续执行结果为 **PARTIAL / BLOCKED**，不是 FULL_PASS。

在原始 Task27 场景中完成了新的场景身份绑定、五-Cube 全量 `planning_only` 规划门禁，以及独占双臂力矩接管。MoveIt 和 Task27 执行器能够把关节轨迹送到新力矩桥，力矩桥在真实 Isaac 物理步中连续写入 2705 帧且 `position_arm_write_count=0`。正式执行在加载轨迹阶段因严格的 applied-effort readback 不一致而 fail-closed；没有进入吸附、抬升、搬运或释放。因此五-Cube 任务实际完成数为 `0/5`，G3 接触力和内部力调节没有建立。

这次结果保留了失败证据，没有放宽力矩、速率、readback、碰撞或接触门禁，也没有修改原始 Task27 场景、FR3 资产、L 型工具、Surface Gripper、D6、Cube 或碰撞几何。

## 执行基线与真实来源

- 仓库：`dreamerToTOE/dual-arm-embodied-palletizing`
- 分支：`side-suction-palletizing`
- 起始同步提交：`c076eff73ce9a6872975c494432cdaa80ba3e758`
- 最终提交：见报告末尾
- 原始场景入口：`isaac/scripts/task27_five_cube_center_insert_scene.py`
- Isaac 根：`/World/Task27`
- 双臂：`/World/left_fr3`、`/World/right_fr3`
- ROS 图：`/World/Task27ROSGraph`
- 当前 checkout 的 Task27 scene wrapper SHA256：`74eb11999b799681ec93b124d58b854c32018f1e23219a5ff07f7918fd201801`
- wrapper 实际加载的绝对 Task26 scene：`/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task26_truck_box_scene.py`
- 绝对 Task26 scene SHA256：`43aa7c2eb6a5667da5e94eaaef347229fe6178ec58d6a10d12dddf14ba2aa9e7`
- 实际加载的原始 feed/Surface Gripper bridge：`/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task26_truck_box_bridge.py`
- 绝对 Task26 bridge SHA256：`0f7123657ea8422cf0efd468e7bcd59624e144ae8c9ca315a82af57576b68dc6`

Task27 bridge 的 arm 位置写入被新入口替换为七关节力矩控制；手指目标、供料、导轨、Surface Gripper 和 Ground Truth 仍由原始 bridge 管理。新入口位于 [task28_task27_torque_bridge.py](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/isaac/scripts/task28_task27_torque_bridge.py)，GUI/ROS 一次运行器位于 [task28_task27_one_pass.py](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/isaac/scripts/task28_task27_one_pass.py)。

## 阶段门禁

| 阶段 | 结果 | 证据与边界 |
|---|---|---|
| G1 场景身份、DOF、来源绑定 | PASS | 原始 Task27 场景、绝对 source hash、双臂 9-DOF 顺序均记录；没有替代 fixture。 |
| G1 五-Cube planning-only | PASS | `issue1_planning_only_20261031_0200`；MoveIt exit `0`，Task27 executor exit `0`，日志明确五批 IK/FCL 和共同 HOME 通过；未发布 joint/suction/feed 命令。 |
| G1 物理抓取与搬运 | BLOCKED | 正式力矩执行在 applied-effort readback 门禁处停止；没有完成一件物理 Cube 的抬升、转运和释放。 |
| G1 连续安全证明 | NOT_ESTABLISHED | 原始 HOME→Cube 接近路径仍保持 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`。planning-only 的离散/采样 FCL 结果没有替代连续证明。 |
| G2 独占 arm effort ownership | PASS（接管层） | 正式尝试 2705 个物理帧；arm mode=`force`、arm Kp/Kd=0、`position_arm_write_count=0`，手指策略未接管。 |
| G2 Task27 live cooperative QP | NOT_ESTABLISHED | 新桥接记录 `solve_count=0`、`NOT_IMPLEMENTED_TASK27_LIVE_QP`；本次只验证了力矩 sink 和轨迹目标捕获，不能把它称为 QP 通过。 |
| G2 loaded tracking | BLOCKED | `TorqueReject('APPLIED_READBACK')`；请求力矩与 Isaac applied-command readback 在高动态阶段不一致，接口 fail-closed。 |
| G3 observer source | CANDIDATE_ONLY | 仅记录模型 `g/M/c` 与 solver projected effort；没有真实 FR3 `tau_J`，没有把 command echo 当作传感器。 |
| G3 独立接触 oracle | NOT_RUN | 没有建立独立 PhysX 接触力/力矩 oracle，也没有向估计器注入接触真值。 |
| G3 net/internal wrench | NOT_ESTABLISHED | 没有完成双臂加载 A/B、实际抓取矩阵、净 wrench 和内部力对照。 |
| Task27 五-Cube 任务 | BLOCKED | 供料 bridge 自动使 Cube_01 到达第一个供料槽；实际双侧吸附、抬升、搬运、释放完成数 `0/5`，其余四件未执行。 |

## 关键运行结果

### 1. GUI/T0 复核

非 headless Isaac Sim 4.5 GUI 运行了原始 Task27 场景并完成 90 帧力矩接管演示。证据：

- [issue1_run_20261031_003717_evaluation.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/issue1_run_20261031_003717_evaluation.json)
- [issue1_run_20261031_003717_process_proof.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/issue1_run_20261031_003717_process_proof.json)
- [issue1_run_20261031_003717_scene_loaded.png](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/issue1_run_20261031_003717_scene_loaded.png)
- [issue1_run_20261031_003717_torque_final.png](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/issue1_run_20261031_003717_torque_final.png)

该演示仍只是 takeover/hold/small-motion 证据，不是 Task27 五-Cube 成功。

### 2. planning-only 五-Cube

运行器：

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
/home/ubuntu2004/isaacsim-4.5.0/python.sh \
isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --planning-only --timeout-s 420 \
  --run-id issue1_planning_only_20261031_0200
```

结果：MoveIt `0`、Task27 executor `0`、4033 个 torque hold 帧、0 个 arm position write。执行器日志最后报告 `Task27 planning-only batch 5 PASS`，且明确没有发布 joint、suction 或 feed command。证据目录：[issue1_planning_only_20261031_0200](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_task27_integration/evidence/issue1_planning_only_20261031_0200)。

### 3. 正式 torque-mode 一次执行

运行器：

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
/home/ubuntu2004/isaacsim-4.5.0/python.sh \
isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --timeout-s 420 \
  --run-id issue1_integrated_20261031_0145
```

MoveIt overlay 使用机器上只读的既有 `franka_fr3_moveit_config`，然后叠加当前 checkout 的 `ros_ws/install`。MoveIt 正常启动，Task27 进入真实规划和轨迹发送。结果：

- torque bridge 物理帧：`2705`
- torque writes：`2705`
- arm position writes：`0`
- 左/右轨迹目标捕获：`755 / 381`
- 失败：`TorqueReject('APPLIED_READBACK')`
- executor 在 fail-closed 后被停止，未进入 suction/transport

证据目录：[issue1_integrated_20261031_0145](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_task27_integration/evidence/issue1_integrated_20261031_0145)。前 3 次启动失败也保留在 `issue1_integrated_20261031_0100`、`0115`、`0130`：分别记录了 ROS ABI 污染和缺少 `franka_fr3_moveit_config`，之后已用干净 ROS 环境与只读 overlay 修复，不能把这些环境失败误记为控制器失败。

## G3 预检与回归

严格 observer adapter、R8 effort ownership、Task27 profile 和 G3-R4 预检单元测试均通过：

- G1 validator + G1-2A tests：`296 passed`
- G3-P2-4-R4 tests：`15 passed`
- R8 effort tests：`15 passed`
- T0 tests：`7 passed`
- 本次新增/相关回归合计：`333 passed`
- ROS 工作区：`colcon build --symlink-install`，6 个包完成

这些测试没有把 G3 的模型候选提升为真实接触观测，也没有改变 `S2 NOT_READY`、`S3 BLOCKED` 或连续安全状态。

## 未完成事项与下一步

1. 先为 Task27 原始 FR3 在力矩模式下解决 applied-effort readback 的物理定义、时刻和误差容差问题；不能通过删除门禁或把 projected effort 当作传感器解决。
2. 把已有 R12/R14 task-space QP 真正接入 Task27 live target，并在同一物理步记录 solver 状态、约束残差、q/dq/J/M/c/g 与最终 effort。
3. 在不加载 Cube 的单臂受控接触中接入独立 PhysX contact oracle，完成 observer 方向、幅值、时延和零偏验证。
4. 只有上述门禁通过后，才继续一件 Cube 的双侧吸附、抬升、搬运、释放，再考虑五-Cube 顺序 `+2,-2,+1,-1,0`。

本次不进入真实机器人、ROS 实机命令、Sim2Real、D6 研究或 G3 内力实验；原始接近路径继续保持未建立连续安全证明。

## Git 交付

- 提交内容：新 Task27 力矩 bridge、一次运行器、GUI/Isaac 原始运行证据、规划-only 与正式执行日志、本报告。
- 工作树要求：提交前执行 `git status --short --branch`，必须为空变更。
- 远端：`origin/side-suction-palletizing`。

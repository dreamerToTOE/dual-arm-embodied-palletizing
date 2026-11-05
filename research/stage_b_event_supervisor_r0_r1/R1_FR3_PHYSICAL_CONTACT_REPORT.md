# R1 FR3 物理接触烟测报告

## 结论

R1 状态：`R1_DEEP_WALL_CONTACT_PASS`。

这是一个边界明确的 Isaac Sim 4.5 单批实验，控制架构为 `FREE_SPACE_TRAJECTORY_POSITION_DRIVE`。实际 FR3 关节执行了自由空间接近、工具/Cube 接触、Cube 推进和深墙接近。实验没有直接给 Cube 施加世界力，也没有 teleport measured Cube。它证明了物理接触与位移链路，不证明混合力控、吸附模型或 jam recovery 已通过。

为避免误读，本实验标记为 `FR3_PHYSICAL_SIDE_TOOL_CONTACT_SMOKE`；suction command、Surface Gripper closed state 和工具/Cube PhysX 接触分别记录，未把高 D6 限值当成经过验证的吸附力。

## 运行

- run id：`r1_event_supervisor_physical_push01`
- scene：Task27 five-cube center-insert scene，实际执行 batch 1
- `moveit_exit=0`，`executor_exit=0`，`status=COMPLETED`
- `runtime_events.jsonl`：`RUN_STARTED → FREE_SPACE_POSITION_DRIVE → TRIAL_COMPLETED`
- execution time scale：4.0；单个 Isaac Sim 实例
- 原始日志、截图和压缩 trace 位于 `evidence/r1_event_supervisor_physical_push01/`

## 物理证据

`physical_trace_summary.json` 报告 15,688 个 post-step frame、65,844 个 PhysX contact report 事件、45,359 个 tool/Cube 事件和 6,452 个 Cube/deep-wall 事件。Cube 的初始到最终欧氏位移为 `0.6463748 m`。第一次工具/Cube 事件在 physics step 2,152（约 35.867 s），第一次深墙/Cube 事件在 step 9,236（约 153.933 s），记录到的接触冲量范数峰值约 `3.5992 N·s`。

trace 同时保存两臂关节位置/速度、TCP world position、Cube pose/velocity、控制 owner 和接触点。接触来源是 `PhysX_simulation_contact_report`，状态来源是 `Task26BatchedFeedBridge articulation and Cube runtime state`。

最终 executor Ground Truth 行为：

- `cell_error=1.716 mm`
- `+X deep wall_gap=+0.367 mm`
- `+Y wall_gap=+1.676 mm`
- 三项均满足原始 3 mm wall-gap acceptance
- 推入峰值力矩约 `29.84 Nm`，低于保留的 80 Nm gate

因此可以分别确认：工具/Cube 物理接触、Cube 实际位移、正常深墙接触、最终几何 acceptance。不能由该实验推出真实 FR3 `tau_J` 等价、吸盘承载力、混合力控或自然 jam recovery 成功。

## 既有失败和对照证据

- `r1_sweep_diagnosis/summary.json` 保留了旧固定锚点 QP 的精确触发：第 607 步 `LOCAL_SWEEP_LIMIT`，上一帧相对 TCP 已为 19.1997 mm，QP 状态为 `QP_OPTIMAL`。这是全局自由空间轨迹撞上局部锚点域，不是求解器不可行。
- `r3_position_precontact03` 证明位置 bridge 在无接触预接近中完成真实 FR3 motion。
- `r4_physical_push02` 保留了先前同场景的 71,854 个接触事件和 0.6468 m Cube 位移，作为重复实验对照。
- 两个早期 R3 repair 失败（缺失原始 feed bridge）及完整旧 torque JSONL 已压缩保存，未删除失败证据。

## 物理边界

本实验没有运行 force-feedback controller、QP contact controller、R2-R4 adaptive selector 或 RL。`FCL PASS` 仅表示规划/碰撞检查通过；R1 结论来自实际 Isaac/PhysX 执行和 contact report。任何 recovery 研究都必须先提供可验证的 controlled hold、attachment state 和重新规划候选。

# R2-B side-compaction handoff 审计

## 实际最终状态

physical05 在深墙座入后记录：Cube 中心约 `[1.09977, 0.06012, 0.26006] m`，座入 gap `+1.497 mm`，z error `.063 mm`；右 TCP `[1.03908,0.06030,0.25964] m`，左 TCP `[0.79007,-0.31984,0.55962] m`，吸盘状态 `left=OPEN, right=CLOSED`。q/dq、TCP、command、Cube 和 suction 都保存在 trace 中。

## 根因

planner 的 SIDE_HIGH_APPROACH 初始检查直接在 `t=0.000 s` 拒绝：`right_fr3_side_suction <-> task26_cube_1`。physical05 报告接触位置 `(0.939,0.043,0.277)`，penetration `0.444 mm`；physical04 为同一 pair、penetration `0.571 mm`。planner 同时列出 right side-suction link origin `(0.784,0.060,0.340)` 和多个 collision primitive，说明初始工具几何与已座入 Cube/重抓后的实际状态没有一致地传递到 handoff preflight。

随后出现的 `both side suctions are OPEN` 是 safe-abort 处理结果，不是根因。它不能被解释成吸盘失败导致碰撞，也不能用关闭 FCL 来绕过。

## 判定

- 这是 planned initial overlap / actual-vs-planned state consistency 问题，不能仅凭 PhysX 接触认定为 intended contact。
- 没有证明 side-compaction 的 phase-specific contact policy 是安全的；handoff 保持 `PREFLIGHT_BLOCKED`。
- 未禁用 FCL、未放宽碰撞、未删除 Cube collider、未改 torque/joint limits，也没有自动尝试修复。
- 需要的最小新 primitive 是一个能从实际 regrasp/座入状态生成并独立验证 side-tool 初始无重叠的 handoff receipt；在该 primitive 存在前不能执行 side-compaction。

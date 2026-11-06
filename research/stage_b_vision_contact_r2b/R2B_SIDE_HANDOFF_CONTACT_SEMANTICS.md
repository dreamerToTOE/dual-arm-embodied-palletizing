# R2-B side handoff 与接触语义审计

## 保留的约束

本分支没有修改 FCL、碰撞阈值、后壁几何或吸盘约束，也没有加入自动 handoff、jam recovery 或控制器重设计。离线碰撞接近仍只表示几何结果，不表示物理卡死。

先前 physical05 的 side-compaction preflight 记录了 `right_fr3_side_suction <-> task26_cube_1` 的 t=0 初始重叠，穿透约 0.444 mm（physical04 为 0.571 mm）。安全路径因而保持 `PREFLIGHT_BLOCKED`，并保留 downstream “both side suctions are OPEN” 的安全处理。

physical07 的日志显示 side-high-approach、side-contact、side-press 的 FCL preflight/execution 段均被请求并选出候选，随后形成 298 个 executor 区间样本；最终 `SIDE_COMPACTION LIFT execution failed`。这不是 jam 成功，也不是接触观测足以支持恢复的证据；该轮没有绕过初始语义或强行放宽碰撞约束。

## 决策

P2 handoff 继续为 `BLOCKED/UNKNOWN`。要重新开启，最小新增 primitive 是带显式 attachment confidence、tool-Cube contact receipt 和 side-suction 开闭状态的可执行 handoff preflight；在主开发会话验证这些 observables 前，不做物理恢复结论。

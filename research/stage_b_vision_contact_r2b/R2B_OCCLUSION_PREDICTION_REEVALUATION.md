# R2-B 丢帧预测复评

physical05 的 55 帧全部检测到 ArUco，因此没有自然的“检测失败 dropout”；真正存在的是最长 `90.650 s` 无采样 blind interval。报告把两者分开，未把 blind interval 称为连续光学遮挡。

## blind interval 上的因果比较

| 策略 | 样本 | 平均误差 | P95 | 说明 |
|---|---:|---:|---:|---|
| P0 last valid visual，authored K | 6 | 85.3 mm | 261.8 mm | 长间隔保持上一视觉状态 |
| P1 visual velocity，authored K | 5 | 94.4 mm | 275.3 mm | 短历史速度外推，长间隔不稳定 |
| P2 TCP propagation | 6 | UNKNOWN | UNKNOWN | 没有满足独立附着置信门控 |

cy=341 比较下 P0/P1 均值分别为 `78.7/90.5 mm`，仍是厘米级。P2 现在要求“双吸盘闭合 + tool-Cube PhysX contact support”才进入评分；单独的 suction Boolean 返回 `UNKNOWN`，不再称为 independently confirmed rigid attachment。当前实验没有足够的自然视觉缺口来证明 P2 可用，也没有任何结果支持 3 mm final-placement acceptance。

预测器只使用历史视觉、机器人反馈、TCP 和显式附着门控；Cube/PhysX 状态仅用于离线评分。未来若 attachment confidence 失效或可能滑移，必须返回 `UNKNOWN` 并继续增大不确定性。

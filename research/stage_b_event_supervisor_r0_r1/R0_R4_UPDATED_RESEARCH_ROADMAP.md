# R0–R4 更新研究路线

## 已完成的 R0/R1 边界

R0 提供了 typed runtime events、free-space 与 local-QP 的所有权分离、时间窗口 stall 诊断、观测新鲜度检查和保留原始几何/碰撞/力矩阈值的运行日志。R1 已完成单批 FR3 工具/Cube/深墙物理接触烟测。R0 状态为 `R0_GATE_REFACTOR_PARTIAL`，R1 状态为 `R1_DEEP_WALL_CONTACT_PASS`。

## R2：接触状态估计与上层操作选择

建立以下状态的估计器：自由空间接近、有效工具接触、正常推进、深墙接触、侧墙接触、提前阻塞、FR3 tracking failure、attachment loss 和 observation uncertainty。估计器必须区分 PhysX contact identity、姿态/速度和未验证的力通道。上层选择器应在真实可行候选中比较操作，并与透明 deterministic rule baseline 对照；仅有状态机不构成两层规划证据。

## R3：下层自适应控制与有界恢复

实现 joint-space、Cartesian、低刚度/阻抗、在力通道验证后才可启用的 hybrid force/position、controlled hold、withdraw 和 recontact。恢复流程固定为：

```text
Stop → Diagnose → Withdraw → Correct → Recontact → Reinsert
```

每次恢复必须有限次、从非穿透初始状态开始，并把 upper-layer candidate、lower-layer feasibility 和结果写入事件日志。

## R4：匹配条件研究评估

比较 M0 fixed trajectory/reference、M1 deterministic event switching、M2 contact-state-aware adaptive switching、M3 M2 加有界 recovery planning。匹配场景和独立随机种子必须一致。指标包括最终完成、loaded contact、wall gap、pre-impact velocity、已验证 contact impulse、切换连续性、tracking error、jam detection accuracy、false positive、recovery success/time 和计算成本。只有实际候选评估与反馈显示改善，才能声称两层规划有优势。

本任务不自动执行 R2-R4、RL 或 G2-G4；它只留下可复现的事件、接触 trace、失败 receipt 和路线入口。

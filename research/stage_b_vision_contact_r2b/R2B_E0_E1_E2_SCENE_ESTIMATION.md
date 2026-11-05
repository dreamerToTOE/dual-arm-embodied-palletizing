# R2-B E0/E1/E2 场景估计

实现位于 `estimator/contact_estimators.py`：

* E0 是几何/速度规则的多标签边估计，同时输出 operation mode；边标签与模式不是同一个枚举。
* E1 在 E0 上增加 3 帧 dwell hysteresis。
* E2 是小型因果、physics-constrained Bayesian-like filter；视觉丢失时只做 prediction-only，不使用 oracle contact。

在既有 R2 trace 上用 oracle pose 做离线审计，E0/E1/E2 均完成 15,688 帧重放。E0 模式为 APPROACH 9,234 帧、DEEP_SEAT 6,454 帧；E1 仅在起始切换产生 2 帧 UNKNOWN；E2 同样得到 APPROACH 9,234、DEEP_SEAT 6,454。最大边概率包括 CUBE_DEEP_WALL 0.998、CUBE_TABLE 0.935；这些数值证明接口可运行，不证明在线接触真值。

在每帧视觉不可用的探针中 E2 全部返回 `prediction_only` 并增加不确定度。由于没有有效的带标注 V2 操作轨迹，E2 相对 E0/E1 的在线优势为 **`E2_ADVANTAGE_UNPROVEN`**。禁止将这个离线重放称为物理恢复成功。

`evidence/r2b_camera_observer_smoke20/estimator_event_journal.jsonl` 是单帧 V2 qualification 事件日志：E0 输出多标签边，E1 输出待确认模式；日志显式标记 oracle contact 已排除。它证明事件接口与隔离策略，不构成 live manipulation episode。

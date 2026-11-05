# R2-B 后墙相机动态可见性

## 新 physical episode

新运行数为 **1**：`r2b_rear_wall_physical05`。选择 `rear_11=[1.70,-0.35,0.80] m`，因为它与 rear_17 具有相同的静态箱体清距 `.380 m`，且本次实际观察验证了该位置；rear_17 的静态 TCP 点清距更大，但没有新的动态运行证据。

相机时间线有 55 个 RGB/深度样本，首尾跨度 `99.867 s`，中位帧间隔 `0.100 s`，对应实际约 `10.0 Hz`。最大未采样间隔为 `90.650 s`；这是采集/执行时间线的 blind interval，不是连续光学遮挡证明。55/55 帧检测到 marker，authored K 下平均位置误差 `43.1 mm`、姿态 `4.38°`；离线 cy=341 比较为 `37.0 mm`、`2.70°`。

## 以 executor 阶段为分母的可见性

- supply、approach、attachment、coordinated transport、placement/release、single-arm insertion、deep-wall arrival 均实际执行，但本次相机时间线在其 executor 区间没有帧，分类为 `EXECUTED_WITHOUT_CAMERA_SAMPLES`。
- release retreat 有 2 帧、检测率 100%；single-arm recontact 有 11 帧、检测率 100%。
- auxiliary side compaction 在 preflight 即被阻塞，没有执行样本，分类 `PREFLIGHT_BLOCKED`。

因此不能把 55/55 解释为全任务检测率，也不能说已验证插入/深墙期间的连续视觉可见性。高频采样设置是有效的，但执行器事件和相机采集的时间段没有覆盖整个物理技能序列。

## 安全 envelope

rear_11 的静态箱体清距 `.380 m`，相机体 x 方向清距 `.370 m`；假设柱半径 `.10 m`、线缆包络 `.05 m` 时，physical05 TCP 点到柱表面的最小距离 `.498 m`。该检查只覆盖一条记录的 TCP 点，不是完整 FR3 链、支架和线缆的 FCL swept certificate；因此安全 gate 仍为保守通过、完整证书未建立。

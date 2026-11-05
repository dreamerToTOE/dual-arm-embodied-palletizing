# R2-B 全任务视觉可观测性报告

## 新增物理 episode

`r2b_rear_wall_physical04` 是本研究新增的 bounded Isaac Sim 物理执行，使用 `rear_02`、单个固定 Eye-to-Hand RGB-D 相机、无腕部相机。每 120 个物理步采集一次，共 51 个 RGB/深度样本；相机时间线与 physical trace 通过 `physics_step/time_s` 对齐。每条 trace 含 `q_actual`、`dq_actual`、TCP、controller command receipt、suction command/state、Cube 状态、阶段字段和接触事件摘要。

关键执行证据：9,821 个 trace 帧；39,509 个接触事件；31,009 个 tool-Cube 接触事件；334 个深墙接触事件；Cube 位移 `.6026 m`。执行在 `SIDE_COMPACTION` preflight 因 `both side suctions are OPEN` 安全中止，executor exit `1`；runner 的 summary 状态 `COMPLETED` 不代表 batch 成功。

## 十阶段可见性

| 阶段 | 结果 |
|---|---|
| cube supply | EXECUTED_OBSERVED，29 帧，26 检测，89.7% |
| dual-arm approach | EXECUTED_OBSERVED，24 帧，24 检测，100% |
| dual-arm attachment | EXECUTED_OBSERVED，6 帧，2 检测，33.3% |
| coordinated transport | EXECUTED_OBSERVED，11 帧，7 检测，63.6% |
| placement/release | NOT_EXECUTED |
| single-arm recontact | NOT_EXECUTED |
| single-arm insertion | NOT_EXECUTED |
| deep-wall arrival | NOT_EXECUTED |
| auxiliary side compaction | NOT_EXECUTED（仅进入 preflight） |
| release/retreat | NOT_EXECUTED |

有效 authored 内参帧的平均 Cube 位置/姿态误差为 `42.1 mm / 9.41°`；离线 `cy=341` 比较为 `23.4 mm / 7.83°`。后者未进入在线控制。

## 失败与控制执行记录

- physical01：当前 worktree 缺少 ROS workspace，`MOVEIT_EXIT_EARLY:1`，无可用相机帧。
- physical02：执行超时，marker 未附着到 Cube；有 RGB-D 文件但不能作为本任务姿态验证。
- physical03：任务执行完成，但深度传感器初始化重置了相机视图，图像与元数据外参不一致；保留为标定失败证据。
- physical04：相机视图修复后获得本报告的有效部分观测，但在侧向 compaction preflight 中止。

因此，本报告证明的是供给、接近、附着和运输阶段的**部分观测可行性**，不是完整放置、恢复或物理 jamming 成功。

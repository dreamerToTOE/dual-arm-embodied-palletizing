# R2-S0 物理接触时间线（中文内部证据报告）

## 结论范围

本阶段没有新启动 Isaac Sim，也没有做 RL、硬件或物理卡滞恢复实验。R2 新增物理实验数为 **0**；本报告重放并分析已有的 1 个完整名义接触回放：`r1_event_supervisor_physical_push01`，来自 R1 已完成的 Isaac 运行。证据中的另一个完整推入回放是 `r4_physical_push02`，`r3_position_precontact03` 是仅到 HIGH_PRE_CONTACT 的门禁运行；它们被保留但没有与本 R2 结果混合统计。

该 R1 运行摘要：15,688 个物理帧、65,844 个原始 PhysX 接触事件、Cube 总位移 0.6463748 m，控制模式 `FREE_SPACE_TRAJECTORY_POSITION_DRIVE`，退出码 0。原始 trace、接触事件、运行事件和日志均未改写。

## 时间线

| 事件 | 首帧 | 仿真时间 | 证据 |
|---|---:|---:|---|
| Cube/Table 接触 | 1 | 0.016667 s | PhysX 接触报告 |
| 左工具/Cube 接触 | 2152 | 35.866669 s | 4 个左侧 cup 对 |
| 右工具/Cube 接触 | 2966 | 49.433336 s | 4 个右侧 cup 对 |
| Cube x 超过初始位置 5 mm | 3600 | 60.000003 s | Cube pose |
| Cube 速度 x 首次超过 0.01 m/s | 3584 | 59.733336 s | Cube velocity |
| Cube/深端墙接触 | 9236 | 153.933341 s | `/World/Task27/TruckBox/WallDeep` |

接触边在 trace 中按物理帧出现的数量为：`CUBE_TABLE` 14,033、`LEFT_TOOL_CUBE` 3,884、`RIGHT_TOOL_CUBE` 6,500、`CUBE_DEEP_WALL` 6,452。原始事件数仍以 summary 的 65,844 为准，边计数是按帧去重后的辅助统计。

Cube 在工具接触前基本保持在 x≈0.5 m；接触后约 59.6 s 开始明显前进，随后在约 x≈0.77 m 处停留，再向深端墙运动。全程末端 TCP、关节位置/速度、Cube pose/velocity 和控制 owner/mode 均在 trace 中可用。

## 可观测性边界

- `Cube/Table`、左/右工具/Cube、深端墙/Cube：有 PhysX contact report，可作为**离线参考标签**。
- 侧墙/Cube：R1 monitor 没有订阅该路径，故整个 R2 数据集标为 `UNKNOWN`；没有从 Cube 位姿反推侧墙接触。
- 吸盘命令、闭合反馈、命令推进量、命令时间戳：没有与该物理 trace 同步记录，均标为 `UNKNOWN`。
- 本 trace 没有可用于“命令在前、Cube 不动”的同步推进命令，因此不能从该运行标注 premature jam。
- PhysX 接触邻近或几何 gap 不是物理卡滞证明；离线的规划/接近结果也不是恢复成功证明。

## 复现产物

`dataset/r2_timeline.jsonl.gz` 逐帧保存 q/dq、TCP、Cube 状态、几何特征、可用性字段和单独的 `oracle_reference`。`oracle_reference` 只用于评估，估计器输入不读取它。时间线图见 `figures/r2_contact_timeline.png`。原始输入的 SHA-256：

- `physical_trace.jsonl.gz`: `e6b37db09f714660b2f1c87468da0c88ed7fa8ee6ab232c25c40efde3c403eb9`
- `contact_events.jsonl.gz`: `b4630092b9c721a8703aa59366c7a274a8e6e083c13f32f33063ede74602e4af`

因此 S0 对名义接触时间顺序和已有观测字段是可复现的；对侧墙、吸盘和异常卡滞仍是数据不完整。

# R2 E0/E1 估计器比较

## 重放设置

`python3 research/stage_b_contact_graph_r2/scripts/replay_estimators.py` 对 15,688 帧、1 个 episode 重放 E0 和 E1。估计器输入不含 `oracle_reference`；PhysX 标签只在评估阶段使用。帧标签高度相关，且不是独立物理真值，因此精度只表示“与该 trace 的参考报告相符”。没有 held-out episode，也没有异常卡滞标签。

## 结果

| 指标 | E0 | E1 |
|---|---:|---:|
| 工具接触事件 frame accuracy | 0.840324 | 0.840579 |
| 工具接触 precision / recall | 0.995671 / 0.660377 | 0.996083 / 0.660651 |
| 深墙事件 frame accuracy | 0.999873 | 0.999873 |
| 深墙 precision / recall | 0.999690 / 1.000000 | 0.999845 / 0.999845 |
| 工具首事件相对参考延迟 | -0.350 s（提前） | -0.317 s（提前） |
| 深墙首事件相对参考延迟 | -0.0167 s（提前） | +0.0167 s |

E0 状态计数为 FREE_SPACE 4,383、TOOL_CONTACT 4,065、NORMAL_ADVANCEMENT 786、DEEP_WALL_ARRIVAL 6,454；E1 另有初始 UNKNOWN 2 帧，其他主要计数为 FREE_SPACE 4,383、TOOL_CONTACT 4,065、NORMAL_ADVANCEMENT 786、DEEP_WALL_ARRIVAL 6,452。E1 的驻留逻辑抑制了少量边界抖动，但不会增加缺失的观测通道。

工具接触的提前是几何接近先于 PhysX 接触报告，不应被称作“提前物理检测成功”；深墙提前/延迟只有一个物理帧量级。表面接触边没有被 E0/E1 当作闭合接触估计，侧墙保持 UNKNOWN。

## 卡滞与复杂估计器判定

该 episode 没有同步 `command_advance_m` 或命令时间戳，也没有独立的 premature jam 标签。因此“实际 premature jam 是否出现”：**没有可证实的实例**；不是“没有发生”的物理证明。E0/E1 的 `SUSPECTED_PREMATURE_BLOCKAGE` 只在单元测试中用合成的“有推进命令、低 Cube 速度”输入覆盖，不能算自然实验结果。

状态过渡、未知值和时间延迟的离线重放可复现，故本层状态为 `R2_CONTACT_ESTIMATOR_OFFLINE_PASS`。在线 observer 尚未验证；复杂估计器必要性与优势为 `UNPROVEN`，不支持引入 GNN 或训练 RL。

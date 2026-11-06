# R2-B Sensor Pipeline 最终决策

## Gate

**`R2B_SENSOR_PIPELINE_PARTIAL`**

本轮确认并修复了采集调度根因，physical07 证明固定 rear_11 相机可以连续产生带 physics receipt 的 RGB-D 文件对；但完整门仍未通过：图像 writer 速率低于 5 Hz 目标下限，viewport 没有曝光级 frame id，独立多视角标定未完成，side handoff 仍不安全，executor 也没有完成完整 manipulation episode。

## 分项结果

| 项目 | 结果 | 依据 |
|---|---|---|
| P0 调度根因 | REPAIRED / VALIDATED | threshold scheduler、writer 等待、physical07 1550/1550 pairs |
| 连续观测 | PARTIAL | physics receipt 6.021 Hz；writer 图像 4.735 Hz；marker 1296/1550 |
| RGB-D 同步语义 | PARTIAL | 同调度步文件对；无曝光级 frame id |
| 独立标定 | BLOCKED/PARTIAL | static 55.1 mm、physical05 37.4 mm；cy341 仅 evaluation |
| side handoff | BLOCKED/UNKNOWN | 保留 FCL；physical07 lift settle failure |

## 代码与证据

分支为 `research/stage-b-r2b-sensor-pipeline-repair`，基线为 `98d26087ce3b31fd6af2fc041d878f447eb5d0a8`。核心代码在 `isaac/scripts/task28_task27_one_pass.py`；回归测试和手动 harness 在 `research/stage_b_vision_contact_r2b/tests/`。physical06/07 的 RGB、depth、contact trace 保留在本地但因体积被 `.gitignore`；compact summary、timeline、phase receipts 和报告随分支提交。

后续研究门是独立非共面标定和显式 handoff primitive 的安全 preflight。RL、GNN、自动切换、完整 HiDex 和物理 jam recovery 均不在本任务范围内。

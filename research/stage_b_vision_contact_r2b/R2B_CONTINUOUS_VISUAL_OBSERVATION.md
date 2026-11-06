# R2-B 连续视觉观测结果

## episode 与全局观测

`physical07` 是本轮 post-repair 的有界 episode。runner 的 `summary.json` 为 `status=COMPLETED`，但 `timeout_reached=true`、`executor_exit=-15`；日志随后报告 side-compaction lift 未在 25 s 内 settle。因此这是观测验证，不是完整操控成功。

相机从 physics step 6 开始记录，1550/1550 RGB-D 对成功，深度有限值比例为 100%，capture/depth error 为 0。writer-based 图像率为 4.735 Hz，physics receipt 为 6.021 Hz，最大 receipt gap 为 0.2333 s。ArUco 检出为 1296/1550（83.61%）；最长 17.33 s 是 marker 未检出区间，不是相机未采样区间。对比之下，physical05 的 90.65 s 是真正的未采样盲区。

## executor 收据与相机样本

以下数字来自 `evidence/r2b_rear_wall_physical07/execution_phase_receipts.json`。阶段边界来自 executor 日志，不使用 Cube GT 阈值；每行分母只在该阶段区间内计算。

| 阶段 | camera samples | ArUco detections | rate |
|---|---:|---:|---:|
| cube supply | 7 | 7 | 100.0% |
| dual-arm approach | 267 | 265 | 99.25% |
| dual-arm attachment | 58 | 18 | 31.03% |
| coordinated transport | 86 | 3 | 3.49% |
| placement/release | 21 | 21 | 100.0% |
| release/retreat | 120 | 120 | 100.0% |
| single-arm recontact | 157 | 157 | 100.0% |
| single-arm insertion | 197 | 197 | 100.0% |
| deep-wall arrival | 6 | 6 | 100.0% |
| side compaction attempt | 298 | 191 | 64.09% |

side-compaction 已进入执行选择和执行区间，收据状态为 `EXECUTION_FAILED`：最终 lift 未在 25 s 内 settle。不能用几何 phase 标签替代 executor 收据。视觉数据足以验证采集连续性和阶段收据关联，不能证明接触恢复或物理 jam recovery。

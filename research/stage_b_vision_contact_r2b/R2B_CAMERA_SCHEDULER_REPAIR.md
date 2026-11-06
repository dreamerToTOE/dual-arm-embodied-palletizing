# R2-B 相机调度修复与验证

## 调度改动

修改文件：`isaac/scripts/task28_task27_one_pass.py`。

1. 用 `next_capture_step` 阈值替代精确 modulo；一次跨过多个 deadline 时保留当前样本并计数 `missed_deadlines`。
2. 将相机采集接入初始化、MoveIt 等待和 executor 循环，避免 executor 尚未启动时没有样本。
3. 对 viewport writer 使用有限的 4 次常规 render update，最多追加 11 次；记录 `render_wait_updates`，不把等待后的 physics step 当作原始收据。
4. RGB、depth、调度步、physics receipt、render wait、writer 完成和错误分别写入 `camera_timeline.jsonl`；`camera_acquisition_summary.json` 汇总请求、成功、失败、跳过 deadline、间隔和 writer latency。
5. `close()` 幂等，异常路径也保存 timeline 和 summary。

## physical07 数值

证据文件：`evidence/r2b_rear_wall_physical07/r2b_camera/camera_acquisition_summary.json`。

| 指标 | 数值 |
|---|---:|
| requested frames | 2573 |
| successful RGB-D pairs | 1550 |
| failed frames | 0 |
| missed deadlines | 1023 |
| writer-based successful rate | 4.735 Hz |
| physics receipt rate | 6.021 Hz |
| median writer gap | 0.2141 s |
| max physics receipt gap | 0.2333 s |
| max writer latency | 0.7308 s |

1023 个 missed deadline 表示 writer 等待期间的调度合并，不表示 1023 个连续光学 dropout；timeline 的 physics receipt 没有 90 s 级空洞。writer-based 图像率略低于本轮目标 5–15 Hz 下限，故不能把这轮写成目标频率 PASS。

## 回归验证

`tests/test_camera_scheduler.py` 覆盖错过 modulo 边界、多个 deadline 合并、重复 physics step 和 RGB 失败。`tests/test_sensor_pipeline_artifacts.py` 检查 physical05 的盲区/检测区分、校准证据完整性及 physical07 的 RGB-D 对齐字段。`python3 -m pytest -q research/stage_b_vision_contact_r2b/tests` 通过 13 项；`py_compile` 和 `git diff --check` 也通过。

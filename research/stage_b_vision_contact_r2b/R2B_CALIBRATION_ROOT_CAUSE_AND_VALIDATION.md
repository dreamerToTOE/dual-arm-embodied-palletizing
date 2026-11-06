# R2-B 标定根因与独立验证

## 当前状态

标定仍为 `PARTIAL`，没有把任何拟合参数提升为在线标定。静态 manifest 的 12/12 SHA-256、train/holdout 不相交、JSON 聚合值重算均通过；这证明证据文件一致，不证明相机外参或 Cube pose 已达到控制精度。

现有 `cy=340.493`（报告中四舍五入为 `cy=341`）只是固定焦距下的 viewport principal-point 拟合，运行时仍使用 authored `cy=360`。静态 holdout 平均 world-pose 残差约 55.1 mm，physical05 holdout 约 37.4 mm，远高于 3 mm 控制阈值。

## physical07 对比

`visual_qualification.json` 给出 1550 个 RGB-D 时间点，其中 authored intrinsics 检出 1296/1550（83.61%），平均位置误差 41.1 mm、姿态误差 6.61°。将 `cy=341` 仅用于离线重算时，平均位置误差 41.3 mm、姿态误差 5.19°；它没有形成稳定的独立校准收益，也没有验证完整的 K、畸变、`T_world_camera` 和 marker-to-Cube 外参。

因此本轮不做 calibration PASS，不把 `cy=341` 写回运行时，也不据此支持 3 mm 级接触控制。下一步最小补充是非共面多 marker 的独立 train/holdout，带完整内参、畸变和外参验证。

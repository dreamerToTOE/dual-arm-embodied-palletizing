> 兼容 Issue #7 的交付文件名；正文以 [`R2B_E0_E1_CONTACT_ESTIMATION.md`](./R2B_E0_E1_CONTACT_ESTIMATION.md) 为准。

# R2-B E0/E1 接触状态估计


日期：2026-11-06  
研究分支：`research/stage-b-r2b-contact-recognition`  
校准父分支：`research/stage-b-r2b-camera-precision`  
校准实现提交：`915ddd44e4067c8687c0542e297b24c03485fa42`  
校准报告提交：`f374d75fb2ee7d6c008c89c1123541cbda860edc`  

## E0

实现位于 `research/stage_b_vision_contact_r2b/estimator/stage_b_contact.py`，输出以下状态：

`FREE_OR_RECONTACT`、`SINGLE_ARM_PUSHING`、`PUSH_PROGRESS`、`DEEP_WALL_NEAR`、`DEEP_WALL_CONTACT_POSSIBLE`、`SIDE_CONTACT_POSSIBLE`、`STALLED_OR_AMBIGUOUS`、`UNKNOWN`。

同时输出多个 contact-edge 概率：left/right tool-Cube、Cube-table、Cube-deep-wall、Cube-side-wall。边缘概率与 skill label 分离，允许同时存在多个可能关系。工具距离、墙面/桌面几何和 commanded advancement 只产生 proximity/possible 证据；没有力、有效接触传感器或同等证据时，`contact_confirmed_from_available_evidence` 固定为 false。

输入缺失、视觉低置信或相机过期时返回 `UNKNOWN`；“靠近”不会被写成 loaded contact，“命令前进但没有观测进度”也不会被写成真实 jam。

## E1

E1 在 E0 外包一层因果时间逻辑：保存上一帧位置和时间，计算已观测 x 速度；仅在不超过 0.25 s 的短窗口做前向预测；通过 dwell/hysteresis 抑制单帧抖动；预测窗口耗尽后返回 `UNKNOWN` 并清空 pending state。预测帧的置信度被限制在 0.35–0.55，避免伪装成真实观测。

没有使用未来帧、Cube ground truth、PhysX contact 或 suction closure 作为 oracle。测试覆盖了候选 pending、短暂 dropout、过期后 UNKNOWN，以及 selector 对 stale 输入的拒绝。

## physical07 实际评估

脚本：`scripts/evaluate_stage_b_contact.py`；证据：`evidence/r2b_rear_wall_physical07/stage_b_contact_evaluation.json`。

| 指标 | 数值 |
|---|---:|
| 视觉帧 | 1550 |
| RGB 可用帧 | 1296（83.61%） |
| E0 UNKNOWN | 255 |
| E1 UNKNOWN | 276 |
| E0 DEEP_WALL_NEAR | 138 |
| E1 DEEP_WALL_NEAR | 155 |
| shadow 输出 HOLD_UNKNOWN | 1550 |
| 相机仿真 age 中位数 / 最大值 | 0.133 / 0.200 s |
| writer latency 中位数 / 最大值 | 0.166 / 0.731 s |

按阶段，single-arm insertion 为 197 帧、197 次 detection，E1 给出 192 `FREE_OR_RECONTACT`、2 `DEEP_WALL_NEAR`、3 `UNKNOWN`；deep-wall arrival 为 6/6 detection，E1 为 6 `DEEP_WALL_NEAR`；auxiliary side compaction 为 298 帧、191 次 detection，E1 为 142 `DEEP_WALL_NEAR`、38 `FREE_OR_RECONTACT`、118 `UNKNOWN`。这些只是几何候选状态，不是加载接触标签。

## 独立指标边界

当前 receipts 只描述阶段区间，没有每帧 loaded-contact、stalled/jam 或 side-contact truth。故 confusion matrix、precision、recall、F1、transition delay、false-positive contact 和 false-stalled 数值均为 `UNAVAILABLE`；没有把 Cube/PhysX 字段转成在线标签，也没有发明 jam case。该单 episode 不能同时作为 unrestricted tuning 和独立 ground truth。

## 2026-11-06 runtime integration addendum

基线报告的 `UNAVAILABLE` 指当时没有接入 raw physical trace。当前分支已生成独立
oracle sidecar，并在 causal E0/E1 输出后离线计算 possible-pair 与
`LOADED_SUPPORTED` 指标；这些数字不进入在线输入。严格 loaded 标签下 E1 F1
仍低（right tool 0.004、deep wall 0.086），所以结论仍不是接触识别 PASS。详见
`R2B_CONTACT_ORACLE_LABEL_AUDIT.md` 与 `R2B_E0_E1_SHADOW_EVALUATION.md`。

结论：E0/E1 已在真实导出的因果 RGB/timestamp 输入上运行并能按不确定性 abstain，但科学上尚未完成“接触识别性能”验证。

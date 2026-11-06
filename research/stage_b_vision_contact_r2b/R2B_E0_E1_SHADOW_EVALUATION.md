# R2-B E0/E1 与 shadow 评估

评估输入是 physical07 RGB 加 latest-past runtime receipt；oracle labels 只在
离线评分阶段 join。结果文件：
`evidence/r2b_rear_wall_physical07/runtime_integration/e0_e1_shadow_evaluation.json`。

视觉有效率为 1,296/1,550 = 83.6%。E0 状态计数为 UNKNOWN 255、FREE 756、
SINGLE_ARM_PUSHING 393、DEEP_WALL_NEAR 138、SIDE_CONTACT_POSSIBLE 8；E1 为
UNKNOWN 276、FREE 751、SINGLE_ARM_PUSHING 364、DEEP_WALL_NEAR 155、SIDE_CONTACT
POSSIBLE 4。shadow 只输出 `HOLD_UNKNOWN` 1,186 与 `CONTINUE_PUSH` 364，未做
controller switching。

以 `pair_present` 为可能接触离线标签时，E0/E1 的右 tool F1 为 0.822/0.839，
deep-wall F1 为 0.547/0.564；这只是 proximity/possible 评分。以严格
`LOADED_SUPPORTED` 标签评分时，E1 F1 为：left tool 0.269、right tool 0.004、
table 0.427、deep wall 0.086。因而不能把 proximity 结果写成 loaded-contact
识别成功，也不能宣称 jam/recovery 成功。

安全边界通过：selector read-only、无运动命令、无 suction override、无 live
mode switch；视觉缺口超过短预测窗或 TCP 缺失时保持 UNKNOWN/HOLD。

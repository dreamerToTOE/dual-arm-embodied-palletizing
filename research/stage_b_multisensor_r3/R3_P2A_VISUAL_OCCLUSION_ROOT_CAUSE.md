# R3-P2A Stage-B 视觉缺失逐帧审计

日期：2026-11-07。基线 `c01e18a1e30e7118a77e8e51f0856c304a74d08f`。
**目标 A：PARTIAL。107 张全部定位、复检并观察；88 张有 probable 箱壁遮挡证据，
19 张原因仍 UNKNOWN。没有把未检出自动解释为机械臂遮挡。**

## 原始文件与集合身份

raw 位于本地 `stage-b-r2b-sensor-pipeline-repair/research/stage_b_vision_contact_r2b/evidence/r2b_rear_wall_physical07/`。
1550 RGB、1550 depth 均实际存在；完整路径与源哈希见 `evidence/p2a_occlusion/summary.json`。
阶段来自 `execution_phase_receipts.json` 的 `auxiliary_side_compaction`，物理步区间
12462–15479；按该区间与 receipt 时间筛选均得到 298 帧、191 检出、107 未检出。
实际图像步号为 12469–15441。没有使用旧 GT-derived visual.phase。

107 张的精确步号、receipt time、RGB/depth 文件名、各自 SHA-256、深度 finite fraction、
原状态、重新检测状态和人工图像判断均在 `107_frame_review.csv`。相同默认 ArUco
检测器重新运行：**107/107 仍未检出**，并非依赖旧缓存状态猜测原因。

## 图像证据和判断

全部 107 张全景与局部图逐格检查，保存 5 张 `full_sheet_*.jpg`、5 张
`crop_sheet_*.jpg`。前 19 张另作放大复查 `borderline_review.jpg`；5 张原始 PNG
代表帧也已保存。图像分类没有使用 Cube GT，局部裁剪框是固定图像区域。

| 类别 | 帧数 | 证据与限制 |
|---|---:|---|
| PARTIAL_MARKER_OCCLUSION_BY_WALL | 88 | 步号14569–15441；标记下缘/外黑边与前侧箱壁相交或被截断，工具在其旁边；probable |
| UNKNOWN | 19 | 标记大部分可见，贴近墙沿；轻微边缘遮挡、像素栅格化与 decoder 敏感性尚不能区分 |
| 完全工具/机械臂遮挡 | 0 已确认 | 没有由未检出推定此类 |
| 视野外 / 仅因极端视角 / 明确照明问题 | 0 已确认 | 没有足够图像证据单独归因 |

类别是可复核的 plausibility 判断，不是独立人工双标注或像素级 segmentation 真值。
88 帧的相似图像不是 88 次独立因果实验；没有通过换相机或移墙来证明反事实。
没有修改相机、检测器参数或 P1 估计器来提高此处统计。

## 缺失段、采集与机器人状态

从 release receipt 结束 119.617 s 起，到实际 trace 结束 257.983 s：Stage B 共
818 张，711 检出、107 未检出。侧压共 15 段连续未检出；前 14 段各1–2张，
最后一段 **88 张**：242.816679–257.350013 s，已观测缺失跨度 **14.533334 s**。
上一有效帧至最后已观测缺失为 **14.700001 s**；记录结束前没有恢复检出，因此
这段是右删失，不能称“完整遮挡时长14.53秒”。单张失败的已观测跨度为0，完整
缺失区间只能被相邻有效帧约束，JSON 同时保留这些定义。

298 次侧压采集均 `frame_pair_ok=true`，RGB/depth 缺文件均为 **0**。
107 次是成功采集后的 marker 未检出。另有 **199 个名义调度时隙被跳过**，来自
旧调度器 `scheduler_skipped_intervals`，它们不是107张失败图片；实际采样间隔
median/max=0.166667/0.200000 s，不能称持续10Hz视觉。

最长段内有 **873 个约60Hz机器人状态样本**，q/dq、USD TCP、吸盘状态/命令、
controller command 字段均873/873可用；双臂 suction observation 都是有效bool，
controller sequence有805个不同值。命令是目标与 receipt，不是实测q，也不代表
每条均对齐到图像曝光。历史 TCP 是 USD link readback；独立 q→FK 在新探针核验。
这些量仍可用于后续间歇观测研究，但不证明 Cube 无滑移或附着成立。

executor 日志 wall→sim 回归把阶段结束映射到290.192 s，而真实 trace只到257.983 s。
审计裁剪到实际记录，不把额外32.21秒回归外推当成“没有采集图像”。旧 side LIFT
执行失败照常保留，不能归咎于视觉缺失或提升为完成了 side compaction。

## 决定

视觉在此场景应作为间歇测量源。P2B 接口需要携带图像年龄、缺失段和时间可信度，
缺帧不自动触发附着/接触结论。本轮不移动相机，不研究 Stage-A、离桌或侧翻。

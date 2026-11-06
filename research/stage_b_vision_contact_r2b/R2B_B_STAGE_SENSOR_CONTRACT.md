> 兼容 Issue #7 的交付文件名；正文以 [`R2B_STAGE_B_OBSERVATION_INTERFACE.md`](./R2B_STAGE_B_OBSERVATION_INTERFACE.md) 为准。

# R2-B Stage B 因果观测接口


日期：2026-11-06  
研究分支：`research/stage-b-r2b-contact-recognition`  
校准父分支：`research/stage-b-r2b-camera-precision`  
校准实现提交：`915ddd44e4067c8687c0542e297b24c03485fa42`  
校准报告提交：`f374d75fb2ee7d6c008c89c1123541cbda860edc`  

## 实现

接口位于：

- `research/stage_b_vision_contact_r2b/observer/contract.py`
- `research/stage_b_vision_contact_r2b/observer/stage_b_adapter.py`

`StageBObserverFrame` 现在承载：RGB Cube 位姿、置信度、观测时间、q/dq、FK TCP、控制器参考/命令、吸盘 command/state、scene-side source stamp 与 health、sensor age、uncertainty、显式 `attachment_confidence`。字段缺失时使用空/None 与 `UNKNOWN`，不会从 Cube USD、PhysX contacts 或 offline trace 回填。

`frame_from_records()` 是唯一的兼容接入点；`causal_payload()` 断言 `oracle_contact_pairs` 不进入在线 estimator payload。oracle 仍可留在 `to_dict()` 的离线诊断通道，不能作为在线输入。

## 因果边界

- `visual_cube` 只来自 RGB 记录；`confidence`、marker id、reprojection/latency 和 camera source provenance 原样保留。
- q/dq/TCP、controller commands、suction feedback、attachment confidence 只有收到 runtime receipt 才进入输入；缺失时 estimator 必须 abstain。
- `SourceStamp.age_s`、`sensor_health` 和 `uncertainty` 用于 stale/missing 边界；默认相机延迟来自时间线，而非未来帧。
- `attachment_confidence` 可以是每侧 `[0,1]` 或 `None`；没有“吸盘闭合即刚性附着”的隐式规则。

## 实际数据限制

physical07 当前导出包含 RGB detections、camera timeline 和 execution phase receipts，但本工作树没有与每个视觉帧对齐的 q/dq/TCP/command/suction stream。评估脚本因此明确将这些字段视为 UNKNOWN，不能把 receipt 中的 Cube/PhysX 评价字段当在线传感器。

## 验证

- `causal_payload()` 单元测试确认 oracle 不泄漏。
- `StageBObserverFrame.validate()` 检查时间、q/dq 长度、视觉置信度和 attachment confidence 范围。
- `PYTHONPATH=. python3 -m pytest -q research/stage_b_vision_contact_r2b/tests`：17 passed。

该接口完成的是可审计的传感器契约，不等于 runtime observer 已经接入 cal24，也不等于 physical07 已经具备完整本体状态流。

## 2026-11-06 runtime integration addendum

上述段落记录的是基线导出状态。当前独立 runtime 分支已从保留的 physical07
`physical_trace.jsonl` 生成 latest-past causal observations：1,550/1,550 相机帧
匹配 q/dq/TCP/suction 行，203 帧 command receipt 仍为 UNKNOWN，attachment
confidence 仍为 UNKNOWN。该接入没有改变本契约的缺失字段语义；详见
`R2B_STAGE_B_RUNTIME_SENSOR_INTEGRATION.md`。

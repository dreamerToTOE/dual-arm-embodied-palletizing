# R2-B Stage B 最终门禁决定


日期：2026-11-06  
研究分支：`research/stage-b-r2b-contact-recognition`  
校准父分支：`research/stage-b-r2b-camera-precision`  
校准实现提交：`915ddd44e4067c8687c0542e297b24c03485fa42`  
校准报告提交：`f374d75fb2ee7d6c008c89c1123541cbda860edc`  

## 最终状态

**`R2B_B_STAGE_SHADOW_BLOCKED`**

实现、因果接口、E0/E1、shadow selector 和实际 physical07 运行均已完成；Gate 0 的 cal24→Cube transfer 失败，且实际 episode 没有逐帧独立接触标签，因此不能升级为 `PASS` 或 `PARTIAL` 的可验证接触识别结论。

## 门禁表

| 门 | 结果 | 证据 |
|---|---|---|
| 校准分支远端可见、祖先关系正确 | 通过 | `f374d75fb2ee7d6c008c89c1123541cbda860edc`；远端 ref 已核对 |
| cal24 参数进入实际 runtime observer | 阻断 | runtime 仍是 ID23/旧协议，未接四 marker + cal24 K |
| Cube 附着板静态 transfer probe | 阻断 | 12 张图 ID10–13 全部 0 检出，0 个可用 PnP 视图 |
| causal Stage B observation contract | 实现完成 | adapter、source age/health、uncertainty、attachment confidence、oracle 隔离 |
| E0/E1 在实际因果数据运行 | 完成但不可评分 | 1550 帧；E0/E1 状态和 UNKNOWN 统计已写证据 |
| 独立接触 precision/recall/F1/delay | 不可用 | receipts 无逐帧 loaded-contact/stall labels |
| read-only shadow selector | 实现完成、评估阻断 | 1550/1550 HOLD_UNKNOWN；无 live switch |
| 物理安全门 | 未通过 | side-compaction lift/FCL/joint-tracking 历史问题仍在 |

## 本窗口实际做了什么

- 新建并使用隔离分支 `research/stage-b-r2b-contact-recognition`。
- 运行 1 次有界静态 transfer probe；没有重跑完整 Task27，没有新增 wrist camera，没有 E2/RL/GNN/paper reproduction。
- 用 physical07 做 RGB/timestamp 因果压力评估，明确缺失本体输入和独立标签，不伪造 jam/接触指标。
- 保留 FCL、碰撞、力矩、关节限制和吸附边界；没有 live controller switching。

## 解除阻断所需的最小下一步

1. 修正 cal24 生成代码的 marker 可见面/manifest 同一几何，并让 runtime observer 真正加载四 marker、cal24 K、rear_11 和 Cube 变换。
2. 先完成一组可检测的静态 Cube 附着板视图，再做短动态 holdout；记录相机、q/dq、FK TCP、commands、suction 与 attachment confidence 的同一时间轴。
3. 单独取得 loaded contact / stalled / side-contact 评价标签，再报告 E0/E1 confusion matrix、F1、transition delay 和 false positives。
4. 另设安全验证门后才讨论任何实际 skill transition。

本分支的最终代码/报告提交 SHA 以推送后 `git rev-parse HEAD` 和 `git ls-remote` 的核对值为准；校准父提交和远端 ref 已在上文给出。

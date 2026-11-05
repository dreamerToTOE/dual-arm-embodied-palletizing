# R2 可行性结论

## 顶层判定

**`R2_CONTACT_ESTIMATOR_OFFLINE_PASS`**。

判定理由是：已有 1 个完整名义 Isaac 接触 episode 可以被固定输入、固定阈值、固定时间线的 E0/E1 因果回放重现；数据集、预测文件、比较指标和测试均已生成。这个状态只覆盖离线名义接触估计，不覆盖在线 observer、物理卡滞恢复或复杂估计器优势。

## 实际实验计数与观察

- R2 新启动的物理实验：**0**。
- R2 分析的已完成名义物理 episode：**1**（`r1_event_supervisor_physical_push01`）。
- 该 episode：15,688 帧、65,844 原始接触事件；观察到 Cube/Table、左/右工具/Cube、深端墙/Cube；侧墙/Cube 为 UNKNOWN。
- 另有已保留的 R0/R1 证据（一个 precontact-only 运行和另一个完整推入运行），不与本次单 episode 指标拼接。
- R2-S4（人为 yaw/侧向偏差、真实 premature stall）有效物理试验：**0**。

## 对问题的回答

- E0/E1 能在离线名义 trace 上区分接近、工具接触、推进和深墙到达；E1 只带来因果驻留和少量边界抑制。
- 当前数据没有可证实的实际 premature jam。没有同步 commanded advancement、suction state、side-wall contact 和已验证 force channel，不能输出在线 jam 概率或 recovery success。
- 接触图 schema 对“多边未知状态、不同边同时激活、观察缺失”有清楚表示；但单一名义 episode 不足以证明图模型比简单规则更必要。复杂估计器状态为 **UNPROVEN**。
- 现有技能原语足以支持双臂接近/吸附、COMMON_LIFT/X/Y/DESCENT、单臂分片 +X 推入、深墙门禁、SIDE_COMPACTION、释放/退出；缺少统一的 observer-frame 转移接口，不能可靠地把技能状态切换与物理接触事件绑定。

## 下一道门

最小追加原语是带统一 physics-step 时间戳的 `StageBObserverFrame`：命令 id/推进量、Cube receipt、左右 suction command/state、深墙和侧墙 contact 摘要，以及经过主开发会话确认的力/反力字段。接入并通过一条名义回放后，再运行有限的正常、侧墙接触和命令停滞试验；在此之前保持物理恢复实验 deferred，不启动 RL 或完整 HiDex 实现。

## 可复现文件

- 数据集：`dataset/r2_timeline.jsonl.gz`
- 估计器预测：`derived/estimator_predictions.jsonl.gz`
- 比较结果：`derived/r2_comparison.json`
- 时间线图：`figures/r2_contact_timeline.png`
- 代码与测试：`estimator/`、`scripts/`、`tests/`

本报告在提交时记录新分支 commit SHA；主分支 `side-suction-palletizing` 和先前 R0/R1 分支不修改。

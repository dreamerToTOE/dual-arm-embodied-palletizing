# R2 技能接口与状态转移审计

审计对象为当前分支继承的 `ros_ws/src/fr3_dual_palletize/src/task26_truck_box_push_in.cpp`；行号来自该文件当前版本。

| 技能/阶段 | 已有原语与门禁 | R2 观察 |
|---|---|---|
| 双臂接近/建立侧面接触 | HIGH_PRE_CONTACT → OUTER_DESCENT → CONTACT，左→右微时序；约 3417–3513 行 | 有 FCL 与实测 TCP/Cube 对齐门禁；侧墙状态没有进入 observer 图 |
| 吸附建立 | `validateDualSideAttachment()` 901–947 行；`/left|right/suction_command` 和 state 订阅 958–967 行；约 4846–4866 行闭合并复核 | 执行器有闭合反馈，但 R1 PhysicalTrace 没有同步保存，无法做 R2 时间对齐 |
| 承载/运输 | COMMON_LIFT、COMMON_X_TRAVEL、COMMON_Y_ALIGN、COMMON_DESCENT_TO_ENTRY 4908–4976 行 | 共同轨迹和 FCL 可用；没有 contact→transport 的统一 observer 事件接口 |
| 落桌/释放 | COMMON_DROP_TO_TABLE 5009–5021 行，随后 `openBothAndConfirm()` 2959–2974 行 | 释放确认存在，但未进入本次 trace schema |
| 单臂 +X 推入 | `executePushWithSupervision()` 2686–2696 行；分片执行和 Cube receipt 等逻辑约 2703 行以后；调用 5362–5376 行 | 代码有命令推进和停滞判据，但本 R1 run 的 PhysicalTrace 没保存 commanded advance/命令时间戳，因此不能重建因果卡滞标签 |
| 深墙验收 | 深墙 receipt/`validDeepWallSeat` 约 5380–5415 行 | 可与 PhysX 深墙参考边对齐；这不是恢复成功证明 |
| 侧向压紧 | SIDE_COMPACTION 的 HIGH_APPROACH→CONTACT→PRESS→WITHDRAW/LIFT/RETURN，5804–5907 行 | 有现成技能链；侧墙 contact 仍未监视，不能验证“侧压触发”条件 |
| 释放/退出 | pusher release 5882–5887 行，后续 clearance/handoff | 有安全释放路径；缺少 observer 对失吸、退出命令和状态的统一事件接口 |

## 缺口

1. 需要一个带 physics step/time、command id、commanded advance、Cube receipt、左右吸盘 command/state、侧墙/深墙 contact 摘要的 `StageBObserverFrame`；目前只能从两个不同日志源按墙上时间排序，不能声称同步。
2. `PhysicalTrace` 需要订阅 `/World/Task27/TruckBox/WallSide*`（或实际场景中的侧墙 collider），把侧墙边从 UNKNOWN 变为可观测。
3. 需要经过主开发会话验证的力/反力通道；R2 不重开已经暂停的 force-observation debugging。
4. 需要一个保留真实几何扰动的预馈接口，才能运行 R2-S4 的 yaw/侧向偏差/推进停滞条件。当前 R2-S4 物理扰动试验数为 0。

## 最小下一步

先只扩展 observer frame 和 PhysicalTrace 字段，不增加控制器：将命令和反馈在同一 physics step 记录，补齐侧墙订阅和 suction state；通过一条名义推入回放检查时间对齐后，再安排正常、侧墙接触、真实命令停滞三类有限物理试验。没有这条接口，不能把在线自适应切换或 jam recovery 宣称为已验证。

# R3-P2A 最终决定

日期2026-11-07；分支 `research/stage-b-r3-multisensor-plan`，起点
`c01e18a1e30e7118a77e8e51f0856c304a74d08f`。沿用独立工作树；旧用户修改、
R2证据、Task26/27控制器及成功判据、P0/P1实现和结果均保持原样。

**总状态PARTIAL。本轮停止P2A，没有开始P2B或在线融合。**

| 目标 | 决定 | 数值证据 |
|---|---|---|
| A：107次未检出审计 | PARTIAL | 298采集、191检出；107张全部定位复检。88张probable箱壁遮挡下缘，19张UNKNOWN |
| B：动态时间接口 | PARTIAL | 最终30/30帧有单调SDK frame/time与共同回调配对，30/30可索引机器人历史；图像内容时刻仍未证实 |
| C：有界动态验证 | PARTIAL | 最终17个相同可评分帧XY P95从9.930降到3.593mm，但4个可辨识运动样本仍有33.333ms时间差，超过一tick |

## 已交付

- `R3_P2A_VISUAL_OCCLUSION_ROOT_CAUSE.md`、`R3_P2A_DYNAMIC_TIME_ALIGNMENT.md`：
  精确帧集合、联系表、缺失段、旧时间字段语义、全部尝试与失败。
- `time_alignment.py`、`scripts/capture_p2a_dynamic.py`：共同Camera回调采集，
  240样本有界q/dq/FK历史，连续量插值后重算FK，离散量只保持过去值，不外推。
- `scripts/audit_p2a_occlusion.py`、`evaluate_p2a_alignment.py`、冻结条件和
  `p2a_time_field_map.json`、逐帧CSV/JSONL、原图/深度与独立oracle。
- 更新 `R3_RESEARCH_PLAN.md`：限定落桌后的Stage B、持续桌面支撑假设、间歇视觉，
  P2A先于P2B。没有删除旧P0/P1结论或新增离桌/侧翻研究要求。

## 有效证据和限制

侧压107次是采集成功后的未检出，RGB/depth缺文件均0；另有199个名义调度槽跳过。
最长连续88张未检出，已观测跨度14.533s、右删失；期间873个机器人状态样本均有
q/dq、TCP、双臂suction bool及命令receipt，805个不同command sequence。
历史TCP不是独立FK，闭合bool也不能自证刚性附着。

共7次短Isaac启动：2次启动失败、2次几何无效、2次时序/姿态核验不足的诊断，
最后1次正式桌面复测。所有重要失败保留；没有完整Task27或Stage-A任务运行。
最终479个物理样本均kinematic=true、姿态角0、z=.260m，scale=.12。

最终组30帧，18检出接受，17帧双方都能评分；XY median/P95/max由
**0.667/9.930/10.263mm → 0.667/3.593/4.013mm**。首个旧request缺历史，保留UNKNOWN。
独立FK对PhysX TCP最大差0.000436mm；478个半tick CPU插值查询最大差0.000386mm。
CPU共 **44 passed**；旧P1估计器和config没有修改。仅复制Git索引文件到隔离目录后，
主实验summary及逐帧CSV与记录完全一致，不依赖physical07本地大文件；回执为
`evidence/p2a_clean_replay_verification.json`。

同SDK时间匹配physics的数值残差为0，但这不是曝光验证。最终4个运动样本的
图像投影最佳时段距SDK时间33.333ms（时段自身宽16.667ms）；还混有成像/传播偏差，
不能认证通用固定相机延迟。同步渲染诊断也没有消除问题，且其XY P95曾退化，
失败结果没有删除。没有用GT拟合时间offset或修正在线估计。

## P2B接口与入口建议

`evidence/p2a_tabletop_evaluation/p2b_timestamped_interface.jsonl`可直接用于接口评审：
`frame_id/pair_id/reported_render_time_s/arrival_*`、q/dq、独立FK、过去的suction与
controller command、P1视觉pose及来源hash；Cube GT/PhysX oracle不进入该文件。

当前30帧全部 `measurement_time_s=null`、`timing_status=UNCERTAIN`、
`online_fusion_eligible=false`。SDK时间下的robot状态仅供研究重放，不冒称可信曝光对齐。

下一步应在相同桌面范围内，用实际4.5 RationalTime/render graph绑定及独立可辨识的
帧时刻验证，关闭当前33ms差异，或建立可验证的测量时间区间；再开展P2B间歇视觉
与机器人状态融合。应继续保留到达延迟、观测年龄、时间不确定性和缺失原因。
不要求研究小幅离桌、Stage-A、侧翻、真机、RL、附着/接触分类或技能切换。

# R3-P2A 动态时间关联与有界验证

日期：2026-11-07；基线 `c01e18a1e30e7118a77e8e51f0856c304a74d08f`。
**目标 B：PARTIAL；目标 C：PARTIAL。SDK 帧身份、共同回调配对和机器人时间索引已
实测，但图像内容与 SDK time 的一致性尚未达到16.7ms目标，不批准在线融合。**

## 旧时间字段不能当曝光时间

完整 field→producer→physical meaning→consumer 在 `p2a_time_field_map.json`。
当前基线 `R2BPhysicalCamera.capture_if_due` 在 screenshot request 后执行4–15次
nested app.update，再读独立 depth；物理时钟持续推进。截图异步写出，没有权威
RGB frame ID，depth没有与截图共同的 frame ID，因此不能证明同帧RGB-D。

更重要的是，**保留的 physical07 receipt 与当前源代码的打点位置不完全一致**：
1550条都缺少后来代码的 `render_wait_updates/render_request_return_wall_s`。
其 `rendered_sensor_frame_wall_s` 距 request median仅70.0µs，depth stamp仅70.3µs；
writer完成则median/P95/max=0.166263/0.217051/0.730847s。历史生产脚本的精确revision
没有在run内绑定，不能用当前代码重新解释这些旧字段为render completion。该补充
不改写P0/P1报告，也不恢复历史曝光时间。

| 字段 | 真正含义 / 消费限制 |
|---|---|
| physics_step / time_s | trace post-step计数及累计dt；物理状态时间 |
| physics_state_time_s | 请求前physics receipt；不是图像时刻 |
| capture_request_wall_s | monotonic请求提交；不等于render |
| rendered_sensor_frame_wall_s | 旧记录仅为请求附近host打点；名称不能证明render |
| physics_time_after_render_wait_s | 后续更新到达的物理状态；仅漂移诊断 |
| depth_acquisition_wall_s | host读回路径打点；旧记录语义不足以证明depth曝光 |
| writer_complete_wall_s | 写盘/到达完成；永不作为曝光或render时间 |

## 最小新采集与接口

`capture_p2a_dynamic.py` 只在研究场景工作，不导入ROS bridge或启动Task27 executor。
保留原Task27 table、工具、墙和 rear_11 camera；Cube暂作kinematic桌面刺激，不是
物理推入成功实验。持续支撑为实验假设，z=.260m、yaw=0；腕关节目标仅±15mrad。

本机Isaac 4.5 `Camera._data_acquisition_callback` 对指定render product的NEW_FRAME
先同步evaluate SDG graph，再读ReferenceTime、RGB和distance_to_image_plane。
研究子类记录共同callback和ReferenceTime，消费者clone整份current_frame，之后才写盘。
不在采集adapter中调用nested app.update，不再用viewport screenshot当RGB源。

`rendering_frame` 来自ReferenceTime numerator；`rendering_time` 由SDK的
`get_sim_time_at_swh_frame`映射。**不能把numerator/denominator直接除成仿真秒**。
还实际探测了新版名称 `get_sim_time_at_time`，本机要求`omni.fabric.RationalTime`
对象，两整数形式不支持；TypeError保留，没有假设API不可用，也没有静默改用它。
实现依据是已安装4.5源码及其hash；未套用6.x接口。ReferenceTime的一般定义见
[NVIDIA官方说明](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.11.35/source/extensions/omni.replicator.core/docs/API.html)。

`RobotHistory` 最多240个样本；q/dq来自articulation，TCP由既有冻结URDF
`RobotModel.fk(side,q)`独立计算。连续量在相邻physics样本间插值q后重算FK，离散
suction/commands仅保持过去样本；不外推，超出缓存/大时间缝隙返回UNKNOWN。
新探针没有Surface_Gripper实例，suction observation/command明确None；旧physical07
通道可用性已单列，未伪造OPEN反馈。

原始observations与oracle分文件。P1像素预测和P2B接口写出后，才读取Cube/PhysX
oracle评分。导出`p2b_timestamped_interface.jsonl`包含frame/pair身份、reported time、
arrival、robot q/dq/FK、独立suction/command和视觉pose；**measurement_time_s=None、
timing_status=UNCERTAIN、online_fusion_eligible=false**。ALIGNED仅指给定SDK时间
能在history中求值，不代表该SDK时间已被证实是图像内容时刻。

## 七次有界尝试及修复

全部尝试都重新创建Isaac场景；没有完整Task27或Stage-A运行。

| run | 结果 | 证据/修复 |
|---|---|---|
| dynamic01 | 30组，但图像/场景无效 | 运行中构造SingleRigidPrim重写带后缀scale，Cube模型失真；post-step USD TCP读回也有大偏差。保留图、日志、源码快照 |
| dynamic02 | 无完成manifest、0图 | 初始化中断，SimulationApp.close掩盖异常；不以退出码0判成功 |
| dynamic03 | 0图，FAILED | 增加关闭前异常回执，确认StopIteration：刚体封装重写了task26_pose的xform op名 |
| dynamic04 | 30组，但几何无效 | 生命周期提前、TCP oracle改为PhysX tensor后FK一致，但封装仍将Cube变成单位scale；保留巨大Cube失败图 |
| dynamic05 | 30组、479 physics样本、8s动态 | scale修复后的诊断；缺少姿态oracle核验，不能作为最终桌面验证 |
| dynamic06 | 30组、480 physics样本、约8s动态 | 同步渲染诊断；未消除时间问题，亦缺少姿态核验 |
| dynamic07 | 30组、479 physics样本、8s动态 | 显式identity姿态、kinematic flag、oracle quaternion/USD matrix；去掉周期跳变。作为最终桌面验证 |

安装的XFormPrim `_set_xform_properties`只识别无后缀`xformOp:scale`，默认重排时忽略
`scale:task26_size`而加入单位scale；研究adapter用reset_xform_properties=False解决，
不修改旧Task26/27代码。失败01/04的全部raw留在本地，Git保留原图代表、日志、
轨迹、manifest和失败源码；02/03启动失败也保留。

初始刺激05/06包含运动/停顿/反向，x=.80→1.00→.84m；6s周期重启有40mm命令重置。
该人工不连续是probe限制，未从最大误差中删除，尤其影响同步组的39.33mm反例。
它不是可用于机器人执行的轨迹，也没有被宣称为物理接触实验。

## 最终桌面验证：dynamic07

最终复测去掉周期跳变，并显式保持姿态。479/479个oracle样本读回kinematic=true、
姿态角=0，z误差最大约1e-8m、USD scale=.12。只核验受控实验条件，不研究侧翻或
离桌恢复；05/06外观/姿态未充分核验的帧不混入本组。SDK时间与图像内容仍有差异，
因此这些条件核验不构成时间PASS。

30个共同callback RGB-D，18检出并被P1接受，双方可评分的相同集合17帧；旧request
早于history的首帧仍UNKNOWN。XY median/P95/max：旧request关联
**0.667/9.930/10.263mm**，SDK render关联 **0.667/3.593/4.013mm**。
旧时间关联造成的oracle XY差为4.999995/13.333321/13.333380mm。

request→SDK render均133.333ms；SDK render→consumer物理时间均33.333ms；
callback→arrival host median/P95/max=30.765/34.010/36.092ms，writer serialization
49.202/53.339/54.630ms。frame/time严格单调，缺时间字段0，history可求值30/30，
共同RP/reference元数据30/30一致；但实际测量时间仍全部30帧UNCERTAIN。

最终组独立FK对PhysX TCP位置差最大0.000436mm；478个实际history半tick查询
最大0.000386mm。真实图像仍全部落physics网格，无非零alpha图像时间；CPU插值
验证不能冒称实际异步曝光验证。RGB检测区域depth-plane残差median/P95/max=
1.471/1.565/1.565mm，配对身份仍受SDK不提供独立子帧号的限制。

最终4个可辨识运动样本的投影最佳时段宽度为一个tick；SDK时间距该时段均
**33.333ms**。这超过16.7ms初始目标，不用GT减去固定两tick来制造通过。

## 诊断对照：05/06，同图旧request vs SDK render

以下只保留诊断过程，不与最终07合并或当成已核验的桌面验证：

两组各30帧；每组16检出，15个P1接受，双方都能评分的相同集合14帧。
每组首个旧request早于history开始，明确1个UNKNOWN，未外推补齐。

| 指标 median / P95 / max | dynamic05 app.update | dynamic06 blocking render |
|---|---:|---:|
| request→SDK render | 133.33 / 133.33 / 133.33 ms | 166.67 / 166.67 / 166.67 ms |
| SDK render→consumer物理时间 | 33.33 / 33.33 / 33.33 ms | 0 / 0 / 0 ms |
| callback→arrival host | 29.92 / 31.18 / 31.89 ms | 24.69 / 34.20 / 37.26 ms |
| writer serialization | 48.83 / 50.80 / 53.62 ms | 49.23 / 50.91 / 51.01 ms |
| 旧时间关联的oracle XY差 | 13.33 / 13.33 / 13.33 mm | 16.67 / 16.67 / 40.00 mm |
| 同14图，视觉XY误差：旧request | 4.14 / 9.99 / 10.26 mm | 2.64 / 13.50 / 14.11 mm |
| 同14图，视觉XY误差：SDK render | 1.66 / 4.71 / 4.96 mm | 3.05 / 16.64 / 39.33 mm |

按SDK时间找最近physics sample及累计dt对core time的残差均0；frame/time都严格
单调，时间缺字段0，history可求值30/30。两组各30/30通过共同callback/RP/reference
元数据检查；RGB检测区域depth与已知marker平面的残差median/P95/max约
1.471/1.565/1.565mm。不过API不提供独立RGB/depth子帧号，这些一致性不是曝光证明。

独立URDF FK对PhysX TCP位置差：run05 median/P95/max=
0.000232/0.000362/0.000436mm；run06最大0.000541mm。
实际image时间在physics网格上，未需要非零alpha插值；另以真实history做
478/479个半tick CPU插值查询，FK对插值PhysX位置最大差0.000386/0.000484mm。
CPU单测覆盖非线性FK重算、过去离散状态、禁止外推、缓存淘汰及坏数据。

## 为何仍是PARTIAL

SDK时间落在physics网格是**元数据一致性**，不是独立成像验证。离线用固定K/T和
marker几何，将真实Cube轨迹投影到图像角点，寻找±0.101s内最匹配的场景时段；
不拟合offset，不回写预测。静止/多解时段不提供时间信息，诊断05/06排除这类时段后每组仅
7个可辨识运动样本（最匹配时段自身有一个tick宽度）。SDK时间距该时段的残差
median/P95/max两组均 **33.33/56.67/66.67ms**，超过一个tick。

投影诊断仍混有角点栅格化、标定误差和USD→PhysX→render状态传播影响，不能把
33ms直接认证为通用camera固定时延，更不能按GT减两tick来制造PASS。
同步渲染诊断组不仅未消除该差异，真实视觉评分还退化。最终07在去掉周期跳变、
补齐姿态核验后，4个可辨识样本仍存在33.333ms差异。因此当前既不能证明精确
render-time等于可见场景时刻，也不能独立证明RGB/depth每个子buffer的曝光身份。

目标C完成了有界、可复现的A/B和失败保留，但**精确动态时间对齐仍PARTIAL**。
停止P2A；P2B可以评审/重放该接口，在线融合需先完成实际4.5 RationalTime/render
状态绑定验证，或得到独立可验证的测量时间区间。继续只研究已落桌Stage B，
不把离桌、Stage-A、附着分类或技能切换增加为前置研究。

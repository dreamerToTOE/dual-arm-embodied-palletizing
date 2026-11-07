# R3-P0/P1 最终决定

日期：2026-11-07。工作分支 `research/stage-b-r3-multisensor-plan`；起点为用户指定
`8af7ee236f7b06afe893ca1efc7b0126b798f4c7`。改动全部在
`research/stage_b_multisensor_r3/`，不合并 main/side-suction-palletizing。

**总状态：PARTIAL。P0 PASS；P1 在桌面条件成立时的精度改善通过独立 A/B，
支撑状态和动态曝光同步仍有明确限制。停止于 P1。**

## 实现与证据

- `planar_pose_estimator.py`：V0 6DoF PnP；V1 固定 table z/roll/pitch，优化 x/y/yaw；
  对明显 lift/tilt、executor carry 矛盾、残差过大输出 UNKNOWN/UNCERTAIN。
- `scripts/audit_r3_p0.py`、`r3_p0_sensor_manifest.json`：实际 raw availability、
  field→producer→consumer、原始输入哈希、timestamps 及污染路径。
- `scripts/build_pose_dataset.py`、`evaluate_planar_pose.py`：观察值和 GT 分文件，
  先写预测再打开 oracle；相同输入/模型条件，对照 matched accepted frames。
- 冻结配置与 CPU tests、一次静态 Isaac capture、原始失败、代表反例、逐帧预测、
  投影/时序诊断都已保留，重放不依赖本地 6 GB 大图像目录。

## 关键数值（V0→V1）

| 独立组 / 相同接受帧 | XY median | XY P95 | XY max | gap abs P95 |
|---|---:|---:|---:|---:|
| 新静态：12 个有效桌面姿态 | 11.34→1.32 mm | 33.03→2.18 mm | 41.37→2.50 mm | 31.87→2.93 mm |
| physical06：271 个匹配桌面帧 | 22.76→2.18 mm | 39.13→4.48 mm | 64.81→13.67 mm | 37.33→3.29 mm |

静态 yaw median/P95=0.525/1.789°；physical06=0.650/2.167°。V1 CPU median/P95
约 3.63/4.70 ms（静态）和 4.19/5.39 ms（轨迹，已检出帧，不含图像检测/writer）。

新静态总 16 帧、15 检出；12 个桌面帧全检出接受，4 个无效支撑反例中 1 个小幅离桌
仍被接受。physical06 固定每三帧采样 498 receipt，294 检出，V1 接受 286；440 个
oracle 桌面帧接受 271，58 个非桌面帧中错误接受 15。拒绝/缺失和反例没有从报告中抹除。

## 已隔离的错误来源

旧 metadata 用 `fy=600`，实际 SDK 渲染模型为 `fy≈711.11`；首次旧 K 对照使 V1
变差，失败结果保留。独立 SDK 读回与投影确认，主 A/B 给 V0/V1 同时修正 K；
actual cell front=61.5 mm 也同时用于两组，没有通过 GT 逐帧调参。

旧 R2 runtime adapter 的 Cube GT velocity 混入 causal controller_reference，旧
visual.phase 由 GT 推断；R3 不复用这些字段。旧 Task27 控制器仍用 `/cube_poses` GT，
本研究没有把其行为冒称无 GT。cal24 的 5.01/10.86 mm 是 camera-center calibration
指标，未当作 Cube 在线误差。

## 验证和失败

- CPU：R3 新增 12 项与 R2 原有 24 项，共 **36 passed**；py_compile 通过。V0 与旧
  `estimatePoseSingleMarkers` 在相同角点/几何下的数值一致性也有断言。
- 隔离复制 compact 数据与代码后重放，两组独立留出的科学指标与记录完全一致
  （耗时除外）；不访问原始 RGB-D，回执为 `evidence/compact_replay_verification.json`。
- 相机几何、旋转/平移、scale 拒绝、lift/tilt、噪声、oracle 输入拒绝、hash tamper、
  matched A/B、真实小幅离桌反例均有测试或保留的失败记录。
- 新 Isaac：**1 次**，16 个 static scene pose，0 dynamic episode，0 全 Task27，
  0 控制写入，0 collision geometry 改动。完整 log 保留；关闭阶段 USD rotate reset
  warning 发生在 capture complete 之后，未用于遮掩失败。

## 推荐 P2 入口条件

1. V1 只作为“支撑条件下”的视觉校验；将 executor lifecycle 与独立支撑/运动证据
   引入 gating，优先解决 20 mm 小幅离桌仍能通过的反例，禁止用 GT support label 当输入。
2. 在短、有界观测中记录权威 render/frame id 与 q/dq/TCP/commands 时序，减少现有
   0.13–0.20 s receipt/render 歧义；不要把 writer 完成时间当曝光时间。
3. 独立验证 q→FK TCP、tool→Cube transform 和 attachment confidence；吸盘 closed
   或 FK 刚性假设不能自证无滑移。GT/PhysX 保持独立评分通道。
4. 仍保留历史 FCL/side LIFT 故障，P2 研究不等于 live switching 准入；本轮未实现
   FK fusion、PPO/SAC、live selection 或 controller redesign。

历史最终 3 mm 落位判据未改。静态部分误差接近该尺度，也不认证物理接触或最终落位。

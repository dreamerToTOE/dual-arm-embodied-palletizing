# R2-B 相机精度最终决策

日期：2026-11-06  
分支：`research/stage-b-r2b-camera-precision`  
实现提交：`915ddd4`

## 决策

**`R2B_CAMERA_PRECISION_PARTIAL`**

## 已接受

- 修复了镜像 marker raster 和缺失 quiet zone 两个零检测根因。
- 用四个合法 ArUco marker 替代单小平面 marker，保留固定 `rear_11` 相机。
- 将可见码面从 backing 平面显式建模到 `x=0.008 m`。
- 固定主点为图像中心后，从 8 个训练视图拟合焦距；4 个 holdout 视图仍独立评分。
- `cal24` 结果为 5.01 mm holdout 中位数、10.86 mm P95、0.35° 旋转中位数、1.08 px 重投影中位数。
- 新增 marker grid 自检；R2-B 测试套件 14 项通过。

## 不能宣称 PASS 的原因

静态 holdout P95 高于 3 mm，且本次没有验证真实 RGB-D 噪声、动态时间对齐、运动模糊、全任务遮挡、物理接触/吸附或 loaded placement。低重投影误差也不能替代这些证据。`cal25–cal27` 的布局诊断没有产生更好的可复现最终证据，因此不覆盖 `cal24`。

## 后续门

先完成非共面或更大基线外部标定和连续动态 holdout；随后把相机状态与机器人本体、吸附反馈和接触 oracle 对齐。通过这些门之前，视觉只能支持进度/粗定位研究，E2、RL、自动 skill switching 和 3 mm 最终放置验收继续暂停。

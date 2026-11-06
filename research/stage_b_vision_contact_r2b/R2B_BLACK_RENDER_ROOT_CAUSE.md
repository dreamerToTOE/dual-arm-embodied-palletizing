# R2-B 黑帧渲染根因与修复验收

日期：2026-11-06  
目标分支：`research/stage-b-r2b-runtime-integration`  
基线：`36f68d642b44aa485e8d59dca78d9135c8933355`

## 已测事实

旧探针 `scripts/r2b_cal24_transfer_probe.py` 的 12 张 PNG 都已写盘，尺寸为
1280×720、RGBA、alpha 恒为 255；RGB 最大值约 76–109，灰度均值中位数为
`0.397/255`，图像中只有暗色 viewport 网格，没有可见 Cube、桌面或 FR3。
因此这是渲染内容失败，不是 RGB writer 未完成。证据为：

`evidence/r2b_cal24_transfer_probe01/transfer_result.json`。

同一 Task27 场景、同一 RTX renderer 和同一 rear_11 视点的已成功图像
`evidence/r2b_rear_wall_physical07/scene_loaded.png` 能看见完整场景；cal24
校准图像也均为约 0.78–0.81 MB 的可见场景图。

## 根因判断

Task26/Task27 场景没有自带 Light Prim，RaytracedLighting 需要 Kit 默认视口灯光。
所有已知可见的采集路径都会执行：

```python
omni.kit.commands.execute(
    "SetLightingMenuModeCommand",
    lighting_mode="Default",
    usd_context_name="",
)
```

该调用存在于 `task28_task27_one_pass.py` 的 `enable_lighting()`、
`r2b_camera_observer_smoke.py`、`r2b_rear_wall_candidate_sweep.py` 和
`r2b_camera_precision_calibration.py`，但旧 transfer probe 完全缺失。旧 probe
的主动相机、相机位姿矩阵、写盘等待和文件尺寸均正常，所以当前最小且高置信的
根因是默认视口灯光没有初始化。修复实验仍需保存命令返回值并确认正向图像，不能
只凭代码推断通过。

## 修复验收条件

1. 在任何 ArUco 处理前保存一张不含 marker 的 Task27 场景 RGB 图，并用
   `view_image` 确认 Cube 可见。
2. 记录 active camera、`world_from_camera`、render-product path、renderer、
   lighting command result、physics timestamp、PNG 尺寸、通道及 RGB
   min/max/mean/std。
3. 确认 writer 已完成后再计算直方图；文件存在不等于图像内容有效。
4. 若使用 `isaacsim.sensors.camera.Camera.initialize()`，必须像已验证的
   `R2BPhysicalCamera` 一样在 initialize 后重新调用 `set_camera_view` 并重新
   记录外参，因为 initialize 可能重置相机 xform。

## 尚未通过的事项

本报告只确认旧探针的黑帧根因，不能把旧的 0/12 ArUco 检出解释为 marker 失败，
也不能证明修复后的 Cube marker 转移已经通过。marker 的父 Cube 缩放和物理可行性
必须在正向 RGB 之后单独验收。

## 正向验收结果

`evidence/r2b_cube_marker_transfer_probe03/scene_smoke_metadata.json` 已完成正向
验收：`SetLightingMenuModeCommand` 返回成功，active camera 为
`/World/R2B_RearWallCamera`，render product 已记录，PNG 为 1280×720 RGBA、alpha
有效，灰度均值 `83.03/255`，warm-up physics timestamp `7.292301 s`。因此黑帧
修复通过；后续仍需把标记几何/PnP 位姿指标单独判定，不能把可见性通过等同于定位通过。

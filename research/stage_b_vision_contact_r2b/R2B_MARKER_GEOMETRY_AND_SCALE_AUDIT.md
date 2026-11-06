# R2-B marker 几何与父缩放审计

日期：2026-11-06  
审计对象：`scripts/r2b_cal24_transfer_probe.py`  
基线：`36f68d642b44aa485e8d59dca78d9135c8933355`

## 已测不一致

`Cube_01` 的 USD 变换包含约 `0.12` 的父级 scale（见
`isaac/scripts/task26_truck_box_scene.py:_box()`）。transfer probe 的
`_make_board()` 已将 marker 的 Y/Z 偏移除以 `PARENT_SIZE_M`，因此这些偏移在
继承 Cube scale 后恢复为实际的 ±100 mm。相同函数的 marker cell 实际中心约为
Cube 中心 `+X=68.0 mm`，1.5 mm 厚 cell 的外表面约为 `+68.75 mm`。

但 `marker_corners_local()` 返回的是米制值，随后第 125 行又把这些值乘以含
`0.12` scale 的 `cube_world_from_local`。旧 manifest 因此记录了：

| 量 | 实际 authored 子几何 | 旧 manifest |
|---|---:|---:|
| marker front X offset | 约 68.75 mm（代码描述为 69.5 mm） | 8.34 mm |
| marker center Y offset | ±100 mm | ±12 mm |
| marker center Z offset | ±100 mm | ±12 mm |

例如旧 `manifest.json` 的 ID10 corner X 为 `0.50834 m`，而 authored cell
front 应接近 `0.56875 m`（Cube 中心 X=`0.500 m`）。这是坐标单位混用，不是
PnP 残差或可用任意偏移修复的问题。

## 物理可行性结论

四个 60 mm marker 的中心分布为 Y/Z 各 ±100 mm，整体约 260 mm；它不能作为
120 mm Cube 的直接 Cube-mounted cal24 board。cal24 四 marker 目标必须继续作为
独立外部相机标定靶。旧四-marker 原型的 backing 约 84 mm；当前 Cube 集成改用
单个 60 mm marker 和 72 mm backing（每侧 6 mm quiet border），并明确
marker-to-Cube 刚体变换。

旧四 marker probe 的 white backing 使用 USD Cube `size=1`、scale=`0.70*marker_size`，
实际全宽约 84 mm，不是 42 mm；其 quiet border 在几何上足够。当前紧凑 Cube marker
改为 72 mm backing，60 mm marker 两侧各 6 mm quiet border，也必须在正向灯光
渲染中复核。旧 `marker_corners_local()` 的注释称“half cell thickness”，却加了
完整 1.5 mm，所以历史模型的外表面还存在 0.75 mm 偏差；新模型应明确采用实际
cell front，避免再引入该偏差。

## 必须采用的验证方式

米制 marker geometry 与 USD normalized local geometry 必须分开。对每个 marker：

1. 从 authored cell/marker prim 的 `ComputeLocalToWorldTransform()` 提取实际四角；
2. 用独立的 marker-to-Cube 米制刚体变换计算期望四角；
3. 在无旋转、Cube 旋转和父 scale 场景分别比较，记录最大角点误差；
4. 误差超阈值时拒绝该 board，不能在评估结果中添加经验 offset。

该审计尚未证明 Cube marker 已在 RGB 中成功检测；旧 transfer probe 的 0/12 结果
发生在近黑场景中，不能作为几何检出结论。

## 紧凑 Cube marker 的复测

修正为单个顶面 ID23 marker 后，`r2b_cube_marker_transfer_probe01` 先证明了场景灯光和
几何一致性，但 0/6 检出；图像显示 white backing 与 cell 前表面共面，黑色 code 被
白 backing 遮住。将 backing 后移后，`r2b_cube_marker_transfer_probe03` 达到 6/6 检出，
几何最大误差 `1.53e-9 m`。这证明了渲染与 marker 几何链路已工作，但同一组静态
holdout 的 PnP 位姿误差中位数约 47.2 mm、P95 54.3 mm，不能当作 Cube 定位精度
通过；这组数字应保留为独立失败结果。

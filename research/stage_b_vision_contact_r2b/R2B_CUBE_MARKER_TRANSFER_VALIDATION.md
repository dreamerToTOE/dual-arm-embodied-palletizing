# R2-B Cube marker 静态转移验证

日期：2026-11-06；分支：`research/stage-b-r2b-runtime-integration`。

cal24 四 marker、260 mm 目标仍是外部标定靶，不能挂到 120 mm Cube 上。本次
Cube 集成使用顶面单个 ID23、60 mm marker 和 72 mm 白色 quiet backing；marker
front offset 为 61 mm。`r2b_cube_marker_transfer_probe03` 固定
`/World/R2B_RearWallCamera`、读取 cal24 K，使用六个静态 teleported holdout 视图，
没有运行 Task27 操作，也没有写 controller。

## 结果

| 指标 | 结果 |
|---|---:|
| RGB 正向烟雾 | 通过；1280×720 RGBA，灰度均值 83.03/255 |
| marker 检出 | 6/6（100%） |
| authored/expected 角点最大误差 | 1.53e-9 m |
| 重投影误差中位数 | 0.307 px |
| Cube 位置误差中位数 / P95 | 47.2 mm / 54.3 mm |
| Cube 姿态误差中位数 | 1.65° |

RGB 和 USD 几何链路通过；位姿精度失败。单平面顶面 marker 在 0.93–1.18 m
距离、较大俯视角下出现深度歧义，低重投影误差不能替代 Cube 位姿误差。因静态
定位未通过 3 mm 级控制门，本轮不做动态短 episode，也不把结果写成 cal24→Cube
transfer PASS。

证据：`evidence/r2b_cube_marker_transfer_probe03/manifest.json`、
`transfer_result.json`、`scene_smoke_metadata.json`。旧近黑场景和 backing
z-fight 运行保留为历史诊断，最终指标只采用 probe03。

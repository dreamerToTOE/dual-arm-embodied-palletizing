# R2-B 相机多帧校准验证

## 方法

使用 static sweep04 的 12 个有效多姿态渲染视图。6 个视图作为训练集，`rear_02/rear_03/rear_08/rear_12/rear_14/rear_17` 作为未参与拟合的静态 holdout；physical05 的 55 帧作为独立动态 holdout。拟合只估计同一渲染产品的有效主点，保留 authored `fx=711.111`、`fy=600`；平面单 marker 不能同时独立识别焦距和主点。`T_world_camera`、marker-to-Cube 仍保留 authored 值。

## 结果

| 数据 | 帧数 | 重投影误差 | Cube 位置误差 |
|---|---:|---:|---:|
| authored cy=360，physical05 | 55 | — | 43.1 mm |
| 旧离线 cy=341，physical05 | 55 | — | 37.0 mm |
| 本次训练拟合 cy=340.49，static train | 6 | 0.290 px | 45.2 mm |
| 本次拟合，static holdout | 6 | 0.356 px | 55.1 mm |
| 本次拟合，physical05 holdout | 55 | 0.225 px | 37.4 mm |

静态 holdout 依视角/深度变化明显：z=.80 m 的四个 holdout 约 `17–99 mm`，z=1.20 m 的两个 holdout 约 `61–96 mm`。这说明低重投影误差并不等于稳定的绝对世界位姿。

## 审计结论

本次验证是独立于 physical05 末端帧的多帧/多姿态主点验证，但不能称 calibration PASS：残余误差仍为厘米级，外参和 marker-to-Cube 仍未由外部标定目标独立验证，且训练/holdout 只覆盖同一模拟板。没有引入世界坐标固定 offset，也没有把 `cy=340.49` 写入运行时估计器。

最小后续需求是同一渲染产品下的非共面或多 marker 标定板，联合验证 K、畸变、相机外参和 marker-to-Cube 旋转；在此之前视觉状态不足以支持 3 mm 最终放置验收。

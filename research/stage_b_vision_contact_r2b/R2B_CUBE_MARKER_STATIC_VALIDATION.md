# R2-B Cube marker 静态验证

本任务只新增一次 bounded Isaac 静态实验：`r2b_cube_marker_90mm_probe01`。
marker 为顶面 90 mm ID23，108 mm quiet backing，完整 backing 仍在 120 mm Cube
面内，没有改 Cube collision geometry，也没有运行 Task27 动态操作。

| 配置 | 检出 | 位移中位数 / P95 | 姿态中位数 | 重投影中位数 |
|---|---:|---:|---:|---:|
| 60 mm（既有 probe03） | 6/6 | 47.2 / 54.3 mm | 1.65° | 0.307 px |
| 90 mm（本任务） | 5/6 | 24.2 / 36.9 mm | 1.32° | 0.417 px |

90 mm 的像素条件改善了位姿误差，但 deep-wall-arrival 视图被容器前壁部分遮挡，
检出率降为 83.3%，且仍不能支持精细接触决定。80 mm 的几何余量可行，但没有在
本任务中额外消耗 Isaac 运行；它只能作为待验证的中间候选，不能被写成已通过。

因此静态定位仍不满足决策所需精度，停止在此，不运行动态实验，不认证 physical
contact 或最终 3 mm placement。

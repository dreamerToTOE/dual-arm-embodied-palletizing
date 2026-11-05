# R2-B V0/V1/V2 对比

| 版本 | 输入 | 帧/可用率 | 结果 |
|---|---|---:|---|
| V0 | 既有 R2 trace 的 oracle pose（审计基线） | 15,688 / 100% | 平均位置误差 0、延迟 0；不是可部署感知器 |
| V1 | 固定 seed=20261105 的噪声/延迟/丢帧 proxy | 15,688 / 90.06% | σ=1 mm，2 帧=33.3 ms，平均误差 1.68 mm，P95 3.00 mm |
| V2 | Isaac 实际 RGB 图像 + OpenCV ArUco/PnP，另存实际深度 | 1 / 100% | 重投影 0.066 px，confidence 0.992，评估平移误差 13.4 mm |

V0/V1 来自既有 R2 单 episode 离线 trace；V2 是真实渲染相机的单帧 qualification，不是新的 manipulation episode。V2 仍需多帧、遮挡、光照和真实 command 时间戳验证后，才能作为在线控制观测。

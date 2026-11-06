# R2-B 单 marker 位姿误差根因

分支：`research/stage-b-r2b-pose-diagnosis`；基线：`4e040d9ea21281762cf29499278cb57aac96e15e`。
证据为已有 `r2b_cube_marker_transfer_probe03` 六张 RGB 图，没有重新使用 Cube
真值修正在线估计。

六张图的 USD expected corners 用固定 rear_11 相机和 cal24 K 投影后，与检测角点的
平均残差为 1.19–2.48 px，最大残差为 1.93–3.98 px；marker 宽约 20.5–35.4 px，
高约 32.6–42.6 px，入射角约 56.9–64.7°。检测中心存在持续的负 x 偏差，且角点
量化在小 marker 上占据较大比例。

坐标审计显示 USD camera→OpenCV 使用列基 `A=[M0,-M1,-M2]`，`det(A)=1`；
marker corner 顺序为 TL/TR/BR/BL，Cube→marker offset 为 61 mm。用合成刚体
变换做 round-trip 后，投影和 PnP 变换一致。marker authored geometry 与 manifest
误差仍为约 `1.53e-9 m`，所以原先的 double-scale 已排除。

根因是小型、强倾斜的共面方形在当前图像分辨率下的平面姿态条件不足：1–3 px 的
角点误差会改变 marker 法向，随后把相机中心横向偏移数厘米。低 reprojection error
不能证明三维 Cube pose 正确。没有证据支持把错误归因于 Cube marker placement 或
单纯矩阵乘法错误。

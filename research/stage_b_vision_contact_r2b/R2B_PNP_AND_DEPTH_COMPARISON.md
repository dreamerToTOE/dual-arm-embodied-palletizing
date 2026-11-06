# R2-B PnP 与深度比较

离线脚本：`scripts/diagnose_single_marker_pose.py`；输出：
`evidence/r2b_cube_marker_transfer_probe03/pose_diagnosis.json`。

| 方法 | 位移中位数 | 位移 P95 | 姿态中位数 | 重投影中位数 |
|---|---:|---:|---:|---:|
| ITERATIVE | 47.2 mm | 54.3 mm | 1.65° | 0.307 px |
| IPPE_SQUARE | 51.2 mm | 101.7 mm | 1.78° | 0.308 px |
| IPPE 两假设 | 80.4 mm | 122.0 mm | 58.3° | 0.473 px |
| SQPNP | 121.0 mm | 187.9 mm | 124.3° | 4.914 px |

IPPE 没有改善第一假设，第二假设明显错误；不能用 ground truth 选择正确假设。
ITERATIVE 在 ±0.25 px 角点扰动下的单视图位移 P95 最高约 118 mm，说明误差主要
来自像素/平面条件，而非可用的 PnP 算法替换。

probe03 只保存 RGB，没有同帧 depth。physical07 的 depth 属于另一段 legacy
episode，且没有 exposure-level synchronization 证明，未用于融合或假设选择。因而
本任务没有伪造 PnP-depth 对比，也没有改变在线 K 或 pose estimator。

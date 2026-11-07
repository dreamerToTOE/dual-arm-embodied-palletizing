# R3-P1 平面约束 Cube 定位 A/B

## 冻结模型与独立性

V0：同一 marker corners，OpenCV SOLVEPNP_ITERATIVE 的无约束 6DoF，再用固定
marker→Cube 几何转换。V1：仅在桌面假设相容时固定 z=0.260 m、roll=pitch=0，
以 Huber 重投影残差优化 x/y/yaw。无 FK、上一帧或 depth 融合。

两组完全共用：60 mm 顶面 ID23、61.5 mm front offset、120 mm Cube、table=0.200 m、
rear_11 T，以及 SDK 有效 K：fx=fy=711.1111、cx=640、cy=360、畸变=0。没有从评估
Cube GT 拟合 K/T、选择 PnP 假设或增加位姿 offset。

冻结配置 `config/p1_frozen.json` SHA-256：
`165645241bffa2664b61a2394e9d5503ea13f22781500cf93069c91c2ea6b54b`。
在查看 physical06 和新静态留出结果前冻结；physical05/07 是已看过的开发诊断，
不冒称独立验证。没有把同一轨迹邻帧随机拆分训练/测试；本模型没有拟合训练参数。

V1 门：executor 明确 carry/placement 矛盾则拒绝；无约束视觉给出的高度偏差超过
半个 Cube（60 mm）、tilt>10° 或平面残差>2 px 则 UNKNOWN/UNCERTAIN。这些门只检测
可见矛盾。通过表示 **plane-compatible estimate**，不证明物体受到桌面支撑。
局部 Jacobian/至少 1 px 噪声给出 sigma，未包含标定、模型、时间系统偏差，不当作
已校准误差界。小幅离桌存在不可辨识反例，详见下文。

## 首次失败与有界修复

首次沿用旧 metadata fy=600 时，physical05 的 V0 XY median=42.30 mm，V1=88.80 mm，
平面约束反而变差。失败输出保留于 `evidence/diagnostic_authored_k/ab_failure.json`。
没有为改善指标事后收紧拒绝阈值。

检查安装的 Isaac 4.5 Camera 源码，确认 `get_vertical_aperture` 根据 horizontal
aperture×height/width 计算，`get_intrinsics_matrix` 对应 square pixels。一次有界
静态运行读回 K=711.111084（float32）同时保留 USD authored aperture=36/24。
静态 holdout 的实际 authored cell corners 投影平均残差从旧 K 的 5.43 px 降至
SDK K 的 0.974 px；角点几何误差 1.54e-9 m。该检查仅离线评分，没有用这些点拟合 K。
旧 cached corners 从原始图像抽查重检 10/10 一致。

## 独立留出结果

### A：新静态场景组 `r3_static_holdout01`

一次 Isaac 运行，16 个姿态：12 个桌面 recontact/entry/progress/deep-wall/side
preparation 姿态，x/y/yaw 与旧六张诊断图不同；4 个 lift/roll/pitch/小幅离桌反例。
均为时间停止后的 static scene pose capture，无运动控制写入，不是动态 manipulation。
Cube/table/墙/tool collision geometry 未修改。

| 指标（12 个桌面帧，相同接受集合） | V0 | V1 |
|---|---:|---:|
| XY median / P95 / max | 11.34 / 33.03 / 41.37 mm | **1.32 / 2.18 / 2.50 mm** |
| yaw median / P95 | 0.516 / 2.070° | 0.525 / 1.789° |
| deep-wall gap abs median / P95 | 10.56 / 31.87 mm | **1.79 / 2.93 mm** |
| x/y 绝对误差 P95 | 31.35 / 12.32 mm | 2.06 / 1.02 mm |
| reprojection median | 0.289 px | 0.638 px |
| estimator CPU median / P95 | 0.313 / 0.369 ms | 3.63 / 4.70 ms |

桌面 12/12 检出并接受；全部场景检出 15/16。V1 全部接受 13/16，4 个无效支撑反例
中 1 个未检出、2 个被视觉门拒绝、**20 mm 小幅离桌被错误接受**。该例说明低
reprojection 不能证明桌面支撑。没有将其从失败率删除。

### B：独立历史轨迹 `physical06_holdout`

按固定 stride=3 取 498/1492 个 receipt，覆盖整个 episode，不按误差挑帧；294 检出
（59.04%），其余 204 是 detector/capture 失效。用 GT 仅在评分侧划分桌面标签：
z 距 0.260 m≤8 mm 且 tilt≤2°，共 440 帧；该判据不进入推断。

| 指标（桌面、V1 接受 271 帧） | V0 同帧 | V1 同帧 |
|---|---:|---:|
| XY median / P95 / max | 22.76 / 39.13 / 64.81 mm | **2.18 / 4.48 / 13.67 mm** |
| yaw median / P95（相同帧） | 0.601 / 2.757° | 0.650 / 2.167° |
| gap abs median / P95（相同帧） | 18.78 / 37.33 mm | **1.76 / 3.29 mm** |
| x/y 绝对误差 P95（V1） | — | 4.27 / 1.43 mm |
| estimator CPU median / P95 | 0.334 / 0.425 ms | 4.19 / 5.39 ms |

V0 全部桌面接受 278/440，V1 271/440；V1 桌面 missing/rejection=38.41%，其中多数
是未检出。V1 全 498 帧接受 286，rejection/missing=42.57%。58 个 oracle 非桌面
帧中 **15 个被错误接受**，主要为 lift/drop 过渡的 10–64 mm 高度偏差。该历史
运行缺少可靠 executor phase receipts，V1 没有用 GT 排除这些帧。

`evidence/p1_holdout_results.json` 保留各自接受集合与 matched acceptance 的全部
XY/yaw/gap/max/拒绝指标，避免通过拒绝困难帧夸大改善。表中误差使用 matched 集合；
每组各自全部接受帧的完整指标也保留在 JSON。V0/V1 的 K 修复同时进行，主对照没有内参混用。

## 动态时间与不确定性

physical06 receipt→render-wait=0.1333 s；writer latency median/P95/max 为
0.168/0.221/0.454 s。GT 在 render-wait 内位移 P95=4.75 mm、max=17.91 mm。
图像实际曝光时间不明，既不能指定为起点也不能指定为终点，因此动态误差包含
潜在关联偏差，没有用 GT 位移补偿估计。physical07 的对应位移 max=18.82 mm，
writer max=0.731 s，故不把毫秒级 CPU 求解称为端到端闭环时延。

静态 V1 局部 sigma median≈1.54 mm，动态≈1.85 mm，但观测到的 max 更大，证明
局部协方差不是完整置信界。没有模拟真实 RGB-D 噪声或宣称曝光级同步。

## 结论与边界

**P1：PARTIAL，平面支撑条件成立时精度改善已被独立验证。** 更低 reprojection
不是目标：V1 重投影残差稍大但 XY/gap 明显改善。支撑状态检验仍有反例，不允许
把所有 V1 接受帧当作桌面支撑已证实。

决策需求应分开：厘米级 gross re-localization/较大 clearance 可以参考 V1 和
不确定性；几毫米墙 gap 附近应输出 unresolved/NEAR 候选；加载接触需要独立
力/运动/吸附证据。历史最终落位 3 mm 判据保持原样，静态部分数值接近 3 mm
不构成 final placement 或 physical contact 认证。本轮停止于 P1。

新 Isaac 次数：**1**（16 静态姿态，0 动态 episode，0 全 Task27 复跑）。退出码 0，
manifest 完整；关闭时出现 rotate-op reset 的 USD type warning，保留完整 isaac.log，
该 warning 发生在 `R3_STATIC_CAPTURE_COMPLETE` 后，不据此隐藏或提升结果。

# R2-B 单 marker 位姿诊断最终决定

最终状态：**R2B_POSE_DIAGNOSIS_PARTIAL**。

交付分支：`research/stage-b-r2b-pose-diagnosis`；交付 SHA：
`3fa69d4d3b0eadeb8c62f1346ed09704d85cee25`。

已建立可信根因：60 mm 顶面共面 marker 在 rear_11 的 56.9–64.7° 入射角和约
20–42 px 边长下，对 1–3 px 角点误差高度敏感；ITERATIVE、IPPE 和 SQPnP 均不能
消除该偏差。USD 几何、相机 frame、corner order 和矩阵 round-trip 已通过 CPU
回归测试。

90 mm / 108 mm 顶面候选在一次新静态 Isaac 运行中把位移中位数从 47.2 mm 降至
24.2 mm，但深墙到达视图出现遮挡，5/6 检出，P95 仍为 36.9 mm。因此定位问题
尚未修复到可用于精细 Stage B 几何决定的程度，不能启动动态实验或宣称 pose PASS。

shadow skill gate 已通过回归：旧的 172 个非插入 CONTINUE_PUSH 提议降为 0；
缺失 attachment confidence 会安全保持 HOLD_UNKNOWN。该安全修复不代表在线
mode switching 已启用。

当前剩余阻塞：需要非共面/更正视的物理可行标记布局，或同帧、具曝光级同步的
depth/运动先验，随后重新做独立静态 holdout。保持 FCL、joint/torque limits、
suction assumptions、oracle separation；不启动 E2、RL、GNN、live switching 或
full recovery。

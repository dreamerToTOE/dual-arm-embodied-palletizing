# R2-B 技能切换准备度

状态：**INCONCLUSIVE / NOT READY FOR PHYSICAL RECOVERY**。

已具备：统一观测帧、oracle 与 estimator 输入隔离、实际 RGB-D 相机、实际可见 ArUco、OpenCV PnP、E0/E1 多标签接口、E2 的遮挡 prediction-only 路径。

最小缺口：

1. 在真实 Task27 command 流中写入同一 physics step 的 command id、stamp 和 commanded advancement；
2. 运行多帧 V2 操作轨迹，记录可见性、遮挡、延迟和相机外参误差；
3. 在主会话确认接触/力观测可解释后，才执行 bounded nonpenetrating variation 和可执行 recovery skill。

在缺少第 1–3 项时，不得声称视觉状态估计驱动了真实恢复，也不得将 E2 选为已验证的控制切换器。

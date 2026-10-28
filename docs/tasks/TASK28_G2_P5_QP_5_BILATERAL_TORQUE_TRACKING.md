# Task28 G2-P5-QP-5：双臂显式 effort 参考跟踪

**最终状态：`G2_P5_QP_5_BILATERAL_HOLD_ONLY`。** 该状态只表示两套 FR3 articulation 在隔离 Isaac 场景中通过了预注册的静态 effort 保持；五段 QP-4 参考的移动跟踪未完成。没有 Cube、吸盘、D6、桌面接触、ROS、生产 bridge 或物理机器人执行。

## 范围和冻结输入

实验使用冻结 USD（SHA-256 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`）、左右基座 `[0.65,-0.6,0]` 与 `[0.65,0.6,0]`、QP-4 归档的初始 `q14` 和物体姿态。每个 articulation 的实际 DOF 名称和顺序均为七个 `fr3_joint1..7` 后接两个 `fr3_finger_joint1..2`，索引为 `0..8`；两侧均设置 `force` 模式和零 implicit stiffness/damping。手指使用完整九维 gravity+PD 保持（目标 `[0.04,0.04] m`），没有把七维 effort 用零值填充。

预注册参数包括 physics `dt=1/120 s`、静态 240 步、五个 20 ms 区间和 60 步最终保持、R3 的 `kp=[4,4,4,4,3,3,2,20,20]`、`kd=[1,1,1,1,0.8,0.8,0.5,2,2]`、effort bounds `[60]*7+[10,10]` 及逐步 rate bounds `[100]*7+[10,10]`。G1 闭链门限保持平移 `1e-4 m`、旋转 `1e-3 rad`，没有放宽。

## 参考语义

QP-4 的 `q_next` 是区间终点，`qdot` 是区间起点速度；采样器首段用 `q_next-0.02*qdot` 恢复起点，后续用上一段终点，区间内采用

\[
q_{ref}(t)=q_{start,k}+(t-t_{start,k})qdot_k .
\]

静态门禁结束后，移动阶段重新建立零点，只用于这段合成的 0.1 s 参考游标；实际 Isaac physics step、绝对时间、左右同一步读取仍全部记录。物体姿态只作为虚拟参考，以归档端点作确定性 SE(3) 插值，没有根据实际 TCP 反拟合。

## 结果

正式运行目录：`offline/task28_p5_qp_5/evidence/formal_run_v1/`。完整采样 315 physics steps，静态 240 帧、移动/最终保持 75 帧，wall time 约 15.9 s。

静态保持通过：

- 最大闭链平移残差：`1.9321745339e-7 m`；最大旋转残差：`1.8506117311e-6 rad`。
- 最大 live link7 FK 误差：`4.7369407091e-7 m`、`8.1799150953e-7 rad`。
- 最大 commanded/applied effort readback 差：`9.4814176776e-7`。
- 位置、速度、限位、九维 effort 形状和时间记录门均通过。

移动阶段在零基索引 2（第三个记录帧）停止接受：最大记录闭链平移 `9.6898547409e-5 m`，但左侧抓取平移门 `LEFT_GRASP_TRANSLATION_MISMATCH` 触发，故不把该路径标为跟踪通过，也没有继续放宽门限或重投影。该结果说明静态保持成立，不能证明五段参考的闭链运动可行。

四次开发/预注册前失败运行均保存在 `evidence/failed_runs/`：第一次是新增采集器错误地复用左臂 q，第二次暴露未受控初始化步造成的闭链旋转超限，第三次暴露首段 q_next 误采样，第四次暴露绝对 physics 时间与移动参考零点混用。所有失败原始帧和状态文件均保留；最终正式运行没有覆盖它们。

负控包含错误 DOF 映射、异步/乱序参考、无效/陈旧状态、错误 Cube ID 和非支持 acceleration，均由独立门函数 fail-closed 拒绝。机器可读汇总和源文件哈希在 `offline/task28_p5_qp_5/evidence/summary.json` 与 `source_hashes.json`。

## 回归与复现

- 历史回归：`789 passed`，日志 `evidence/regression_historical_789.log`。
- QP-4：`53 passed`，日志 `evidence/regression_qp4_53.log`。
- QP-5：`5 passed`，日志 `evidence/regression_qp5_5.log`。

Isaac 命令、环境、预注册哈希、资产哈希、runtime DOF 身份、physics step 和输出哈希均见 `summary.json`。这些是 CPU/Isaac 隔离实验记录，不是 ROS servo 周期、实际 effort 传感器、吸附力或实体 Cube 运输测量。

## 结论和剩余限制

本任务没有修改 QP-4、G1 验证器、冻结 fixture、ACM 或历史任务。当前只可报告 `G2_P5_QP_5_BILATERAL_HOLD_ONLY`。在获得 review 前，不得把该结果升级为双臂移动候选通过。

后续若获授权，必须先分析第 2 个移动采样点的抓取几何残差，确认时间参考、姿态插值和 effort readback 语义，再决定是否设计新的预注册实验。仍缺少实际 Cube/吸盘接触、有限吸力、内部力反馈、碰撞梯度/连续安全证书、ROS bridge 时序、实机 effort 标定及执行授权。

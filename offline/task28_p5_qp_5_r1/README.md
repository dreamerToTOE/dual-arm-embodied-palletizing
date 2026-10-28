# Task28 G2-P5-QP-5-R1

R1 是 QP-5 的独立时间契约和 fail-closed 诊断层。原 QP-5 目录和证据只读保留。

- `forensic.py`：读取原始 75 个移动帧，比较原采集器混合时刻、同一时刻和 post-step 下一时刻假设。
- `collector.py`：控制器在 pre-step 同时读取两臂并采样关节/虚拟物体参考，推进一个 physics step 后，以 post-step 时刻重新计算参考并立即执行门禁。
- `runtime_gate.py`：检查源身份、时间、DOF、q/dq、effort/rate、readback、FK、完整 G1 闭链和跟踪门。
- `evaluate.py`：独立复核正式输出及“首失败后无后续 physics step”。

正式运行前固定参数见 `preregistration.json`。运行失败帧和启动失败均保存在 `evidence/`，没有覆盖 QP-5 原始证据。

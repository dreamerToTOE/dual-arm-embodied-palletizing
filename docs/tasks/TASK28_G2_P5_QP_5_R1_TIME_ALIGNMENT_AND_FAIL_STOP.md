# Task28 G2-P5-QP-5-R1：时间对齐与 fail-closed 验证

**最终状态：`G2_P5_QP_5_R1_FAIL_CLOSED_DIAGNOSTIC_PASS`。** R1 成功验证了时间契约、独立复算和首个门禁失败后的运行时停止；修正后的五段 QP-4 参考仍被实际几何门拒绝。因此这不是移动跟踪通过，也不代表实体 Cube、吸附、连续安全或实机执行。

## 归档取证

原 QP-5 正式移动证据 `offline/task28_p5_qp_5/evidence/formal_run_v1/tracking_observations.json` 的 75 帧未被修改。原采集器的关节参考时间是 `max(0,t-dt)`，物体参考时间是 `t`，且记录的是 physics step 前状态；原采集器在离线评估前完成全部 75 个移动/最终保持步。

R1 独立复算在原始 TCP 上得到：

- 帧索引 2、`t=0.0166666675359 s`：左绝对抓取平移 `0.1533140925 mm`，右绝对抓取平移 `0.0823962569 mm`，左右相对平移 `0.0968985474 mm`。因此左绝对门（0.100 mm）先失败，而相对门当时仍通过。
- 帧索引 3、`t=0.0250000013 s`：左右相对平移达到 `0.1693979726 mm`，相对门随后失败。
- 同一时刻插值的 QP 参考本身仍保持很小的数学闭链残差；R1 没有用回溯时间偏移替换实际结果。

每帧的实际 q/dq、原始/匹配时刻参考、物体时刻、TCP、绝对/相对闭链、跟踪、饱和和 rate 状态保存在 `evidence/forensic_replay/replay.json`。

## 修正后的时间契约

R1 控制器在 pre-step 时刻 `t` 同时读取左右状态，并以同一个游标采样两臂 q/qdot 和虚拟物体 SE(3) 参考，计算完整九维 gravity+PD effort 后推进一个 Isaac physics step。post-step 状态以实际 `t+dt` 时间戳记录，并与 `t+dt` 的参考比较。QP-4 的五个 20 ms 区间及其 interval-start `qdot` 保持不变；没有假设三步 physics 等于一个 20 ms 区间，也没有生成 qdd。移动游标在静态门禁结束时从零开始，最终保持只有在整个 0.1 s 移动段通过后才允许进入。

静态阶段每步都检查；移动阶段首个失败后抛出显式 `REJECTED`，不发送零 effort 或 hold-last fallback，也不再执行后续 physics step。正式运行的 post-step 状态和 gate 结果由独立 `evaluate.py` 复核。

## R1 正式运行

正式输出位于 `offline/task28_p5_qp_5_r1/evidence/formal_run_v1/`。使用与 QP-5 相同的 FR3 USD、初始 q14/物体姿态、R3 增益、effort/rate/speed 限制和冻结 G1 门限；没有 Cube、吸盘、D6、桌面、ROS 或 production bridge。

- 静态保持：240 步全部通过。
- 移动：仅执行 2 个 physics step；post-step `t=0.0166666675359 s` 首次拒绝。
- 首失败：`LEFT_ABSOLUTE_TRANSLATION`。
- 首失败残差：左绝对平移 `0.1540633891 mm`，右绝对平移 `0.0821964790 mm`，左右相对平移 `0.0982531125 mm`；相对 0.100 mm 门尚未失败。
- 首失败后：未执行剩余 10 个移动 step，未执行 60 步 final hold；无零 effort fallback。
- 独立评估确认：`continued_after_first_failure=false`。

这说明时间对齐和 fail-closed 机制已建立，但不支持把原始参考称为可移动执行轨迹。失败更接近实际几何/控制跟踪限制，而不是原始混合时间假设单独造成的结果。

## 负控与测试

R1 focused tests 覆盖同一时刻 q/物体契约、post-step dt、20 ms 与 1/120 s 不可整数替换、DOF/侧身份、时钟、闭链、跟踪、effort 饱和/rate、首失败后禁止继续和 qdd 不可用。负控结果在 `summary.json`。

本次回归：历史 789、QP-4 53、QP-5 5、R1 6 均通过。正式和开发启动失败的 console/status 文件均归档，源文件与 artifact SHA 位于 `evidence/source_hashes.json` 和 `evidence/summary.json`。

## 限制和结论

R1 仍是隔离 Isaac 虚拟几何实验。它没有测量实际吸力、D6 反力、Cube 接触、内部力、ROS 时序、实机 effort 或连续碰撞安全，也没有修改 QP-4、QP-5 原始实现、G1 门限、fixture、ACM 或历史证据。

因此本任务只报告 `G2_P5_QP_5_R1_FAIL_CLOSED_DIAGNOSTIC_PASS`，停止在 R1，不自动进入后续任务。

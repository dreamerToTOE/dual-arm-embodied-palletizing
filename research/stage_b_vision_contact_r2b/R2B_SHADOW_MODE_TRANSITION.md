# R2-B Shadow mode 选择


日期：2026-11-06  
研究分支：`research/stage-b-r2b-contact-recognition`  
校准父分支：`research/stage-b-r2b-camera-precision`  
校准实现提交：`915ddd44e4067c8687c0542e297b24c03485fa42`  
校准报告提交：`f374d75fb2ee7d6c008c89c1123541cbda860edc`  

## 实现

只读 selector 位于 `research/stage_b_vision_contact_r2b/shadow/mode_selector.py`，输出：

- `CONTINUE_PUSH`
- `REQUEST_DEEP_WALL_CHECK`
- `SIDE_APPROACH_READY`
- `HOLD_UNKNOWN`

策略只读取 `ContactEstimate`、不确定性和可选的 `side_preflight_passed` receipt。`UNKNOWN` 或 uncertainty ≥ 0.65 一律 `HOLD_UNKNOWN`；side contact 没有显式 preflight 也保持 HOLD；deep-wall possible 只请求检查，不直接切换控制器。输出记录 supporting state/edges、confidence、时间戳、当前 supervisor mode（若有）及差异，但不发命令。

## 安全边界

代码没有 motion write、controller ownership change、FCL bypass、torque/joint-limit relaxation、suction override 或 side-compaction trigger。没有实现 live automatic skill switching。此前 physical07 的 side compaction lift joint-tracking failure 和 FCL/preflight 问题仍视为未解决。

## 实际 shadow 运行

在 physical07 的 1550 帧上，q/dq、TCP、commands 和 suction receipts 缺失；adapter 按契约产生 UNKNOWN/缺失本体字段，selector 因此输出：

- `HOLD_UNKNOWN`: 1550/1550
- 其他模式：0

这是预期的安全拒绝，不能解释成 selector 发现了 1550 次失败接触。当前证据没有逐帧 supervisor snapshot，所以不能给出可信的 supervisor transition confusion；selector 的 stale rejection 和 side preflight rejection 由单元测试覆盖。

## 决策

该组件具备 shadow-only 的接口和拒绝边界，但尚未满足“在独立真实接触标签上评估模式转换”的门。下一步必须先接入同时间轴 q/dq/TCP/commands/suction，再用独立加载接触标签比较提议模式、延迟和 premature transition；在此之前保持只读。

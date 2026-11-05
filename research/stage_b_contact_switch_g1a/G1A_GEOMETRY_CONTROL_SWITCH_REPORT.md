# G1-A 几何触发控制切换报告

## 结论

`G1A_PROXY_PASS`，`G1A_FR3_PASS` 不成立。三种候选控制器在 Isaac Sim 最小 Cube/Table/DeepWall 场景中都实际施加了 PhysX 物理命令并改变了 Cube 运动；C1 和 C2 的模式切换由几何墙间隙与 Cube 速度触发，未读取墙力。三种条件各有三次最终固定协议重复，结果完全一致。

这是一个**物理 proxy 实验**，不是双 FR3 控制。C0 在当前 proxy 中用预定义固定目标和恒定 +8 N advancement command 代表固定推进基线；它不是 FR3 position servo，因此结果只对该 proxy 控制接口负责。它证明了控制命令确实改变了代理对象的运动和命令时序，不证明机器人关节控制、混合力/位置控制优越性、吸盘完整性或 jam recovery。

## 实验实现

脚本 [`scripts/g1a_geometry_switch.py`](scripts/g1a_geometry_switch.py) 在新 Isaac Sim 场景中创建动态 Cube、原始尺寸桌面和深墙。播放后只通过 dynamic-control `apply_body_force` 施加世界 +X 命令，没有设置 Cube pose 或 velocity。每步记录：

- target center、实际 Cube pose/velocity、墙间隙；
- active command 与下一物理步 command；
- controller mode、切换步/时间、命令不连续度；
- Cube/Wall pair、原始事件冲量、矩阵力、睡眠状态；
- 修正动量 ledger、最终间隙、是否建立 loaded contact。

控制器只读取 `cube_wall_gap_m` 和 Cube `velocity_world_m_s`：

- **C0**：预定义固定几何目标 `x=0.63 m` 的恒定推进 proxy（`+8 N`），不切换。
- **C1**：当间隙 `≤3 mm` 且运动状态满足门槛时切换到 `GEOMETRY_HOLD`，停止推进命令。
- **C2**：当间隙 `≤10 mm` 时进入 `BLEND`，12 个物理步将推进命令从 `+8 N` 连续降到 0，再进入 `REDUCED_HOLD`。

墙力只作诊断记录，不参与 mode switch；本实验没有调用 force feedback。

冻结配置：[`configs/g1a_proxy_protocol.json`](configs/g1a_proxy_protocol.json)，SHA-256：`acafa28d8a96a9cfb6dffd9b5aa1f4ec566c723644f52a41d703dcee7cba3ef1`。

## 匹配重复结果

最终比较使用 `g1a_C{0,1,2}_final_{03,04,05}`，每次 150 个物理样本；每个 controller 三次独立 fresh-start。所有重复的运行时质量、重力、碰撞几何和命令 hash 相同。

| 条件 | 切换时间 | 最终墙间隙 | 几何 3 mm 接受 | loaded wall | 墙接触步数 | 峰值墙力 | 过渡命令不连续 |
|---|---:|---:|---|---|---:|---:|---:|
| C0 固定推进 | — | -0.000000293 m | 3/3（数值容差内） | 3/3 | 131 | 38.9589 N | 0 N |
| C1 几何停止 | 0.333333 s | 0.000001435 m | 3/3 | 3/3 | 131 | 38.9589 N | 8.0 N |
| C2 平滑几何切换 | 0.316667 s | 0.000001376 m | 3/3 | 3/3 | 131 | 38.9589 N | 0.6667 N |

C0 的 `-0.293 µm` 是低于 `0.1 mm` 数值容差的微小接触重叠，仍满足本实验的几何接受判据。C1 的离散停止发生在 Cube 已接近墙的物理步，因此它并没有避免墙加载；C2 也建立了墙接触。三者具有相同的几何接受和 loaded-contact 结果，不能据此声称 C1/C2 优于 C0，也不能声称力峰值降低。

C1/C2 的可重复行为差异在命令层面：C1 有 `8 N` 的离散命令跳变，C2 在 blend 段最大跳变约 `0.667 N`，mode 序列稳定为 `APPROACH → BLEND → REDUCED_HOLD`。这支持“平滑切换已物理实现”的 proxy 结论，但仍不是混合力/位置控制。

## 失败与修复记录

- `g1a_C0/C1/C2_probe_01`：保留了首版 PD proxy；三个条件都因 PD 命令降到静摩擦阈值以下而停在约 19.426 mm 间隙，未发生墙接触。这是 implementation/configuration probe，不纳入最终性能比较；原协议见 [`configs/g1a_proxy_protocol_probe_pd.json`](configs/g1a_proxy_protocol_probe_pd.json)。
- `g1a_C2_final_01`：保留了 `KeyError('trigger_speed_m_s')`，18 个样本后非零退出。修复配置字段后，使用同一冻结协议完成 final_02/final_03–05。
- `g1a_C0/C1_final_01`、final_02：保留为早期修复后的重复；最终公平表只使用协议 hash 完全一致的 final_03–05。

## FR3 状态与比较解释

本任务没有把命令接入 FR3 关节/末端控制接口；`fr3_execution=false`，因此 `G1A_FR3_PASS` 不成立。没有把 Cube proxy 运动称作机器人运动，也没有运行物理吸盘闭链。

公平比较只能支持以下有限结论：

- C0 vs C1：在同一最小场景中，几何触发停止实际改变了 command/mode，但最终接触和几何接受相同。
- C1 vs C2：C2 的 12 步平滑下降降低了切换命令不连续度，但没有显示任务或墙力优势。
- 力峰值指标虽在最小场景可观测，却不用于切换，也没有用于声称控制器优越；fuller-scene 力通道仍不可靠。

## G1-A 判定与 G1-B 门槛

最终判定：**`G1A_PROXY_PASS`**，不是 `G1A_FR3_PASS`。结果只证明几何/运动状态驱动的物理 proxy 切换已实现并可重复。它不支持 jam recovery、混合力/位置力控或论文级方法优越性结论。

G1-B 仍阻塞，直到 fuller-scene 的接触力/支撑缺口根因得到可重复解释，且 FR3 命令接口能在实际仿真机器人上执行同一类切换。下一步若继续，应先做 FR3 无吸盘、无墙力反馈的短程末端位置切换，并单独报告关节/末端实际运动；不得把本 proxy 结果直接升级为 FR3 结果。

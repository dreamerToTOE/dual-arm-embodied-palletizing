# G0-R 力观测与 DeepWall ledger 修复报告

## 结论

`G0_R_PASS`（限定于最小 Cube/Table/DeepWall 场景）。三次固定协议、全新 Isaac Sim 启动的运行均完成：加载墙接触、一次有界撤回脉冲、至少 12 个物理步的持续分离；修正后的动量 ledger 与实际运动一致。这个结论不覆盖 fuller Task27 场景，也不改变原 G0 的 `G0_PARTIAL` 结论。

## 修复内容

原 `g0_c2_wall.py` 在 post-step 行中记录的是“当前样本之后施加、下一物理步生效”的命令，却没有把上一行命令产生的外力冲量放进当前步 ledger。新脚本 [`scripts/g0r_c2_wall.py`](scripts/g0r_c2_wall.py) 明确记录：

- `active_force_world_N`：当前 pre/post 物理步实际生效的力；
- `active_force_dt_s` 与 `active_force_impulse_world_Ns`；
- `next_commanded_force_world_N`：当前 post-step 之后给下一物理步的命令；
- 接触法向事件冲量、摩擦力乘实际 physics `dt`、重力冲量和修正/未修正残差。

ledger 只使用 contact-report 原始法向冲量和摩擦接口；矩阵力/冲量用于交叉核对，不与事件冲量相加，避免同一 PhysX 求解器结果双计数。事件 callback 步号要求 `step_at_callback + 1 == sample step`。未使用 wall-clock dt。

## 冻结协议与运行

协议文件：[`configs/g0r_c2_protocol.json`](configs/g0r_c2_protocol.json)，SHA-256：`d75bbe65c92f3b321a94a5ba15fc07d29fb6818cf6723ff8de425f04b7d82081`。

固定排程：

1. `t < 0.5 s`：零力；
2. 首次加载前：`+8 N` 世界 +X；
3. 加载后固定保持 `+1 N`，持续 `0.6 s`；
4. 撤回只施加一个物理步 `-10 N`；之后零力保持，直到 12 个连续分离样本。

旧 G0 C2-02 的 `-5 N` 以及 C2-03 的 `-12 N` 都不是本协议的复现；它们保留在旧分支证据中。每个新运行的 summary 都保存协议 hash 和脚本 hash。

最终三次运行：

| run | loaded 起点 | first separated | 峰值墙法向力 | 修正残差 L∞ | 未修正残差 L∞ | 分离接受 |
|---|---:|---:|---:|---:|---:|---|
| `g0r_c2_final_01` | 49 | 86 | 38.9648 N | 2.13e-5 N·s | 0.166667 N·s | PASS |
| `g0r_c2_final_02` | 49 | 86 | 38.9648 N | 2.13e-5 N·s | 0.166667 N·s | PASS |
| `g0r_c2_final_03` | 49 | 86 | 38.9648 N | 2.13e-5 N·s | 0.166667 N·s | PASS |

三次运行均无 Cube pose/velocity teleport，质量 `0.8000000119 kg`、重力 `[0,0,-9.81000042] m/s²`，事件步对齐为真。撤回后 12 个样本的墙法向力与事件法向冲量均低于卸载门槛；但原始 collider pair 仍在 12 个样本中出现，且间隙已经约 1.5 mm。这是 contact-offset / pair-report 语义，不是承载接触；因此卸载判据使用力和冲量，不使用 pair identity 单独判定。

## ledger 结果

未修正 ledger 在第一次 `+8 N` 激励步产生约 `+0.133333 N·s` 的 X 残差，在 `+1 N` 保持和撤回步继续产生命令遗漏，最大约 `0.166667 N·s`。加入当前物理步的命令冲量后，三次运行的最大未解释冲量仅 `2.13e-5 N·s`，显著低于协议 `0.07848 N·s` 门槛；这证明原问题是记账时序缺项，而不是把矩阵与事件重复相加。

## fuller-scene 诊断

运行 `g0r_c4_full_diagnostic_01` 是实际 Isaac Sim fuller Task27 场景预检，结果仍为 `FAILED`。诊断文件见 [`g0r_full_scene_diagnostic.json`](g0r_full_scene_diagnostic.json)。

- 12 个样本全部 awake；运行时质量、重力、TGS 和 stabilization 正确。
- 有效材质为 `/World/Task27/HighFrictionMaterial`，静摩擦 0.90、动摩擦 0.75、恢复系数 0。
- Cube/Table 矩阵冲量与事件冲量最大差为 0；样本 `sensor_step == step`，world frame 有效，观测年龄约 0.297–0.472 ms；没有 Cube pose 写入。
- 48 个工具 cup/Cube pair events 的峰值原始冲量为 0；它们的分离量约 3 mm，属于零加载的接触 offset 报告。没有其它 Cube 接触冲量。
- 支撑力均值 `3.299012 N`，期望 `7.848000 N`，相对误差 `57.96%`；积分支撑缺口 `0.909798 N·s`，与预检 z 残差 `0.909792 N·s` 数值相近。

最后两个数在数学上必然相关，因为残差包含重力与报告支撑力的差；它们不是两个独立物理传感器，也没有在本任务中证明根因。根因结论保持 **UNKNOWN**，fuller-scene 力通道仍禁止接入控制器。

## 剩余边界

G0-R 只把最小 DeepWall 通道推进到 `PASS`。它没有验证吸盘状态、fuller-scene 力转移或 FR3 关节/工具控制。下一阶段若使用墙力，必须明确限定为本最小场景的观测；G1-B 力反馈仍保持阻塞。

# TASK28 G2-P5-QP-6 R12：B/C 跟踪误差归因与联合达标验证

## 结论

R12 以 R11 推送 HEAD `180b1adecbba91bd30acf563578188fcd026afa2` 为只读基线。R11 的 τ=`0.10 s` 启动速度捕获保留；新增一个全局、统一应用于 V10-A/B/C 的反馈增益缩放 `0.75`：

```text
object_gain  = 0.75 * R11 object_gain
relative_gain = 0.75 * R11 relative_gain
```

QP 硬约束、关节速度/位置限制、努力和速率限制、场景、fixture、Cube 身份、无接触门和验收门限均未改变。R12 独立复测结果为 V10-A/B/C 全部通过冻结门限，且 V10-A 的结果保持不变。

## R11 B/C 误差归因

逐帧证据保存在 `offline/task28_p5_qp_6_r12/evidence/error_decomposition.json`；R11 原始帧仍保留在 `offline/task28_p5_qp_6_r11/evidence/diagnostic/`。

- V10-B 峰值在 physics step `226`，仿真时间 `3.766667 s`，对应虚拟轨迹时间约 `0.7333 s`、早期加速段。目标位移约 `32.75 µm`，共同对象误差 `0.299829 mm`；同一步 `FK(q_ref)` 参考误差约 `0.226202 mm`，实际相对参考滞后约 `0.093259 mm`。
- V10-C 峰值在 physics step `225`，仿真时间 `3.750000 s`，对应虚拟轨迹时间约 `0.7167 s`、早期加速段。目标位移约 `30.82 µm`，共同对象误差 `0.315761 mm`；同一步参考误差约 `0.251402 mm`，实际相对参考滞后约 `0.088458 mm`。
- 两组在 ownership 后第一段均出现反馈触发的 qdot 跃升：step 184 开始，B/C 的 qdot 约 `0.00241/0.00247 rad/s`，step 188 达到约 `0.00575/0.00587 rad/s`；对应 common-task residual 达到 `2.40699e-4/2.59259e-4`。A 在同一段 qdot 仅约 `7.3e-6 rad/s`，common residual 约 `1.48e-9`。
- B/C 的最大相对 TCP 平移分别为 `0.071728/0.062615 mm`，远低于 `0.150 mm`；因此相对运动没有主导共同对象失败。
- R11 B/C 的所有 QP 状态均为 `QP_OPTIMAL`，硬相对残差峰值约 `1.15e-10/1.13e-10`，qdot 峰值约 `0.00575/0.00600 rad/s`，低于 `0.12 rad/s`。没有约束激活或数值不可行证据。

证据等级为：启动 ownership 层的共同根因是直接逐帧支持的描述性结论；B/C 特有的早期位姿反馈软任务 mismatch 和 qpath/reference 偏移是主要支持项；“quintic 轨迹公式本身有缺陷”没有得到证据支持。

## 离线定向试验

R12 运行前执行了只读 CPU 半隐式代理敏感性试验，固定 R11 归档的 qdot、M、Kp、Kd、dq0 和目标序列，仅缩放记录的反馈相关 qdot。结果用于预注册统一参数，不是验收结果：

| 全局缩放 | A 预测峰值 | B 预测峰值 | C 预测峰值 |
|---:|---:|---:|---:|
| 1.00 | `0.203 mm` | `0.294 mm` | `0.315 mm` |
| 0.75 | `0.203 mm` | `0.230 mm` | `0.246 mm` |
| 0.50 | `0.246 mm` | `0.253 mm` | `0.253 mm` |
| 0.25 | `0.373 mm` | `0.377 mm` | `0.377 mm` |

统一缩放 `0.75` 在 R12 Isaac 运行前冻结。该试验没有重解 QP，运行结果由真实独立闭环复测确认。

## R11 与 R12 定量对比

| 工况 | R11 Cube 峰值 | R12 Cube 峰值 | R11 相对 TCP | R12 相对 TCP | R12 状态 |
|---|---:|---:|---:|---:|---|
| V10-A | `0.199777 mm` | `0.199777 mm` | `0.029872 mm` | `0.029872 mm` | PASS |
| V10-B | `0.299829 mm` | `0.237183 mm` | `0.071728 mm` | `0.065286 mm` | PASS |
| V10-C | `0.315761 mm` | `0.247617 mm` | `0.062615 mm` | `0.059173 mm` | PASS |

R12 的努力峰值分别为 `28.0089/28.0168/28.0179 Nm`，注册努力速率峰值分别为 `3.902/4.413/4.438 Nm/s`；三组均在既有 R8 限制内。QP 硬相对残差峰值分别为 `1.13e-10/1.13e-10/1.13e-10`，三组过程隔离、缓存隔离、正常退出和资源清理均通过。

## 联合验收与安全边界

V10-A、V10-B、V10-C 均为 210 个控制命令/臂，Cube 位移为 0，吸盘和 D6 命令为 0，独立 evaluator 均返回 `PASS_NO_CONTACT_ACCURACY_GATE_PASS`。V10-A 没有牺牲：其输出和 R11 完全一致。

R12 只验证 A/B/C 联合达标，因此没有运行 V10-D 或 FORMAL。原始 HOME→Cube 接近路径仍标记为 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`；离散无碰撞结果没有被提升为连续安全证明。没有实际接触、Cube 位移、吸盘、D6、ROS 命令或真实机器人执行。

## 回归与交付

完整回归为历史 `1051` 项加 R12 聚焦 `19` 项，共 `1070 passed`。R12 聚焦测试覆盖全局缩放预注册、只读代理敏感性、τ=`0.10 s` 数学、QP 负例和连续安全阻断证据。

机器证据：

- `offline/task28_p5_qp_6_r12/evidence/error_decomposition.json`
- `offline/task28_p5_qp_6_r12/evidence/feedback_gain_sensitivity.json`
- `offline/task28_p5_qp_6_r12/evidence/evaluation.json`
- `offline/task28_p5_qp_6_r12/evidence/process_isolation_audit.json`
- `offline/task28_p5_qp_6_r12/evidence/regressions/summary.json`

## 未解决问题与下一阶段建议

连续安全仍未建立：原始 HOME→Cube 接近路径的区间验证保留未解析碰撞/支撑区间，离散无碰撞结果不能替代连续证明。R12 的反馈增益缩放只改善了这三个虚拟闭环工况的跟踪误差，未证明接触、吸盘、Cube 搬运或真实命令可行性。

若获得后续授权，建议先以冻结的 `tau=0.10 s` 与全局反馈缩放 `0.75` 保持同一安全约束和场景，复核更长时间窗与扰动下的误差和约束裕度，再独立解决连续碰撞/支撑区间证明；在此之前不运行 V10-D、FORMAL 或任何真实执行。

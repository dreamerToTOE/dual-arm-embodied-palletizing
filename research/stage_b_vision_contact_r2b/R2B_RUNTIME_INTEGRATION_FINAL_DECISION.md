# R2-B Stage B 运行时集成最终决定

最终状态：**BLOCKED（实现完成，Stage B 晋级门未通过）**。

已完成的门：

- 黑帧根因修复并以正向 RGB smoke 验收；lighting command、相机、render product、
  尺寸、通道、直方图和 physics timestamp 已落盘。
- marker USD normalized local 与米制 expected corners 分离审计；父级 0.12 scale
  只应用一次，角点误差 1.53e-9 m。
- physical07 q/dq、FK TCP、suction、controller receipt 已按 latest-past 接入；
  missing/stale TCP 现在让 E0/E1 返回 UNKNOWN。
- PhysX contact 保持独立 oracle，区分 pair presence、possible 和 loaded-supported；
  side wall 无支持时不造标签。
- E0/E1/shadow 已重放评估，shadow 仍只读，FCL、碰撞、torque、joint-limit 和
  suction 约束未放宽。

阻断门：

- 顶面单平面 ID23 静态转移虽然 6/6 检出、重投影 0.307 px，但 Cube 位置误差
  中位数 47.2 mm、P95 54.3 mm，远未达到 3 mm 级静态定位门。
- 因静态门失败，没有运行动态短 episode，也没有把现有 physical07 的执行轨迹
  解释为新 marker 的动态 transfer 或完整 contact-recognition validation。
- controller command 缺少与 physics step 的严格 wall-time 因果标记；attachment
  confidence 仍 UNKNOWN。

因此本分支可交付的是可审计的传感器接入、黑帧修复、oracle 标签和安全 shadow
评估，不能作为在线 Stage B contact recognition PASS。后续最小研究动作是改善
Cube marker 的可观测几何（例如相机正视的可见面或非共面约束）后重新做独立静态
holdout；在该门通过前不做动态操作、E2/RL/GNN 或 live switching。

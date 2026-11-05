# R2-B 最终研究决定

**决定：`R2B_DATA_PARTIAL; B2_SMOKE_PASS; E2_ADVANTAGE_UNPROVEN; PHYSICAL_GATE_DEFERRED`。**

本分支基于 `a308882aa7e1f1a9427584ddbc469808f6ef1974`，实现并测试了 Stage B observer contract、真实 Isaac RGB-D 相机、附着式 ArUco/PnP、V0/V1/V2 资格脚本以及 E0/E1/E2 小型估计器。真实渲染单帧中 marker 检出成功，重投影误差 0.066 px，深度帧有效值比例 0.7856。

研究结论是“感知链路可行，但在线控制/恢复尚未被验证”。V0/V1 是离线基线和 proxy；V2 只有单帧 smoke，E0/E1/E2 没有带真实操作时间戳的视觉接触标签；因此不支持两级恢复选择器或物理 jam/recovery 结论。下一步最小动作是把 command 时间对齐和已验证接触观测接入同一条多帧 Task27 episode，再重复 V2/E0/E1/E2 和 bounded variation gate。

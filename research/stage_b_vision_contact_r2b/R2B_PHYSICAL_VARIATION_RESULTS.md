# R2-B 物理变化与接触结果

本分支没有启动新的物理 manipulation、jamming、recovery 或 RL 试验。`r2b_camera_observer_smoke20` 在播放前把 `Cube_01` 放到 authored perception slot，只用于验证 RGB-D 和 marker；它不是接触实验，`fresh_physical_manipulation_episode=false`，oracle contact 记录为空。

因此本阶段：

* 真实接触边、夹持保持、卡滞和恢复成功率：**未测量**；
* offline proximity：只可解释为几何接近，不能解释为物理 jam；
* Physical variation gate：**DEFERRED**，等待主开发会话修复并验证 contact-force observability、command 时间对齐和可执行 recovery skill。

这遵守 Issue #4 的“先通过 nominal V2 + observer，再做 bounded physical variations”顺序，没有制造人工失败。

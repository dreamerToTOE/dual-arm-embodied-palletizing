# R2-B 接触标签语义审计

本分支继续使用上一阶段构建的独立 PhysX oracle；E0/E1 不读取它。标签区分：
几何 proximity、callback pair presence、impulse-supported event、penetration proxy
和 independently measured loaded contact。

`LOADED_SUPPORTED` 只表示 simulation callback 中存在非零 impulse 或超过 0.1 mm
penetration 且 pair 持续至少两步的保守 proxy。penetration 不是接触力测量，不能写成
真实 loaded force。`CUBE_SIDE_WALL` 仍为 unsupported/UNKNOWN；没有 fabricated jam、
attachment 或 recovery positive。

本任务没有调 E2，也没有把 earlier pair-present F1 当成 loaded-contact 识别成功。
Pose 诊断只使用 oracle 做离线误差计算；在线 shadow gate 使用 executor skill、
controller owner、传感器 health 和 attachment confidence。

# R2-B 观测帧契约

实现位置：`observer/contract.py`。`StageBObserverFrame` 将 physics step、仿真时间、q/dq、TCP、控制器参考、command id/stamp/commanded advancement、吸盘命令/状态、视觉位姿和 sensor health 放入同一帧；`StageBFrameBuffer` 只允许单一 owner 且强制 step 单调递增。

`estimator_payload()` 不包含 `oracle_contact_pairs`，而 `to_dict()` 仅把它作为旁路诊断保存。这样 E0/E1/E2 不能读取 PhysX 接触标签。相机源、关节回读、TCP FK 和物理回调均写入 `SourceStamp`。

Isaac smoke `r2b_camera_observer_smoke20` 生成 1 个真实仿真帧：q/dq/TCP/相机源均为 `PRESENT`，command 字段在无操作命令的感知 smoke 中为 `null`。关节命令字段已留在契约中，须在实际 Task27 执行器运行时填充。

验证：`python3 -m pytest -q research/stage_b_vision_contact_r2b/tests` → **4 passed**。

V2 后处理生成 `estimator_event_journal.jsonl`；事件记录明确排除了 oracle contact，且保留 physics step/time 对齐。

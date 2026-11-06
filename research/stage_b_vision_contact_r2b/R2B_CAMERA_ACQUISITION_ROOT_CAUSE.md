# R2-B 相机采集根因

## 结论

根因在采集调度和写盘握手，不在 `rear_11=[1.70,-0.35,0.80]` 的视点本身。旧实现使用 `physics_step % every_steps == 0`，但 `capture_viewport_to_file()` 后的 `app.update()` 会让物理步一次前进多个 tick；错过一个整除边界后，后续永远不会补采。旧路径还把 RGB 请求、深度读取和文件写完混在一起，无法区分调度收据、传感器渲染和 writer 完成。

## 证据

- `physical05` 有 55/55 个 ArUco 检出、没有光学 dropout，但最大未采样间隔为 90.65 s。这是采集调度盲区，不应称为视觉不检测。
- `physical06` 的第一轮修复已经避免了整除边界永久丢失，但 writer 检查早于异步文件落盘，记录了 379 个假失败；该轮只作为诊断探针。
- `physical07` 使用修复后的 writer 等待和启动阶段采集：1550/1550 RGB-D 文件对成功、capture/depth error 均为 0，physics receipt 最大间隔 0.2333 s。它说明原始 90 s 级盲区已经消失。

## 修复后的语义

`R2BPhysicalCamera` 现在使用 `next_capture_step` 阈值调度。当前步超过预约步时只写一对当前 RGB-D，并记录 `scheduler_skipped_intervals`、`missed_deadlines` 和调度滞后；不会静默丢掉整个调度相位。每条记录分开保存 physics receipt、capture request、render-wait 完成、depth acquisition 和 writer 完成时间。

`capture_viewport_to_file` 仍需要有限的嵌套 `app.update()` 才能完成异步 writer；修复没有把这段推进伪装成物理观测，而是记录 `render_wait_updates` 和 `physics_step_after_render_wait`，后者只作诊断。相机启动时从首次观测到的 physics step 激活调度，避免把相机尚未启用的时间计为丢帧。

## 限制

viewport API 没有返回硬件 frame id，因此 `rendered_sensor_frame_wall_s` 表示有界 render wait 完成时刻，`writer_complete_wall_s` 表示文件可读时刻；它们不是外部相机曝光时间。RGB 和 depth 是同一调度步的一对文件，不能据此宣称曝光级同步。该限制与校准残差共同使最终门保持 PARTIAL。

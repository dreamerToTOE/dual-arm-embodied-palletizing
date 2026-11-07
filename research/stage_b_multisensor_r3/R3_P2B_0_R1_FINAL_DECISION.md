# R3-P2B-0-R1 覆盖与运动插值决定

基线 `3128c6340e0a43cd7c5021b5633157103cb1a8bb`；分支保持 R3。

**总评 PARTIAL；受限离线 P2B-1 推荐选项 A：精确实测机器人状态。** 采集覆盖已解决；动态插值未整体通过原冻结门槛。固定命令括号的线性插值在本次独立腕关节探针达标，但不默认授权向任意 Task27 运动转移。B 不作为默认入口；C 无必要。新的用户任务授权随后开展离线 P2B-1，本提交先完整保存 R1 边界。

## 覆盖缺口与修复

旧 `[0,2.71666680835)` 来自先创建/订阅相机 RP、再 warmup 和初始化 articulation，期间没有实测 q/dq。原证据重放仍得到 12,150 个无支持候选，完全保留。

新版本 `scripts/capture_p2b0_r1.py` 不初始化早期 Camera RP。先 warmup、初始化手柄、真实读取两臂 q/dq/FK，然后才创建 `force_new=True`、独立命名的 RenderProduct，并在此之后创建 Cube 标记和写入新的 RGB/depth 出生编码。只接受解码匹配出生编码的图像。因此新的因果下界来自采集资源和新可见标记的实际出生事件，而不是用 SDK 时间或位置拟合截掉过去。无预初始化 q/dq 补写。

同一次有界运行中重建采集流一次，物理时钟不重置：

| epoch | 实测下界/创建时刻 s | birth code | 机器人样本 | 缺步 |
|---|---:|---:|---:|---:|
| 0 | 2.26666678488 | 41809 | 193 | 0 |
| 1 | 5.46666695178 | 23718 | 193 | 0 |

三元身份仍为 run/epoch/time_base；每个 Camera 对象固定持有自己的 stream_id/epoch，不用全局新身份重标旧回调。旧 RP 暂停、解除 annotator、销毁后新建不同路径；新 stream 再测一次真实边界状态。边界物理步在两个 epoch 各有一次真实读取，不是同一 epoch 重复步。

结果：96/96 保存帧（90 动态、6 settle）出生编码通过、候选网格全覆盖，8,970/8,970 候选有状态；每帧采集/bracketing 不支持时长 median/P95/max=0/0/0 s，两个 epoch 缺步均 0。6 次尚未初始化的 frame 事件没有被包装成有效图像。旧 epoch 实际保留图片被新 epoch 拒绝；仅重标元数据但保留旧图片也被出生编码拒绝，回执见 `stale_buffer_negative.json`。

完整视觉时间集合仍是 `[epoch_birth,callback_entry]`，宽度 median/P95/max=1.533333/2.966667/3.100000 s。没有减固定偏移或制造点时间。这个出生证据限定新采集/新标记图像的因果起点，不证明任意物体的 Fabric 传播有统一精确时刻。跨 epoch 旧图像不能沿用；同一 epoch 的全部历史别名仍保留。

**采集范围/相邻实测括号覆盖不等于连续 q/dq 插值通过。** 精确模式只能查询真实物理样本；非网格候选仍 UNKNOWN，不能把连续区间缩成物理网格。

## 独立运动与冻结评估

仅一次 Isaac 运行，31.28 s wall；未修改控制器、驱动、安全限值或后壁相机。局部关节目标每 3 app updates 更新一次，包含静止、正/负匀速目标段、停启和多种速度，幅度限值沿用 0.015 rad。它是动力学机器人/运动学 Cube 同步探针，不是吸附接触实验。

采集前冻结 `config/p2b0_r1_frozen.json`，原 q/dq/TCP/姿态门槛分别 0.0002 rad、0.001 rad/s、0.020 mm、0.0002 rad，逐项 CPU 测试与旧配置相同。线性与 Hermite 公式源码同时快照并哈希；未根据误差拟合系数。Hermite 的 dq 为 q 三次多项式的解析导数。

两个相反奇偶留出划分，从新轨迹实测 60 Hz 序列形成 30 Hz 端点，共 370 个真实留出时刻。每个时刻只进入一个划分；预测只接收端点，不接收留出 q/dq。全部预测写出并封存后，才打开独立 PhysX TCP 参考。两方法使用相同目标、参考、门槛。exact 对照在端点稀疏历史拒绝全部留出点，不以 nearest lookup 伪造成功；完整历史则 386/386 精确查询成功，q/dq 零差仅是检索一致性。

386 个状态、772 arm-samples 的独立 FK 位置误差 median/P95/max=0.000235/0.000381/0.000465 mm，姿态 max=9.61951e-7 rad。Core 与累计物理时间、SDK 与 Fabric 映射时间一致，但不据此宣称图像内容时刻精确。

| 方法/原始 370 目标 | q median/P95/max rad | dq median/P95/max rad/s | TCP median/P95/max mm | 姿态 max rad | 原门槛全项达标 |
|---|---|---|---|---:|---:|
| linear | 7.55489e-6 / 5.48065e-5 / 1.36852e-4 | 7.59640e-5 / .00388290 / .00878059 | .001369 / .008628 / .021246 | 1.36627e-4 | 289/370 |
| Hermite | 8.78061e-8 / 3.71784e-5 / 7.49073e-5 | .01605095 / .01605168 / .01844587 | .000323 / .005848 / .027194 | 7.49881e-5 | 0/370 |
| exact 稀疏历史 | 无输出 | 无输出 | 无输出 | 无输出 | 拒绝370/370 |

原始方法数值输出覆盖均 370/370，不等于验证接受。预先规定的同命令括号 guard 接受 251/370（67.8378%）、拒绝119/370；linear 的 251/251 全项达标，max q/dq/TCP/姿态为 1.85966e-5 rad / .000186018 rad/s / .007409 mm / 1.87699e-5 rad。Hermite 即使此 guard 后仍 0/251 全项达标，保持不支持。

| 分层 | 目标数 | linear max q / dq / TCP / rotation | Hermite max q / dq / TCP / rotation |
|---|---:|---|---|
| command transition 附近 ±1 tick | 179 | 1.36852e-4 / .00878059 / .021246 mm / 1.36627e-4 | 7.49073e-5 / .01844587 / .027194 mm / 7.49881e-5 |
| stable motion | 131 | 1.20699e-5 / .000120894 / .002073 mm / 1.25420e-5 | 3.73220e-7 / .01606298 / .000463 mm / 1.03093e-6 |
| stable hold | 60 | 1.57654e-5 / .000157486 / .007025 mm / 1.60190e-5 | 6.69460e-7 / .01605596 / .000454 mm / 1.00689e-6 |

分层由已记录命令的变化及目标速度定义，不以评分误差挑样本。stable motion 是目标匀速段中的非切换邻域，记录确有 FR3 关节运动；不能理解为所有关节严格匀速。Hermite 的位置改进没有带来 dq 通过，不能只挑位置结果宣称更准确。原 PhysX q/dq 输出不自动满足给定时间尺度下的连续导数一致性；没有修饰测量或更改原驱动来补过门槛。

## P2B-1 的受限输入

见 `evidence/p2b0_r1_input_contract.json`。默认通过 `ReadinessHistory.exact_at(identity,t)` 消费实测 q/dq 和重新计算的独立 FK，不调用继承的试验性插值入口。物理样本优先，禁止非网格 snapping、跨 reset 查询或外推。

可区分：精确实测值；本次局部腕运动上经验验证的同命令线性括号；没有验证/失败的跨命令线性和 Hermite；有界但时间未知的视觉；None/UNAVAILABLE 的 suction/contact/force。已观测插值通过不构成不同控制器、运动或时间尺度的自动认证。选项 A 足以开展受限离线研究，保留非网格时间 UNKNOWN；不授权在线控制。

`Surface_Gripper.is_closed()` 是软件反馈，不是独立物理附着证据；新探针没有此实例。未建立附着条件时禁止由 FK 推断 Cube 位姿。视觉点时间始终 null，完整区间及候选引用原样传递。

## 文件与复现

新增 collector `scripts/capture_p2b0_r1.py`、契约 `history_contract_r1.py`、评估器 `scripts/evaluate_p2b0_r1.py`、CPU 测试 `tests/test_history_contract_r1.py`、冻结配置、原始证据 `evidence/r3_p2b0_r1_holdout01/` 和预测/评分 `evidence/p2b0_r1_evaluation/`。原 P2B-0 与更早结果和源文件均不修改。

```bash
# 只在需要重新采集时运行；不要复用已有输出目录名
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  research/stage_b_multisensor_r3/scripts/capture_p2b0_r1.py --run-id r3_p2b0_r1_reproduce
python3 research/stage_b_multisensor_r3/scripts/evaluate_p2b0_r1.py \
  --run research/stage_b_multisensor_r3/evidence/r3_p2b0_r1_holdout01 --out /tmp/p2b0_r1_replay
python3 -m pytest -q research/stage_b_multisensor_r3/tests \
  research/stage_b_vision_contact_r2b/tests/test_aruco.py \
  research/stage_b_vision_contact_r2b/tests/test_cube_marker_geometry.py \
  offline/task28_g1_1/tests/test_independent_fk.py offline/task28_g1_1/tests/test_model.py
```

CPU 回执 `evidence/p2b0_r1_cpu_tests.txt`；两次独立目录回放的比较见 `evidence/p2b0_r1_replay_verification.json`。本阶段没有融合、连接估计或 online shadow；随后仅按新用户 P2B-1 任务开展离线工作。

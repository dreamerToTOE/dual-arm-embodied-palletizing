# R3-P2B-1 离线 Cube 状态估计决定

日期：2026-11-07。用户指定基线 `3128c6340e0a43cd7c5021b5633157103cb1a8bb`；先完成并推送 R1 `5f02408764029b723e561e0611220b972f05b232`，随后按新授权继续 P2B-1。分支仍为 `research/stage-b-r3-multisensor-plan`。

**最终决定：PARTIAL。** 已实现并独立评估 V1/K1/F1、连接原型和消融，但没有获得融合总体优于视觉保持的证据。三个命题分别为：

| 命题 | 决定及证据 |
|---|---|
| 给定有效连接假设，可以预测 Cube 位姿 | 在受控 PhysX 固定关节夹具中有条件支持。新独立轨迹连接有效段 K1 XY median/P95/max=0.095/0.262/0.302 mm |
| 可从本轮观测辨识真实连接状态 | 未建立。正向 CONNECTED 没有独立观测支持；主原型输出 UNKNOWN 或刚性关系不一致的反证，不把软件反馈当物理证明 |
| 整体系统在遮挡下可靠 | 不支持。新留出长缺失 F1 XY P95=34.183 mm，恢复后 2 s 内未回到 3 mm；大多数连接状态仍 UNKNOWN |

不进入 online shadow、控制器集成、自动重吸附、RL、skill switching 或实机。全部新增文件在本研究目录，旧结果和 Task26/27 源文件保持不变。

## 1. 可用输入及适用范围

采用 R1 的选项 A：同次运行初始化后的实测双臂 q/dq、独立 URDF FK，60 Hz 精确物理采样点；不使用未验证插值、nearest-step snapping 或命令替代反馈。相机 RP 在首条实测状态后创建，新 RGB/depth birth token 与 run/epoch/stream 身份验证因果下界，完整视觉区间保留。

相机位置、外参和 K 与原后壁配置相同；1280×720，沿用 P1/R2 像素与深度箱面拟合。所有 4 条完整轨迹各 303 帧，均通过出生/全候选机器人网格覆盖；有效视觉位姿分别 291、294、297、300 帧。帧质量失败保留为 UNKNOWN，没有用 Cube GT 修正标记检测或点时间。离线历史容量在评分前冻结为 4096，保留每条 1213 个原始实测状态；采集本身保存全部行，不用 1200 行容量丢弃初始化锚点。

`Surface_Gripper.is_closed()` 是软件反馈，不是压力、密封或独立物理附着证据。新夹具探针没有 Surface_Gripper 实例，suction feedback 为 None，不冒充 OPEN/CLOSED。安装版 SDK 的 measured joint efforts/reaction wrench API 存在，但这些轨迹没有记录这些量；applied efforts 是命令量，不能作为外部力。详细来源见 `evidence/p2b1_signal_availability.json`。没有补造力/压力通道，也没有将 PhysX contact 标签输入估计器。

机器人命令和 q/dq 都保留，但没有将 tracking residual 的变化直接当连接标签：重力、跟踪滞后、模型误差等也会造成差异。本原型没有完成这些因素的独立可辨识性验证，不能声称所有可用 proprioception 都在信息论上无法辨识；结论仅限本实现及实际采到的通道。

## 2. 新仿真与独立标签

场景只从桌面支撑的 Cube 开始；实际动力学 Cube、原 FR3 position drives 和原工具几何保持。为独立检验条件预测，研究探针创建显式 PhysX 固定关节夹具，将 Cube 与 link8 连接；标签来自夹具是否启用，并在评分中用独立 PhysX TCP/Cube 相对变换验证。**这是受控机械连接，不是验证了真实吸盘接触、密封或 Surface_Gripper。**

初始关节设置只用于建立 Stage-B 研究初始条件，随后有界位置驱动产生真实运动、读取实测反馈。没有改动原驱动参数、关节限值、安全逻辑、控制器或成功标准。连接释放只在 oracle 回执中记录；可观测命令始终请求保持初始连接，因此估计器不能从命令日志直接读出释放时刻。滑移通过释放后的横向速度扰动产生，不用位置 GT 伪造观测。

| 轨迹 | 起始 Cube x / motion period | 标签及用途 |
|---|---|---|
| connected | .870 m / 9 s | 始终启用夹具；首版独立留出，修复后归为开发复核 |
| loss | .855 m / 11 s | 8 s 释放，无横向滑移；主原型无法可靠从历史别名中辨识，修复后开发复核 |
| slip | .880 m / 7.5 s | 8 s 释放并施加横向扰动；修复后开发复核 |
| verification | .873 m / 10.3 s | 修复后新采集的独立留出，9 s 释放并滑移；全部参数在读其标签前冻结 |

每条动态控制 20 s，前 2 s 静止，约 15 Hz 图像。新独立轨迹实际 Cube 运动范围 x=35.597 mm、y=13.419 mm；z=0.259997–0.260012 m，max tilt=0.0295°。全部四条桌面支撑检查通过；启用夹具期间相对位置误差 max≤0.336 mm，验证不是空 joint flag。独立 q→FK 对 PhysX TCP 的四条轨迹最大位置差≤0.000762 mm。

合计 6 次有界 Isaac 尝试：一次 7 臂关节值写入 9-DoF articulation 的初始化失败；修正以保留原夹爪关节值后一次 3 s 开发探针通过；四次完整采集。失败 source/traceback、所有 raw ROI、depth、状态、oracle 和 console 均保存。没有无界重跑。

遮挡实验使用相同原始图像上的**离线观测缺失掩码**：0.7 s、3 s、14.533334 s，以及恢复。它们检验信息缺失，不声称重新制造了真实墙体遮挡；同一轨迹上的多个掩码不是独立物理实验。GT 用于评分，不用于估计输入。

## 3. 方法与不确定性

- **V1**：保持最新有效历史视觉 XY，不进行 FK 位移传播。其 XY 与 FK 无关；共同初始注册和连接列的共享辅助诊断另列，不把它当 vision-only 附着识别成功。
- **K1**：仅由初始视觉和静止 FK 历史注册 tool→Cube，再由当前精确 FK 传播。CONNECTED 明确标记为初始连接假设；未根据 GT 给它自动断连或恢复。因此释放后失效是有意保留的条件基线反例。
- **F1**：四维 XY/velocity CV 状态模型、协方差预测和低频视觉校正。连接不能独立确认时，刚性 FK 约束暂停；机器人历史用于检查初始刚性关系是否与视觉的所有支持时间候选矛盾。没有用“工具靠近”或 is_closed 绑定 Cube。

连接输出采用 CONNECTED/UNKNOWN/DISCONNECTED 三值接口。主 F1 的正向 CONNECTED 证据没有建立；当前只有 K1 的显式条件假设可设置 CONNECTED。运动一致、无异常或软件闭合都输出 UNKNOWN；连续两次视觉相对全部支持候选违反阈值时输出推断的 DISCONNECTED，置信度不伪装成标定概率。无新独立连接测量的物理步返回 UNKNOWN，不用状态保持制造连续确认。

所有输入 `measurement_time_s=null`，完整 `[birth,callback_entry]` 连续区间和机器人网格引用保存。F1 将未知视觉年龄建模为区间上的 uniform nuisance variable，采用 Gaussian moment approximation，而非给每图选择一个时刻。若 z=p_now−d·v_now+noise，则：

`H=[I,−E(d)I]`，`S=H P Hᵀ + Var(d)(P_vv+v vᵀ)+R`，交叉协方差为 `P Hᵀ`。这是明确的近似时间先验，不是已测得的延迟分布或形式化置信界。连续候选没有被缩成网格；刚性反证的步间 padding 也只是本实验模型参数，没有宣称验证了任意连续机器人插值。

输出含 XY、XY velocity、位置协方差/名义半径、观测年龄区间、连接状态/原因、有效性和 rigid-constraint 状态。velocity 没有独立精度认证。估计器的参数文件只包含模型/噪声参数，不含场景的释放或滑移时间表。所有 episode 的预测文件写出并哈希封存后，评分器才打开 Cube/夹具 oracle。

## 4. 首版失败、修复与新留出

首版把 z 先平移 `v·E(d)`，却仍用当前位置 H 更新 velocity，造成正反馈。首批 connected/fully_visible F1 max=245.427 mm，显著差于 V1。源码保留为 `offline_cube_estimator_v0.py`，原始预测/评分保留在 `evidence/p2b1_evaluation/`，并有紧凑失败回执 `p2b1_v0_failure/`。

修复使用上述延迟测量算子和交叉协方差，保持原噪声参数不变。CPU 可复现反例中首版 max=987.292 mm，修正版=22.819 mm。随后新采集 verification，才评价修复后的独立效果。前三条不能再冒充修复后的盲测。

新独立 fully_visible 首版 F0 median/P95/max=9.076/138.507/148.189 mm；修正版 F1=24.206/70.451/75.857 mm。极端误差减小，但 median 变差，**没有总体改善结论，更没有胜过 V1**。断连反证后的零速重置及宽时间先验仍会导致不合理传播；这是被实验否定的原型策略，不是物理速度观测或可靠接触切换。

## 5. 新独立留出主指标

以下均为当前物理步 Cube XY 与独立 PhysX 真值的误差，单位 mm；覆盖为 1205/1213=99.3405%，不等于精度合格率。约 0.66% 初始状态因无视觉锚点不输出。3 mm 仅用于描述恢复操作尺度，不等于视觉估计准确度或历史 placement gate 的通过。

| 场景 | V1 median/P95/max | K1 median/P95/max | F1 median/P95/max |
|---|---|---|---|
| fully_visible | 0.000055 / 1.657 / 13.144 | 13.240 / 29.401 / 30.156 | 24.206 / 70.451 / 75.857 |
| short 0.7 s | 0.000055 / 2.210 / 13.144 | 13.240 / 29.401 / 30.156 | 24.186 / 70.390 / 75.792 |
| long 14.533 s | 29.438 / 34.061 / 34.822 | 13.240 / 29.401 / 30.156 | 28.845 / 34.183 / 34.922 |
| recovery 3 s | 0.000055 / 17.698 / 18.460 | 13.240 / 29.401 / 30.156 | 22.020 / 64.076 / 68.991 |

K1 连接有效段 median/P95/max=0.095/0.262/0.302 mm；失效段=15.854/29.918/30.156 mm。这个对比支持条件预测，但也明确显示条件失效后不能继续绑定。极小视觉 median 来自理想仿真深度/已知几何及停止后的静态样本，不是实机视觉纳米级准确度，也没有把 cal24 camera-center 标定残差当在线 Cube 误差。

长缺失结束后第一步旧预测误差：V1=29.438、K1=15.799、F1=28.764 mm。第一次视觉恢复更新后 V1=0.000053 mm、F1=10.972 mm；V1 在 0.03333 s / 1 次更新内低于 3 mm，F1 和 K1 在 2 s 窗口内没有恢复。3 s 缺失的 F1 在恢复首更新后约0.962 mm，但后续仍可能再次漂移，不能据单次恢复宣称可靠。

全时序曲线见 `evidence/p2b1_evaluation_r1/error_curves.png`；逐物理步预测完整保存。F1 fully_visible 的名义位置半径 median/P95≈116/414 mm，经验覆盖为100%，说明范围极宽；这不是毫米级可靠性的证据。

## 6. 连接混淆、漏检与消融

新独立 fully_visible 主 F1，按有效预测物理步计：

| GT\预测 | CONNECTED | UNKNOWN | DISCONNECTED |
|---|---:|---:|---:|
| 夹具启用 | 0 | 545 | 0 |
| 夹具释放 | 0 | 498 | 162 |

UNKNOWN=86.556%，已连接时误报断连=0；严格未识别断连（包括 UNKNOWN）=498/660。长缺失时变为 UNKNOWN=96.763%，断连39、UNKNOWN621/660。不能用“没有误报”掩盖未知率，也不能把 UNKNOWN 当检测正确。

事件层面，可见滑移的首次断连反证延迟约0.2167 s；长缺失时约8.4167 s。loss 无横向滑移的旧独立场景保留为反例：时间别名及缺少独立通道使主原型无法可靠发现释放。各方法的物理步混淆矩阵和事件级延迟均保存；K1 的 CONNECTED 是假设，不是检测器。四条轨迹/同一场景多掩码不能被当作大量独立统计样本。

新独立 fully_visible 消融 XY P95：F1=70.451 mm；共同初始化后去掉机器人反证/重置=27.574 mm；去掉后续视觉=34.837 mm。机器人信息提供了模型失配线索，但当前将其用于零速重置会损害 XY，未验证正贡献。

把 SDK 时间强行当点时间的消融 P95=3.475 mm；长缺失后可1次更新恢复。但该点时间没有独立支持，**只能作为敏感性反例，不能被选为通过方案**。主 F1 保留完整时间区间。全部12个场景/7个方法的 median/P95/max、覆盖、UNKNOWN、样本/事件漏检和恢复更新数见 `NUMERICAL_RESULTS.md`、`summary.json`、`secondary_diagnostics.json`。

## 7. physical07 的有效复用与边界

复用已经去 GT 的独立历史摘录：真实长视觉未检出段观测跨度14.533334 s，右删失；873 个连续 q/dq/软件反馈样本。连接原型在无视觉且无独立连接观测时873/873 UNKNOWN。原始 trace 有 Cube GT，但 sanitized 摘录无有效初始视觉锚点，旧曝光关联未验证、物理附着标签也不完整，因此不在这里伪报其完整 XY/连接混淆评分，更不与新运行拼接。原 side-LIFT 失败照常保留。

## 8. 文件、复现与检查

主要新增：`offline_cube_estimator.py`、首版失败比较器 `offline_cube_estimator_v0.py`，`scripts/capture_p2b1_bounded.py`、`evaluate_p2b1.py`、`replay_p2b1_inputs.py`、`summarize_p2b1.py`、`plot_p2b1.py`，三份 `config/p2b1*_frozen.json`，CPU/负面用例和所有原始/预测/评分证据。

```bash
# 重采仅在需要时运行，输出名称必须全新；仿真使用已有 R1 RationalTime binding
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  research/stage_b_multisensor_r3/scripts/capture_p2b1_bounded.py \
  --run-id r3_p2b1_verification_reproduce --condition verification

# 从原始RGB-D重建全部预测，之后才打开GT评分
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 research/stage_b_multisensor_r3/scripts/evaluate_p2b1.py \
  --runs research/stage_b_multisensor_r3/evidence/r3_p2b1_connected_holdout01 \
         research/stage_b_multisensor_r3/evidence/r3_p2b1_loss_holdout01 \
         research/stage_b_multisensor_r3/evidence/r3_p2b1_slip_holdout01 \
         research/stage_b_multisensor_r3/evidence/r3_p2b1_verification_holdout01 \
  --out /tmp/p2b1_full_replay

# 快速重放已封存观测输入，逐文件验证全部预测哈希、独立重算全部场景评分
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 research/stage_b_multisensor_r3/scripts/replay_p2b1_inputs.py \
  --source research/stage_b_multisensor_r3/evidence/p2b1_evaluation_r1 --out /tmp/p2b1_fast_replay

python3 -m pytest -q research/stage_b_multisensor_r3/tests \
  research/stage_b_vision_contact_r2b/tests/test_aruco.py \
  research/stage_b_vision_contact_r2b/tests/test_cube_marker_geometry.py \
  offline/task28_g1_1/tests/test_independent_fk.py offline/task28_g1_1/tests/test_model.py
```

90 项相关 CPU 测试通过，含错误时钟/epoch、stale/duplicate、缺状态、GT注入、错误连接证明、未来视觉、重置、无锚点、协方差和首版正反馈反例。回放12个场景的全部预测 SHA 和全部评分一致，见 `p2b1_replay_verification.json`；归档独立目录复核见 `p2b1_clean_replay_verification.json`。运行依赖版本随回执记录，不声称跨任意数值库逐位一致。

原始采集见 `evidence/r3_p2b1_*holdout01/manifest.json` 和 snapshots；`oracle*.jsonl` 始终是隔离评估资料。`p2b1_evaluation_r1/prediction_manifest.json` 记录 oracle_inputs=[]、冻结 source/config SHA 和预测 SHA。初始化失败和两个评分器边界修复（缺首条 Cube oracle、NumPy bool 序列化）保存在 `r3_p2b1_fixture_check01/`、`p2b1_evaluation_failure/`，没有制造边界标签。

## 9. P2B-2 建议

暂不进入在线 shadow 或控制集成。先在仿真中记录并审计独立可用的 solver efforts/reaction wrench，区分软件反馈、执行器命令、模拟测量与接触 GT；在新的独立释放/无滑移/滑移对照中检验连接可辨识性。修正或拒绝反证后的零速重置与宽时间区间视觉更新，继续用 V1/K1 对照，不以更窄但未验证的时间戳换取低误差。只有当新独立数据证明连接与位置两项均有效，才讨论 P2B-2；本轮到离线评估停止。

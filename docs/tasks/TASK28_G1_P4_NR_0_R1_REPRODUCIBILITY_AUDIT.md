# Task28 G1-P4-NR-0-R1｜独立离线复现与研究有效性审计

审计日期：2026-10-26（Asia/Seoul）
仓库：`dreamerToTOE/dual-arm-embodied-palletizing`
分支：`side-suction-palletizing`
期望 HEAD：`94118e2984810e3251e04e5fce93d3a77226c7c0`

## 审计结论

**G1_P4_NR_0_R1_PARTIAL**。

远端/本地提交完整性、P4 方法对应关系、真实树搜索、父边回溯、冻结验证器复用、466 项回归以及三 seed 的离线科学结果均通过独立核对。研究有效性仍为 PARTIAL：本批只有三个固定 seed，查询位于无新增障碍的开阔单 Cube 场景，主搜索没有触发拒绝；G1-2B-1 的运动段闭链和连续碰撞维度仍为 `INCONCLUSIVE`。本审计不把原 G1-P4-NR-0 的 PASS 扩展为连续安全、完备性、实时性或物理有效性。

本报告和 `offline/task28_p4_nr_0/evidence/r1_reproducibility_audit.json` 在历史 R1 审计完成时是两个未跟踪文件，当时未提交、未推送。随后用户单独授权仅归档这两个审计文件；该归档不修改 NR-0 原有 53 文件提交中的内容。本报告中的 HEAD、工作树和 fsck 描述均指历史审计起点，归档交付状态另行核验。

这里的独立复现指重新生成输出和单独字段核对，使用同一主机及依赖环境；不声称外部人员独立审计或跨平台复现。

## 1. 提交、远端与基线保护

审计开始时独立 checkout 的工作树没有已跟踪文件改动，分支为 `side-suction-palletizing`。`git ls-remote` 返回的远端分支 SHA 与本地 HEAD 完全相同：

```text
94118e2984810e3251e04e5fce93d3a77226c7c0
```

HEAD 是一个有效 Git commit，树为 `48bbbff36cfdfb7df9d1c8a9128995f8919b27b4`，父提交为要求的 `5417da1a76a2a496f2e14993cc765a45f2133aff`。`git fsck --full` 没有报告 missing、corrupt 或 fatal 对象；输出仅包含 16 个未引用 blob（没有 dangling tree/commit/tag），不影响当前分支可达提交。提交标题为 `feat(task28): add bounded P4-inspired NR planning baseline`。

原提交中的 369 个受保护文件逐字节重新计算 SHA256，全部匹配 `protected_baseline.json`。原 artifact manifest 的 52 个新基线文件和 7 个 gzip 归档的字节/解压校验也匹配。fixture、ACM、模型、G1-1/R1/B0/B1、历史 Task24/26/27 文件均未改动。

## 2. 算法真实性与验证器边界

静态审计确认实现确实保留了 P4 的关键结构，而不是把旧 A-stage 路径改名：

- `projection.py` 构造左右两臂各 6 维的 SE(3) 残差，形成 12×14 block-diagonal 几何 Jacobian，使用 SVD Moore–Penrose Newton 更新、相对截断 `1e-12`、关节限位拒绝和回溯；没有调用旧路径 builder 或旧 least-squares IK 作为投影器。
- `planner.py` 从冻结状态锚点和固定 RNG 进行物体 SE(3) 采样，执行 goal bias、nearest/range、NR 投影和单状态门禁。投影中点由物体位姿 geodesic midpoint 与端点关节均值构成，递归两半在叶边检查前完成。
- 只有通过中点状态、叶边 B0 和 FCL 门禁的边才插入树。最终 transport 是从真实 `parent` 指针回溯得到，未预先生成旧路径后再贴上 planner 标签。独立事件审计得到的父链包含 dead branch 被排除，且与结果中的 transport 逐节点一致。
- `validation.py` 继续调用原 G1-1 单状态门、G1-2A-R1 规则、G1-2B-0 采样边验证和 G1-2B-1 连续验证。B1 的连续碰撞和运动闭链状态没有被升级，任何候选仍为 `OFFLINE_CANDIDATE_UNCERTIFIED`，`execution_authorized=false`。

P4 原始来源审计与本 FR3 适配的界线记录在 `p4_correspondence.md/json`。本实现是 P4-inspired：固定物体条件下的 12 行适配残差、FR3 14 关节、物体 SE(3) 采样和当前门禁属于工程适配；二维冗余纤维没有显式探索，米/弧度残差没有做无量纲归一，也没有概率完备性声明。

## 3. 独立回归

按基线报告的命令在新临时目录重新运行全部测试：

```text
/tmp/task28_g1_p4_nr_0_r1_tests2/regression_466.txt
```

完整测试命令（从仓库根目录执行）：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0:offline/task28_g1_2b_1 \
python3 -m pytest offline/task28_g1_1/tests offline/task28_g1_2a/tests \
  offline/task28_g1_2b_0/tests offline/task28_g1_2b_1/tests \
  offline/task28_p4_nr_0/tests -q -p no:cacheprovider
```

结果为 **466 passed in 83.65 s，退出码 0**。原归档记录为 466 passed in 83.25 s。审计中的第一条测试包装命令因 `PYTHONPATH` 目录名错误而在收集阶段失败，纠正后测试通过；另一次实验包装预先创建了 runner 要求不存在的输出目录，因而在规划开始前被拒绝，随后改用未创建的新目录完成复现。两次包装错误没有被计入科学失败或覆盖原证据。测试日志全文和退出码原文已嵌入机器记录，不依赖临时日志路径存续。

## 4. 三 seed 新目录复现

复现实验使用原登记的 seed、预算和线程环境，输出目录为：

```text
/tmp/task28_g1_p4_nr_0_r1_exp2
```

历史复现命令（只列出，不在归档任务重新执行）：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
timeout --signal=TERM --kill-after=10s 900s \
python3 -u offline/task28_p4_nr_0/run_experiments.py \
  --output /tmp/task28_g1_p4_nr_0_r1_exp2
```

再次复现时必须选用另一个尚不存在的输出目录。原始 `offline/task28_p4_nr_0/evidence/experiments_v1` 未被覆盖。每个 seed 的完整 `result.json.gz` 和 `search_events.jsonl.gz` 均重新生成。归档时再次只读比较现存数据，精确排除六个测量字段名：`planning_time_s`、`computation_time_s`、`runtime_s`、`CPU_wall_s`、`control_wall_s` 和 `peak_main_RSS_KiB`。不使用 `*time*` 等通配过滤；合成 R1 时间线中的 `time_s`、evidence identity、姿态、关节状态、事件顺序和其余全部科学字段均保留并比较。

环境为 Python 3.10.12、NumPy 1.24.4、SciPy 1.8.0、FCL 0.7.0，CPU-only，两个 BLAS/OMP 线程变量均为 1。新鲜 summary 直接记录 Python 和平台；包版本及 FCL 库 SHA256 绑定已提交的 `dependency_identities.json`，并在归档时通过读取当前安装元数据/库字节核验一致，未执行新的数值实验。测试与实验退出码均为 0。

| seed | 节点/树节点 | NR 尝试/失败 | 叶 B0 | 平移路径 (m) | 最大闭链平移 (m) | 最大闭链旋转 (rad) | 最小限位余量 (rad) | 归档/复现 planning wall (s) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 28 | 7 / 7 | 226 / 0 | 224 | 0.472028130 | 4.634665e-6 | 5.877242e-6 | 0.263211426 | 87.333071 / 86.293949 |
| 29 | 12 / 12 | 194 / 0 | 192 | 0.302033828 | 5.519578e-6 | 1.340348e-5 | 0.269222715 | 58.911033 / 58.989599 |
| 30 | 14 / 14 | 266 / 0 | 264 | 0.319608867 | 6.437022e-6 | 7.291397e-6 | 0.267831508 | 69.779446 / 69.442049 |

三个 seed 的 per-seed summary、完整 result 和所有搜索事件在剥离上述测量字段后均完全相同。事件数量分别为 691、605、825；每个事件中的投影、状态门禁、叶边、树插入和候选计数与归档一致。复现实验总 CPU wall 为 226.960954 s，归档为 228.383145 s；复现 peak RSS 为 268016 KiB，归档为 254392 KiB。这些是机器/负载相关的性能结果，不是科学输出。

## 5. 为什么三个成功 seed 没有主搜索拒绝

事件级统计显示没有隐藏失败：seed 28/29/30 的 `projection/state_gate/leaf_edge` 分别为 `226/226/224`、`194/194/192`、`266/266/264`；状态门禁 invalid 为 0，所有叶边均为 `sampled_edge_valid`，每个完整候选一次。主统计的 collision、closure、joint-limit 和 other rejection 均为 0，NR failure 也均为 0。

这不是保守碰撞几何被跳过，也不是全空间容易的证明。当前固定查询在以下条件下确实较宽裕：没有新增障碍；候选物体中心主要从起点/目标的 x=`0.65..0.82`、y=`-0.075..0` 和 lift 高度运动；每个投影从 HOME 或已接受节点 warm-start，且 NR 有严格的每臂平移/旋转收敛门。独立 FK 对照仍运行，最大 USD/URDF 误差约 2.1e-8 m，说明不是只比较同一 FK 函数。

零 closure rejection 还受到对象条件投影结构的影响：每个采样物体位姿 `X` 直接同时构造左右 `X·G_i` 目标，因此 NR 候选天然落在同一物体条件闭链附近；原 G1-1 仍用独立 FK/SE(3) 门重新检查，所以这不是完全循环，但并非高压力闭链测试。零 collision rejection 的具体原因是 support 边界的 Cube/table 零深度接触由原 support-contact 合同接受；原始 FCL reference 会记录接触，但总体 collision gate 为 valid、reasons 为空，transport 段无残余碰撞，未把 support 接触当成漏检。最大闭链误差仍低于 G1 门，最小 joint margin 仍大于 0.263 rad。

## 6. 负控审计

- **seed 128** 是实际 `Planner.plan` 查询，但在必要 reach sphere 预条件处以 `GOAL_OUTSIDE_NECESSARY_REACH_BOUND` 结束，0 个 sample、NR 调用和边检查。因此它是 planner-level early rejection，不是一次完整采样路径失败，也不能用来估计搜索失败率。
- **seed 129** 是独立 NR 重构后送入原单状态门的错误 grasp 控制，不是完整规划运行。输入物体旋转约 0.2 rad，左右 grasp 对象姿态出现约 0.2 rad mismatch，同时 FCL 检出 4 对 tool/Cube 碰撞。`path_length` 和 continuous status 为 N/A。
- **seed 130** 是独立 NR 重构后送入原单状态门的载荷/table 控制。闭链误差约 `3.10e-11 m / 8.14e-11 rad`，joint margin 约 0.362 rad，但 Cube/table 有 4 个接触、约 20 mm 穿透，clearance 为 `-0.02 m`。它也不是完整 planner path failure。

负控证据保留了原始计数与分类；129/130 的 planner rejection counter 为 0 是因为它们没有调用完整 `plan` rejection 路径，不能改写成 planner 通过。

另一个研究合同缺口是 runner 使用 `any(run.found)` 生成总状态，理论上一个 seed 失败仍可能产生 PASS。当前三个 seed 实际全部 found，且 acceptance evidence 明确检查了三者 sampled+R1，所以本批不受污染；NR-1 必须按 seed 分别报告并要求所有预注册 seed 的结果。

## 7. 连续安全与研究边界

三个候选的六段 B0、独立 19 帧 R1 和静态门禁结果在复现中保持一致。B1 对 joint limits 的连续状态为 VERIFIED，但运动段闭链和连续 collision/combined 状态仍为 `CONTINUOUS_VERIFICATION_INCONCLUSIVE`；没有连续证书、没有 Isaac/ROS/MoveIt/机器人/吸 suction 执行，也没有动力学、噪声、延迟或真实接触载荷证据。

因此本轮确认的是：在冻结离线模型中，P4-inspired NR-RRT 的实现和三 seed 科学输出可复现。它不能推出路径连续安全、规划完备性、成功概率、实时性能或物理可执行性。

## 8. 建议的 G1-P4-NR-1 预注册小实验（仅方案）

本节只冻结方案，不实现、不执行 NR-1。fixture、Cube_01/index0、URDF、ACM、碰撞几何、吸附配置和原验证器保持不变。

**查询族 H1：固定 fixture 起终姿态的冗余分支。** 物体起点和目标严格使用 fixture manifest。用独立 NR projector、固定 endpoint seed 映射 `{131: start, 132: goal}` 和固定 anchor restart 顺序 `[HOME, joint-midpoint, uniform restart 0..5]` 生成 q14；选定 restart 规则在实验前冻结，若 endpoint 生成失败则记录 query invalid，不调参、不换分支。将 q14 与物体 SE(3) geodesic 同时做 128 等距区间的 baseline 检查。

**查询族 H2：固定的 endpoint branch-pair。** 使用同一 fixture 物体起终姿态，但采用预注册的第二个成功 endpoint restart pair `{133: start, 134: goal}`；如果该 pair 不产生两个有效端点，记录为未触发 hard case。H2 只测试冗余分支差异，不改变 fixture 或碰撞允许矩阵。

baseline 是 128 段 `X` geodesic 加 q14 线性插值，在每个点运行原 G1-1 单状态和 G1-2B-0 门禁。只有 baseline 记录到第一个可量化 closure、joint-limit、FCL 或 phase/contact 失败，该 query 才算合格 hard case；baseline 通过则记录 `NOT_HARD`，禁止事后挑选或修改参数。

对每个合格 query 使用固定 planner seeds `{131,132,133}`，沿用当前 PlannerConfig、NR 上限、RBS 深度、B0 采样和 R1 合同，不增加 obstacle、不放宽 ACM、不改变门限。记录每个 seed 的 endpoint status、baseline 首个失败点、planner sampled/edge/closure/limit/collision counters、最大残差、最小 margin、完整 parent path 和 wall time。成功标准是 planner 为该合格 query 找到通过 sampled+B0+独立 R1 的替代路径；若没有替代路径，完整保存失败。B1 仍单独保留 INCONCLUSIVE，不授予执行授权。

该设计先证明“直线基线确实失败”，再测试搜索是否能找到替代路径，避免把当前开阔场景的无拒绝结果误当成规划能力。它不包含 PCS/RSS、QP、RL、物理实验或连续验证器扩展。

## 最终推荐

`G1_P4_NR_0_R1_PARTIAL`：可重复性和算法真实性通过；研究有效性需等待预注册的 hard-case/alternative-path 实验，以及后续独立解决连续碰撞和运动闭链证书。审计到此停止，不自动开始 NR-1。

## 9. 两文件证据归档

本次仅归档本 Markdown 和 `offline/task28_p4_nr_0/evidence/r1_reproducibility_audit.json`。未重跑测试或规划，未修改任何模型、fixture、门限、ACM、验证器、科学验收标准或历史证据。NR-0 的离线 sampled PASS、R1 复现通过、整体研究有效性 PARTIAL 和未获连续安全认证分别保留。

机器记录的 `archival` 段保存以下持久信息：

- 测试命令、日志全文、退出码原文，及各原始文本的 bytes/SHA256。
- 历史实验命令、Python/平台记录、依赖身份来源、线程变量、退出记录、控制台和完整 fresh summary。
- 11 个旧/新 JSON 或 gzip 文件的原始 SHA256、字节数、精确科学规范化摘要、事件记录数和一致性结果；对应原文件全部保留在 NR-0 提交中。
- 精确排除上述六类测量字段的比较 Python 源码及其 SHA256。JSONL 逐条规范化后追加 LF，保留事件顺序；JSON 对键排序、紧凑编码为 UTF-8 后计算 SHA256。
- 已提交的 manifest、依赖身份、保护清单、artifact 清单、验收记录与原审阅记录的 SHA256；再次核验 369 个原文件和 52 个 NR-0 清单文件未变。

`/tmp/task28_g1_p4_nr_0_r1_exp2` 仅作为本次可读的历史来源，不是永久归档位置。新鲜大体积 raw gzip 没有加入提交；其科学内容通过规范化摘要与已提交的 NR-0 完整归档逐项绑定，性能摘要和小型退出/日志记录则嵌入两个归档文件。临时目录消失后仍可按记录重做实验并比较科学输出，但无法从本提交单独恢复新鲜 raw 文件中逐调用的耗时字节；这项证据保存边界明确保留，不影响或扩大原科学结论。

在仓库根目录，只重做已存在结果的字段比较可执行下列命令（不启动规划器）：

```bash
python3 -c 'import json; p="offline/task28_p4_nr_0/evidence/r1_reproducibility_audit.json"; d=json.load(open(p)); exec(d["archival"]["comparison"]["reproduction_python"])' /path/to/existing/fresh_output
```

NR-1 仍仅为第 8 节历史建议草案；归档没有完善、实施或授权该实验协议。到证据归档完成后停止。

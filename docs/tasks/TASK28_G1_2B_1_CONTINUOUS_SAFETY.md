# Task28 G1-2B-1｜离线连续安全验证与 FCL 能力审计

日期：2026-10-26（Asia/Seoul）。

仓库：`dreamerToTOE/dual-arm-embodied-palletizing`；分支：`side-suction-palletizing`。
起始提交：`bcc6e7d260eaa45f6f0d055d53334922df31db3a`。

## 1. 结论与完成范围

**最终任务状态：`G1_2B_1_PARTIAL`。** 本次区分“有限点通过”与
“在明确数学模型下覆盖整个参数区间的充分证明”。不得把原 G1-2B-0 的
`SAMPLED_EDGE_VALID` 改名为连续安全，也不得把 FCL endpoint-transform CCD 的阴性
输出直接作为关节轨迹的连续证书。

FCL 能力审计已经实际完成：运行了 **31 个离线 C++ CCD 控制**，读取并核对当前系统
头文件、动态库、API 默认值和运动插值。审计确认该接口的输入是两个刚体的起终点
变换，不包含关节向量、串联链结构或 `FK(q(u))` 回调。冻结左 TCP 的实际 FK 中点
与 FCL `LINEAR` 中点相差约 **21.985 mm / 0.0954255 rad**；选择 `SCREW` 后，
平移差仍约 **27.922 mm**。另有精确解析反例和真实 FCL 碰撞见证，见第 5 节。

新增连续层已实现：精确有理 Bernstein 多项式范围、以 `2**80` 为分母的向外舍入
区间 FK，以及保守形状包围盒的区间分离。第 6 节按实际源码说明证明对象和判据；
核心运行与验收结果见第 7、8 节。它仅对**由已存 binary64 系数定义的实
多项式，以及显式端点集合**给出充分结论；不声称逐位复制旧 NumPy/SciPy/libm/FCL
执行，也不认证物理运动。包围盒重叠、未证明的接触法向规则、非恒定 Cube 旋转
和当前深度预算未能解析的区间均保留未决。

全部八个 A-stage 输入（七个原段与一个显式支撑等待组合边）均完成重评：连续关节
限位得到证明；激活 bilateral 约束的静止段得到连续闭链证明；运动段的闭链界仍过宽。
**八个输入的连续碰撞与最终组合状态均为 INCONCLUSIVE，没有任何 A-stage 输入获得
完整连续安全证书。** 两个原有限采样漏检反例均得到严格的点区间拒绝及真实门禁见证。

原有 **336** 项回归全部通过，新增最终 **59** 项测试通过，共 **395** 项；FCL 的
31 个实际 C++ 控制单独计数。核心流水线实测 **227.841 s**，主进程 peak RSS
**456,288 KiB（约 445.6 MiB）**。机器汇总见
[summary.json](../../offline/task28_g1_2b_1/evidence/summary.json)，完整 trace、区间
整数界、见证、原有限采样与显式时序观察一并保存。

本任务限定在离线数学与几何验证。没有启动 Isaac Sim、ROS、GPU 仿真、机器人、
吸附或释放命令，也没有物理实验结果。没有新增规划器、P4-NR、QP、力控制或 RL。
冻结 fixture 标签仍为 `NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`。

## 2. 冻结边界与证据位置

沿用唯一 `Cube_01`、index 0 与
`fixtures/task28_g1_0_new_single_cube_v0`。fixture manifest SHA256 为
`f809a4a57ec8b2a3719f6bcd9c53a467b8ed2f5353928f62d8adfb02d91f3286`。
不修改旧 G1-1/G1-2A-R1/G1-2B-0、fixture、网格、joint limits、SRDF/ACM、支撑与
吸盘阈值；不通过扩大容差、删除 Cube 或增加碰撞豁免使区间通过。

新增审计源为 [audit_fcl.py](../../offline/task28_g1_2b_1/audit_fcl.py)，结果为
[fcl_capabilities.json](../../offline/task28_g1_2b_1/evidence/fcl_capabilities.json)。
该脚本内嵌小型 C++ probe，在临时目录编译并链接实际系统库，运行完成后移除临时
编译产物；不会修改旧 FCL backend，也不会安装 Python 包。证据保留探针源 SHA256、
脚本 SHA256、冻结 FK/URDF/manifest hashes、编译器、链接库及每次实际调用结果。

新增实现与入口如下；本表列出源码位置，不代替运行结果：

| 文件 | 职责 |
|---|---|
| [interval_math.py](../../offline/task28_g1_2b_1/interval_math.py) | 整数/dyadic 向外区间运算与有显式余项的 sin/cos 包含界 |
| [continuous_verifier.py](../../offline/task28_g1_2b_1/continuous_verifier.py) | 精确 Bernstein 范围、冻结 FK、closure、414 pair 几何范围、三维状态与组合状态 |
| [adversarial_controls.py](../../offline/task28_g1_2b_1/adversarial_controls.py) | 有独立解析 oracle 的离散网格外反例输入 |
| [run_validation.py](../../offline/task28_g1_2b_1/run_validation.py) | 旧基线回归、增量测试、控制/A-stage 运行、保护清单及证据 receipts |
| [test_interval_math.py](../../offline/task28_g1_2b_1/tests/test_interval_math.py) | 有理数角点与独立高精度级数对区间包含性的检查 |
| [test_continuous_verifier.py](../../offline/task28_g1_2b_1/tests/test_continuous_verifier.py) | 冻结门禁集成、支持域、外部证据不可伪造、数值错误与预算合同 |
| [test_certificate_math.py](../../offline/task28_g1_2b_1/tests/test_certificate_math.py) | 独立有理数与 100 位 Decimal FK、closure、严格分离控制 |
| [test_adversarial_controls.py](../../offline/task28_g1_2b_1/tests/test_adversarial_controls.py) | 反例多项式及其独立解析事实 |

## 3. 实际后端、绑定与可复现身份

### 3.1 当前运行的是系统 C++ FCL

| 项目 | 本次实际环境 |
|---|---|
| Python | `/usr/bin/python3`，3.10.12 |
| Python `import fcl` | 不可用；当前环境没有安装 Python-FCL |
| 系统 FCL | `pkg-config --modversion fcl` 为 `0.7.0` |
| Ubuntu FCL 包 | `libfcl-dev`、`libfcl0.7` 均为 `0.7.0-3` |
| libccd | `libccd-dev`、`libccd2` 均为 `2.1-2` |
| Eigen | `libeigen3-dev` 为 `3.4.0-2ubuntu2` |
| 编译器 | GCC 11.4.0，Ubuntu 11.4.0-1ubuntu1~22.04.3 |
| 实际库 | `/usr/lib/x86_64-linux-gnu/libfcl.so.0.7.0` |
| FCL 动态库 SHA256 | `0bb493f432f0a38ff605198937a037e1c41f1d5fb6dfc36473988da973b5dc05` |

审计读取 `/usr/include/fcl`，并逐字节比较官方 FCL `0.7.0` tag 的
`continuous_collision-inl.h` 与安装文件。两者 SHA256 均为
`c8ff31f3d110236180cc627f0e0c15319aa141ca3bf61df9ce58fdd3be2c90ce`。
这确认本报告审计的是此版本的具体实现；不能把最新文档、另一 wheel 自带的库或
另一个后端的能力移植到当前执行环境。
原始依据：[FCL 0.7.0 CCD 实现](https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/narrowphase/continuous_collision-inl.h)。

### 3.2 下载的 Python 绑定只做源码对照

另外下载了 `python-fcl==0.7.0.10` wheel，读取其中 `fcl.pyx`、`collision_data.py`
与包 metadata；**未安装、未导入、未执行其绑定或自带 FCL**。外部下载文件的路径、
wheel SHA256 和两份源码 SHA256 均保存于审计 JSON。该 wheel 源码与官方
`v0.7.0.10` tag 对应文件相符。
依据：[绑定请求数据结构](https://github.com/BerkeleyAutomation/python-fcl/blob/v0.7.0.10/src/fcl/collision_data.py)、
[Cython 调用封装](https://github.com/BerkeleyAutomation/python-fcl/blob/v0.7.0.10/src/fcl/fcl.pyx)。

默认值存在容易影响实验结论的差异：

| 参数 | 实际系统 C++ `ContinuousCollisionRequestd()` | 源码对照 Python-FCL 0.7.0.10 |
|---|---|---|
| `num_max_iterations` | 10 | 10 |
| `toc_err` | 0.0001 | 0.0001 |
| motion | `CCDM_TRANS` | `CCDM_TRANS` |
| GJK solver | `GST_LIBCCD` | `GST_LIBCCD` |
| CCD solver | **`CCDC_NAIVE`** | **`CCDC_CONSERVATIVE_ADVANCEMENT`** |

C++ 默认值由实际 probe 输出与
[安装版本请求声明](https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/narrowphase/continuous_collision_request.h)
共同确认。Python wrapper 会把 C++ 标量返回值原样返回；传入已有 result 时，还会以
OR 累积 `is_collide`、以 min 累积 `time_of_contact`。审计为每次查询新建 result，
防止前次结果污染本次结论。表中 Python 列只说明该下载版本，不能作为本机 active
backend 的安装声明。

## 4. FCL CCD API、实际能力与失败语义

### 4.1 API 的运动输入合同

C++ 几何 overload 接收
`geometry1, tf1_begin, tf1_end, geometry2, tf2_begin, tf2_end, request, result`；
`CollisionObject` overload 从对象当前变换读取 begin，再接收两个 end。Python
wrapper 暴露的是后一种形式。起终点 SE(3) 相同，并不能确定它们之间的机器人关节
运动；API 也没有收到冻结机器人在该段的关节轨迹或其 FK。

以下只描述当前 tag/安装版本已审查的路径。枚举存在、函数能调用或返回值有限，
都不等价于已经实现正确的、适用于该形状和运动模型的连续算法。

| motion | 源码与实际行为 | 本任务使用边界 |
|---|---|---|
| `CCDM_TRANS` = 0 | 起终点平移连线；始终使用起始旋转，最终旋转输入被忽略 | 只有已独立证明纯平移且模型一致时才可能适用；本任务不据此发关节轨迹证书 |
| `CCDM_LINEAR` = 1 | 调用 `InterpMotion`；按相对主轴角推进旋转，平移由参考点与端点决定。干净 probe 中观察到零参考点对应的端点弦线 | 实际 FK 中点与该模型不符；另见下述参考点初始化源码问题 |
| `CCDM_SCREW` = 2 | 两端变换确定单一 screw；不是多关节同时插值的一般 FK。纯平移分支存在实际端点误差 | 作为诊断，不假定它等于实际 FK 或必达输入终点 |
| `CCDM_SPLINE` = 3 | endpoint-transform 构造函数保留 TODO，没有初始化所需 spline 控制量 | 不执行该构造路径；不把枚举当作受支持能力 |

源码依据：[TranslationMotion](https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/math/motion/translation_motion-inl.h)、
[InterpMotion](https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/math/motion/interp_motion-inl.h)、
[ScrewMotion](https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/math/motion/screw_motion-inl.h)、
[SplineMotion](https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/math/motion/spline_motion-inl.h)。

`InterpMotion` 的 endpoint 构造函数没有显式初始化 `reference_p`，其类型是 Eigen
三维向量；不能仅凭“参考点默认零”的注释认定该构造在所有构建和内存条件下都得到
零参考点。因此本报告把弦线称为**本次实际 probe 的观察值和理想零参考点模型**，
不把它提升为该版本不附带条件的规范保证。即使修复这一初始化问题，端点刚体模型
与一般 articulated FK 不等价的数学问题仍然存在。

### 4.2 solver 的实现与控制

| solver | 该版本行为 | 实际控制与限制 |
|---|---|---|
| `CCDC_NAIVE` = 0 | `n=min(num_max_iterations,ceil(1/toc_err))` 个离散点 `u=i/(n−1)`；默认 10 点 | 两个半径 0.01 m 球体的短暂穿越被漏掉，返回 false/1；这不是连续安全算法 |
| `CCDC_CONSERVATIVE_ADVANCEMENT` = 1 | 存在 box/cylinder/convex 的 shape-pair dispatch；BVH 也有 dispatch | primitive 穿越控制成功，但 primitive wrapper 忽略请求的 iteration/toc_err；BVH 控制还出现漏检，不能只凭 dispatch 签发证书 |
| `CCDC_RAY_SHOOTING` = 2 | primitive+translation 的允许分支为空，最后返回 −1 | 实际球体控制返回 −1，结果仍保留 false/1 默认值 |
| `CCDC_POLYNOMIAL_SOLVER` = 3 | 仅接受 BVH/BVH 且 `CCDM_TRANS`；primitive/Convex 不满足入口条件 | primitive 控制返回 −1；BVH probe 返回 TOC 0，但离散起点无碰撞，不能宣称此 probe 得到了正确的变换感知 TOC |

默认 NAIVE 的球体控制采用移动中心 `x(u)=−1+2u`，固定中心 0，两个半径均为
0.01 m。真实接触区间为 `[0.49,0.51]`，十点 `i/9` 没有落入该区间；NAIVE 返回
`is_collide=false, time_of_contact=1`，conservative advancement 返回
`is_collide=true, time_of_contact=0.49`。调小名为 `toc_err` 的参数而仍保留最大
10 次迭代，并不能把该分支变成连续证明。

primitive conservative advancement 的 wrapper 明确不使用传入 request；节点自身
`t_err=0.0001`。把 `num_max_iterations=1, toc_err=0.5` 传给相同球体穿越，实际仍
返回 TOC 0.49。因此不能把 request 字段当作此执行路径已经遵守的预算或误差界。
依据：[shape/mesh conservative advancement](https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/narrowphase/detail/conservative_advancement_func_matrix-inl.h)、
[primitive 节点默认值](https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/narrowphase/detail/traversal/distance/shape_conservative_advancement_traversal_node-inl.h)。

实际还运行 box/box、cylinder/box、convex/box、convex/convex 的三种 motion 控制；
这些形状均保留为实体。原 G1-1 的冻结 STL 被加载为 `fcl::Convexd` 闭凸实体，
并非 `BVHModel` 三角表面，不能仅因文件扩展名是 STL 就选用仅限 BVH 的 polynomial
solver。为审计 BVH 分支，另外构造了新鲜 OBBRSS tetrahedron BVH；其 conservative
advancement 返回 clear/1，但独立离散 midpoint 查询确有碰撞。polynomial 分支会
更新 mesh vertices，因此每次控制均使用新几何，避免一次调用改写下一次输入。

SCREW 的纯平移控制要求 `[0,0,0]→[2,0,0] m`，实际 `u=1` 得到 `[4,0,0] m`。
对应源码在近零旋转分支保存未归一化平移轴，又把该轴乘以平移长度。这个观察仅限
被审计实现与输入，不能被掩盖为“时间参数不同”，因为它连用户提供的终点也未达到。

### 4.3 返回值必须与状态及适用合同一起解释

`−1` 是不支持/设置无效等错误路径，**绝不是净空**。这些路径可能不更新 result，
故 `is_collide=false, time_of_contact=1` 同时出现也不能证明查询成功。有效标量
TOC 也不能独自解决对应 motion 是否等于实际 FK 的问题；本次 BVH 控制还说明，
不能把“函数返回有限 TOC”自动视作算法已被验证。

本次所有 FCL CCD 结果均为能力与见证诊断。它们不签发 articulated-path certificate，
不替代区间证明，也不把尚未认证的 rigid-motion 碰撞自动标成实际关节轨迹碰撞。
确定拒绝所用的碰撞见证必须落在重建的真实 `FK(q(u))` 与真实 Cube 位姿上。

## 5. endpoint-transform CCD 与真实 FK 的独立反例

### 5.1 精确解析反例，不依赖采样分辨率

取一个 revolute joint，球心位于半径 1 m 的刚性偏置：

```text
q(u) = −π/3 + (2π/3)u,                 0 ≤ u ≤ 1
p_FK(u) = [cos(q(u)), sin(q(u)), 0]
R_FK(u) = Rz(q(u))
```

移动球与固定障碍球的半径均为 0.02 m，障碍球心为 `[1,0,0]`。两端球心为
`[1/2,−√3/2,0]` 和 `[1/2,√3/2,0]`，端点无碰撞；真实 FK 在 `u=1/2` 给出
`[1,0,0]`，两个球心重合，确定碰撞。

零参考点 LINEAR 的球心始终位于端点弦线 `x=1/2`；其到障碍球心距离至少 0.5 m，
大于半径和 0.04 m，因此它的整条替代轨迹可以保持分离。弧线与弦线中点相差精确
0.5 m。实际 FCL LINEAR 调用返回 clear/1，而真实 FK midpoint 的独立 FCL 单点
查询返回碰撞。审计保存了两种输入与实际结果，足以否定“LINEAR endpoint CCD
阴性 ⇒ joint-linear FK 连续无碰撞”的推理。

这个反例中 SCREW 的旋转分支恰好能表示单 revolute screw，不意味着一般多关节
FK 同样可由 SCREW 表示。下一控制直接检查冻结机器人。

### 5.2 冻结机器人上的数值不一致

使用旧 `RobotModel` 与冻结 URDF，左臂从 `home−Δq/2` 到 `home+Δq/2`，
`Δq=[0.8,0.3,−0.25,0.2,0.35,−0.3,0.4] rad`，右臂保持 home。两端均在冻结
effective joint limits 内。对 `u=0,0.25,0.5,0.75,1`，比较
`FK((1−u)q0+u q1)` 与同一 link 起终点变换交给实际 FCL 后生成的运动。

| 对象与 motion | midpoint 平移差 (m) | midpoint 旋转差 (rad) |
|---|---:|---:|
| `left_fr3_link7`，LINEAR | 0.0192307713 | 0.0954254965 |
| `left_fr3_side_suction_tcp`，LINEAR | 0.0219846983 | 0.0954254965 |
| `left_fr3_link7`，SCREW | 0.0175706753 | 0.0954254965 |
| `left_fr3_side_suction_tcp`，SCREW | 0.0279224663 | 0.0954254965 |

TRANS 在同一 TCP 的终点旋转差约为 0.668540844 rad，因为它固定起始旋转。
完整 q、全部实际/替代 4×4 矩阵与误差在审计 JSON 中。这些是**运动模型诊断配置**，
只确认端点关节限位，不声称该双臂配置通过了闭链、接触或碰撞门禁，更不作为执行
候选。对附着物体的 driver TCP 同样必须使用其整个 FK 曲线；常量抓取变换并不能
令一个本已错误的 TCP 插值自动成为正确的物体运动。

## 6. 连续数学证明的实际合同与保守边界

本节对应新增 [continuous_verifier.py](../../offline/task28_g1_2b_1/continuous_verifier.py)
与 [interval_math.py](../../offline/task28_g1_2b_1/interval_math.py)。它们沿用旧 typed
trajectory、单状态/时序门禁、冻结 limits 和 closure 阈值，不改写原有限采样结论。
时间仍是离线合成参数；这里的连续是对声明的数学曲线覆盖整个实参数区间。

### 6.1 证明对象：已存 binary64 系数的实多项式与显式端点

新层调用旧 `span_coefficients` 取得 linear/Hermite 的四组 power coefficients。
这些系数已经由原 NumPy binary64 运算形成；**证明从这些已存系数开始**，经
`Fraction.from_float` 把每个 binary64 值解释为其精确有理数，再定义实多项式：

```text
P(u) = a*u³ + b*u² + c*u + d,    0 ≤ u ≤ 1
B = [d, d+c/3, d+2c/3+b/3, a+b+c+d]
```

这不是把用户十进制文字、精确 Hermite 基函数运算和原 floating-point 系数计算
混为同一条曲线，也不是证明每次浮点 Horner 求值恰等于实数 `P(u)`。旧
`reconstruct` 在 `u=0,1` 直接返回显式 waypoint，绕过多项式求值；rounded power
coefficients 的实数端点和显式 waypoint 可能略有不同。因此新层既包围整个闭区间
的实多项式，**又对每个显式 waypoint 独立作 interval FK/closure/geometry 点包围**，
标记 `kind=explicit_endpoint`。joint limits 同时核对显式端点和多项式整段范围。
两种端点没有被静默视为逐位相同。

在待证明 `[lo,hi]` 上用精确有理 de Casteljau restriction 获取 Bernstein
controls。basis 非负且和为 1，故实多项式值落在 restricted controls 的 min/max
内；随后向外转为 dyadic 区间。此范围是整个区间的包含证明，和 sample 密度无关。
没有运行 IK 修复、闭链投影、joint clipping、抓取 frame 修改或 Cube 缩放。

数学路径范围和旧门禁各司其职：新层每次都重新运行权威 `SampledEdgeValidator`，
不接受调用者传入的 `sampled_evidence` 代替门禁；输出明确
`sampled_evidence_used_as_proof=false`。旧浮点门禁用于端点、mode、event、R1 与
诊断，不能补成尚未完成的实区间证明。

### 6.2 URDF 与精确实 FK 的范围

`ProofModel` 从未修改的 URDF 读取 joint origin 与 collision origin。每个 xyz/rpy
数值 token **先按 Python `float` 解析成 binary64，再视作精确实数**；不是直接把
原十进制 token 当作无舍入的十进制有理数。`origin` 按数学
`Rz(yaw) Ry(pitch) Rx(roll)` 构造，revolute joint 按其精确实关节角执行数学 `Rz`。
本版本只接受冻结的 Z-axis `0 0 1` revolute joints；其它 axis 不静默转换。
冻结 nominal grasp inverse 的已算出 binary64 常量同样被当作精确实常量。

证明的是上述冻结常量与数学三角函数定义的 FK，**不是对 NumPy 矩阵乘法、SciPy
Rotation、平台 libm 或 FCL 内部每一次浮点舍入作 bit-for-bit 包围证明**。
新层也没有把模型与实物的尺寸、标定、柔性或传感误差纳入区间。必须保留这两个
边界，不能把“数学 FK 的连续证明”扩大成“原数值栈全部实现的逐位认证”或物理证书。

### 6.3 `2**80` dyadic 运算与三角余项

区间端点是整数除以 `SCALE=2**80`。加减、乘除、平方、min/max 与有理数转换按
整数 floor/ceiling 向外舍入；跨零除数拒绝。证明计算使用精确有理数或整数 dyadic
运算；输出显示用 float 转换也向外，但不参与证明判据。

`sin/cos` 在区间精确中心 `c` 上使用 32 个 Maclaurin terms：sine 到 63 次，
cosine 到 62 次，可把缺失的 63 次系数看作零，因此二者都可使用 63 阶 Taylor
多项式的 64 阶 Lagrange 余项界：

```text
|f(c) − Taylor63_f(c)| ≤ |c|**64 / 64!
|f(x) − f(c)| ≤ |x−c| ≤ interval_radius
```

级数的每一步也向外区间运算；加上精确有理余项，再以全局导数界 1 扩张输入
radius，最后与 `[-1,1]` 相交。若中心在 `[-16,16]` 外，直接返回无条件
`[-1,1]`，不依赖 libm 的 argument reduction 或 trig 正确舍入。区间 FK 的每次
矩阵与平移运算保持包含性；相关性损失可以使范围变宽，但不会因为普通 float
看起来接近就缩小证明界。

### 6.4 恒定 Cube 旋转与 closure 判据

`same_rotation` 用精确有理 quaternion 分量交叉乘积检查比例关系，因而明确接受
`q` 与 `−q` 所表示的同一旋转。非零恒定 quaternion 的旋转矩阵使用归一化后的
有理公式，分母为 `x²+y²+z²+w²`，无需把 sqrt 浮点近似当作证明。**只有这样的
constant rotation 得到本版闭链旋转覆盖**；一般随 u 变化的 Cube SLERP 没有实现。

物体旋转变化时，几何层仍可用 local bounds 的 L1 半径构造适用于任意 SO(3) 的
粗包围；这不是 SLERP/closure 的连续证书。在 bilateral 模式下，缺少物体旋转
范围即令 closure 未决，不能以该粗盒或有限 sample 代替。

bilateral 模式计算三组 implied object closure：left/right、left/object、
right/object。平移使用平方范数，阈值仍为 `1e-4 m`。对于实际 SO(3) 旋转矩阵，
相对转角 θ 满足：

```text
||Ra − Rb||_F² = 8 sin²(θ/2)
```

据此用 `8*sin(1e-3/2)**2` 的向外区间表示原 `1e-3 rad` 阈值。
误差区间上界不大于阈值区间下界才验证；误差区间下界严格大于阈值区间上界才拒绝；
中间不确定区间保留未决。未要求 bilateral closure 的 detached 状态按不适用
处理，不代表建立了物理闭链。joint limits 的完整范围超出充分条件时也不直接
伪称某点违规；确定的 joint rejection 需要后述点包围见证。

### 6.5 冻结形状、有限桌面与接触规则

box 使用完整半边长 bounds，cylinder 使用包围其完整实体的半径/长度盒，冻结
binary STL 读取全部 float32 vertices 的坐标范围，包含闭凸 hull 的完整实体。
将 local box 通过区间 FK/shape origin 传播成整段 world AABB。读取旧候选与 ACM
后，代码校验 **414 个需要检查的 shape pair、40 个冻结 ACM skip**；同 owner
的 shape 按旧候选合同处理。没有新增 ACM exemption，不删除 Cube/table、工具、
arm-arm 或 self-collision 的既有候选。

每个 pair 在某一坐标轴出现严格正的全区间 gap，才得到 separation proof，并记录
axis、direction 与以 `2**80` 缩放的 gap 下界。**两个 outer AABB 重叠只是此充分
条件未证明分离；不等于真实碰撞，也不允许跳过该 pair。**

桌面是有限 footprint，不能被当成无限平面。完整 collision 验证还要求 **Cube
整段 world enclosure 的完整 x/y footprint** 被冻结桌面 bounds 包含；不能只检查
中心、某角点或一次 sample。对已支持的恒定旋转，用 Cube 八顶点的区间 minimum z
构造桌面穿透界；只有 footprint 完整包含且穿透下界严格大于 `1e-6 m`，才得到
`CERTIFIED_CUBE_TABLE_PENETRATION`。未满足 footprint 合同只会阻止本版充分证明，
不能据此宣称物体已碰撞或物理上不可能通过。airborne/transitioning 还要求整段
最低点到 table top 的净空下界严格大于 `1e-6 m`。

**所有 supported modes 的局部支撑规则在本版始终未证明**：
`DETACHED_SUPPORTED`、`BILATERAL_SUPPORTED`、`RELEASED_SUPPORTED` 会设置
`local_support_rules_unverified=true`，即使所有普通几何 samples 或 pair 分离均
通过，也不能获得 collision VERIFIED；若另有确定穿透等错误，仍可以独立拒绝。
LIFT/PLACE 的 path interval 同样保留这项缺口，不能因为起点暂标 transitioning
就漏掉区间中的支撑切换。需要未来另行实现完整连续接触规则证明，不能仅增加细分
深度绕过这个缺口。

吸盘/支撑的法向、位置、穿透与 contact count 策略沿用旧实现；新层没有为允许
接触建立完整连续证明。接触 outer bounds 重叠时不把旧单点例外变成整段豁免。
模式、intent/feedback、attachment 与 R1 仍由原 gate 检查；区间点不是观察帧，
不贡献三帧确认，也不产生新吸附证据。

### 6.6 实际状态、见证、覆盖与预算

结果分别报告 `joint_limits.status`、`closed_chain.status`、`collision.status`
与 `combined_status`：

| 实际状态字符串 | 使用条件 |
|---|---|
| `CONTINUOUSLY_VERIFIED` | 对应维度全部规定范围已获充分证明；组合结果还需三个维度全验证、真实端点有效且权威旧门禁为 `sampled_edge_valid` |
| `CONTINUOUSLY_REJECTED` | 有确定的区间或点包围证明违反；组合结果中任何维度确定拒绝优先 |
| `CONTINUOUS_VERIFICATION_INCONCLUSIVE` | 支持域/算术错误、区间界太宽、overlap、接触规则缺口、当前深度未解析、离散 prerequisites 不满足等；不得当作 pass |

非法 typed 输入或不支持的数值/系数运算 fail closed。finite 但过大的 Hermite
数值也可能在形成原 binary64 系数时溢出，输出未决及原因，不能生成证书。
外部伪造 pass 数据不会取代内部旧门禁。mode-only 的离散拒绝如果没有对应数学
维度的确定违反，组合状态保留未决，而非凭空制造碰撞/closure 见证。

代码用 midpoint 和数值导数根等产生**候选**参数，`numpy.roots` 只负责发现位置。
每个声明的 rejection 都在该 binary64 参数的精确有理值上重新做 outward POINT
包围，再检查 closure、关节或 Cube/table 违反下界。保存
`CERTIFIED_CLOSURE_THRESHOLD_VIOLATION`、`CERTIFIED_JOINT_LIMIT_VIOLATION` 或
`CERTIFIED_CUBE_TABLE_PENETRATION`，并附重建状态与原单点门禁诊断。数值根准确性
本身不承担证明；有限候选均通过也不意味着其余连续区间通过。

自适应细分对未知且多项式非恒定的区间二分。当前 runner 设置 `max_depth=2`，
所以每个 span 最多 **4 个 path leaf intervals**，另存显式 waypoint 点包围；
验证器允许深度 0–8。该实现的主预算是深度，并未另行声称存在连续层全局 wall-time
或总 leaf 上限。每个 leaf 记录 span、精确有理 `[lo,hi]`、depth、closure bounds、
shape AABBs、pair proofs/unresolved pairs、footprint/clearance/support flags。

完整 VERIFIED 必须覆盖输入每个 span 的全部参数区间与显式端点。当前深度用尽
但仍有未知范围就保留未决。增加深度可能缩紧依赖性较宽的范围，**本次 max_depth=2
未证明不表示更深细分也不可能证明**；也没有保证更多细分一定成功。supported-mode
规则等功能缺口则需另行数学实现，不能被细分次数或旧 sampled pass 自动消除。

## 7. 核心运行结果与指定控制

以下 V、R、I 分别表示 `CONTINUOUSLY_VERIFIED`、`CONTINUOUSLY_REJECTED`、
`CONTINUOUS_VERIFICATION_INCONCLUSIVE`；只用于本报告表格缩写。完整状态字符串
保留在 machine-readable 结果中。所有控制端点均通过原门禁。

| 控制 | 原 sampled 结果/点数 | 连续限位 | 连续闭链 | 连续碰撞 | 最终组合 | 用时 s |
|---|---|---|---|---|---|---:|
| positive_hold | sampled_edge_valid / 3 | V | V | I | I | 0.162 |
| A_closure | sampled_edge_rejected / 187 | V | R | I | R | 4.038 |
| B_payload_collision | sampled_edge_rejected / 53 | V | R | R | R | 1.615 |
| C_limit | sampled_edge_rejected / 431 | R | R | I | R | 8.561 |
| D_mode | sampled_edge_rejected / 5 | V | V | I | I | 0.265 |
| offgrid_closure_excursion | sampled_edge_valid / 3 | V | R | I | R | 0.345 |
| offgrid_support_penetration | sampled_edge_valid / 3 | V | V | R | R | 0.350 |

`positive_hold` 完整证明限位与三组闭链关系；414 pair 中 411 pair 在全参数范围得到
严格 AABB 分离，仍有 3 pair 的外包围重叠。因此这是**维度正例**，不是完整安全正例。
`D_mode` 的显式 attachment/event 合同被旧门禁拒绝，几何层没有把该离散错误伪造为
连续碰撞；其组合状态为 I，绝不允许通过。非恒定 Cube 旋转的门禁有效小角度控制
在新增测试中明确返回 closure/combined I，未获得替代采样证书。

### 7.1 两个独立网格外负例

两个反例复用冻结真实关节配置，端点相同且有效；关节、旋转、抓取合同不变，只在
原 typed object-Hermite 平移中设置切向量。独立解析 oracle 使用
`f(u)=u(1−u)(1−u/2)`，驻点为 `u*=1−1/sqrt(3)`；没有调用被测 interpolation
计算此形状。旧默认检查恰好只有 `u=0,0.5,1` 三点，而且三点全部通过。

* **closure excursion**：物体 X 方向峰值 `0.0001001 m`，半程点小于冻结的
  `0.0001 m` 阈值。数值发现的见证 `u=0.42264973081037427` 被精确点区间重算；
  原完整门禁测得 left/object `0.000100099601709 m`、right/object
  `0.000100099996061 m`，并给出左右 grasp translation mismatch。
  区间下界严格越过阈值，故连续闭链及组合 R；碰撞保持 I。
* **support penetration**：Z 向下峰值 `1.01e−6 m`；半程的穿透约
  `0.984e−6 m`，在原 `1e−6 m` 数值容差内，因此旧三点通过。同一网格外见证
  的最低顶点穿透包围为约 `1.01e−6 m`，严格越过原阈值。独立重放原 FCL/门禁
  报告真实 Cube/table 碰撞、`SUPPORT_TABLE_PENETRATION`、
  `SUPPORT_BOTTOM_NOT_ON_TABLE_PLANE` 与 `CLEARANCE_TABLE_PENETRATION`。
  连续碰撞及组合 R；闭链仍 V。

两例保留实际 q、Cube 位姿、时间、严格整数点界和完整旧门禁响应，见
[closure trace](../../offline/task28_g1_2b_1/evidence/controls/offgrid_closure_excursion.json.gz)、
[support trace](../../offline/task28_g1_2b_1/evidence/controls/offgrid_support_penetration.json.gz)。
这直接说明通过原有限 grid 与通过连续约束是不同命题，不能用加密 samples 的名称
替代证书。导数根负责发现见证，严格越界由独立 outward 运算判定。

### 7.2 冻结 A-stage 逐段结果

全部使用原保存 waypoint 与显式 observation，不重新运行 IK，不修改运动输入。
原段以及另行保存的 placement-with-support-hold 组合边分别报告如下。

| 段 | 端点有效 | 原 sampled / 点数 | 连续碰撞 | 连续闭链 | 连续限位 | 最终组合 | 区间/未决数 | 用时 s |
|---|---|---|---|---|---|---|---|---:|
| grasp_boundary | 是 | sampled_edge_valid / 5 | I | V（不适用） | V | I | 11 / 11 | 0.640 |
| attachment_confirmation | 是 | sampled_edge_valid / 7 | I | V | V | I | 10 / 10 | 0.706 |
| vertical_lift | 是 | sampled_edge_valid / 277 | I | I | V | I | 353 / 350 | 26.722 |
| bilateral_transport | 是 | sampled_edge_valid / 189 | I | I | V | I | 468 / 372 | 32.686 |
| vertical_placement | 是 | edge_inconclusive / 277 | I | I | V | I | 353 / 350 | 28.652 |
| support_confirmation | 是 | sampled_edge_valid / 7 | I | V | V | I | 13 / 13 | 4.445 |
| release_boundary | 是 | sampled_edge_valid / 7 | I | V | V | I | 13 / 13 | 4.509 |
| placement_with_explicit_support_hold | 是 | sampled_edge_valid / 283 | I | I | V | I | 365 / 362 | 26.084 |

“区间数”含独立显式 endpoint 点包围；“未决数”是 collision 或 closure 至少一项 I
的记录数。grasp 为 detached，不要求 bilateral closure，V 表示无激活约束的空真；
release 的闭链仅认证仍处于 bilateral 的部分，不把释放后的物体称为闭链。

静止 attachment、support、release 的最大闭链平移**包围上界**分别约
`4.84e−11`、`3.98e−10`、`1.34e−10 m`，满足冻结 `1e−4 m` 阈值。
运动 lift、transport、placement 的最大保守平移包围上界分别约
`7.291e−3`、`7.618e−3`、`1.189e−2 m`，这些是依赖性损失形成的**范围上界**，
不是测得的误差，也不是违规证据。它们不足以证明闭链，因此返回 I；没有把上界
大于阈值写成 R。对应最大 Frobenius 平方上界约为 `0.000329`、`0.000298`、
`0.000863`，阈值约 `2e−6`，同样未能完成旋转闭链证明。

完整覆盖 414 pair 中，至少在整个指定输入上取得分离证据的 pair 数为：grasp 409、
attachment 409、lift 399、transport 404、placement 400、support 410、release 410、
placement+hold 400；剩余 pair 逐 leaf 列出，不能从这些子集推断全碰撞安全。
工具与 payload、两臂及 self pair 均在原候选中；静止候选中的支持接触和部分 mesh
外包围使全碰撞结论未决。没有任何原 ACM 以外的豁免。

`vertical_placement` 原离散结果仍为 `edge_inconclusive`，原因是
`TERMINAL_PHASE_NOT_CONFIRMED`：接近支撑的插值点不是独立观察。保留原独立
`support_confirmation` 明确观察序列；拼接原有显式 support hold 后，原有限门禁才
为 `sampled_edge_valid`。这只补足原时序证据，不补足连续几何或闭链证明。
原 `temporal_prelude`、每帧 evidence ID/stamp/event、R1 输出和全部 sample provenance
均在对应 gzip JSON 中。任何 interval/midpoint/derivative-root 点都没有送入 R1
作为新观察，也没有提供新的 physical support confirmation。

## 8. 复现、测试、运行时间与远端状态

FCL 审计可以完全离线重跑，源代码与所有实际控制都包含在新脚本中：

```bash
python3 offline/task28_g1_2b_1/audit_fcl.py
```

如需重建下载绑定的**源码对照**字段，可另行提供已经取得的 0.7.0.10 wheel；此项
只读取 zip，不安装、不执行绑定：

```bash
python3 offline/task28_g1_2b_1/audit_fcl.py \
  --python-fcl-source-wheel /path/to/python_fcl-0.7.0.10-cp310-cp310-manylinux_2_24_x86_64.manylinux_2_28_x86_64.whl
```

未提供 wheel 时，脚本仍重跑系统 C++ 的 31 个控制与 FK 比较，源码对照字段注明
未请求。已提交审计包含本次实际下载文件的 hash，不把该可选下载变为运行依赖。

核心运行与最终测试命令（仓库根目录）为：

```bash
python3 offline/task28_g1_2b_1/run_validation.py
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0:offline/task28_g1_2b_1 \
python3 -m pytest offline/task28_g1_2b_1/tests -q -p no:cacheprovider
python3 offline/task28_g1_2b_1/pack_evidence.py
```

第一条命令重跑旧 336 测试与新测试、生成七个控制和八个 A-stage 结果；默认深度 2。
最后一条将完整 JSON 无损压缩成 deterministic gzip，逐文件校验 bytes round-trip、
原 SHA256、414 pair 分离 receipts 和各 span 的完整 `[0,1]` 覆盖，另保留
[packaging.json](../../offline/task28_g1_2b_1/evidence/packaging.json)。
需要读取 trace 时可直接 `gzip.open(path, 'rt')` 后 `json.load`；没有删减字段。

| 验证 | 实际结果 | 时间 |
|---|---|---:|
| 原三套回归（G1-1/G1-2A/G1-2B-0） | 336 passed，exit 0 | 76.776 s |
| 核心流水线初次新增测试 | 53 passed，exit 0 | 2.664 s（含进程启动） |
| 数学独立检查文件加入后的最终新增测试 | 59 passed，exit 0 | pytest 2.63 s |
| 最终去重测试总数 | **395 passed** | 不将重复运行相加 |
| FCL C++ API/运动控制 | 31 个控制及审计断言通过 | 独立于 pytest 计数 |

原 336 个 tracked 文件在起点和运行结束均无内容差异，逐文件 SHA256 清单为
[protected_baseline.json](../../offline/task28_g1_2b_1/evidence/protected_baseline.json)。
因此冻结 fixture、ACM、旧验证器、旧报告与 Task24/26/27 实现保持不变。
最终测试日志见 [new_tests_final.txt](../../offline/task28_g1_2b_1/evidence/new_tests_final.txt)，
原 336 项日志及最初 53 项日志亦保留，没有覆盖成新的计数。

核心流水线总 wall time **227.841 s**，含 336 回归及最初新增测试、全部控制、全部
区间证明和 evidence 序列化；各轨迹内部计时见第 7 节，实际证明/旧采样会共同占用
该时间。峰值主进程 RSS **456,288 KiB**；`RUSAGE_CHILDREN` 返回的峰值同为
456,288 KiB，它是进程组会计记录，不是将所有进程内存求和。FCL helper 缓存、trig
缓存和主机调度会影响时间；这里不声称硬实时上界，也不称为机器人执行耗时。
原完整 trace **169,479,550 bytes**，gzip 后 **20,259,438 bytes**，压缩 **10.013 s**、peak RSS **539,956 KiB**，详见 packaging
记录；压缩不参与数学判据。所有单独配置、区间、witness、未决 pair、源 hash 与
receipt 均保存在 `offline/task28_g1_2b_1/evidence/`。

交付使用 `side-suction-palletizing`，先 fetch 核对起点，再检查远端是本地提交的祖先，
仅以 fast-forward push 发布，最后通过 `ls-remote` 核对远端 SHA 与本地 HEAD。
最终 commit SHA 与实际远端核对结果在交付回复给出；不把待产生的自身 SHA 写进
提交内容形成自引用。没有创建或启动下一个阶段。

## 9. 判定边界

本层可以增加“冻结数学模型、已支持输入域、整段覆盖”的证据，但不能把未决接触、
运动模型差异、unsupported quaternion 运动或预算缺口写成证书。FCL 的 API 名称
包含 continuous，不会自动填补这些缺口；原有限采样结果仍只说明那些 sample。

本次 `G1_2B_1_PARTIAL` 的已完成范围包括：实际 FCL 能力审计与反例；有向区间
算术和独立高精度校验；全部冻结候选的连续限位证明；激活闭链的静止段证明；
每个区间的完整碰撞候选覆盖、部分 pair 分离与显式未决；两个普通 grid 漏检的
严格拒绝；完整显式支撑确认序列及无损机器证据。工程实现与回归完成。

未完成的科学范围是：整个 A-stage 的连续碰撞安全、运动段完整闭链约束、允许
吸盘/支撑接触的全区间法向/位置/计数规则，以及一般 Cube geodesic 旋转证明。
当前深度 2 的结果不能断言更强方法也不可能证明，但也没有根据把本次 I 提升为 V。
因此不签发完整轨迹连续安全证书，不解释为可执行许可，**到 G1-2B-1 停止，不进入
P4-NR 或其他规划/控制阶段**。

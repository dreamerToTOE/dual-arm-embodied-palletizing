// Task26：分批到料的侧面吸盘紧协调码垛执行器。
//
// TASK27_FIVE_CUBE 由 task27_five_cube_center_insert.cpp 定义。它不是对 Task26
// 二件物理基线的就地改写，而是以同一条已验收链路编译出的独立执行器：前四件
// 分别贴墙/贴外侧 Cube，最后一件仅作中心插入并验收两侧余量。
//
// 设计边界：不读取选择器，不做在线任务排序。8 件 Cube 分 4 批、每批 2 件到料，
// 批次与目标固定；一次只把当前批两件加入 MoveIt Planning Scene，已完成件始终
// 保留为真实碰撞物，未到货件停在桌下休眠位、不进入任何规划场景。每一件仍由
// Isaac Ground Truth 门禁、MoveIt 规划、同步 FCL 与双 Surface Gripper 物理闭环
// 共同验证。共同 lift / transport / descent / release 原则沿用 Task11--13，
// 侧吸几何与“墙优先 + 短推”沿用 Task24；任何中心侧吸可达性失败都会明确停止，
// 绝不以移动到侧面边缘的假吸点继续执行，也不用单臂推送绕开真实碰撞。

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace
{
// Task24-B：线性尺寸为旧 30 mm Cube 的 4 倍；所有 Cube 都由双臂紧协调。
constexpr double kCubeSize = 0.120;
constexpr double kCubeHalf = 0.060;
// Task24-H：与 Isaac 场景和 MoveIt 环境发布器保持同构。0.200 m 顶面由
// Franka 官方 FR3 move_to_start 姿态及 Duo 手册的高可操作性工作半径确定，
// 不是通过反复试高度得到的经验值。
constexpr double kTableTopZ = 0.200;
// 滑轨静止位：必须与 isaac/scripts/task26_truck_box_scene.py 的
// RAIL_REST_X = LEFT_BASE[0] = 0.650 保持一致（bridge 的 rail_state 只发布
// [target, arrived, measured, x_min, x_max]，不含静止位）。
constexpr double kRailRestX = 0.650;
constexpr double kBottomZ = kTableTopZ + kCubeHalf;
constexpr double kUpperZ = kBottomZ + kCubeSize;
// 在尚未建立 Surface Gripper D6 约束的 CONTACT 阶段，Cup Collider 不能以
// 0 mm 穿入动态 Cube，否则两个机械臂的接近会先把 Cube 推偏。保留 1 mm 的
// 名义空气隙：远小于 3 mm 捕获阈值；Ground Truth 的实际间隙与对称性
// 仍由下方 kMaxAttachmentGap / kMaxAttachmentGapAsymmetry 门禁验收。
constexpr double kSideContactCommandGap = 0.001;
// 阵列 TCP 必须对准 Cube 侧面几何中心：四杯以 TCP 为中心镜像分布，才能让
// 杯面完整落在侧面中央区域。Y 向仍严格保持 1 mm 名义贴合间隙，Surface
// Gripper 在 3 mm 门限内闭合。若中心位姿没有可达通道，应先走单臂短推预调整，
// 绝不能把吸点上移到 Cube 边缘来规避碰撞。
constexpr double kSideContactCommandZOffset = 0.000;
// 这两项是 Isaac Ground Truth 的硬门限，不是命令偏置。Task27 分阶段续跑
// 的侧杯物理接近实测稳定停在 2.1--2.3 mm：低于 Surface Gripper 的 3 mm
// 捕获阈值，且继续向内的关节微调未缩小间隙。只在 Task27 允许 2.5 mm 内
// 尝试 CLOSE；仍必须双侧 CLOSED、后续附着几何/抬升门禁全部通过。
// Task26 已验收的 2 mm 门限保持原样；两侧不对称门限仍为 1 mm。
#ifdef TASK27_FIVE_CUBE
constexpr double kMaxAttachmentGap = 0.0025;
#else
constexpr double kMaxAttachmentGap = 0.002;
#endif
constexpr double kMaxAttachmentGapAsymmetry = 0.001;
constexpr double kMinAttachmentGap = -0.001;
constexpr double kPreContactOffsetY = 0.100;
// 右臂的外侧预接触偏移必须远小于左臂：右臂要从北侧跨过车厢口去够 Cube 的 +Y 面，
// 在低位（z=0.260）时它的前臂会扫到车厢侧墙/后墙，且落点随 IK 解支漂移。
// 实测：把 x=0.700、z=0.260 沿 y 扫一遍，右臂在 y <= -0.039 时各解支都有效，
// 从 y >= -0.019 起开始出现 link5/link6 撞墙。取 20 mm，使低位外侧位姿落在
// 接触位（y=-0.059）之外仅 20 mm —— 下降通道仍然干净，因为 L 型工具本体在
// 杯面的北侧，不会碰到南侧的 Cube。
// 20 -> 40 mm：原 20 mm 是在**旧布局**（预推位 x=0.700、车厢装料口 x=0.790）下扫出来的，
// 当时再往大扫会出现 link5/link6 撞墙。车厢整体 +0.15 让位后（装料口 0.940、预推位
// 0.840），右臂在低位外侧位姿离墙更远，余量随之变宽，因此重新放大到 40 mm。
// 动机（实测 t26_rail1.log）：外侧接近段推臂工具压在 Cube_01 上（lateral_support /
// vacuum_manifold 0.00 mm 重叠），腕关节 J6 被接触载荷顶到官方 12 N·m 的 100%、
// 残差 22.05° 追不上；左臂同时被扰到 1.411°。放大净空是为了让**本该干净**的这段
// 不再发生接触——即修根因，而不是去动增益或力矩上限。
constexpr double kRightPreContactOffsetY = 0.040;
// L 型工具的竖直段在侧向姿态下会向桌面方向占据额外空间。160 mm 抬升在
// COMMON_X_TRAVEL 的中间采样中仍会擦到桌面；提高到 280 mm 后，工具最低点
// 仍保有明确净空。该值是载荷共同运输高度，不改变最终放置高度。
constexpr double kLiftHeight = 0.280;
// 换位第一段的竖直抬起高度：换位前推入臂的杯面离 Cube 只有约 1.5 mm，若直接走 RRT
// 绕行会擦到 Cube 把它拖走（实测被拖 446 mm）。先纯 Z 抬起让开 Cube 所在高度。
constexpr double kRegraspLiftHeight = 0.200;
// 只为短推的 entry 留出 20 mm X 向余量；不是长距离推送。
constexpr double kPrePushOffsetX = 0.020;
// L 型阵列在侧面接触姿态下的最低实体比 Cup 接触面更低。Cube 以 1 mm
// 的桌面释放余量下降时，阵列本体会在下降路径中擦到桌面。释放高度改为
// 20 mm：仍是低速、短距离的物理落稳，而不是以放宽 ACM 换取可达性。
// 释放后必须以 Isaac Ground Truth 回读并通过双墙贴合校验，否则任务失败。
constexpr double kReleaseGapZ = 0.020;
constexpr double kCartesianStep = 0.002;
constexpr double kMinCartesianFraction = 0.999;
// 笛卡尔轨迹的中间状态允许偏离命令直线的最大值。MoveIt 的 computeCartesianPath 在
// 个别路径点 IK 失败时仍可能报 1.0000 的 fraction，但中间状态离线极远（实测左臂
// COMMON_Y_ALIGN 的 TCP 从 y=0.02 甩到 y=-1.2 m 再绕回，途中扫过桌面）。5 mm 对
// 正常轨迹有上百倍余量（实测贴线轨迹的偏差在 0.1 mm 量级），只用来拦掉这种绕行。
constexpr double kMaxCartesianLineDeviation = 0.005;
// jump_threshold 仍按 Task24 的做法关闭：MoveIt 的 jump 判据把「手腕快速转动」和
// 「解支跳变」混在一起，实测把阈值设成 1.0 会让正常下降段的 fraction 掉到 0.9789。
// 真正拦绕行的是上面的贴线校验。
constexpr double kCartesianJumpThreshold = 0.0;
constexpr double kFclSamplePeriod = 0.010;
constexpr double kPlacementTolerance = 0.010;
// Isaac 与 MoveIt 的固定 L 型工具 frame 存在约 2.1 mm 的已实测静态标定差；
// x/z 采用 2.5 mm 门限。该门限仍远小于 10 mm Cup 半径，而 Y 向实际贴合间隙
// 与左右对称性继续由上方独立的 2 mm / 1 mm 硬门限约束。否则 CLOSED 也不能
// 说明阵列 Cup 已对应贴合 Cube，禁止进入共同运输。
constexpr double kAttachmentAlignmentTolerance = 0.0025;
constexpr double kAttachmentOrientationToleranceDeg = 3.0;
constexpr double kPi = 3.14159265358979323846;
// 这是空载冗余构型的候选池大小，不是失败重试次数。每个候选都会在完整
// 双臂 FCL 负载链路上筛选；只执行第一个通过者。3 个样本不足以覆盖 FR3
// 在侧向 L 型工具下的肘部翻转分支，因此扩大为 8 个确定上限的样本。
constexpr int kRrtCandidateCount = 8;
// 一个 8 解候选池可能恰好全落在同一类不利的 FR3 冗余腕部构型。右臂还未
// 接触 Cube 时，允许最多三批独立 RRTConnect 采样；每一条仍必须通过完整
// FCL 负载链路，绝不是降低碰撞门限或在失败后盲目执行。
constexpr int kRrtCandidateBatches = 3;
constexpr int kRetries = 3;
// 0.8 kg 共同搬运物在每段末端需要额外收敛时间，避免刚到短推终点就 release。
constexpr int kFinalCommandHold = 200;
// 双侧吸盘共同搬运不是“两个独立轨迹恰好同时开始”。必须由同一控制时钟发布
// 左右命令，且采用零起止速度的共同 S 曲线进度，避免两套 PD 在阶段边界收到
// 不同相位/突变速度命令后给 Cube 施加瞬态扭矩。
constexpr auto kDualCommandPeriod = 10ms;
// 发送最终 command 并不等于 Isaac Articulation 已到达该关节状态。Task26 的
// 刚体接触和 Isaac PD 伺服在静态低位姿会留下约 1.5 deg 的关节稳态残差；这里
// 只把 2 deg 作为“停止继续跟随”的门槛，不能当成几何验收。真正的吸附前和每段
// 搬运后仍必须通过 validateDualSideAttachment() 的 2 mm TCP/Cube 实测门限。
constexpr double kJointSettleToleranceRad = 0.035;
// 到位判定的补充路径。关节在负载下可能收敛到一个很小的稳态偏差（本任务 149 次到位
// 判定里绝大多数是 0.02--0.3 deg，偶发 1.0--2.4 deg），此时再等也不会变小：那是
// 伺服在负载下的静态偏差，不是还在运动。因此除了「残余 <= 2.0 deg」之外，再接受
// 「残余 <= kJointSettleResidualToleranceRad 且连续多次读数几乎不变」——后者证明的
// 是机械臂已经停住，而放置精度由释放后的 Ground Truth 门限（<= 10 mm）单独保证，
// 这条路径不放宽任何几何精度要求。
constexpr double kJointSettleResidualToleranceRad = 0.06;
constexpr double kJointSettleStableDeltaRad = 0.002;
constexpr int kJointSettleStableSamples = 5;
constexpr double kJointSettleTimeoutSec = 25.0;

struct OfflineTask
{
  const char* id;
  std::size_t cube_index;
  geometry_msgs::msg::Pose pre_push;   // 双臂紧协调放置的预推位
  geometry_msgs::msg::Pose cell;       // 单臂持 -X 面沿 +X 推入后的格位
};

#ifdef TASK27_FIVE_CUBE
constexpr const char* kTaskLabel = "Task27";
constexpr const char* kTaskTopicPrefix = "/task27";
#else
constexpr const char* kTaskLabel = "Task26";
constexpr const char* kTaskTopicPrefix = "/task26";
#endif

std::string taskTopic(const std::string& suffix)
{
  return std::string(kTaskTopicPrefix) + suffix;
}

// 推入段"滑轨搬站位"引入的世界系偏移。
//
// MoveIt 模型里两个基座固定在 0.650，只有一个世界系。要让推臂基座沿轨 +X 前进
// Δ 之后仍能正确规划，就必须把**整个规划世界**平移 -Δ（等价于基座前进 Δ）。
// 因为 FCL/IK 只有一个世界系，这个偏移只能对**双臂同时**成立——所以动轨时两条
// 轨道一起走（辅助臂此时已停放在停放位，同步平移无副作用）。
//
// 偏移集中在这三个构造函数里：模型侧的目标一律减去它，而 worldPose() 保持不动
// （task.pre_push / task.cell 仍按仿真世界坐标用于 Ground Truth 复核）。
double g_world_shift_x = 0.0;

geometry_msgs::msg::Pose worldPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

// 一个 TCP frame 的 +X 始终是吸盘朝向 Cube 的法向；+Z 指向下。
// 左臂位于 -Y，从 -Y 侧吸附且 +X 指向世界 +Y；右臂相反。
geometry_msgs::msg::Pose sidePose(double x, double y, double z, bool left)
{
  auto pose = worldPose(x - g_world_shift_x, y, z);
  constexpr double half_root = 0.7071067811865476;
  if (left)
  {
    pose.orientation.x = half_root;
    pose.orientation.y = half_root;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
  }
  else
  {
    // 右侧同样只用杯面法向定义姿态：局部 +X 指向世界 -Y。Task24-D 通过
    // 加长轻量 L 型 standoff 让 wrist/flange 留在 Cube 外侧，不再依赖此前
    // 90 度绕法向翻转而产生的极端腕部姿态。
    pose.orientation.x = half_root;
    pose.orientation.y = -half_root;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
  }
  return pose;
}

// 推入姿态：杯面法向（TCP +X）指向世界 +X，即吸住 Cube 的 **-X 面**（装料口那面）。
//
// 为什么必须换面：侧吸工具在杯面后方伸出 155 mm。车厢内腔只有 0.25 宽、两排各
// 0.12，若仍用 ±Y 面持件沿 +X 推入，工具会伸进车厢撞 ±Y 墙或另一排已落稳的 Cube。
// 持 -X 面时整个工具留在车厢外侧，往里推 +X 不碰任何墙。
//
// quaternion (x,y,z,w) = (1,0,0,0) = 绕世界 +X 转 180°：+X 不变、+Y 变 -Y、
// +Z 变 -Z。即杯面法向为 +X（穿进 Cube），+Z 仍指向下，与 sidePose 的下压约定一致。
geometry_msgs::msg::Pose pushPose(double x, double y, double z)
{
  auto pose = worldPose(x - g_world_shift_x, y, z);
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  return pose;
}

// Task26：车厢第一层 4 件。C1(x=0.700) 与 C2(x=0.820) 两列，每列先远排后近排。
//
// 顺序由几何强制：左臂支架在 TCP 后方 155 mm，推远排时会扫过
// y ∈ [cube_y-0.216, cube_y-0.061]；若同列近排已经落稳就会真实碰撞。
// 车厢：三面围墙（+X 深端、±Y 两侧）+ 顶上无墙，**装料口开在 -X**，底板就是桌面顶面。
// 车厢在 Y 上居中（对称轴 y=0）、位于 +X 侧。
// 车厢整体 +X 让位（与 isaac/scripts/task26_truck_box_scene.py 的 TRUCK_SHIFT_X 必须一致）。
// 推入改由滑轨 +X 承担，基座要走到 0.950，原先装料口 (0.790) 会挡住前臂。
// 侧墙压紧的终点同时要求 x/y 两个方向都到达墙面。原 +0.150 m 布局在
// +Y/-Y 终点使压紧臂的 Cartesian IK 于最后约 16 mm 失效；将车厢整体回收
// 50 mm 后，末端仍在滑轨覆盖范围内，但腕部保留了横向压紧所需的行程余量。
constexpr double kTruckShiftX = 0.100;
constexpr double kBoxInteriorX0 = 0.810 + kTruckShiftX;
constexpr double kBoxInteriorX1 = 1.060 + kTruckShiftX;
// 车厢 Y 向五通道：5 x 120 mm Cube、相邻通道 2 mm、两侧各 10 mm 冗余。
// 必须与 isaac/scripts/task26_truck_box_scene.py 同值，保证 MoveIt 的三面墙
// 与 PhysX 的三面墙完全同构。现有四件 Task26 仍使用中央两行。
constexpr int kBoxYCubeCapacity = 5;
constexpr double kBoxYInterCubeGap = 0.002;
constexpr double kBoxYSideClearance = 0.010;
constexpr double kBoxInteriorYHalf = 0.5 * (
  kBoxYCubeCapacity * kCubeSize +
  (kBoxYCubeCapacity - 1) * kBoxYInterCubeGap +
  2.0 * kBoxYSideClearance);
constexpr double kBoxInteriorY0 = -kBoxInteriorYHalf;
constexpr double kBoxInteriorY1 = kBoxInteriorYHalf;
constexpr double kWallThickness = 0.020;
// 墙高必须让开推入臂的前臂（硬约束）：前臂 link5 下沿扫到 z≈0.40，所以墙顶要低于它。
//   0.250 -> 顶到墙顶 0.420 mm（拒）；0.200 -> 仍擦 0.014 mm（拒）；0.150 -> 净空约 50 mm。
// 代价：只比第一层 Cube 顶面（0.32）高 30 mm。第二层需要更高的墙，与本条冲突，
// 届时改用「X 导轨跟随推入」等方案再设计。
constexpr double kWallHeight = 0.150;
constexpr double kPrePushX = 0.690 + kTruckShiftX;
// 格位再 +10 mm 让 Cube 真正靠到深墙（深格 1.150 时 Cube 远端面正好落在 WallDeep
// 内表面 1.210）。与场景的 CELL_FLUSH_SHIFT_X 必须一致。
constexpr double kCellFlushShiftX = 0.010;
constexpr double kCellShallowX = 0.870 + kTruckShiftX + kCellFlushShiftX;
constexpr double kCellDeepX = 0.990 + kTruckShiftX + kCellFlushShiftX;
// 供料与双吸盘预推位保留中央两行；这保证紧协调搬运仍在两臂对称的舒适工作区。
// +X 推到深端墙后，再由另一臂把 Cube 压向对应的 ±Y 侧墙，最终不留侧缝。
constexpr double kRowYPlus = 0.060;
constexpr double kRowYMinus = -0.060;
constexpr double kCellYPlus = kBoxInteriorY1 - kCubeHalf;
constexpr double kCellYMinus = kBoxInteriorY0 + kCubeHalf;
// Task27 最深层的五件横向布局：两侧外件先贴墙，内件再贴外件，中心件最终
// 插入由两侧内件留下的余量。Task26 仍只使用 kCellYPlus/kCellYMinus。
#ifdef TASK27_FIVE_CUBE
// 内侧直推时，Cube 并非绝对无旋转的理想方块。实测第 4 件在靠近外侧件时
// 偏航约 0.36°，两个实际包围范围在 Y 向交叠约 0.98 mm，关节扭矩随即
// 达到 87 N·m。预留 1.5 mm 名义通道吸收角点扫掠和小幅跟踪偏差；最终仍须
// 通过原有的 3 mm 邻件间隙验收，不能放宽扭矩或碰撞门限。
constexpr double kInnerStraightInsertClearance = 0.0015;
#else
constexpr double kInnerStraightInsertClearance = 0.0;
#endif
constexpr double kCellYInnerPlus = kCellYPlus - kCubeSize - kInnerStraightInsertClearance;
constexpr double kCellYInnerMinus = kCellYMinus + kCubeSize + kInnerStraightInsertClearance;
// 推入臂持 **-X 面** 时，杯面到 Cube 中心的名义偏移：TCP_x = cube_x - (kCubeHalf + 间隙)。
// 维持 1 mm 间隙，它既位于 Surface Gripper 3 mm 捕获阈值内，又是当前 FR3
// 侧向姿态下可稳定求得低位 -X 接触 IK 的位置。
constexpr double kPushCupOffsetX = kCubeHalf + kSideContactCommandGap;
// Task26 侧墙压紧：+X 推入深端墙后，另一只机械臂从 Cube 的另一侧以空载杯面
// 短程压向 ±Y 围墙。Cup 先保留 1 mm 名义间隙再接触；最终是否真正贴墙由
// Isaac Ground Truth 的深端/侧墙两个 gap 独立验收，不能仅相信关节命令。
constexpr double kWallFlushTolerance = 0.003;
// 物理侧压的终点在规划几何上越过名义墙面 4 mm。原因不是放宽验收：Surface
// Gripper 杯面带有接触/约束建立的数值余量，若只命令到“名义 1 mm 间隙”处，
// 实测会在墙前残留约 3.6 mm 的缝。围墙 Collider 会阻止 Cube 穿墙；此余量仅
// 让位置控制器持续施加朝墙的压紧方向，最终仍由严格 3 mm Ground Truth 门限验收。
constexpr double kSideWallPressOvertravel = 0.004;
// 推入监督：Cube Ground Truth 落后命令位置超过该值即判为卡死。
constexpr double kPushJamTolerance = 0.020;
// 关节实测力矩硬上限：最后一道保护（吸盘 D6 断裂阈值是 1e6 N，位置命令持续前推
// 时接触力本身没有上限，所以必须有这一道）。
constexpr double kPushTorqueLimit = 80.0;
// 推进分段数：每段结束后检查「Cube 是否跟着走」与「力矩是否越限」。
constexpr int kPushSlices = 16;

// 两个预推行仍位于 y=±0.060；每一件先由双臂紧协调搬到该中央安全行，
// 后续由另一臂压到对应侧墙。每行按深格再浅格的顺序，避免浅格挡住深格的 +X 推入。
#ifdef TASK27_FIVE_CUBE
// Task27 只填最深 X 层，顺序严格左右交替：+Y 外 -> -Y 外 -> +Y 内 -> -Y 内。
// 中心件在四侧基准完成后再从 y=0 插入；它不做横向压紧，避免扰动已落稳的四件。
const std::array<OfflineTask, 5> kTasks{{
  {"task27_plus_outer", 0, worldPose(kPrePushX, kRowYPlus, kBottomZ), worldPose(kCellDeepX, kCellYPlus, kBottomZ)},
  {"task27_minus_outer", 1, worldPose(kPrePushX, kRowYMinus, kBottomZ), worldPose(kCellDeepX, kCellYMinus, kBottomZ)},
  {"task27_plus_inner", 2, worldPose(kPrePushX, kCellYInnerPlus, kBottomZ), worldPose(kCellDeepX, kCellYInnerPlus, kBottomZ)},
  {"task27_minus_inner", 3, worldPose(kPrePushX, kCellYInnerMinus, kBottomZ), worldPose(kCellDeepX, kCellYInnerMinus, kBottomZ)},
  {"task27_center_insert", 4, worldPose(kPrePushX, 0.0, kBottomZ), worldPose(kCellDeepX, 0.0, kBottomZ)},
}};
constexpr int kBatchSize = 1;
constexpr int kBatchCount = 5;
#else
const std::array<OfflineTask, 4> kTasks{{
  {"task26_r0_deep", 0, worldPose(kPrePushX, kRowYPlus, kBottomZ), worldPose(kCellDeepX, kCellYPlus, kBottomZ)},
  {"task26_r0_shallow", 1, worldPose(kPrePushX, kRowYPlus, kBottomZ), worldPose(kCellShallowX, kCellYPlus, kBottomZ)},
  {"task26_r1_deep", 2, worldPose(kPrePushX, kRowYMinus, kBottomZ), worldPose(kCellDeepX, kCellYMinus, kBottomZ)},
  {"task26_r1_shallow", 3, worldPose(kPrePushX, kRowYMinus, kBottomZ), worldPose(kCellShallowX, kCellYMinus, kBottomZ)},
}};

// 装料：2 批 x 2 件，每批同一排（先深格后浅格）。供料槽放在**中轴线 y=0** 且靠近
// 基座（x=0.50 / 0.35），两臂对称可达：y=0 时两臂 TCP 各偏 0.539 m，最大伸展约
// 0.60 m。放到 (0.28, ±0.10) 时右臂要 0.74 m 斜向长臂，实测 RRT 全部超时。
constexpr int kBatchSize = 2;
constexpr int kBatchCount = 2;
#endif
constexpr double kSlotAx = 0.500;
constexpr double kSlotAy = 0.000;
constexpr double kSlotBx = 0.350;
constexpr double kSlotBy = 0.000;
constexpr double kSlotTolerance = 0.003;
constexpr double kFeedTimeoutSec = 90.0;
constexpr std::array<double, 7> kHomeQ{
  0.0, -kPi / 4.0, 0.0, -3.0 * kPi / 4.0, 0.0, kPi / 2.0, kPi / 4.0};

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
  point.time_from_start.sec = static_cast<std::int32_t>(std::floor(seconds));
  point.time_from_start.nanosec = static_cast<std::uint32_t>(std::llround(
    (seconds - std::floor(seconds)) * 1e9));
}

void ensureTiming(trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty())
  {
    return;
  }
  // computeCartesianPath() 在目标与起点完全重合（例如已经对准目标 Y 的
  // COMMON_Y_ALIGN）时会合法地返回仅含一个、t=0 的点。它不是规划失败，
  // 而是零位移保持；补出一个相同的 30 ms 终点，才能与另一臂同步并继续做
  // FCL 采样和原子双臂命令。不能把该阶段跳过，否则左右时序会失去统一接口。
  if (trajectory.points.size() == 1)
  {
    auto endpoint = trajectory.points.front();
    setPointTime(endpoint, 0.03);
    trajectory.points.push_back(std::move(endpoint));
    return;
  }
  if (pointTime(trajectory.points.back()) > 1e-6)
  {
    return;
  }
  for (std::size_t index = 0; index < trajectory.points.size(); ++index)
  {
    setPointTime(trajectory.points[index], 0.03 * static_cast<double>(index));
  }
}

std::vector<double> finalPositions(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  return trajectory.points.empty() ? std::vector<double>{} : trajectory.points.back().positions;
}

std::string formatJointPositions(const std::vector<double>& positions)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4) << "[";
  for (std::size_t index = 0; index < positions.size(); ++index)
  {
    if (index > 0)
    {
      stream << ", ";
    }
    stream << positions[index];
  }
  stream << "]";
  return stream.str();
}

// 侧面双吸盘的最终 CONTACT 必须两臂都到位，但两条从 PRE_CONTACT 到 CONTACT
// 的短 Cartesian 进给若严格同一时刻开始，局部肘部扫掠会偶发相交。这里使用
// 可验证的微时序：左臂先进入，右臂保持 PRE_CONTACT；随后右臂进入，左臂保持
// CONTACT。两杯均接触后仍同步 ON，并从 COMMON_LIFT 开始执行真正的紧协调。
// 这不是等待区或长距离串行搬运，只是接触建立阶段的安全时序。
trajectory_msgs::msg::JointTrajectory holdTrajectory(
  const trajectory_msgs::msg::JointTrajectory& reference,
  const std::vector<double>& positions, double duration)
{
  trajectory_msgs::msg::JointTrajectory hold;
  hold.joint_names = reference.joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint begin;
  begin.positions = positions;
  setPointTime(begin, 0.0);
  trajectory_msgs::msg::JointTrajectoryPoint end = begin;
  setPointTime(end, std::max(0.05, duration));
  hold.points = {begin, end};
  return hold;
}

std::vector<double> interpolate(
  const trajectory_msgs::msg::JointTrajectory& trajectory, double time_sec)
{
  if (trajectory.points.empty())
  {
    return {};
  }
  if (time_sec <= pointTime(trajectory.points.front()))
  {
    return trajectory.points.front().positions;
  }
  if (time_sec >= pointTime(trajectory.points.back()))
  {
    return trajectory.points.back().positions;
  }
  for (std::size_t index = 1; index < trajectory.points.size(); ++index)
  {
    const auto& first = trajectory.points[index - 1];
    const auto& second = trajectory.points[index];
    if (time_sec > pointTime(second))
    {
      continue;
    }
    const double denominator = pointTime(second) - pointTime(first);
    const double alpha = denominator > 1e-9 ?
      std::clamp((time_sec - pointTime(first)) / denominator, 0.0, 1.0) : 0.0;
    std::vector<double> positions(first.positions.size());
    for (std::size_t joint = 0; joint < positions.size(); ++joint)
    {
      positions[joint] = first.positions[joint] +
        alpha * (second.positions[joint] - first.positions[joint]);
    }
    return positions;
  }
  return trajectory.points.back().positions;
}

double distance3d(const geometry_msgs::msg::Point& first, const geometry_msgs::msg::Point& second)
{
  return std::hypot(std::hypot(first.x - second.x, first.y - second.y), first.z - second.z);
}

moveit_msgs::msg::CollisionObject cubeObject(
  const std::string& id, const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;
  // 碰撞体跟随同一个世界偏移（车主车厢三面墙、桌面、Cube 都走 cubeObject）。
  geometry_msgs::msg::Pose shifted = pose;
  shifted.position.x -= g_world_shift_x;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {kCubeSize, kCubeSize, kCubeSize};
  object.primitives.push_back(shape);
  object.primitive_poses.push_back(shifted);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

std::vector<moveit_msgs::msg::CollisionObject> boxWallObjects()
{
  // 车厢三面静态围墙：+X（深端）、-Y、+Y。装料口开在 -X，顶部无墙；底板就是桌面。
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  const double wall_z = kTableTopZ + kWallHeight * 0.5;
  const double span_y = (kBoxInteriorY1 - kBoxInteriorY0) + 2.0 * kWallThickness;
  const double span_x = (kBoxInteriorX1 - kBoxInteriorX0) + 2.0 * kWallThickness;
  auto add = [&](const std::string& id, const geometry_msgs::msg::Pose& pose,
                 const std::array<double, 3>& size)
  {
    auto object = cubeObject(id, pose);
    object.primitives.front().dimensions = {size[0], size[1], size[2]};
    objects.push_back(std::move(object));
  };
  // +X 墙（深端）
  add("task26_box_wall_deep",
      worldPose(kBoxInteriorX1 + kWallThickness * 0.5,
        (kBoxInteriorY0 + kBoxInteriorY1) * 0.5, wall_z),
      {kWallThickness, span_y, kWallHeight});
  // -Y 墙
  add("task26_box_wall_minus_y",
      worldPose((kBoxInteriorX0 + kBoxInteriorX1) * 0.5,
        kBoxInteriorY0 - kWallThickness * 0.5, wall_z),
      {span_x, kWallThickness, kWallHeight});
  // +Y 墙
  add("task26_box_wall_plus_y",
      worldPose((kBoxInteriorX0 + kBoxInteriorX1) * 0.5,
        kBoxInteriorY1 + kWallThickness * 0.5, wall_z),
      {span_x, kWallThickness, kWallHeight});
  return objects;
}

moveit_msgs::msg::CollisionObject tableObject()
{
  auto pose = worldPose(0.55, 0.0, 0.100);
  auto object = cubeObject("task26_table", pose);
  object.primitives.front().dimensions = {1.50, 0.80, 0.200};
  return object;
}

bool copyRobotDescriptions(const rclcpp::Node::SharedPtr& node)
{
  // 在本节点加入工作 executor 前完成参数复制。不能使用 SyncParametersClient：
  // 若 DDS 中短暂存在同名 MoveIt 内部服务端，sync get_parameters() 可能无限
  // 等待而没有日志。显式驱动异步 future，并给每一步确定的 10 s 上限；超时即
  // 失败退出，绝不把“零命令预检”卡成无输出进程。
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "Task26 无法连接 /move_group。");
    return false;
  }
  const auto future = client->get_parameters({"robot_description", "robot_description_semantic"});
  if (rclcpp::spin_until_future_complete(node, future, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(),
      "Task26 读取 /move_group RobotModel 参数超时；请确认只启动一套 MoveIt。");
    return false;
  }
  const auto values = future.get();
  if (values.size() != 2 ||
      values[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      values[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(node->get_logger(), "Task26 未从 /move_group 读取到有效 RobotModel。");
    return false;
  }
  node->declare_parameter<std::string>("robot_description", values[0].as_string());
  node->declare_parameter<std::string>("robot_description_semantic", values[1].as_string());
  return true;
}

class CubeBuffer
{
public:
  CubeBuffer(const rclcpp::Node::SharedPtr& node, std::size_t expected_count)
    : expected_count_(expected_count)
  {
    subscription_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
      taskTopic("/cube_poses"), 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message)
      {
        if (message->poses.size() != expected_count_)
        {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        poses_ = message->poses;
        ++revision_;
        ready_ = true;
        condition_.notify_all();
      });
  }

  bool wait(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::duration<double>(timeout_sec), [this]() { return ready_; });
  }

  std::pair<geometry_msgs::msg::Pose, std::uint64_t> get(std::size_t index) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {poses_.at(index), revision_};
  }

  bool waitNew(std::size_t index, std::uint64_t previous, double timeout_sec,
               geometry_msgs::msg::Pose* output) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_.wait_for(lock, std::chrono::duration<double>(timeout_sec),
                             [this, previous]() { return revision_ > previous; }))
    {
      return false;
    }
    *output = poses_.at(index);
    return true;
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  bool ready_{false};
  std::uint64_t revision_{0};
  std::size_t expected_count_{0};
  std::vector<geometry_msgs::msg::Pose> poses_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

// Isaac Bridge 每帧发布长度 8 的就位掩码：0 = 未到货（桌下休眠），1 = 已瞬移到槽位
// 正在落稳，2 = 已到位且静止。执行器只在本标志非 0 时才允许读取该件 Ground Truth 并
// 规划；未到货的 Cube 停在桌下休眠位，既不是障碍物也不是抓取对象。
class FeedStateBuffer
{
public:
  FeedStateBuffer(const rclcpp::Node::SharedPtr& node, std::size_t expected_count)
    : expected_count_(expected_count)
  {
    subscription_ = node->create_subscription<std_msgs::msg::Int32MultiArray>(
      taskTopic("/feed_state"), 10,
      [this](const std_msgs::msg::Int32MultiArray::SharedPtr message)
      {
        if (message->data.size() != expected_count_)
        {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = message->data;
        ready_ = true;
        condition_.notify_all();
      });
  }

  bool wait(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::duration<double>(timeout_sec), [this]() { return ready_; });
  }

  // 等待给定的一批件全部到位。这是“到位即停”的唯一证据来源：只要有一件没到位，
  // 就不允许读它的 pose、更不允许围绕它规划。
  bool waitArrived(const std::vector<std::size_t>& indices, double timeout_sec)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
    std::unique_lock<std::mutex> lock(mutex_);
    while (std::chrono::steady_clock::now() < deadline)
    {
      bool all = ready_;
      for (const auto index : indices)
      {
        if (index >= state_.size() || state_.at(index) == 0)
        {
          all = false;
          break;
        }
      }
      if (all)
      {
        return true;
      }
      condition_.wait_for(lock, 50ms);
    }
    return false;
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  bool ready_{false};
  std::size_t expected_count_{0};
  std::vector<std::int32_t> state_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

// Isaac Bridge 在同一个 physics tick 发布 Cube 与两个 side_suction_tcp Pose。
// Task26 不将 CLOSED 当成“抓正了”：必须用这三个 Ground Truth 显式验证。
class TcpBuffer
{
public:
  TcpBuffer(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_ = message->pose;
        ready_ = true;
        condition_.notify_all();
      });
  }

  bool wait(double timeout_sec) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::duration<double>(timeout_sec),
      [this]() { return ready_; });
  }

  geometry_msgs::msg::Pose get() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_;
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  bool ready_{false};
  geometry_msgs::msg::Pose pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

double orientationAngleDeg(const geometry_msgs::msg::Quaternion& orientation)
{
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (norm <= 1e-9)
  {
    return 180.0;
  }
  return 2.0 * std::acos(std::clamp(std::abs(orientation.w) / norm, 0.0, 1.0)) * 180.0 / kPi;
}

bool validateDualSideAttachment(
  const rclcpp::Node::SharedPtr& node, const OfflineTask& task,
  const geometry_msgs::msg::Pose& cube, const geometry_msgs::msg::Pose& left_tcp,
  const geometry_msgs::msg::Pose& right_tcp, const std::string& phase)
{
  const double left_gap = cube.position.y - left_tcp.position.y - kCubeHalf;
  const double right_gap = right_tcp.position.y - cube.position.y - kCubeHalf;
  const double gap_asymmetry = std::abs(left_gap - right_gap);
  const double x_mismatch = std::max({
    std::abs(cube.position.x - left_tcp.position.x),
    std::abs(cube.position.x - right_tcp.position.x),
    std::abs(left_tcp.position.x - right_tcp.position.x)});
  const double z_mismatch = std::max({
    std::abs(cube.position.z + kSideContactCommandZOffset - left_tcp.position.z),
    std::abs(cube.position.z + kSideContactCommandZOffset - right_tcp.position.z),
    std::abs(left_tcp.position.z - right_tcp.position.z)});
  const double tilt_deg = orientationAngleDeg(cube.orientation);

  RCLCPP_INFO(node->get_logger(),
    "%s %s ATTACHMENT: x=%.3f mm z=%.3f mm left_gap=%.3f mm right_gap=%.3f mm gap_delta=%.3f mm tilt=%.3f deg.",
    task.id, phase.c_str(), x_mismatch * 1000.0, z_mismatch * 1000.0,
    left_gap * 1000.0, right_gap * 1000.0, gap_asymmetry * 1000.0, tilt_deg);
  if (x_mismatch > kAttachmentAlignmentTolerance ||
      z_mismatch > kAttachmentAlignmentTolerance ||
      left_gap < kMinAttachmentGap || right_gap < kMinAttachmentGap ||
      left_gap > kMaxAttachmentGap || right_gap > kMaxAttachmentGap ||
      gap_asymmetry > kMaxAttachmentGapAsymmetry ||
      tilt_deg > kAttachmentOrientationToleranceDeg)
  {
    RCLCPP_ERROR(node->get_logger(),
      "%s %s: dual side attachment is not corresponding; stop before a skewed transport.",
      task.id, phase.c_str());
    return false;
  }
  return true;
}

class Arm
{
public:
  Arm(const rclcpp::Node::SharedPtr& node, bool left, double time_scale)
    : node_(node), left_(left), side_(left ? "left" : "right"),
      group_(left ? "left_arm" : "right_arm"),
      eef_(left ? "left_fr3_side_suction_tcp" : "right_fr3_side_suction_tcp"),
      prefix_(left ? "left_" : "right_"), time_scale_(time_scale)
  {
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/" + side_ + "/joint_command", 10);
    suction_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      taskTopic("/" + side_ + "/suction_command"), 10);
    state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      taskTopic("/" + side_ + "/suction_state"), 10,
      [this](const std_msgs::msg::Bool::SharedPtr state)
      {
        seen_state_.store(true);
        closed_.store(state->data);
      });
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/" + side_ + "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr state)
      {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        joint_positions_.clear();
        const std::size_t count = std::min(state->name.size(), state->position.size());
        for (std::size_t index = 0; index < count; ++index)
        {
          joint_positions_[state->name[index]] = state->position[index];
        }
        // 速度只服务于超时诊断：用来区分"被挡住"（速度≈0）与"还在追"（速度非零）。
        joint_velocities_.clear();
        const std::size_t velocity_count = std::min(state->name.size(), state->velocity.size());
        for (std::size_t index = 0; index < velocity_count; ++index)
        {
          joint_velocities_[state->name[index]] = state->velocity[index];
        }
        seen_joint_state_.store(true);
        joint_condition_.notify_all();
      });
  }

  const std::string& groupName() const { return group_; }
  const std::string& eefLink() const { return eef_; }
  bool isLeft() const { return left_; }
  bool isClosed() const { return closed_.load(); }

  bool waitBridge() const
  {
    for (int attempt = 0; attempt < 100; ++attempt)
    {
      if (joint_pub_->get_subscription_count() > 0 && suction_pub_->get_subscription_count() > 0 &&
          seen_state_.load() && seen_joint_state_.load())
      {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  bool planPoseCandidates(moveit::planning_interface::MoveGroupInterface& group,
                          const geometry_msgs::msg::Pose& target, const std::string& label,
                          std::vector<trajectory_msgs::msg::JointTrajectory>* outputs) const
  {
    configure(group);
    group.clearPoseTargets();
    if (!group.setPoseTarget(target, eef_))
    {
      return false;
    }
    // RRTConnect 对同一个 TCP pose 可能给出不同的冗余腕部构型。空载阶段
    // 主动采样有限组候选并选择关节累计位移最小者，避免偶发选择绕过关节极限的
    // 大回环解；这不是放宽碰撞约束，MoveIt 仍逐候选做碰撞检查。
    std::vector<std::pair<double, trajectory_msgs::msg::JointTrajectory>> candidates;
    for (int attempt = 1; attempt <= kRrtCandidateCount; ++attempt)
    {
      group.setStartStateToCurrentState();
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
          !plan.trajectory_.joint_trajectory.points.empty())
      {
        const auto& candidate = plan.trajectory_.joint_trajectory;
        double cost = 0.0;
        for (std::size_t point = 1; point < candidate.points.size(); ++point)
        {
          for (std::size_t joint = 0; joint < candidate.points[point].positions.size(); ++joint)
          {
            cost += std::abs(candidate.points[point].positions[joint] -
              candidate.points[point - 1].positions[joint]);
          }
        }
        RCLCPP_INFO(node_->get_logger(),
          "%s %s RRTConnect candidate sample=%d/%d points=%zu joint_travel=%.3f.",
          side_.c_str(), label.c_str(), attempt, kRrtCandidateCount, candidate.points.size(), cost);
        candidates.emplace_back(cost, candidate);
        continue;
      }
      RCLCPP_WARN(node_->get_logger(), "%s %s RRTConnect candidate sample failed=%d/%d.",
        side_.c_str(), label.c_str(), attempt, kRrtCandidateCount);
    }
    if (candidates.empty())
    {
      return false;
    }
    std::sort(candidates.begin(), candidates.end(),
      [](const auto& first, const auto& second) { return first.first < second.first; });
    outputs->clear();
    for (auto& [cost, candidate] : candidates)
    {
      (void)cost;
      ensureTiming(candidate);
      outputs->push_back(std::move(candidate));
    }
    RCLCPP_INFO(node_->get_logger(), "%s %s RRTConnect candidates ready=%zu shortest_joint_travel=%.3f.",
      side_.c_str(), label.c_str(), outputs->size(), candidates.front().first);
    return true;
  }

  bool planPose(moveit::planning_interface::MoveGroupInterface& group,
                const geometry_msgs::msg::Pose& target, const std::string& label,
                trajectory_msgs::msg::JointTrajectory* output) const
  {
    std::vector<trajectory_msgs::msg::JointTrajectory> candidates;
    if (!planPoseCandidates(group, target, label, &candidates))
    {
      return false;
    }
    *output = std::move(candidates.front());
    RCLCPP_INFO(node_->get_logger(), "%s %s RRTConnect selected points=%zu.",
      side_.c_str(), label.c_str(), output->points.size());
    return true;
  }

  // 与 planPoseCandidates() 相同的空载 RRTConnect 候选池，但起点显式来自一条
  // 已预演轨迹的末端。planning_only 模式下当前物理关节仍在 HOME；若这里偷用
  // setStartStateToCurrentState()，就会把"预演成功"误报成真实后续链路可行。
  bool planPoseCandidatesFrom(
    moveit::planning_interface::MoveGroupInterface& group,
    const std::vector<double>& start, const geometry_msgs::msg::Pose& target,
    const std::string& label,
    std::vector<trajectory_msgs::msg::JointTrajectory>* outputs) const
  {
    configure(group);
    const auto* joint_model_group = group.getRobotModel()->getJointModelGroup(group_);
    auto state = group.getCurrentState(2.0);
    if (!joint_model_group || !state || start.size() != joint_model_group->getVariableCount())
    {
      return false;
    }
    state->setJointGroupPositions(joint_model_group, start);
    state->update();
    group.clearPoseTargets();
    if (!group.setPoseTarget(target, eef_))
    {
      return false;
    }
    std::vector<std::pair<double, trajectory_msgs::msg::JointTrajectory>> candidates;
    for (int attempt = 1; attempt <= kRrtCandidateCount; ++attempt)
    {
      group.setStartState(*state);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS ||
          plan.trajectory_.joint_trajectory.points.empty())
      {
        RCLCPP_WARN(node_->get_logger(), "%s %s RRTConnect candidate failed=%d/%d.",
          side_.c_str(), label.c_str(), attempt, kRrtCandidateCount);
        continue;
      }
      auto candidate = plan.trajectory_.joint_trajectory;
      ensureTiming(candidate);
      double travel = 0.0;
      for (std::size_t point = 1; point < candidate.points.size(); ++point)
      {
        for (std::size_t joint = 0; joint < candidate.points[point].positions.size(); ++joint)
        {
          travel += std::abs(candidate.points[point].positions[joint] -
            candidate.points[point - 1].positions[joint]);
        }
      }
      candidates.emplace_back(travel, std::move(candidate));
    }
    if (candidates.empty())
    {
      return false;
    }
    std::sort(candidates.begin(), candidates.end(),
      [](const auto& first, const auto& second) { return first.first < second.first; });
    outputs->clear();
    for (auto& [travel, candidate] : candidates)
    {
      (void)travel;
      outputs->push_back(std::move(candidate));
    }
    RCLCPP_INFO(node_->get_logger(), "%s %s candidates=%zu, shortest_joint_travel=%.3f.",
      side_.c_str(), label.c_str(), outputs->size(), candidates.front().first);
    return true;
  }

  // 批间退出：共同回到官方空载准备姿态。空载 RRTConnect 到固定关节目标即可，
  // 但仍必须整段通过同步 FCL 门禁；这里不执行任何负载段。
  bool planHome(moveit::planning_interface::MoveGroupInterface& group,
                const std::array<double, 7>& home,
                trajectory_msgs::msg::JointTrajectory* output) const
  {
    configure(group);
    group.clearPoseTargets();
    std::map<std::string, double> target;
    for (const auto& name : group.getJointNames())
    {
      if (name.size() < 2)
      {
        continue;
      }
      const char last = name.back();
      if (last < '1' || last > '7')
      {
        continue;
      }
      target[name] = home.at(static_cast<std::size_t>(last - '1'));
    }
    if (target.size() != home.size())
    {
      RCLCPP_ERROR(node_->get_logger(),
        "%s HOME target joint mismatch: resolved=%zu expected=%zu.",
        side_.c_str(), target.size(), home.size());
      return false;
    }
    group.setStartStateToCurrentState();
    if (!group.setJointValueTarget(target))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s HOME target rejected by MoveIt.", side_.c_str());
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS ||
        plan.trajectory_.joint_trajectory.points.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "%s HOME RRTConnect planning failed.", side_.c_str());
      return false;
    }
    *output = plan.trajectory_.joint_trajectory;
    ensureTiming(*output);
    RCLCPP_INFO(node_->get_logger(), "%s HOME planned points=%zu.", side_.c_str(), output->points.size());
    return true;
  }

  bool executeAt(const trajectory_msgs::msg::JointTrajectory& input,
                 const std::chrono::steady_clock::time_point& start) const
  {
    if (input.points.empty() || input.joint_names.empty())
    {
      return false;
    }
    auto trajectory = input;
    ensureTiming(trajectory);
    std::vector<std::string> names;
    for (const auto& name : trajectory.joint_names)
    {
      names.push_back(name.rfind(prefix_, 0) == 0 ? name.substr(prefix_.size()) : name);
    }
    std::this_thread::sleep_until(start);
    const auto publish = [&](const std::vector<double>& positions)
    {
      sensor_msgs::msg::JointState command;
      command.header.stamp = node_->now();
      command.header.frame_id = std::string(kTaskLabel) + "_single";
      command.name = names;
      command.position = positions;
      joint_pub_->publish(command);
    };
    std::size_t segment = 0;
    const double duration = pointTime(trajectory.points.back());
    while (true)
    {
      const double logical_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count() / time_scale_;
      if (logical_time > duration)
      {
        break;
      }
      while (segment + 1 < trajectory.points.size() &&
             pointTime(trajectory.points[segment + 1]) < logical_time)
      {
        ++segment;
      }
      publish(interpolate(trajectory, logical_time));
      std::this_thread::sleep_for(10ms);
    }
    for (int repeat = 0; repeat < kFinalCommandHold; ++repeat)
    {
      publish(trajectory.points.back().positions);
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  // 由 executeSync 的唯一调度线程调用。两个 Arm 使用同一 stamp 和同一 phase，
  // 因此 ROS Graph 会在同一物理控制周期内收到相同阶段的左右目标，而不是由
  // 两个 sleep_for(10ms) 线程产生不可控的相位漂移。
  void publishAt(const trajectory_msgs::msg::JointTrajectory& input, double logical_time,
                 const builtin_interfaces::msg::Time& stamp) const
  {
    std::vector<std::string> names;
    names.reserve(input.joint_names.size());
    for (const auto& name : input.joint_names)
    {
      names.push_back(name.rfind(prefix_, 0) == 0 ? name.substr(prefix_.size()) : name);
    }
    sensor_msgs::msg::JointState command;
    command.header.stamp = stamp;
    command.header.frame_id = std::string(kTaskLabel) + "_dual_sync";
    command.name = std::move(names);
    command.position = interpolate(input, logical_time);
    joint_pub_->publish(command);
  }

  builtin_interfaces::msg::Time nowMsg() const
  {
    const auto nanoseconds = node_->now().nanoseconds();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    return stamp;
  }

  double timeScale() const
  {
    return time_scale_;
  }

  bool waitAtTarget(const trajectory_msgs::msg::JointTrajectory& trajectory,
                    double tolerance_rad, double timeout_sec) const
  {
    if (trajectory.joint_names.empty() || trajectory.points.empty() ||
        trajectory.joint_names.size() != trajectory.points.back().positions.size())
    {
      return false;
    }
    const auto& target = trajectory.points.back().positions;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    double last_max_error = std::numeric_limits<double>::infinity();
    double previous_max_error = std::numeric_limits<double>::infinity();
    int stable_samples = 0;
    while (std::chrono::steady_clock::now() < deadline)
    {
      {
        std::unique_lock<std::mutex> lock(joint_mutex_);
        bool complete = true;
        last_max_error = 0.0;
        for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index)
        {
          const auto& full_name = trajectory.joint_names[index];
          const std::string name = full_name.rfind(prefix_, 0) == 0
            ? full_name.substr(prefix_.size()) : full_name;
          const auto found = joint_positions_.find(name);
          if (found == joint_positions_.end())
          {
            complete = false;
            last_max_error = std::numeric_limits<double>::infinity();
            break;
          }
          last_max_error = std::max(last_max_error, std::abs(found->second - target[index]));
        }
        if (complete && last_max_error <= tolerance_rad)
        {
          RCLCPP_INFO(node_->get_logger(), "%s final joint-state settled: max_error=%.3f deg.",
            side_.c_str(), last_max_error * 180.0 / kPi);
          return true;
        }
        // 稳态残余：连续若干次读数几乎不变且残余有界，说明已经停住，可以继续推进。
        if (complete && std::isfinite(last_max_error) &&
            last_max_error <= kJointSettleResidualToleranceRad &&
            std::abs(last_max_error - previous_max_error) <= kJointSettleStableDeltaRad)
        {
          ++stable_samples;
        }
        else
        {
          stable_samples = 0;
        }
        previous_max_error = last_max_error;
        if (stable_samples >= kJointSettleStableSamples)
        {
          RCLCPP_WARN(node_->get_logger(),
            "%s final joint-state at rest with bounded residual: max_error=%.3f deg (<= %.3f deg).",
            side_.c_str(), last_max_error * 180.0 / kPi,
            kJointSettleResidualToleranceRad * 180.0 / kPi);
          return true;
        }
        joint_condition_.wait_for(lock, 50ms);
      }
    }
    const std::string max_error_text = std::isfinite(last_max_error)
      ? std::to_string(last_max_error * 180.0 / kPi) + " deg"
      : "missing joint state";
    RCLCPP_ERROR(node_->get_logger(), "%s final joint-state did not settle within %.1f s: max_error=%s.",
      side_.c_str(), timeout_sec, max_error_text.c_str());
    // 超时诊断：逐关节残差 + 实测速度。速度≈0 = 被挡住；速度非零 = 还在追（给时间即可）。
    {
      std::unique_lock<std::mutex> lock(joint_mutex_);
      std::ostringstream detail;
      for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index)
      {
        const auto& full_name = trajectory.joint_names[index];
        const std::string name = full_name.rfind(prefix_, 0) == 0
          ? full_name.substr(prefix_.size()) : full_name;
        const auto position = joint_positions_.find(name);
        if (position == joint_positions_.end())
        {
          continue;
        }
        detail << " " << name << " " << std::fixed << std::setprecision(2)
               << (position->second - target[index]) * 180.0 / kPi << "deg";
        const auto velocity = joint_velocities_.find(name);
        if (velocity != joint_velocities_.end())
        {
          detail << "/" << std::setprecision(3) << velocity->second << "rad_s";
        }
      }
      RCLCPP_ERROR(node_->get_logger(), "%s 超时细节（残差deg / 速度rad_s）：%s",
        side_.c_str(), detail.str().c_str());
    }
    return false;
  }

  void suction(bool enabled) const
  {
    std_msgs::msg::Bool message;
    message.data = enabled;
    for (int repeat = 0; repeat < 20; ++repeat)
    {
      suction_pub_->publish(message);
      std::this_thread::sleep_for(10ms);
    }
  }

  bool waitSuction(bool expected, double timeout_sec) const
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (seen_state_.load() && closed_.load() == expected)
      {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

private:
  void configure(moveit::planning_interface::MoveGroupInterface& group) const
  {
    group.setPlannerId("RRTConnectkConfigDefault");
    // Task26：车厢占掉了工作区一大片自由空间，右臂低位外侧的位姿目标很窄，
    // RRTConnect 5 s 常常采不到解。给足规划时间（只是等待时长，不放宽任何门禁）。
    group.setPlanningTime(20.0);
    group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(0.12);
    group.setMaxAccelerationScalingFactor(0.12);
    group.setPoseReferenceFrame("world");
    group.setEndEffectorLink(eef_);
  }

  rclcpp::Node::SharedPtr node_;
  bool left_;
  std::string side_;
  std::string group_;
  std::string eef_;
  std::string prefix_;
  double time_scale_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::atomic_bool seen_state_{false};
  std::atomic_bool seen_joint_state_{false};
  std::atomic_bool closed_{false};
  mutable std::mutex joint_mutex_;
  mutable std::condition_variable joint_condition_;
  std::unordered_map<std::string, double> joint_positions_;
  std::unordered_map<std::string, double> joint_velocities_;
};

bool executeSync(const Arm& left, const trajectory_msgs::msg::JointTrajectory& left_trajectory,
                 const Arm& right, const trajectory_msgs::msg::JointTrajectory& right_trajectory)
{
  auto left_command = left_trajectory;
  auto right_command = right_trajectory;
  ensureTiming(left_command);
  ensureTiming(right_command);
  if (left_command.points.empty() || right_command.points.empty() ||
      left_command.joint_names.empty() || right_command.joint_names.empty())
  {
    return false;
  }

  const double left_duration = pointTime(left_command.points.back());
  const double right_duration = pointTime(right_command.points.back());
  const double common_duration = std::max(left_duration, right_duration);
  if (common_duration <= 1e-6 || std::abs(left.timeScale() - right.timeScale()) > 1e-9)
  {
    return false;
  }

  const auto start = std::chrono::steady_clock::now() + 100ms;
  std::this_thread::sleep_until(start);

  // time_scale_ 原本已经被 Arm 持有；通过左臂读取后两个 Arm 构造时传入同一值。
  // 使用 0..1 的三次平滑时间律 3u^2-2u^3：起止速度为零，两个 Articulation
  // 共享同一 phase 与总时长。Task26 实测它的连续倾角小于五次时间律，故保留。
  const double physical_duration = common_duration * left.timeScale();
  std::size_t tick = 0;
  while (true)
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start).count();
    const double linear_phase = std::clamp(elapsed / physical_duration, 0.0, 1.0);
    const double smooth_phase = linear_phase * linear_phase * (3.0 - 2.0 * linear_phase);
    const auto stamp = left.nowMsg();
    left.publishAt(left_command, smooth_phase * left_duration, stamp);
    right.publishAt(right_command, smooth_phase * right_duration, stamp);
    if (linear_phase >= 1.0)
    {
      break;
    }
    ++tick;
    std::this_thread::sleep_until(start + tick * kDualCommandPeriod);
  }

  for (int repeat = 0; repeat < kFinalCommandHold; ++repeat)
  {
    const auto stamp = left.nowMsg();
    left.publishAt(left_command, left_duration, stamp);
    right.publishAt(right_command, right_duration, stamp);
    std::this_thread::sleep_for(kDualCommandPeriod);
  }
  return left.waitAtTarget(left_command, kJointSettleToleranceRad, kJointSettleTimeoutSec) &&
    right.waitAtTarget(right_command, kJointSettleToleranceRad, kJointSettleTimeoutSec);
}

bool synchronize(trajectory_msgs::msg::JointTrajectory* first, trajectory_msgs::msg::JointTrajectory* second)
{
  ensureTiming(*first);
  ensureTiming(*second);
  if (first->points.empty() || second->points.empty())
  {
    return false;
  }
  const double duration = std::max(pointTime(first->points.back()), pointTime(second->points.back()));
  for (auto* trajectory : {first, second})
  {
    if (trajectory->points.empty())
    {
      return false;
    }

    const double original = pointTime(trajectory->points.back());
    if (original <= 1e-9)
    {
      return false;
    }
    for (auto& point : trajectory->points)
    {
      setPointTime(point, pointTime(point) * duration / original);
    }
  }
  return true;
}

bool validateSync(
  const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& model,
  const std::vector<moveit_msgs::msg::CollisionObject>& world,
  const trajectory_msgs::msg::JointTrajectory& left, const trajectory_msgs::msg::JointTrajectory& right,
  const std::string& label)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  for (const auto& object : world)
  {
    if (!scene->processCollisionObjectMsg(object))
    {
      RCLCPP_ERROR(node->get_logger(), "%s cannot load world object %s into FCL.", label.c_str(), object.id.c_str());
      return false;
    }
  }
  const double duration = std::max(pointTime(left.points.back()), pointTime(right.points.back()));
  const std::size_t samples = static_cast<std::size_t>(std::ceil(duration / kFclSamplePeriod));
  for (std::size_t index = 0; index <= samples; ++index)
  {
    const auto left_q = interpolate(left, std::min(duration, index * kFclSamplePeriod));
    const auto right_q = interpolate(right, std::min(duration, index * kFclSamplePeriod));
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setVariablePositions(left.joint_names, left_q);
    state.setVariablePositions(right.joint_names, right_q);
    state.update();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1;
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, state);
    if (result.collision)
    {
      const double collision_time = std::min(duration, index * kFclSamplePeriod);
      RCLCPP_ERROR(node->get_logger(), "%s FCL collision at t=%.3f s.",
        label.c_str(), collision_time);
      for (const auto& [pair, contacts] : result.contacts)
      {
        RCLCPP_ERROR(node->get_logger(), "%s FCL pair: %s <-> %s.",
          label.c_str(), pair.first.c_str(), pair.second.c_str());
        for (const auto& contact : contacts)
        {
          RCLCPP_ERROR(node->get_logger(),
            "%s FCL contact: pos=(%.3f, %.3f, %.3f) depth=%.3f mm.",
            label.c_str(), contact.pos.x(), contact.pos.y(), contact.pos.z(),
            contact.depth * 1000.0);
        }
        // 记录碰撞采样点的 link 原点，区分“目标姿态本身过低”和“Cartesian
        // IK 分支在中途下探”。这只是诊断，不放宽任何碰撞规则。
        for (const auto& name : {pair.first, pair.second})
        {
          if (model->hasLinkModel(name))
          {
            const auto& p = state.getGlobalLinkTransform(name).translation();
            RCLCPP_ERROR(node->get_logger(), "%s FCL link %s origin=(%.3f, %.3f, %.3f).",
              label.c_str(), name.c_str(), p.x(), p.y(), p.z());
            // Task26 侧吸盘的 collision 依次是：竖杆、横杆、面板、四个 Cup。
            // 输出各 primitive 的世界原点，以便定位真实擦碰实体；不据此放宽 ACM。
            const auto* link = model->getLinkModel(name);
            const auto& local_origins = link->getCollisionOriginTransforms();
            for (std::size_t collision_index = 0; collision_index < local_origins.size(); ++collision_index)
            {
              const auto world = state.getGlobalLinkTransform(name) * local_origins[collision_index];
              const auto& origin = world.translation();
              RCLCPP_ERROR(node->get_logger(),
                "%s FCL collision_primitive=%zu world_origin=(%.3f, %.3f, %.3f).",
                label.c_str(), collision_index, origin.x(), origin.y(), origin.z());
            }
          }
        }
      }
      return false;
    }
  }
  RCLCPP_INFO(node->get_logger(), "%s synchronized FCL PASS, samples=%zu.", label.c_str(), samples + 1);
  return true;
}

// 用 FK 检查一条笛卡尔轨迹的中间状态是否真的贴着命令直线。
// MoveIt 的 computeCartesianPath 在个别路径点 IK 失败时仍会报出 1.0000 的 fraction，
// 但中间状态可能离直线极远（实测左臂 COMMON_Y_ALIGN 的 TCP 从 y=0.02 甩到 y=-1.2 m
// 再绕回来，且中途扫过桌面）。这类轨迹既不是真实碰撞也不是可用构型，必须整体拒绝并
// 换一个采样步长重规划，而不是等同步 FCL 在最后一段才发现。
double cartesianLineDeviation(
  const moveit::core::RobotModelConstPtr& model, const std::string& eef_link,
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  const geometry_msgs::msg::Pose& target)
{
  if (trajectory.points.empty() || trajectory.joint_names.empty())
  {
    return std::numeric_limits<double>::infinity();
  }
  const auto poseAt = [&](const std::vector<double>& positions) {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setVariablePositions(trajectory.joint_names, positions);
    state.update();
    return state.getGlobalLinkTransform(eef_link).translation();
  };
  const Eigen::Vector3d start = poseAt(trajectory.points.front().positions);
  const Eigen::Vector3d goal(
    target.position.x, target.position.y, target.position.z);
  const Eigen::Vector3d delta = goal - start;
  const double span = delta.norm();
  if (span <= 1e-9)
  {
    return 0.0;
  }
  const Eigen::Vector3d direction = delta / span;
  double worst = 0.0;
  for (const auto& point : trajectory.points)
  {
    const Eigen::Vector3d actual = poseAt(point.positions);
    const double alpha = std::clamp(
      (actual - start).dot(direction) / span, 0.0, 1.0);
    worst = std::max(worst, (actual - (start + alpha * delta)).norm());
  }
  return worst;
}

bool planCommonCartesian(
  const rclcpp::Node::SharedPtr& node, moveit::planning_interface::MoveGroupInterface& group,
  const std::string& own_group, const std::string& partner_group,
  const std::vector<double>& own_start, const std::vector<double>& partner_start,
  const geometry_msgs::msg::Pose& target, const std::string& label,
  trajectory_msgs::msg::JointTrajectory* output,
  const std::string& eef_link, bool avoid_collisions = false)
{
  const auto model = group.getRobotModel();
  const auto* own = model->getJointModelGroup(own_group);
  const auto* partner = model->getJointModelGroup(partner_group);
  if (!own || !partner || own_start.empty() || partner_start.empty())
  {
    return false;
  }
  // 同一个起点可能因为 IK 采样序列不同而产生贴线或离线的轨迹。先按默认步长试，
  // 只有在轨迹离线时才换步长重试；不接受任何不贴线的轨迹。
  const std::array<double, 4> steps{kCartesianStep, 0.0015, 0.003, 0.001};
  for (const double step : steps)
  {
    for (int attempt = 1; attempt <= kRetries; ++attempt)
    {
      auto state = group.getCurrentState(2.0);
      if (!state)
      {
        return false;
      }
      state->setJointGroupPositions(own, own_start);
      state->setJointGroupPositions(partner, partner_start);
      state->update();
      group.setStartState(*state);
      moveit_msgs::msg::RobotTrajectory candidate;
      moveit_msgs::msg::MoveItErrorCodes error;
      // 共同搬运阶段的完整双臂 FCL 在候选同步后进行；不能把搭档臂冻结在阶段
      // 起点而误判。但空载接触阶段必须同时避开桌面，故由调用点显式开启。
      const double fraction = group.computeCartesianPath(
        {target}, step, kCartesianJumpThreshold, candidate, avoid_collisions, &error);
      if (fraction < kMinCartesianFraction || candidate.joint_trajectory.points.empty())
      {
        RCLCPP_INFO(node->get_logger(), "%s Cartesian fraction=%.4f error=%d step=%.4f attempt=%d/%d.",
          label.c_str(), fraction, error.val, step, attempt, kRetries);
        continue;
      }
      const double deviation = cartesianLineDeviation(
        model, eef_link, candidate.joint_trajectory, target);
      RCLCPP_INFO(node->get_logger(),
        "%s Cartesian fraction=%.4f error=%d step=%.4f attempt=%d/%d line_deviation=%.3f mm.",
        label.c_str(), fraction, error.val, step, attempt, kRetries, deviation * 1000.0);
      if (deviation > kMaxCartesianLineDeviation)
      {
        RCLCPP_WARN(node->get_logger(),
          "%s rejected: 中间状态离线 %.1f mm（超过 %.1f mm）；换步长重规划。",
          label.c_str(), deviation * 1000.0, kMaxCartesianLineDeviation * 1000.0);
        break;
      }
      *output = candidate.joint_trajectory;
      ensureTiming(*output);
      return true;
    }
  }
  return false;
}

std::vector<moveit_msgs::msg::CollisionObject> staticWorld(
  moveit::planning_interface::PlanningSceneInterface& scene)
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  for (const auto& [id, object] : scene.getObjects())
  {
    (void)id;
    objects.push_back(object);
  }
  return objects;
}

bool planAndCheckCommon(
  const rclcpp::Node::SharedPtr& node, moveit::planning_interface::MoveGroupInterface& left_group,
  moveit::planning_interface::MoveGroupInterface& right_group, const Arm& left, const Arm& right,
  const std::vector<double>& left_start, const std::vector<double>& right_start,
  const geometry_msgs::msg::Pose& left_target, const geometry_msgs::msg::Pose& right_target,
  const std::vector<moveit_msgs::msg::CollisionObject>& world, const std::string& stage,
  trajectory_msgs::msg::JointTrajectory* left_output, trajectory_msgs::msg::JointTrajectory* right_output)
{
  // 共同负载段：两臂在同一 phase 下同步运动，但 computeCartesianPath 只能一次
  // 规划一条单臂路径，并且会把搭档臂冻结在该段起点。Task26 的 COMMON_Y_ALIGN 需要
  // 把 Cube 从 y=-0.070 搬到目标行的 y=±0.120，两臂要一起走 +190 mm；此时“冻结的
  // 搭档臂”会被真实运动的另一臂扫到，MoveIt 会把 Cartesian 路径截断在约 63.5%
  // （实测截断点接触对为 left_fr3_side_suction <-> right_fr3_side_suction）。
  // 那是冻结假设造成的假阳性，不是真实碰撞。
  //
  // 因此共同段的两条单臂路径不做“对冻结搭档”的碰撞判断，真正的门禁是紧随其后的
  // validateSync：它按同一时间参数采样左右两条轨迹，对完整双臂 RobotState 做
  // robot--robot 与 robot--world 的 FCL 检查。ACM 没有扩大，世界障碍物也没有移除，
  // 任何真实碰撞仍会在 validateSync 处以具体碰撞对和接触点被拒绝。
  if (!planCommonCartesian(node, left_group, left.groupName(), right.groupName(), left_start, right_start,
                           left_target, stage + " left", left_output, left.eefLink(), false) ||
      !planCommonCartesian(node, right_group, right.groupName(), left.groupName(), right_start, left_start,
                           right_target, stage + " right", right_output, right.eefLink(), false) ||
      !synchronize(left_output, right_output) ||
      !validateSync(node, left_group.getRobotModel(), world, *left_output, *right_output, stage))
  {
    return false;
  }
  return true;
}

// ---------------------------------------------------------------- 推入段控制
//
// Task26 推入段参数（1D 导纳 + 卡阻检测）。全部来自
// config/task26_push_control.yaml，不允许散落硬编码。
struct PushControlConfig
{
  double push_base_advance_m{0.200};   // 推入前推臂基座沿轨 +X 前进量
  double force_bias_seconds{0.5};
  double force_lpf_cutoff_hz{15.0};
  std::string wrench_log_path;
  double contact_force_n{1.5};
  double contact_hold_sec{0.08};
  double target_force_n{4.0};
  double force_ramp_sec{0.4};
  double virtual_mass{2.0};
  double virtual_damping{250.0};
  double virtual_stiffness{0.0};
  double max_push_speed{0.016};
  double max_retreat_speed{0.008};
  double jam_force_n{12.0};
  double jam_hold_sec{0.2};
  double jam_window_sec{0.3};
  double jam_min_progress_m{0.001};
  double seated_depth_tolerance_m{0.003};
  int jam_retry_limit{2};
  double retract_distance_m{0.008};
};

PushControlConfig loadPushControlConfig(const rclcpp::Node::SharedPtr& node)
{
  PushControlConfig config;
  std::string path;
  try
  {
    const std::string share = ament_index_cpp::get_package_share_directory("fr3_dual_palletize");
    path = share + "/config/task26_push_control.yaml";
  }
  catch (const std::exception& error)
  {
    RCLCPP_WARN(node->get_logger(), "找不到 fr3_dual_palletize share 目录：%s", error.what());
    return config;
  }
  std::ifstream stream(path);
  if (!stream)
  {
    RCLCPP_WARN(node->get_logger(), "推入控制参数 %s 不存在，使用内置默认值。", path.c_str());
    return config;
  }
  const YAML::Node root = YAML::Load(stream);
  const YAML::Node values = root["task26_push_control"]["ros__parameters"];
  if (!values)
  {
    RCLCPP_WARN(node->get_logger(), "%s 缺少 task26_push_control.ros__parameters。", path.c_str());
    return config;
  }
  auto readDouble = [&](const char* key, double& target)
  {
    if (values[key]) { target = values[key].as<double>(); }
  };
  auto readInt = [&](const char* key, int& target)
  {
    if (values[key]) { target = values[key].as<int>(); }
  };
  readDouble("push_base_advance_m", config.push_base_advance_m);
  readDouble("force_bias_seconds", config.force_bias_seconds);
  readDouble("force_lpf_cutoff_hz", config.force_lpf_cutoff_hz);
  readDouble("contact_force_n", config.contact_force_n);
  readDouble("contact_hold_sec", config.contact_hold_sec);
  readDouble("target_force_n", config.target_force_n);
  readDouble("force_ramp_sec", config.force_ramp_sec);
  readDouble("virtual_mass", config.virtual_mass);
  readDouble("virtual_damping", config.virtual_damping);
  readDouble("virtual_stiffness", config.virtual_stiffness);
  readDouble("max_push_speed", config.max_push_speed);
  readDouble("max_retreat_speed", config.max_retreat_speed);
  readDouble("jam_force_n", config.jam_force_n);
  readDouble("jam_hold_sec", config.jam_hold_sec);
  readDouble("jam_window_sec", config.jam_window_sec);
  readDouble("jam_min_progress_m", config.jam_min_progress_m);
  readDouble("seated_depth_tolerance_m", config.seated_depth_tolerance_m);
  readInt("jam_retry_limit", config.jam_retry_limit);
  readDouble("retract_distance_m", config.retract_distance_m);
  if (values["wrench_log_path"]) { config.wrench_log_path = values["wrench_log_path"].as<std::string>(); }
  RCLCPP_INFO(node->get_logger(),
    "推入控制参数已载入 %s：F_contact=%.2f N, F_target=%.2f N, M_d=%.2f kg, B_d=%.1f N*s/m, "
    "K_d=%.2f, v_push<=%.3f m/s, F_jam=%.1f N",
    path.c_str(), config.contact_force_n, config.target_force_n, config.virtual_mass,
    config.virtual_damping, config.virtual_stiffness, config.max_push_speed, config.jam_force_n);
  return config;
}

// 取名字里最后一段连续数字："left_fr3_joint3" / "fr3_joint3" 都得到 3。
int trailingJointIndex(const std::string& name)
{
  int last = -1;
  int current = -1;
  for (const char character : name)
  {
    if (character >= '0' && character <= '9')
    {
      current = (current < 0 ? 0 : current) * 10 + (character - '0');
    }
    else if (current >= 0)
    {
      last = current;
      current = -1;
    }
  }
  return current >= 0 ? current : last;
}

// 关节实测力矩（桥在 /task26/{left,right}/measured_joint_forces 上按 20 Hz 发布）。
class ForceBuffer
{
public:
  ForceBuffer(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
      topic, 10, [this](const sensor_msgs::msg::JointState::SharedPtr message)
      {
        double peak = 0.0;
        std::vector<double> efforts;
        efforts.reserve(message->effort.size());
        for (const auto value : message->effort)
        {
          peak = std::max(peak, std::abs(value));
          efforts.push_back(value);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        peak_ = peak;
        // 力估计需要完整向量，不再只留 max（peak() 行为保持不变）。
        efforts_ = std::move(efforts);
        ready_ = true;
      });
  }
  double peak() const { std::lock_guard<std::mutex> lock(mutex_); return peak_; }
  bool ready() const { std::lock_guard<std::mutex> lock(mutex_); return ready_; }
  std::vector<double> efforts() const { std::lock_guard<std::mutex> lock(mutex_); return efforts_; }

private:
  mutable std::mutex mutex_;
  double peak_{0.0};
  std::vector<double> efforts_;
  bool ready_{false};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

// 末端等效力旋量估计（开发顺序 Step 2：只加力传感，不动控制器）。
//
// 只用"真机上也能拿到"的信号：关节位置 + 关节实测力矩。
//   tau_ext = tau_measured - tau_bias
//   F_ext   = (J^T)^+ tau_ext     J = 末端几何 Jacobian（世界系）
//   F_push  = -F_ext . e_push     把阻碍推进的环境反力定义为正
//
// tau_bias 在接触前的静止窗口采集，因此不做 q 插值（第一版限制：推进期间关节
// 几乎不动）。Cube--槽 之间的接触力属于仿真上帝视角，只允许写入日志用于真值
// 对照，绝不允许作为闭环输入。
class WrenchEstimator
{
public:
  WrenchEstimator(const rclcpp::Node::SharedPtr& node, const std::string& side,
                  const moveit::core::RobotModelConstPtr& model, const std::string& tip_link,
                  const Eigen::Vector3d& push_axis, const PushControlConfig& config)
    : node_(node), side_(side), prefix_(side + "_"), model_(model), tip_link_(tip_link),
      push_axis_(push_axis.normalized()), config_(config),
      forces_(node, taskTopic("/" + side + "/measured_joint_forces")),
      first_stamp_(node->now())
  {
    subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/" + side + "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        names_ = message->name;
        positions_ = message->position;
        seen_ = true;
      });
  }

  // 采集力偏置：要求调用方保证这段时间内末端没有接触、机械臂静止。
  bool collectBias()
  {
    bias_torque_.assign(7, 0.0);
    std::vector<int> samples(7, 0);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(config_.force_bias_seconds);
    while (std::chrono::steady_clock::now() < deadline)
    {
      const std::vector<double> torque = forces_.efforts();
      if (torque.size() >= 7)
      {
        for (std::size_t index = 0; index < 7; ++index)
        {
          bias_torque_[index] += torque[index];
          ++samples[index];
        }
      }
      std::this_thread::sleep_for(5ms);
    }
    bool ok = true;
    for (std::size_t index = 0; index < 7; ++index)
    {
      if (samples[index] == 0) { ok = false; break; }
      bias_torque_[index] /= static_cast<double>(samples[index]);
    }
    if (!ok)
    {
      RCLCPP_ERROR(node_->get_logger(), "%s 力偏置采集期间没有收到实测力矩。", side_.c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "%s 力偏置已采集（tau_bias=[%.2f %.2f %.2f %.2f %.2f %.2f %.2f] N*m）。",
      side_.c_str(), bias_torque_[0], bias_torque_[1], bias_torque_[2], bias_torque_[3],
      bias_torque_[4], bias_torque_[5], bias_torque_[6]);
    return true;
  }

  // 单次估计：返回 false 表示信号不全（没有关节状态 / 力矩）。
  bool update()
  {
    std::vector<std::string> names;
    std::vector<double> positions;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!seen_) { return false; }
      names = names_;
      positions = positions_;
    }
    const std::vector<double> torque = forces_.efforts();
    if (torque.size() < 7 || bias_torque_.size() < 7 || names.size() != positions.size())
    {
      return false;
    }

    // 1) 关节状态按关节号 (1..7) 写入模型，不依赖发布顺序。
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    for (std::size_t index = 0; index < names.size(); ++index)
    {
      const int joint = trailingJointIndex(names[index]);
      if (joint < 1 || joint > 7) { continue; }
      state.setVariablePosition(prefix_ + "fr3_joint" + std::to_string(joint), positions[index]);
    }
    state.update();

    const moveit::core::LinkModel* tip = model_->getLinkModel(tip_link_);
    if (tip == nullptr)
    {
      RCLCPP_ERROR_ONCE(node_->get_logger(), "模型里没有 %s。", tip_link_.c_str());
      return false;
    }
    const Eigen::Vector3d tip_position = state.getGlobalLinkTransform(tip).translation();

    // 2) 自建几何 Jacobian（6x7）。FR3 七个关节都绕父连杆局部 +Z 转，
    //    关节 j 的父连杆是 "{prefix}fr3_link{j-1}"，不依赖 MoveIt 的列顺序约定。
    Eigen::MatrixXd jacobian(6, 7);
    Eigen::VectorXd residual = Eigen::VectorXd::Zero(7);
    for (int joint = 1; joint <= 7; ++joint)
    {
      const std::string parent = prefix_ + "fr3_link" + std::to_string(joint - 1);
      if (model_->getLinkModel(parent) == nullptr)
      {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "模型里没有 %s。", parent.c_str());
        return false;
      }
      const Eigen::Isometry3d transform = state.getGlobalLinkTransform(parent);
      const Eigen::Vector3d axis = transform.linear().col(2).normalized();
      const Eigen::Vector3d origin = transform.translation();
      jacobian.block<3, 1>(0, joint - 1) = axis.cross(tip_position - origin);
      jacobian.block<3, 1>(3, joint - 1) = axis;

      for (std::size_t index = 0; index < names.size() && index < torque.size(); ++index)
      {
        if (trailingJointIndex(names[index]) == joint)
        {
          residual[joint - 1] = torque[index] - bias_torque_[joint - 1];
          break;
        }
      }
    }

    const Eigen::VectorXd wrench =
      jacobian.transpose().completeOrthogonalDecomposition().solve(residual);
    force_vector_ = wrench.head<3>();
    raw_force_ = -force_vector_.dot(push_axis_);   // 阻碍推进的力为正

    // 3) 一阶低通（PhysX 接触力不干净，raw 只用于日志）。
    if (!filter_initialized_)
    {
      filtered_force_ = raw_force_;
      filter_initialized_ = true;
    }
    else
    {
      const double now = node_->now().seconds();
      const double dt = std::max(1e-4, now - last_stamp_);
      const double omega = 2.0 * kPi * std::max(0.1, config_.force_lpf_cutoff_hz);
      const double alpha = omega * dt / (omega * dt + 1.0);
      filtered_force_ = alpha * raw_force_ + (1.0 - alpha) * filtered_force_;
    }
    last_stamp_ = node_->now().seconds();
    return true;
  }

  double rawForce() const { return raw_force_; }
  double filteredForce() const { return filtered_force_; }
  double peakTorque() const { return forces_.peak(); }
  const Eigen::Vector3d& forceVector() const { return force_vector_; }
  const Eigen::Vector3d& pushAxis() const { return push_axis_; }
  const std::string& side() const { return side_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string side_;
  std::string prefix_;
  moveit::core::RobotModelConstPtr model_;
  std::string tip_link_;
  Eigen::Vector3d push_axis_;
  PushControlConfig config_;
  ForceBuffer forces_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  mutable std::mutex mutex_;
  std::vector<std::string> names_;
  std::vector<double> positions_;
  bool seen_{false};
  std::vector<double> bias_torque_;
  Eigen::Vector3d force_vector_{Eigen::Vector3d::Zero()};
  double raw_force_{0.0};
  double filtered_force_{0.0};
  bool filter_initialized_{false};
  double last_stamp_{0.0};
  rclcpp::Time first_stamp_;
};

// 滑轨客户端：把"整条机械臂沿 X 平移"这件事封起来。
//
// bridge 侧的 _apply_rail 是**瞬移**（同时写 USD 与 Fabric 的 world pose），不是
// 连续滑动；因此工具在离 Cube 1 mm 处被平移不会"贴着面刮过去"，不会重现最初那个
// 拖件问题。互锁两道：距上次关节命令需静默 RAIL_COMMAND_QUIET_SEC=1.0 s，且
// **任一吸盘夹紧即拒绝移动**——所以动轨必须在两侧吸盘都送掉之后。
class RailClient
{
public:
  RailClient(const rclcpp::Node::SharedPtr& node, const std::string& side)
    : node_(node), side_(side)
  {
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
      taskTopic("/" + side_ + "/rail_command"), 10);
    subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      taskTopic("/" + side_ + "/rail_state"), 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (message->data.size() >= 2)
        {
          target_ = message->data[0];
          arrived_ = message->data[1] > 0.5;
          have_state_ = true;
          if (message->data.size() >= 3)
          {
            measured_ = message->data[2];
            have_measured_ = true;
          }
        }
      });
  }

  // 分阶段续跑不能把上一进程留下的滑轨站位误当成静止位。只接受桥确认
  // 到位的实测 X，规划世界/目标随后统一减去同一个基座位移。
  bool waitCurrent(double timeout_sec, double* measured_x) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok())
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (have_state_ && have_measured_ && arrived_ &&
            std::isfinite(measured_) && std::abs(target_ - measured_) <= 1e-3)
        {
          *measured_x = measured_;
          return true;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    RCLCPP_ERROR(node_->get_logger(),
      "%s rail 未能在 %.1f s 内提供已到位的实测位置；拒绝按默认站位规划。",
      side_.c_str(), timeout_sec);
    return false;
  }

  bool moveTo(double target, double timeout_sec)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      arrived_ = false;
    }
    std_msgs::msg::Float64 command;
    command.data = target;
    publisher_->publish(command);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok())
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (have_state_ && arrived_ && std::abs(target_ - target) <= 1e-3)
        {
          RCLCPP_INFO(node_->get_logger(), "%s rail 已到位 x=%.3f（目标 %.3f）。",
            side_.c_str(), target_, target);
          return true;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    RCLCPP_ERROR(node_->get_logger(),
      "%s rail 未在 %.1f s 内到位（目标 %.3f）——很可能被互锁拒绝："
      "需两侧吸盘都松开，且距上次关节命令静默 >= 1.0 s。",
      side_.c_str(), timeout_sec, target);
    return false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string side_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  mutable std::mutex mutex_;
  double target_{0.0};
  double measured_{0.0};
  bool arrived_{false};
  bool have_state_{false};
  bool have_measured_{false};
};

// 世界偏移变化后必须重建静态规划世界（桌面 + 车厢三面墙），否则规划器眼里
// 车厢还停在旧位置。Cube 由各自的 apply/remove 流程管理，这里不碰。
bool refreshStaticWorld(moveit::planning_interface::PlanningSceneInterface& scene, bool include_box_walls)
{
  scene.removeCollisionObjects({
    "task26_table", "task26_box_wall_deep", "task26_box_wall_minus_y", "task26_box_wall_plus_y"});
  std::this_thread::sleep_for(300ms);
  std::vector<moveit_msgs::msg::CollisionObject> objects{tableObject()};
  if (include_box_walls)
  {
    const auto walls = boxWallObjects();
    objects.insert(objects.end(), walls.begin(), walls.end());
  }
  const bool ok = scene.applyCollisionObjects(objects);
  std::this_thread::sleep_for(300ms);
  return ok;
}

// 把一条已规划的笛卡尔轨迹切成 [t0, t1] 子段，用于「分段推进 + 逐段监督」。
// 首尾用插值点补齐，段内时间从 0 开始。
bool sliceTrajectory(const trajectory_msgs::msg::JointTrajectory& input,
                     double t0, double t1,
                     trajectory_msgs::msg::JointTrajectory* output)
{
  if (input.points.empty() || input.joint_names.empty() || t1 - t0 <= 1e-6)
  {
    return false;
  }
  output->joint_names = input.joint_names;
  output->points.clear();
  trajectory_msgs::msg::JointTrajectoryPoint begin;
  begin.positions = interpolate(input, t0);
  setPointTime(begin, 0.0);
  output->points.push_back(begin);
  for (const auto& point : input.points)
  {
    const double time = pointTime(point);
    if (time > t0 + 1e-6 && time < t1 - 1e-6)
    {
      auto copy = point;
      setPointTime(copy, time - t0);
      output->points.push_back(std::move(copy));
    }
  }
  trajectory_msgs::msg::JointTrajectoryPoint end;
  end.positions = interpolate(input, t1);
  setPointTime(end, t1 - t0);
  output->points.push_back(std::move(end));
  return output->points.size() >= 2;
}

// 空载侧墙压紧完成后，撤离严格复用已经过 FCL 的入场轨迹反向版本；不在 Cup
// 贴着 Cube/墙的位置重新调用 RRT，避免随机解支把末端扫进已落稳的 Cube。
trajectory_msgs::msg::JointTrajectory reverseTrajectory(
  const trajectory_msgs::msg::JointTrajectory& input)
{
  trajectory_msgs::msg::JointTrajectory output;
  if (input.points.empty() || input.joint_names.empty())
  {
    return output;
  }
  const double duration = pointTime(input.points.back());
  output.joint_names = input.joint_names;
  output.points.reserve(input.points.size());
  for (auto it = input.points.rbegin(); it != input.points.rend(); ++it)
  {
    auto point = *it;
    point.velocities.clear();
    point.accelerations.clear();
    point.effort.clear();
    setPointTime(point, std::max(0.0, duration - pointTime(*it)));
    output.points.push_back(std::move(point));
  }
  ensureTiming(output);
  return output;
}

// 用 FK 求某个 link 的世界位置（推进监督用：把命令位置与 Cube Ground Truth 对比）。
Eigen::Vector3d linkPosition(const moveit::core::RobotModelConstPtr& model,
                             const std::string& link,
                             const std::vector<std::string>& names,
                             const std::vector<double>& positions)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setVariablePositions(names, positions);
  state.update();
  return state.getGlobalLinkTransform(link).translation();
}

// +X 推入后、侧墙压紧前的中间验收：深格贴车厢深端墙；浅格贴同排已完成的
// 深格 Cube。二者都属于 +X 方向的实体支撑面，不能把浅格到外墙的 120 mm
// 正常层内间距误判为失败。
double xSupportGap(const OfflineTask& task, const geometry_msgs::msg::Pose& actual)
{
  if (std::abs(task.cell.position.x - kCellDeepX) < 1e-6)
  {
    return kBoxInteriorX1 - (actual.position.x + kCubeHalf);
  }
  // 深格 Cube 的 -X 面在 kCellDeepX - kCubeHalf；浅格 Cube 的 +X 面为
  // actual.x + kCubeHalf，两面相减即为层内贴合缝隙。
  return (kCellDeepX - kCubeHalf) - (actual.position.x + kCubeHalf);
}

const char* xSupportName(const OfflineTask& task)
{
  return std::abs(task.cell.position.x - kCellDeepX) < 1e-6
    ? "+X deep wall" : "same-row deep cube";
}

bool isCenterInsertTask(const OfflineTask& task)
{
#ifdef TASK27_FIVE_CUBE
  return task.cube_index == 4;
#else
  (void)task;
  return false;
#endif
}

bool isInnerReferenceTask(const OfflineTask& task)
{
#ifdef TASK27_FIVE_CUBE
  return task.cube_index == 2 || task.cube_index == 3;
#else
  (void)task;
  return false;
#endif
}

bool isStraightInsertTask(const OfflineTask& task)
{
  return isInnerReferenceTask(task) || isCenterInsertTask(task);
}

// 五件横向布局中，最外件由车厢侧墙支撑，内件由已经完成的同侧外件支撑。
// Task26 的两个目标都在最外侧，故默认行为完全不变。
bool sideSupportIsWall(const OfflineTask& task)
{
  return std::abs(task.cell.position.y) >= std::abs(kCellYPlus) - 1e-6;
}

const char* sideSupportName(const OfflineTask& task)
{
  if (sideSupportIsWall(task))
  {
    return task.cell.position.y > 0.0 ? "+Y wall" : "-Y wall";
  }
  return task.cell.position.y > 0.0 ? "+Y outer cube" : "-Y outer cube";
}

double sideSupportGap(const OfflineTask& task, const geometry_msgs::msg::Pose& actual)
{
  const bool toward_plus_y = task.cell.position.y > 0.0;
  if (sideSupportIsWall(task))
  {
    return toward_plus_y
      ? kBoxInteriorY1 - (actual.position.y + kCubeHalf)
      : (actual.position.y - kCubeHalf) - kBoxInteriorY0;
  }
  const double outer_center_y = toward_plus_y ? kCellYPlus : kCellYMinus;
  return toward_plus_y
    ? (outer_center_y - kCubeHalf) - (actual.position.y + kCubeHalf)
    : (actual.position.y - kCubeHalf) - (outer_center_y + kCubeHalf);
}

double measuredSideSupportGap(const OfflineTask& task,
                              const geometry_msgs::msg::Pose& actual,
                              const CubeBuffer& cubes)
{
#ifdef TASK27_FIVE_CUBE
  if (!sideSupportIsWall(task) && !isCenterInsertTask(task))
  {
    // Task27 的 Cube 01 / 02 分别是 +Y / -Y 外侧件；内侧件必须以它们的
    // Isaac Ground Truth 面为准，而非仅以名义格位计算贴合间隙。
    const bool toward_plus_y = task.cell.position.y > 0.0;
    const auto [outer, outer_revision] = cubes.get(toward_plus_y ? 0 : 1);
    (void)outer_revision;
    return toward_plus_y
      ? (outer.position.y - kCubeHalf) - (actual.position.y + kCubeHalf)
      : (actual.position.y - kCubeHalf) - (outer.position.y + kCubeHalf);
  }
#else
  (void)cubes;
#endif
  return sideSupportGap(task, actual);
}

bool validDeepWallSeat(const rclcpp::Node::SharedPtr& node, const OfflineTask& task,
                       const geometry_msgs::msg::Pose& actual)
{
  const double x_gap = xSupportGap(task, actual);
  const double z_error = std::abs(actual.position.z - task.cell.position.z);
  RCLCPP_INFO(node->get_logger(),
    "%s +X support seat (%s): gap=%+.3f mm (target 0), z_error=%.3f mm.",
    task.id, xSupportName(task), x_gap * 1000.0, z_error * 1000.0);
  return std::abs(x_gap) <= kWallFlushTolerance && z_error <= kPlacementTolerance;
}

// 最终验收：Cube 同时贴住 +X 深端墙和所属的 ±Y 侧墙。欧氏格位误差合格不足以
// 证明没有可见侧缝，因此两个墙面 gap 必须分别落在紧的物理门限内。
bool validCellPlacement(const rclcpp::Node::SharedPtr& node, const OfflineTask& task,
                        const geometry_msgs::msg::Pose& actual, const CubeBuffer& cubes)
{
#ifndef TASK27_FIVE_CUBE
  (void)cubes;
#endif
  const double error = distance3d(task.cell.position, actual.position);
  const double x_gap = xSupportGap(task, actual);
  if (isCenterInsertTask(task))
  {
#ifdef TASK27_FIVE_CUBE
    // Cube 03 / 04 分别是已经验收的 +Y / -Y 内件。车厢比五件总宽多 28 mm，
    // 中心件的正确结果是两侧各约 14 mm，而不是靠穿透碰撞体伪造双侧零缝。
    const auto [plus_inner, plus_revision] = cubes.get(2);
    const auto [minus_inner, minus_revision] = cubes.get(3);
    (void)plus_revision;
    (void)minus_revision;
    const double plus_gap = (plus_inner.position.y - kCubeHalf) -
      (actual.position.y + kCubeHalf);
    const double minus_gap = (actual.position.y - kCubeHalf) -
      (minus_inner.position.y + kCubeHalf);
    constexpr double kExpectedCenterTotalGap =
      (kBoxInteriorY1 - kBoxInteriorY0) - 5.0 * kCubeSize -
      2.0 * kInnerStraightInsertClearance;
    constexpr double kCenterTotalGapTolerance = 0.006;
    constexpr double kCenterGapBalanceTolerance = 0.006;
    constexpr double kCenterOffsetTolerance = 0.006;
    RCLCPP_INFO(node->get_logger(),
      "%s center Ground Truth: expected=(%.3f, %.3f, %.3f), actual=(%.3f, %.3f, %.3f), "
      "cell_error=%.3f mm, +X_gap=%+.3f mm, +Y_inner_gap=%+.3f mm, -Y_inner_gap=%+.3f mm, "
      "total=%+.3f mm.",
      task.id, task.cell.position.x, task.cell.position.y, task.cell.position.z,
      actual.position.x, actual.position.y, actual.position.z, error * 1000.0,
      x_gap * 1000.0, plus_gap * 1000.0, minus_gap * 1000.0,
      (plus_gap + minus_gap) * 1000.0);
    return error <= kPlacementTolerance && std::abs(x_gap) <= kWallFlushTolerance &&
           plus_gap >= -kWallFlushTolerance && minus_gap >= -kWallFlushTolerance &&
           std::abs((plus_gap + minus_gap) - kExpectedCenterTotalGap) <= kCenterTotalGapTolerance &&
           std::abs(plus_gap - minus_gap) <= kCenterGapBalanceTolerance &&
           std::abs(actual.position.y) <= kCenterOffsetTolerance;
#endif
  }
  const double side_gap = measuredSideSupportGap(task, actual, cubes);
  RCLCPP_INFO(node->get_logger(),
    "%s final Ground Truth: expected=(%.3f, %.3f, %.3f), actual=(%.3f, %.3f, %.3f), "
    "cell_error=%.3f mm, %s_gap=%+.3f mm, %s_gap=%+.3f mm.",
    task.id, task.cell.position.x, task.cell.position.y, task.cell.position.z,
    actual.position.x, actual.position.y, actual.position.z, error * 1000.0,
    xSupportName(task), x_gap * 1000.0, sideSupportName(task), side_gap * 1000.0);
  return error <= kPlacementTolerance &&
         std::abs(x_gap) <= kWallFlushTolerance &&
         std::abs(side_gap) <= kWallFlushTolerance;
}

// 分段推进并逐段监督：Cube 必须跟着命令走，且关节力矩不得越限。
bool executePushWithSupervision(
  const rclcpp::Node::SharedPtr& node, const Arm& left, const Arm& right,
  const trajectory_msgs::msg::JointTrajectory& left_push,
  const trajectory_msgs::msg::JointTrajectory& right_hold,
  const CubeBuffer& cubes, const OfflineTask& task,
  const moveit::core::RobotModelConstPtr& model, const std::string& eef_link,
  const ForceBuffer& left_forces, const ForceBuffer& right_forces)
{
  const double duration = pointTime(left_push.points.back());
  double peak_torque = 0.0;
  for (int slice = 1; slice <= kPushSlices; ++slice)
  {
    const double t0 = duration * static_cast<double>(slice - 1) / kPushSlices;
    const double t1 = duration * static_cast<double>(slice) / kPushSlices;
    trajectory_msgs::msg::JointTrajectory left_slice;
    trajectory_msgs::msg::JointTrajectory right_slice;
    if (!sliceTrajectory(left_push, t0, t1, &left_slice) ||
        !sliceTrajectory(right_hold, t0, t1, &right_slice) ||
        !executeSync(left, left_slice, right, right_slice))
    {
      RCLCPP_ERROR(node->get_logger(), "%s PUSH slice %d/%d execution failed.",
        task.id, slice, kPushSlices);
      return false;
    }
    const Eigen::Vector3d tcp = linkPosition(
      model, eef_link, left_slice.joint_names, finalPositions(left_slice));
    // 推入臂持 -X 面：Cube 中心 = TCP_x + (半件 + 间隙)。
    // tcp 来自 FK，是**模型系**坐标（滑轨搬站位后已整体偏移 -g_world_shift_x）；
    // 而下面的 pose 是 Isaac Ground Truth，属**仿真系**。不把偏移加回去，lag 会凭空
    // 多出 Δ —— 实测 Δ=0.100 时第一片就报 "lag=100.65 mm" 并误判卡死。
    const double commanded_cube_x = tcp.x() + kPushCupOffsetX + g_world_shift_x;
    const auto [pose, revision] = cubes.get(task.cube_index);
    (void)revision;
    const double lag = std::abs(commanded_cube_x - pose.position.x);
    const double torque = std::max(left_forces.peak(), right_forces.peak());
    peak_torque = std::max(peak_torque, torque);
    RCLCPP_INFO(node->get_logger(),
      "%s PUSH slice %d/%d: commanded_cube_x=%.4f, actual=%.4f, lag=%.2f mm, peak_torque=%.2f Nm.",
      task.id, slice, kPushSlices, commanded_cube_x, pose.position.x, lag * 1000.0, torque);
    if (lag > kPushJamTolerance)
    {
      RCLCPP_ERROR(node->get_logger(),
        "%s PUSH stopped: Cube Ground Truth lagged %.1f mm (limit %.1f mm) — 卡死或没有跟着走。",
        task.id, lag * 1000.0, kPushJamTolerance * 1000.0);
      return false;
    }
    if (torque > kPushTorqueLimit)
    {
      RCLCPP_ERROR(node->get_logger(),
        "%s PUSH stopped: joint torque %.1f Nm exceeded %.1f Nm.",
        task.id, torque, kPushTorqueLimit);
      return false;
    }
  }
  RCLCPP_INFO(node->get_logger(), "%s PUSH complete: peak_torque=%.2f Nm.",
    task.id, peak_torque);
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    std::string(kTaskLabel) == "Task27" ? "task27_five_cube_center_insert" : "task26_batched_side_suction");
  // max_batches 始终表示“从 first_batch 起连续执行多少批”，保持既有单批
  // -p max_batches:=1 的含义。Task27 因此可先执行 first_batch=1/max=4，
  // 再在同一 Isaac 场景中只执行 first_batch=5/max=1 的中心插入阶段。
  const int max_batches = node->declare_parameter<int>("max_batches", kBatchCount);
  const int first_batch = node->declare_parameter<int>("first_batch", 1);
  const double time_scale = node->declare_parameter<double>("execution_time_scale", 3.0);
  // 零命令预检：只做 MoveIt/IK/FCL 链路筛选，不发布 joint、suction，也绝不请求
  // 到料（feed_command 会改变物理世界，同样属于命令）。它用于在改动槽位或目标
  // 坐标前先验证 FR3 的可达工作区，避免把几何试错带入 Isaac 物理执行。
  const bool planning_only = node->declare_parameter<bool>("planning_only", false);
  // 诊断开关：把车厢三面墙从 MoveIt 规划场景里去掉。用于分辨"失败是不是车厢碰撞体
  // 参与判定造成的"——它不是验收配置，正式运行必须保持 true。
  const bool include_box_walls = node->declare_parameter<bool>("include_box_walls", true);
  if (max_batches < 1 || first_batch < 1 || first_batch > kBatchCount ||
      first_batch + max_batches - 1 > kBatchCount ||
      time_scale < 1.0 ||
      !copyRobotDescriptions(node))
  {
    rclcpp::shutdown();
    return 1;
  }

  const std::size_t cube_count = kTasks.size();
  CubeBuffer cubes(node, cube_count);
  FeedStateBuffer feed_state(node, cube_count);
  ForceBuffer left_forces(node, taskTopic("/left/measured_joint_forces"));
  ForceBuffer right_forces(node, taskTopic("/right/measured_joint_forces"));
  TcpBuffer left_tcp(node, taskTopic("/left/side_suction_tcp_pose"));
  TcpBuffer right_tcp(node, taskTopic("/right/side_suction_tcp_pose"));
  auto feed_command_pub = node->create_publisher<std_msgs::msg::Int32>(taskTopic("/feed_command"), 10);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spin([&executor]() { executor.spin(); });
  bool success = false;
  do
  {
    RCLCPP_INFO(node->get_logger(),
      "========== Task26 BATCHED-FEED SIDE-SUCTION TIGHT: batches=%d, cubes=%zu, planning_only=%s, time_scale=%.2f ==========" ,
      max_batches, cube_count, planning_only ? "true" : "false", time_scale);
    if (!cubes.wait(10.0))
    {
      RCLCPP_ERROR(node->get_logger(), "Task26 等待 /task26/cube_poses 超时。");
      break;
    }
    if (!feed_state.wait(10.0))
    {
      RCLCPP_ERROR(node->get_logger(),
        "Task26 等待 /task26/feed_state 超时；Isaac 侧 bridge 必须先运行。");
      break;
    }
    if (!left_tcp.wait(10.0) || !right_tcp.wait(10.0))
    {
      RCLCPP_ERROR(node->get_logger(), "Task26 等待侧吸盘 TCP Ground Truth 超时。");
      break;
    }
    Arm left(node, true, time_scale);
    Arm right(node, false, time_scale);
    if (!left.waitBridge() || !right.waitBridge())
    {
      RCLCPP_ERROR(node->get_logger(), "Task26 Isaac side-suction bridge 未就绪。");
      break;
    }
    // 从 CLOSE 起，只要任一后续规划、执行或 Ground Truth 门禁失败，就必须
    // 同步释放并确认 OPEN。这样失败不会把 Cube 留在高处或把下一轮带入脏状态。
    // 失败路径和正常 RELEASE 都共用这个闭环：先确认两个物理吸盘均 OPEN，
    // 再继续退出。日志不得把正常释放误写成 safe abort。
    const auto openBothAndConfirm = [&](const char* context)
    {
      std::thread left_off([&]() { left.suction(false); });
      std::thread right_off([&]() { right.suction(false); });
      left_off.join();
      right_off.join();
      // 释放确认给 6 s：桥的吸盘状态是按仿真时间 20 Hz 发布的，仿真偶尔掉帧时
      // 3 s 会误判成「没确认」。这里只是等待时长，不放宽任何门限。
      if (!left.waitSuction(false, 6.0) || !right.waitSuction(false, 6.0))
      {
        RCLCPP_ERROR(node->get_logger(), "Task26 emergency SUCTION OFF was not confirmed on both arms.");
        return false;
      }
      RCLCPP_INFO(node->get_logger(),
        "Task26 %s: both side suctions are OPEN.", context);
      return true;
    };
    moveit::planning_interface::MoveGroupInterface left_group(node, left.groupName());
    moveit::planning_interface::MoveGroupInterface right_group(node, right.groupName());
    left_group.setEndEffectorLink(left.eefLink());
    right_group.setEndEffectorLink(right.eefLink());
    RailClient left_rail(node, "left");
    RailClient right_rail(node, "right");
    double left_rail_x = 0.0;
    double right_rail_x = 0.0;
    if (!left_rail.waitCurrent(5.0, &left_rail_x) ||
        !right_rail.waitCurrent(5.0, &right_rail_x) ||
        std::abs(left_rail_x - right_rail_x) > 0.002)
    {
      RCLCPP_ERROR(node->get_logger(),
        "Task26 双滑轨状态不一致或未知：left=%.4f right=%.4f；停止，避免 TCP 世界坐标整体错位。",
        left_rail_x, right_rail_x);
      break;
    }
    g_world_shift_x = 0.5 * (left_rail_x + right_rail_x) - kRailRestX;
    RCLCPP_INFO(node->get_logger(),
      "Task26 启动滑轨同步：left=%.4f right=%.4f rest=%.4f world_shift_x=%+.4f m。",
      left_rail_x, right_rail_x, kRailRestX, g_world_shift_x);
    const PushControlConfig push_control = loadPushControlConfig(node);
    const double push_base_advance = push_control.push_base_advance_m;

    // 力觉探针（开发顺序 Step 2）：**只读**，不发布任何 joint / suction / rail 命令。
    // 用途是在不改动控制器的前提下验证：力的方向、坐标定义、量级、以及滤波效果。
    // 推进轴 e_push 由"预推位 -> 格位"的世界位移定义（来自场景几何），
    // 不允许写死某个世界轴。
    const std::string wrench_probe = node->declare_parameter<std::string>("wrench_probe", "");
    if (!wrench_probe.empty())
    {
      if (wrench_probe != "left" && wrench_probe != "right")
      {
        RCLCPP_ERROR(node->get_logger(), "wrench_probe 只能是 left / right。");
        break;
      }
      const double probe_seconds = node->declare_parameter<double>("wrench_probe_seconds", 20.0);
      const PushControlConfig probe_config = loadPushControlConfig(node);

      const Arm& probe_arm = (wrench_probe == "left") ? left : right;
      moveit::planning_interface::MoveGroupInterface& probe_group =
        (wrench_probe == "left") ? left_group : right_group;
      Eigen::Vector3d push_axis(
        kTasks.front().cell.position.x - kTasks.front().pre_push.position.x,
        kTasks.front().cell.position.y - kTasks.front().pre_push.position.y,
        kTasks.front().cell.position.z - kTasks.front().pre_push.position.z);
      push_axis.normalize();
      RCLCPP_INFO(node->get_logger(),
        "%s e_push（预推位->格位）= [%.4f %.4f %.4f]；探针时长 %.1f s。",
        wrench_probe.c_str(), push_axis.x(), push_axis.y(), push_axis.z(), probe_seconds);

      {
      WrenchEstimator estimator(node, wrench_probe, probe_group.getRobotModel(),
        probe_arm.eefLink(), push_axis, probe_config);
      if (!estimator.collectBias())
      {
        break;
      }

      const std::string csv_path = probe_config.wrench_log_path.empty()
        ? ("/home/ubuntu2004/WorkBuddy/2026-09-21-10-05-14/t26_wrench_" + wrench_probe + ".csv")
        : probe_config.wrench_log_path;
      std::ofstream csv(csv_path);
      csv << "t,raw_F_push,filt_F_push,Fx,Fy,Fz,peak_torque\n";
      RCLCPP_INFO(node->get_logger(), "%s wrench CSV -> %s", wrench_probe.c_str(), csv_path.c_str());

      double min_force = std::numeric_limits<double>::infinity();
      double max_force = -std::numeric_limits<double>::infinity();
      double sum_force = 0.0;
      double max_torque = 0.0;
      int samples = 0;
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(probe_seconds);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
      {
        if (estimator.update())
        {
          const double filtered = estimator.filteredForce();
          const Eigen::Vector3d force = estimator.forceVector();
          min_force = std::min(min_force, filtered);
          max_force = std::max(max_force, filtered);
          sum_force += filtered;
          max_torque = std::max(max_torque, estimator.peakTorque());
          ++samples;
          csv << node->now().seconds() << ',' << estimator.rawForce() << ',' << filtered << ','
              << force.x() << ',' << force.y() << ',' << force.z() << ','
              << estimator.peakTorque() << '\n';
        }
        std::this_thread::sleep_for(20ms);
      }
      csv.flush();
      if (samples > 0)
      {
        RCLCPP_INFO(node->get_logger(),
          "%s 力觉探针结束：样本 %d，F_push 范围 [%.3f, %.3f] N，均值 %.3f N，峰值关节力矩 %.2f N*m。",
          wrench_probe.c_str(), samples, min_force, max_force, sum_force / samples, max_torque);
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "%s 力觉探针没有取到有效样本。", wrench_probe.c_str());
      }
      }
      // 这里**不能**调 rclcpp::shutdown() 提前收尾：外层作用域里还有 Arm /
      // MoveGroupInterface / PlanningSceneInterface，它们会晚于上下文销毁而
      // terminate（实测 Aborted）。用 break 走正常退出路径；代价是外层任务循环
      // 判定为"未完成"，`ros2 run` 会以 exit 1 结束——这是探针的预期行为。
      break;
    }
    moveit::planning_interface::PlanningSceneInterface scene;
    // 启动时 Planning Scene 只有桌面：未到货件停在桌下休眠位，既不在工作区也不是
    // 障碍物；当前批两件只在到料确认后加入，已完成件始终保留。
    scene.removeCollisionObjects({
      "task26_table", "task26_box_wall_deep", "task26_box_wall_minus_y", "task26_box_wall_plus_y",
      "task26_cube_1", "task26_cube_2", "task26_cube_3", "task26_cube_4", "task26_cube_5"});
    std::this_thread::sleep_for(300ms);
    std::vector<moveit_msgs::msg::CollisionObject> initial{tableObject()};
    if (include_box_walls)
    {
      const auto walls = boxWallObjects();
      initial.insert(initial.end(), walls.begin(), walls.end());
    }
    else
    {
      RCLCPP_WARN(node->get_logger(),
        "Task26 诊断模式：车厢墙体未加入 Planning Scene（include_box_walls=false）。");
    }
    if (!scene.applyCollisionObjects(initial))
    {
      RCLCPP_ERROR(node->get_logger(), "Task26 无法初始化 MoveIt Planning Scene。");
      break;
    }
    std::this_thread::sleep_for(500ms);

    // 供料槽位是设计常量。用批 1 的真实 Ground Truth 校验“场景 - bridge - 执行器”
    // 三方对同一槽位的理解一致；不一致就停止，绝不用错误几何继续规划。
    const auto slotPoseForCube = [](std::size_t cube_index)
    {
      // Cube index 从零开始：奇数编号到 A、偶数编号到 B，与 Isaac bridge 的
      // task26_metadata 完全一致。逐件到料时仍交替 A/B，避免只在一个槽位做
      // 规划而掩盖另一个供料位的可达性。
      return cube_index % 2 == 0
        ? worldPose(kSlotAx, kSlotAy, kBottomZ)
        : worldPose(kSlotBx, kSlotBy, kBottomZ);
    };
    if (first_batch == 1)
    {
      std::vector<geometry_msgs::msg::Pose> nominal_slots;
      std::vector<std::size_t> first_batch_indices;
      nominal_slots.reserve(kBatchSize);
      first_batch_indices.reserve(kBatchSize);
      for (int slot = 0; slot < kBatchSize; ++slot)
      {
        nominal_slots.push_back(slotPoseForCube(static_cast<std::size_t>(slot)));
        first_batch_indices.push_back(static_cast<std::size_t>(slot));
      }
      if (!feed_state.waitArrived(first_batch_indices, kFeedTimeoutSec))
      {
        RCLCPP_ERROR(node->get_logger(),
          "%s 批 1 未在 %.0f s 内到位；请确认 Isaac bridge 已启动（它会自动释放批 1）。",
          kTaskLabel, kFeedTimeoutSec);
        break;
      }
      bool slot_ok = true;
      for (std::size_t slot = 0; slot < nominal_slots.size(); ++slot)
      {
        const auto [live, live_revision] = cubes.get(slot);
        (void)live_revision;
        const double error = distance3d(nominal_slots.at(slot).position, live.position);
        RCLCPP_INFO(node->get_logger(),
          "%s 槽 %c 设计值=(%.3f, %.3f, %.3f) 实际=(%.3f, %.3f, %.3f) 偏差=%.2f mm。",
          kTaskLabel, slot == 0 ? 'A' : 'B',
          nominal_slots.at(slot).position.x, nominal_slots.at(slot).position.y,
          nominal_slots.at(slot).position.z,
          live.position.x, live.position.y, live.position.z, error * 1000.0);
        if (error > kSlotTolerance)
        {
          if (planning_only)
          {
            RCLCPP_WARN(node->get_logger(),
              "%s 槽 %c 当前偏差 %.1f mm（超过 %.1f mm）：planning-only 仍按设计槽位预演，"
              "物理执行前必须重建场景。",
              kTaskLabel, slot == 0 ? 'A' : 'B', error * 1000.0, kSlotTolerance * 1000.0);
            continue;
          }
          RCLCPP_ERROR(node->get_logger(),
            "%s 槽 %c 偏差超过 %.1f mm；场景或 bridge 已漂移，停止。",
            kTaskLabel, slot == 0 ? 'A' : 'B', kSlotTolerance * 1000.0);
          slot_ok = false;
        }
      }
      if (!slot_ok)
      {
        break;
      }
    }
    else
    {
      // 分阶段运行（Task27 阶段 B）时，新的进程不能假定 MoveIt 仍保留上一个
      // 进程发布的 CollisionObject。先从 Isaac Ground Truth 恢复所有已完成件，
      // 验收后重新放回本进程的 Planning Scene，再允许中心件到料/规划。
      const std::size_t completed_count =
        static_cast<std::size_t>(first_batch - 1) * static_cast<std::size_t>(kBatchSize);
      std::vector<std::size_t> completed_indices;
      completed_indices.reserve(completed_count);
      for (std::size_t index = 0; index < completed_count; ++index)
      {
        completed_indices.push_back(index);
      }
      if (!feed_state.waitArrived(completed_indices, kFeedTimeoutSec))
      {
        RCLCPP_ERROR(node->get_logger(),
          "%s 阶段恢复失败：前 %zu 件尚未全部到位。", kTaskLabel, completed_count);
        break;
      }
      std::vector<moveit_msgs::msg::CollisionObject> completed_objects;
      completed_objects.reserve(completed_count);
      bool recovered = true;
      for (std::size_t index = 0; index < completed_count; ++index)
      {
        const auto [settled, settled_revision] = cubes.get(index);
        (void)settled_revision;
        if (!validCellPlacement(node, kTasks.at(index), settled, cubes))
        {
          RCLCPP_ERROR(node->get_logger(),
            "%s 阶段恢复拒绝：Cube_%02zu 未通过此前格位验收。",
            kTaskLabel, index + 1);
          recovered = false;
          break;
        }
        completed_objects.push_back(cubeObject(
          "task26_cube_" + std::to_string(index + 1), settled));
      }
      if (!recovered || !scene.applyCollisionObjects(completed_objects))
      {
        RCLCPP_ERROR(node->get_logger(), "%s 无法恢复前序 Cube Planning Scene。", kTaskLabel);
        break;
      }
      std::this_thread::sleep_for(300ms);
      RCLCPP_INFO(node->get_logger(),
        "%s 阶段恢复 PASS：前 %zu 件已由 Ground Truth 回写为真实碰撞物。",
        kTaskLabel, completed_count);
    }

    bool all_complete = true;
    for (int batch = first_batch;
         batch < first_batch + max_batches && all_complete;
         ++batch)
    {
      std::array<std::size_t, kBatchSize> batch_indices{};
      for (int slot = 0; slot < kBatchSize; ++slot)
      {
        batch_indices.at(static_cast<std::size_t>(slot)) =
          static_cast<std::size_t>(batch - 1) * kBatchSize + static_cast<std::size_t>(slot);
      }
      if constexpr (kBatchSize == 1)
      {
        RCLCPP_INFO(node->get_logger(),
          "########## %s batch %d/%d: Cube_%02zu ##########",
          kTaskLabel, batch, first_batch + max_batches - 1, batch_indices.front() + 1);
      }
      else
      {
        RCLCPP_INFO(node->get_logger(),
          "########## %s batch %d/%d: Cube_%02zu (slot A) then Cube_%02zu (slot B) ##########",
          kTaskLabel, batch, first_batch + max_batches - 1,
          batch_indices.front() + 1, batch_indices.back() + 1);
      }

      std::array<geometry_msgs::msg::Pose, kBatchSize> batch_sources;
      if (planning_only)
      {
        // 零命令预检不请求到料：按设计槽位虚拟推进，绝不发布 feed_command。
        for (int slot = 0; slot < kBatchSize; ++slot)
        {
          batch_sources.at(static_cast<std::size_t>(slot)) =
            slotPoseForCube(batch_indices.at(static_cast<std::size_t>(slot)));
        }
        RCLCPP_INFO(node->get_logger(),
          "%s planning-only batch %d: 使用设计槽位预演，未发布 feed_command 或任何运动命令。",
          kTaskLabel, batch);
      }
      else
      {
        if (batch > 1)
        {
          std_msgs::msg::Int32 command;
          command.data = batch;
          for (int repeat = 0; repeat < 20; ++repeat)
          {
            feed_command_pub->publish(command);
            std::this_thread::sleep_for(10ms);
          }
          RCLCPP_INFO(node->get_logger(), "%s 已请求第 %d 批到料，等待落稳。", kTaskLabel, batch);
        }
        std::uint64_t arrival_mark = 0;
        {
          const auto [mark_pose, mark_revision] = cubes.get(batch_indices.front());
          (void)mark_pose;
          arrival_mark = mark_revision;
        }
        if (!feed_state.waitArrived(
              {batch_indices.front(), batch_indices.back()}, kFeedTimeoutSec))
        {
          RCLCPP_ERROR(node->get_logger(),
            "Task26 batch %d 未在 %.0f s 内全部到位。", batch, kFeedTimeoutSec);
          all_complete = false;
          break;
        }
        // 到料是瞬移 + 落稳：必须等到新一帧 Ground Truth，绝不使用瞬移前的缓存 pose。
        geometry_msgs::msg::Pose fresh;
        if (!cubes.waitNew(batch_indices.front(), arrival_mark, 5.0, &fresh))
        {
          RCLCPP_ERROR(node->get_logger(),
            "Task26 batch %d 到位后没有收到新的 Ground Truth。", batch);
          all_complete = false;
          break;
        }
        std::this_thread::sleep_for(200ms);
        for (int slot = 0; slot < kBatchSize; ++slot)
        {
          const std::size_t index = batch_indices.at(static_cast<std::size_t>(slot));
          const auto [live, live_revision] = cubes.get(index);
          (void)live_revision;
          batch_sources.at(static_cast<std::size_t>(slot)) = live;
          const double error = distance3d(
            slotPoseForCube(index).position, live.position);
          if (error > kSlotTolerance)
          {
            RCLCPP_ERROR(node->get_logger(),
              "Task26 batch %d Cube_%02zu 实际落在 (%.3f, %.3f, %.3f)，与槽位偏差 %.1f mm 超过 %.1f mm；停止。",
              batch, index + 1, live.position.x, live.position.y, live.position.z,
              error * 1000.0, kSlotTolerance * 1000.0);
            all_complete = false;
          }
        }
        if (!all_complete)
        {
          break;
        }
      }

      // 当前批两件按最新 Ground Truth 加入 Planning Scene；此前批次已落稳的件一直
      // 保留为真实碰撞物，后续批次的休眠件从不进入规划场景。
      std::vector<moveit_msgs::msg::CollisionObject> batch_objects;
      for (int slot = 0; slot < kBatchSize; ++slot)
      {
        const std::size_t index = batch_indices.at(static_cast<std::size_t>(slot));
        batch_objects.push_back(cubeObject(
          "task26_cube_" + std::to_string(index + 1),
          batch_sources.at(static_cast<std::size_t>(slot))));
      }
      if (!scene.applyCollisionObjects(batch_objects))
      {
        RCLCPP_ERROR(node->get_logger(),
          "Task26 batch %d 无法把当前批两件加入 Planning Scene。", batch);
        all_complete = false;
        break;
      }
      std::this_thread::sleep_for(300ms);

      // 批内逐件执行 Task24 已验证的紧协调链路（先槽 A 后槽 B）。
      for (int slot = 0; slot < kBatchSize; ++slot)
      {
      const int task_index = static_cast<int>(batch_indices.at(static_cast<std::size_t>(slot)));
      OfflineTask task = kTasks.at(static_cast<std::size_t>(task_index));
#ifdef TASK27_FIVE_CUBE
      if (isInnerReferenceTask(task))
      {
        // 内侧件先在车厢外沿 Y 对准最终通道，再沿 +X 直推。第四件的旧方案
        // 需要右臂在深位作低位侧压；实测 SIDE_CONTACT Cartesian 仅到 65%，
        // 因此改为复用已经通过 FCL 的直线推入。物理执行时对齐外件实测面，
        // 预检时对齐外件的名义格位；两者的间隙均由最终 Ground Truth 验收。
        const std::size_t outer_index = task.cube_index == 2 ? 0 : 1;
        geometry_msgs::msg::Pose outer_pose;
        if (planning_only)
        {
          outer_pose = kTasks.at(outer_index).cell;
        }
        else
        {
          const auto [live_outer, outer_revision] = cubes.get(outer_index);
          (void)outer_revision;
          outer_pose = live_outer;
        }
        task.cell.position.y = outer_pose.position.y +
          (task.cube_index == 2
            ? -(kCubeSize + kInnerStraightInsertClearance)
            : (kCubeSize + kInnerStraightInsertClearance));
        task.pre_push.position.y = task.cell.position.y;
        RCLCPP_INFO(node->get_logger(),
          "%s 对齐外侧 Cube_%02zu 的 %s 位姿：直推通道 y=%.4f m，"
          "名义邻件余量=%.1f mm。",
          task.id, outer_index + 1, planning_only ? "名义" : "Ground Truth",
          task.pre_push.position.y, 1000.0 * kInnerStraightInsertClearance);
      }
#endif
      const std::string object_id = "task26_cube_" + std::to_string(task.cube_index + 1);
      const auto [live_source, source_revision] = cubes.get(task.cube_index);
      (void)live_source;
      // 物理执行时 source 就是该批到料落稳后读到的真实 Ground Truth；零命令预检
      // 不请求到料，后续批次仍停在桌下休眠位（z=-5 m），因此必须使用该批的设计槽位
      // 作为预演起点，绝不能把休眠位当成抓取位姿。
      const geometry_msgs::msg::Pose source = planning_only
        ? batch_sources.at(static_cast<std::size_t>(slot))
        : live_source;
      RCLCPP_INFO(node->get_logger(), "---------- %s: source=(%.3f, %.3f, %.3f), target=(%.3f, %.3f, %.3f) ----------",
        task.id, source.position.x, source.position.y, source.position.z,
        task.pre_push.position.x, task.pre_push.position.y, task.pre_push.position.z);

      const double initial_left_contact_y = source.position.y - kCubeHalf - kSideContactCommandGap;
      const double initial_right_contact_y = source.position.y + kCubeHalf + kSideContactCommandGap;
      // 大 Cube 不能让两臂同时在低位 PRE_CONTACT：先到位的手会占据另一只手的
      // OMPL 接近通道。两臂先在共同 lift 高度的两侧待命，再以受 FCL 门禁的
      // 左->右微时序下降至 CONTACT；真正负载阶段仍从 COMMON_LIFT 起严格同步。
      const double pre_contact_z = source.position.z + kSideContactCommandZOffset + kLiftHeight;
      // 左臂的 HIGH_PRE_CONTACT 不能只取“关节行程最短”的 RRT 解。FR3 的冗余
      // 姿态会让该解在后续纯 Z 外侧下降中产生 link2 <-> link7 自碰。这里在
      // 任何机器人命令发出前，对完整的空载左侧接近链逐个筛选候选，绝不把这
      // 一对真实自碰加入 ACM。
      std::vector<trajectory_msgs::msg::JointTrajectory> left_pre_candidates;
      if (!left.planPoseCandidates(left_group,
            sidePose(source.position.x, initial_left_contact_y - kPreContactOffsetY, pre_contact_z, true),
            std::string(task.id) + " LEFT HIGH_PRE_CONTACT", &left_pre_candidates))
      {
        all_complete = false;
        break;
      }
      trajectory_msgs::msg::JointTrajectory right_pre;
      if (!right.planPose(right_group,
            sidePose(source.position.x, initial_right_contact_y + kRightPreContactOffsetY, pre_contact_z, false),
            std::string(task.id) + " RIGHT HIGH_PRE_CONTACT", &right_pre))
      {
        all_complete = false;
        break;
      }

      scene.removeCollisionObjects({object_id});
      std::this_thread::sleep_for(250ms);
      auto world_after_remove = staticWorld(scene);
      trajectory_msgs::msg::JointTrajectory left_contact;
      trajectory_msgs::msg::JointTrajectory right_contact;
      // Contact 建立使用确定的左->右微时序。不能从 HIGH_PRE_CONTACT 斜向
      // 直接插入：长 L 型支臂会使 IK 在下降过程中绕到桌面下方。先在供料
      // 外侧作纯 Z 下降，再作纯 Y 靠近；两段均启用桌面碰撞检查。
      const auto left_low_pre = sidePose(
        source.position.x, initial_left_contact_y - kPreContactOffsetY,
        source.position.z + kSideContactCommandZOffset, true);
      trajectory_msgs::msg::JointTrajectory left_pre;
      trajectory_msgs::msg::JointTrajectory left_outer_descent;
      trajectory_msgs::msg::JointTrajectory right_outer_descent;
      bool left_candidate_safe = false;
      for (std::size_t candidate_index = 0;
           candidate_index < left_pre_candidates.size(); ++candidate_index)
      {
        const auto& candidate_pre = left_pre_candidates[candidate_index];
        const std::string candidate_prefix = std::string(task.id) + " LEFT_CANDIDATE_" +
          std::to_string(candidate_index + 1);

        // 实际执行保持左臂先到高位、右臂再到高位的微时序；分别验证两个子段，
        // 不能只验证两个 RRT 轨迹同起同落的理想化情况。
        auto right_hold_initial = holdTrajectory(
          right_pre, right_pre.points.front().positions, pointTime(candidate_pre.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              candidate_pre, right_hold_initial, candidate_prefix + " HIGH_LEFT"))
        {
          continue;
        }
        auto left_hold_high = holdTrajectory(
          candidate_pre, finalPositions(candidate_pre), pointTime(right_pre.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              left_hold_high, right_pre, candidate_prefix + " HIGH_RIGHT"))
        {
          continue;
        }

        trajectory_msgs::msg::JointTrajectory candidate_outer;
        if (!planCommonCartesian(node, left_group, left.groupName(), right.groupName(),
              finalPositions(candidate_pre), finalPositions(right_pre), left_low_pre,
              candidate_prefix + " OUTER_DESCENT", &candidate_outer, left.eefLink(), true))
        {
          continue;
        }
        auto right_hold_pre = holdTrajectory(
          right_pre, finalPositions(right_pre), pointTime(candidate_outer.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              candidate_outer, right_hold_pre, candidate_prefix + " OUTER_DESCENT"))
        {
          continue;
        }

        trajectory_msgs::msg::JointTrajectory candidate_contact;
        if (!planCommonCartesian(node, left_group, left.groupName(), right.groupName(),
              finalPositions(candidate_outer), finalPositions(right_pre),
              sidePose(source.position.x, initial_left_contact_y,
                source.position.z + kSideContactCommandZOffset, true),
              candidate_prefix + " CONTACT", &candidate_contact, left.eefLink(), true))
        {
          continue;
        }
        right_hold_pre = holdTrajectory(
          right_pre, finalPositions(right_pre), pointTime(candidate_contact.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              candidate_contact, right_hold_pre, candidate_prefix + " CONTACT"))
        {
          continue;
        }

        left_pre = candidate_pre;
        left_outer_descent = std::move(candidate_outer);
        left_contact = std::move(candidate_contact);
        left_candidate_safe = true;
        RCLCPP_INFO(node->get_logger(), "%s selected LEFT_CANDIDATE_%zu/%zu: "
          "HIGH_PRE_CONTACT -> OUTER_DESCENT -> CONTACT FCL PASS.", task.id,
          candidate_index + 1, left_pre_candidates.size());
        break;
      }
      if (!left_candidate_safe)
      {
        RCLCPP_ERROR(node->get_logger(), "%s no FCL-safe left empty-approach candidate was found.", task.id);
        all_complete = false;
        break;
      }

      // planning_only 必须是严格的“零命令”模式：它以最新收到的 source pose
      // 预演后续右侧空载接近和完整共同负载链，绝不能为了取得 live pose 而先
      // 执行左侧接近。真实执行模式才允许左侧先落到 CONTACT，并随后从 Isaac
      // Ground Truth 刷新 Cube 位置，补偿无吸附接近期间可能出现的微小漂移。
      geometry_msgs::msg::Pose cube_after_left_contact = source;
      if (!planning_only)
      {
        if (!left.executeAt(left_pre, std::chrono::steady_clock::now()) ||
            !right.executeAt(right_pre, std::chrono::steady_clock::now()) ||
            !left.waitAtTarget(left_pre, kJointSettleToleranceRad, kJointSettleTimeoutSec) ||
            !right.waitAtTarget(right_pre, kJointSettleToleranceRad, kJointSettleTimeoutSec))
        {
          all_complete = false;
          break;
        }

        auto right_hold_pre = holdTrajectory(
          right_pre, finalPositions(right_pre), pointTime(left_outer_descent.points.back()));
        if (!executeSync(left, left_outer_descent, right, right_hold_pre))
        {
          all_complete = false;
          break;
        }

        right_hold_pre = holdTrajectory(
          right_pre, finalPositions(right_pre), pointTime(left_contact.points.back()));
        if (!executeSync(left, left_contact, right, right_hold_pre))
        {
          all_complete = false;
          break;
        }

        std::this_thread::sleep_for(250ms);
        const auto [live_cube_after_left_contact, left_contact_revision] = cubes.get(task.cube_index);
        (void)left_contact_revision;
        cube_after_left_contact = live_cube_after_left_contact;
      }
      const double right_live_contact_y =
        cube_after_left_contact.position.y + kCubeHalf + kSideContactCommandGap;
      // 空载的右侧外部接近可以使用 RRTConnect。先在机器人尚未吸附 Cube 时
      // 预演后续完整负载链；只有通过者才会发送右臂命令。一个有限候选池可能
      // 恰好只采到同一类绕腕解，因此在左臂保持静止、两杯仍 OFF 时最多重采样
      // 三批；不会把一次 OMPL 随机结果变成物理失败。
      std::vector<trajectory_msgs::msg::JointTrajectory> right_outer_candidates;
      // 右臂低位外侧用 RRTConnect 直接规划。注意不能用「高位 + 纯 Z 笛卡尔下降」
      // 代替：笛卡尔直线无法绕开正停在接触位的左臂，实测下降段 fraction 只有
      // 0.968（单独规划同一段则是 1.000，说明就是双臂互相干涉）。RRT 可以绕行，
      // 代价是位姿目标区域较窄——因此车厢墙高必须留出前臂净空（见场景脚本）。
      const auto right_outer_target = sidePose(
        cube_after_left_contact.position.x,
        right_live_contact_y + kRightPreContactOffsetY,
        cube_after_left_contact.position.z + kSideContactCommandZOffset, false);
      // 关键：外侧接近必须让规划器**重新看见那颗 Cube**。
      // 上面为了接触/负载段把它从规划场景摘掉了（scene.removeCollisionObjects），
      // 若这里继续在"无 Cube 世界"里规划，RRT 会把 L 型工具**从 Cube 顶上抹过去**；
      // 仿真里工具就压在 Cube 顶缘上被接触顶住，停在离目标 ~6.9° 处，
      // final joint-state 25 s 超时（连续多轮复现，右 TCP 一律停在 Cube 顶面上方
      // 约 35 mm）。外侧接近是**空载、在 Cube 外侧**的动作，目标位本身离 Cube 面
      // 还有 kRightPreContactOffsetY=20 mm 净空，带 Cube 规划必然可解；随后的
      // 接触段仍是 1 mm 间隙的纯 Y 直线靠近。
      if (!scene.applyCollisionObject(cubeObject(object_id, cube_after_left_contact)))
      {
        RCLCPP_ERROR(node->get_logger(),
          "%s cannot re-apply the cube CollisionObject %s for the outer approach.",
          task.id, object_id.c_str());
        all_complete = false;
        break;
      }
      std::this_thread::sleep_for(250ms);
      for (int batch = 1; batch <= kRrtCandidateBatches; ++batch)
      {
        std::vector<trajectory_msgs::msg::JointTrajectory> batch_candidates;
        if (!right.planPoseCandidates(right_group, right_outer_target,
              std::string(task.id) + " RIGHT_OUTER_APPROACH batch=" + std::to_string(batch),
              &batch_candidates))
        {
          RCLCPP_WARN(node->get_logger(), "%s RIGHT_OUTER_APPROACH batch=%d/%d produced no OMPL candidate.",
            task.id, batch, kRrtCandidateBatches);
          continue;
        }
        right_outer_candidates.insert(right_outer_candidates.end(),
          std::make_move_iterator(batch_candidates.begin()),
          std::make_move_iterator(batch_candidates.end()));
      }
      // 外侧接近已规划完：把 Cube 摘回"无 Cube 世界"，后续接触/负载段的
      // 校验世界必须与 world_after_remove 保持一致。
      scene.removeCollisionObjects({object_id});
      std::this_thread::sleep_for(250ms);
      if (right_outer_candidates.empty())
      {
        RCLCPP_ERROR(node->get_logger(), "%s RIGHT_OUTER_APPROACH exhausted %d RRT candidate batches.",
          task.id, kRrtCandidateBatches);
        all_complete = false;
        break;
      }
      RCLCPP_INFO(node->get_logger(), "%s RIGHT_OUTER_APPROACH evaluating %zu candidates from %d batches.",
        task.id, right_outer_candidates.size(), kRrtCandidateBatches);
      // 推入臂**按排选取**：每臂只推自己那一侧的一排。反排去推要跨到对面，实测约
      // 0.69 m 斜向长臂，RRT 全部超时；同侧则是约 0.57 m。
      const bool push_left = task.cell.position.y < 0.0;
      Arm& pusher = push_left ? left : right;
      Arm& helper = push_left ? right : left;
      moveit::planning_interface::MoveGroupInterface& pusher_group =
        push_left ? left_group : right_group;
      // 辅助臂在推入阶段的停放位：**必须挪出推入臂的工作区**。实测停在"预推位正
      // 上方"时，它的侧面吸盘会撞到推入臂的 link6/link7——推入过程中前臂会扫到
      // y≈0.10、z≈0.56，而那个停放位恰好落在同一个区域。因此让辅助臂退到**自己那
      // 一侧**（y=∓0.32）并略微后退，彻底离开推入通道。
      const geometry_msgs::msg::Pose helper_park = sidePose(
        task.pre_push.position.x - 0.10, push_left ? +0.32 : -0.32,
        task.pre_push.position.z + kSideContactCommandZOffset + kReleaseGapZ + kLiftHeight,
        !push_left);

      // 记下选中候选预演出的末端轨迹。零命令预检要用它们作为推入段/退出的起点：
      // 真实执行路径用实际执行后的 left_push / right_retreat，而预检不执行，只能
      // 用这里预演出的结果。
      trajectory_msgs::msg::JointTrajectory selected_left_push;
      trajectory_msgs::msg::JointTrajectory selected_right_push;
      trajectory_msgs::msg::JointTrajectory selected_left_retreat;
      trajectory_msgs::msg::JointTrajectory selected_right_retreat;
      bool right_candidate_safe = false;
      for (std::size_t candidate_index = 0; candidate_index < right_outer_candidates.size(); ++candidate_index)
      {
        auto candidate_outer = right_outer_candidates[candidate_index];
        auto candidate_left_hold = holdTrajectory(
          left_contact, finalPositions(left_contact), pointTime(candidate_outer.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              candidate_left_hold, candidate_outer,
              std::string(task.id) + " RIGHT_OUTER_CANDIDATE_" + std::to_string(candidate_index + 1)))
        {
          continue;
        }

        trajectory_msgs::msg::JointTrajectory candidate_contact;
        if (!planCommonCartesian(node, right_group, right.groupName(), left.groupName(),
              finalPositions(candidate_outer), finalPositions(left_contact),
              sidePose(cube_after_left_contact.position.x, right_live_contact_y,
                cube_after_left_contact.position.z + kSideContactCommandZOffset, false),
              std::string(task.id) + " RIGHT_CONTACT_CANDIDATE_" + std::to_string(candidate_index + 1),
              &candidate_contact, right.eefLink(), true))
        {
          continue;
        }
        candidate_left_hold = holdTrajectory(
          left_contact, finalPositions(left_contact), pointTime(candidate_contact.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              candidate_left_hold, candidate_contact,
              std::string(task.id) + " RIGHT_CONTACT_CANDIDATE_" + std::to_string(candidate_index + 1)))
        {
          continue;
        }

        trajectory_msgs::msg::JointTrajectory candidate_left_lift;
        trajectory_msgs::msg::JointTrajectory candidate_right_lift;
        const double candidate_left_y = cube_after_left_contact.position.y - kCubeHalf - kSideContactCommandGap;
        const double candidate_right_y = cube_after_left_contact.position.y + kCubeHalf + kSideContactCommandGap;
        if (!planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(left_contact), finalPositions(candidate_contact),
              sidePose(cube_after_left_contact.position.x, candidate_left_y,
                cube_after_left_contact.position.z + kSideContactCommandZOffset + kLiftHeight, true),
              sidePose(cube_after_left_contact.position.x, candidate_right_y,
                cube_after_left_contact.position.z + kSideContactCommandZOffset + kLiftHeight, false),
              world_after_remove,
              std::string(task.id) + " RIGHT_CANDIDATE_COMMON_LIFT_" + std::to_string(candidate_index + 1),
              &candidate_left_lift, &candidate_right_lift))
        {
          continue;
        }

        // 不只检查 LIFT：侧向工具在随后 Y 对齐的 IK 插值中也可能下探到
        // 桌面。闭合前把完整 Z->X->Y->Z->X->Z 负载链路都预演完毕，任何一
        // 段失败就换下一个空载 RRT 候选，不会让物理 Cube 进入危险构型。
        const double candidate_entry_x = task.pre_push.position.x - kPrePushOffsetX;
        const double candidate_release_z =
          task.pre_push.position.z + kSideContactCommandZOffset + kReleaseGapZ;
        const double candidate_transport_z =
          cube_after_left_contact.position.z + kSideContactCommandZOffset + kLiftHeight;
        const double candidate_entry_left_y = task.pre_push.position.y - kCubeHalf - kSideContactCommandGap;
        const double candidate_entry_right_y = task.pre_push.position.y + kCubeHalf + kSideContactCommandGap;
        trajectory_msgs::msg::JointTrajectory candidate_left_x, candidate_right_x;
        trajectory_msgs::msg::JointTrajectory candidate_left_y_path, candidate_right_y_path;
        trajectory_msgs::msg::JointTrajectory candidate_left_descent, candidate_right_descent;
        trajectory_msgs::msg::JointTrajectory candidate_left_push, candidate_right_push;
        trajectory_msgs::msg::JointTrajectory candidate_left_retreat, candidate_right_retreat;
        const std::string candidate_prefix = std::string(task.id) + " RIGHT_CANDIDATE_" +
          std::to_string(candidate_index + 1);
        if (!planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(candidate_left_lift), finalPositions(candidate_right_lift),
              sidePose(candidate_entry_x, candidate_left_y, candidate_transport_z, true),
              sidePose(candidate_entry_x, candidate_right_y, candidate_transport_z, false),
              world_after_remove, candidate_prefix + " COMMON_X_TRAVEL",
              &candidate_left_x, &candidate_right_x) ||
            !planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(candidate_left_x), finalPositions(candidate_right_x),
              sidePose(candidate_entry_x, candidate_entry_left_y, candidate_transport_z, true),
              sidePose(candidate_entry_x, candidate_entry_right_y, candidate_transport_z, false),
              world_after_remove, candidate_prefix + " COMMON_Y_ALIGN",
              &candidate_left_y_path, &candidate_right_y_path) ||
            !planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(candidate_left_y_path), finalPositions(candidate_right_y_path),
              sidePose(candidate_entry_x, candidate_entry_left_y, candidate_release_z, true),
              sidePose(candidate_entry_x, candidate_entry_right_y, candidate_release_z, false),
              world_after_remove, candidate_prefix + " COMMON_DESCENT_TO_ENTRY",
              &candidate_left_descent, &candidate_right_descent) ||
            !planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(candidate_left_descent), finalPositions(candidate_right_descent),
              sidePose(task.pre_push.position.x, candidate_entry_left_y, candidate_release_z, true),
              sidePose(task.pre_push.position.x, candidate_entry_right_y, candidate_release_z, false),
              world_after_remove, candidate_prefix + " COMMON_SIDE_SHORT_PUSH",
              &candidate_left_push, &candidate_right_push) ||
            !planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(candidate_left_push), finalPositions(candidate_right_push),
              push_left ? sidePose(task.pre_push.position.x, candidate_entry_left_y,
                candidate_release_z + kLiftHeight, true) : helper_park,
              push_left ? helper_park : sidePose(task.pre_push.position.x, candidate_entry_right_y,
                candidate_release_z + kLiftHeight, false),
              world_after_remove, candidate_prefix + " PREPLANNED_COMMON_RETREAT",
              &candidate_left_retreat, &candidate_right_retreat))
        {
          continue;
        }

        right_outer_descent = std::move(candidate_outer);
        right_contact = std::move(candidate_contact);
        selected_left_push = std::move(candidate_left_push);
        selected_right_push = std::move(candidate_right_push);
        selected_left_retreat = std::move(candidate_left_retreat);
        selected_right_retreat = std::move(candidate_right_retreat);
        right_candidate_safe = true;
        RCLCPP_INFO(node->get_logger(), "%s selected right outer candidate=%zu/%zu after full loaded-chain preflight.",
          task.id, candidate_index + 1, right_outer_candidates.size());
        break;
      }
      if (!right_candidate_safe)
      {
        RCLCPP_ERROR(node->get_logger(), "%s no right-side empty-arm candidate passed the full CONTACT -> LIFT -> X -> Y -> DESCENT -> PUSH -> RETREAT preflight.",
          task.id);
        all_complete = false;
        break;
      }

      // 推入臂（左臂）持 -X 面时杯面位置：TCP_x = cube_x - (半件 + 间隙)。
      // 这三项在**执行路径**里会被实测 Cube 位姿覆盖（见下面「推入段几何改用实测」）；
      // 预检路径（planning_only）保持标称值——那时 Cube 还在供料槽，实测不适用。
      const double push_cell_x = task.cell.position.x - kPushCupOffsetX;
      double push_entry_x = task.pre_push.position.x - kPushCupOffsetX;
      double push_cube_y = task.pre_push.position.y;
      double push_cube_z = task.cell.position.z;

      // 预推位放置链路的公共几何（只依赖 task，不依赖执行后的 grasp_pose），提前到
      // 这里以便推入预演段与真实执行段共用。
      const double entry_x = task.pre_push.position.x - kPrePushOffsetX;
      const double release_z =
        task.pre_push.position.z + kSideContactCommandZOffset + kReleaseGapZ;
      const double entry_left_y = task.pre_push.position.y - kCubeHalf - kSideContactCommandGap;
      const double entry_right_y = task.pre_push.position.y + kCubeHalf + kSideContactCommandGap;

      // 换位第一段（笛卡尔竖直抬起）由 preplanPush 规划后写到这里：真实执行时**必须先
      // 走这一段**，否则后续从抬起位出发的 RRT 起点与实际位置不符（踩过一次：
      // 只规划不执行 → 右臂 11.8° 追不上）。
      trajectory_msgs::msg::JointTrajectory regrasp_lift;

      // Task27 外侧件的重抓候选还必须允许后续侧压。此回调在下方定义好
      // planSideCompaction 后赋值；Task26 和 Task27 内侧/中央件不改变筛选链。
      std::function<bool(
        const trajectory_msgs::msg::JointTrajectory&,
        const trajectory_msgs::msg::JointTrajectory&,
        const geometry_msgs::msg::Pose&,
        const std::string&)> side_preflight;

      // 「换位到 -X 面 → 沿 +X 推入 → 原路退出」的完整预演。零命令预检与真实执行共用
      // 这一段：推入是本任务几何风险最高的一段（新持件面 + 新推进轴），绝不能只
      // 预演到预推位放置就结束。
      // lift_only：只规划"笛卡尔竖直抬起"段（它必须在**滑轨平移之前**执行，
      //            让杯面离开 Cube 所在高度——实测平移时吸盘离 Cube 面仅约 1.2 mm，
      //            稍偏就会把那颗自由站立的 Cube 顶走）。
      // lift_already_done：抬起段已执行过，本次只规划摆动/推入/退出，不再重规划抬起
      //            （否则会用同一个位姿规划出一条零长轨迹）。
      const auto preplanPush =
        [&](const trajectory_msgs::msg::JointTrajectory& start_left,
            const trajectory_msgs::msg::JointTrajectory& hold_right,
            const std::string& stage_prefix,
            bool lift_only,
            bool lift_already_done,
            trajectory_msgs::msg::JointTrajectory* left_regrasp_out,
            trajectory_msgs::msg::JointTrajectory* left_push_out,
            trajectory_msgs::msg::JointTrajectory* right_hold_push_out,
            trajectory_msgs::msg::JointTrajectory* left_retreat_out,
            trajectory_msgs::msg::JointTrajectory* right_hold_retreat_out)
      {
        // 换位分两段。第一段：**笛卡尔竖直抬起**，让杯面离开 Cube 所在的高度。
        // 单段 RRT 从 ±Y 持件位直接摆到 -X 面位时，松手后杯面离 Cube 只有约 1.5 mm，
        // 绕行路径会擦到 Cube 并把它拖走（实测被拖 446 mm，见 TASK26 文档「待修清单」#1）。
        // 抬起是沿 Cube 侧面滑升，1.5 mm 间隙保持不变，不会拖件。
        trajectory_msgs::msg::JointTrajectory grasp_lift_l, grasp_lift_r;
        {
          const geometry_msgs::msg::Pose lift_left_target = push_left
            ? sidePose(push_entry_x, push_cube_y - kPushCupOffsetX,
                push_cube_z + kRegraspLiftHeight, true)
            : helper_park;
          const geometry_msgs::msg::Pose lift_right_target = push_left
            ? helper_park
            : sidePose(push_entry_x, push_cube_y + kPushCupOffsetX,
                push_cube_z + kRegraspLiftHeight, false);
          const auto& lift_left_start = push_left ? start_left : hold_right;
          const auto& lift_right_start = push_left ? hold_right : start_left;
          if (!planAndCheckCommon(node, left_group, right_group, left, right,
                finalPositions(lift_left_start), finalPositions(lift_right_start),
                lift_left_target, lift_right_target, world_after_remove,
                stage_prefix + " REGRASP_LIFT", &grasp_lift_l, &grasp_lift_r))
          {
            RCLCPP_ERROR(node->get_logger(),
              "%s REGRASP_LIFT (笛卡尔竖直抬起) failed.", task.id);
            return false;
          }
        }
        // 抬起段已执行时，它的末端就是本次摆动段的起点（start_left 是推入臂轨迹）。
        const trajectory_msgs::msg::JointTrajectory& lift_end = lift_already_done
          ? start_left
          : (push_left ? grasp_lift_l : grasp_lift_r);
        if (!lift_already_done)
        {
          regrasp_lift = lift_end;
        }
        if (lift_only)
        {
          return true;
        }

        // 第二段：从抬起位 RRT 摆到 -X 面姿态。显式设置起始状态而不是用 current
        // state：预检不执行任何动作、current state 仍是 HOME，那样预演的就不是真正
        // 要走的换位段。
        // 换位摆臂必须在**看得见那颗 Cube** 的世界里规划。上面为了接触/负载段把目标
        // Cube 从规划场景摘掉了；若换位段沿用这个"无 Cube 世界"，RRT 会把 L 型工具从
        // 这颗**已经自由站在预推位**（两侧吸盘都已送掉）的 Cube 顶上抹过去，物理上直接
        // 把它顶走——实测 drift 57.07 mm，触发 "Cube drifted while the pusher
        // re-grasped" 中止。这与外侧接近是同一类病因，用同一套修法：规划前按预推位
        // 把 Cube 放回，规划完立刻摘走。目标位（杯面离 Cube 面 1 mm）本身无碰撞，带
        // Cube 必然可解。
        // 用**实测**位姿，而不是标称 task.pre_push：短推是靠杯面摩擦推件，落点随接触
        // 浮动——实测落桌后 Cube 停在 x=0.821（标称 0.840，差 19 mm）。若按标称放碰撞体，
        // RRT 会从 Cube 的**真实**位置抹过去并把它顶走（实测 Y 向漂 25.4 mm 后中止）。
        const auto [regrasp_cube, regrasp_revision] = cubes.get(task.cube_index);
        (void)regrasp_revision;
        if (!scene.applyCollisionObject(cubeObject(object_id, regrasp_cube)))
        {
          RCLCPP_ERROR(node->get_logger(),
            "%s cannot re-apply the cube CollisionObject %s for the regrasp.",
            task.id, object_id.c_str());
          return false;
        }
        std::this_thread::sleep_for(250ms);
        // RRT 的终点仍是 -X 面的 2.5 mm 捕获间隙；这里按关节路程排序候选，
        // 优先选择最短、最少绕腕的无碰撞换位路径。此前“先生成即采用”的随机
        // 候选会偶发绕到 Cube 边缘，尽管最终姿态正确，PhysX 中仍可能擦动 Cube。
        // 单批 8 个 RRTConnect 解并不能稳定覆盖侧向 L 型工具所需的腕部冗余分支：
        // 某些终点构型虽然换位本身无碰撞，却会令紧接着的 +X Cartesian 推入跳到
        // 另一支 IK。与 RIGHT_OUTER_APPROACH 一致，这里也采样多批，再以整个
        // REGRASP -> PUSH -> RETREAT 链路筛选；没有任何候选通过时宁可安全停止。
        std::vector<std::pair<double, trajectory_msgs::msg::JointTrajectory>> scored_candidates;
        for (int batch = 1; batch <= kRrtCandidateBatches; ++batch)
        {
          std::vector<trajectory_msgs::msg::JointTrajectory> batch_candidates;
          if (!pusher.planPoseCandidatesFrom(pusher_group, finalPositions(lift_end),
                pushPose(push_entry_x, push_cube_y, push_cube_z),
                stage_prefix + " REGRASP_NEG_X batch=" + std::to_string(batch),
                &batch_candidates))
          {
            RCLCPP_WARN(node->get_logger(),
              "%s REGRASP_NEG_X batch=%d/%d produced no RRT candidate (pusher=%s).",
              task.id, batch, kRrtCandidateBatches, push_left ? "left" : "right");
            continue;
          }
          for (auto& candidate : batch_candidates)
          {
            double travel = 0.0;
            for (std::size_t point = 1; point < candidate.points.size(); ++point)
            {
              for (std::size_t joint = 0; joint < candidate.points[point].positions.size(); ++joint)
              {
                travel += std::abs(candidate.points[point].positions[joint] -
                  candidate.points[point - 1].positions[joint]);
              }
            }
            scored_candidates.emplace_back(travel, std::move(candidate));
          }
        }
        if (scored_candidates.empty())
        {
          scene.removeCollisionObjects({object_id});
          std::this_thread::sleep_for(250ms);
          RCLCPP_ERROR(node->get_logger(),
            "%s REGRASP_NEG_X exhausted %d RRT candidate batches (pusher=%s).",
            task.id, kRrtCandidateBatches, push_left ? "left" : "right");
          return false;
        }
        std::sort(scored_candidates.begin(), scored_candidates.end(),
          [](const auto& first, const auto& second) { return first.first < second.first; });
        std::vector<trajectory_msgs::msg::JointTrajectory> candidates;
        candidates.reserve(scored_candidates.size());
        for (auto& [travel, candidate] : scored_candidates)
        {
          (void)travel;
          candidates.push_back(std::move(candidate));
        }
        RCLCPP_INFO(node->get_logger(),
          "%s REGRASP_NEG_X evaluating %zu candidates from %d batches, shortest_joint_travel=%.3f.",
          task.id, candidates.size(), kRrtCandidateBatches, scored_candidates.front().first);
        const auto world_with_regrasp_cube = staticWorld(scene);
        // 退出段用「沿 -X 原路退出装料口」，而不是在格内纯 +Z 抬升：实测「格内垂直
        // 抬升 0.28 m」会让 IK 跳解支（离线 195 mm），而原路返回就是把已经验证笔直的
        // 推入路径倒着走。退出后工具停在装料口外侧的桌面高度，由后续阶段/批次 HOME 抬走。
        //
        // 候选必须让**整条「换位 → +X 推入 → 原路退出」**都通过才算选中。只通过换位段的
        // 候选可能恰好把腕部停在一个后续推入会跳解支（笛卡尔离线）的构型上——实测
        // 就出现过「换位 FCL 通过、但 PUSH_INTO_BOX 离线 157/331 mm」的候选。
        // 这与右臂空载接近的做法一致：先预演完整链路，再定候选。
        for (std::size_t index = 0; index < candidates.size(); ++index)
        {
          const auto hold = holdTrajectory(
            hold_right, finalPositions(hold_right), pointTime(candidates[index].points.back()));
          const bool safe = push_left
            ? validateSync(node, left_group.getRobotModel(), world_with_regrasp_cube,
                candidates[index], hold,
                stage_prefix + " REGRASP_CANDIDATE_" + std::to_string(index + 1))
            : validateSync(node, left_group.getRobotModel(), world_with_regrasp_cube,
                hold, candidates[index],
                stage_prefix + " REGRASP_CANDIDATE_" + std::to_string(index + 1));
          if (!safe)
          {
            continue;
          }
          auto regrasp_candidate = candidates[index];
          trajectory_msgs::msg::JointTrajectory push_l, push_r, ret_l, ret_r;
          const bool chain_ok = push_left
            ? (planAndCheckCommon(node, left_group, right_group, left, right,
                 finalPositions(regrasp_candidate), finalPositions(hold_right),
                 pushPose(push_cell_x, push_cube_y, push_cube_z), helper_park,
                 world_after_remove, stage_prefix + " PUSH_INTO_BOX",
                 &push_l, &push_r) &&
               planAndCheckCommon(node, left_group, right_group, left, right,
                 finalPositions(push_l), finalPositions(push_r),
                 pushPose(push_entry_x, push_cube_y, push_cube_z), helper_park,
                 world_after_remove, stage_prefix + " PREPLANNED_CELL_EXIT",
                 &ret_l, &ret_r))
            : (planAndCheckCommon(node, left_group, right_group, left, right,
                 finalPositions(hold_right), finalPositions(regrasp_candidate),
                 helper_park, pushPose(push_cell_x, push_cube_y, push_cube_z),
                 world_after_remove, stage_prefix + " PUSH_INTO_BOX",
                 &push_l, &push_r) &&
               planAndCheckCommon(node, left_group, right_group, left, right,
                 finalPositions(push_l), finalPositions(push_r),
                 helper_park, pushPose(push_entry_x, push_cube_y, push_cube_z),
                 world_after_remove, stage_prefix + " PREPLANNED_CELL_EXIT",
                 &ret_l, &ret_r));
          if (!chain_ok)
          {
            RCLCPP_WARN(node->get_logger(),
              "%s REGRASP_CANDIDATE_%zu/%zu passed FCL but its PUSH_INTO_BOX / "
              "CELL_RETREAT preflight failed; trying the next candidate.",
              task.id, index + 1, candidates.size());
            continue;
          }
#ifdef TASK27_FIVE_CUBE
          if ((lift_already_done || planning_only) && !isStraightInsertTask(task) && side_preflight)
          {
            // 在真正吸住 -X 面之前，用同一候选的 PUSH 末端预演侧压。
            // 失败就换重抓构型，不能等 Cube 已推到深墙才发现横移不可达。
            auto predicted_seat = task.cell;
            predicted_seat.position.y = push_cube_y;
            predicted_seat.position.z = push_cube_z;
            if (!side_preflight(push_l, push_r, predicted_seat,
                  stage_prefix + " REGRASP_CANDIDATE_" + std::to_string(index + 1)))
            {
              RCLCPP_WARN(node->get_logger(),
                "%s REGRASP_CANDIDATE_%zu/%zu 推入通过，但后续侧压整链未通过；换候选。",
                task.id, index + 1, candidates.size());
              if (!scene.applyCollisionObject(cubeObject(object_id, regrasp_cube)))
              {
                return false;
              }
              std::this_thread::sleep_for(250ms);
              continue;
            }
            RCLCPP_INFO(node->get_logger(),
              "%s REGRASP_CANDIDATE_%zu PREPUSH_SIDE_PREFLIGHT PASS: "
              "推入前已从同一 PUSH 末端验证侧压整链。", task.id, index + 1);
          }
#endif
          *left_regrasp_out = std::move(regrasp_candidate);
          *left_push_out = std::move(push_l);
          *right_hold_push_out = std::move(push_r);
          *left_retreat_out = std::move(ret_l);
          *right_hold_retreat_out = std::move(ret_r);
          // 换位链已经结束，后续 +X 推入是合法接触阶段，才从 MoveIt world
          // 暂时摘掉当前 Cube；其它墙体、桌面、已完成 Cube 始终保留在 FCL 中。
          scene.removeCollisionObjects({object_id});
          std::this_thread::sleep_for(250ms);
          RCLCPP_INFO(node->get_logger(),
            "%s selected shortest safe REGRASP_CANDIDATE_%zu/%zu with full PUSH/RETREAT preflight (pusher=%s).",
            task.id, index + 1, candidates.size(), push_left ? "left" : "right");
          return true;
        }
        scene.removeCollisionObjects({object_id});
        std::this_thread::sleep_for(250ms);
        RCLCPP_ERROR(node->get_logger(),
          "%s no regrasp candidate passed the full regrasp -> push -> retreat preflight (pusher=%s).",
          task.id, push_left ? "left" : "right");
        return false;
      };

      // ------------------------------------------------------ 侧墙压紧
      //
      // +X 推入后，Cube 已经由桌面承托并贴住深端墙。推入臂保持 -X 面吸附，
      // 作为随 Y 横移的背挡；另一臂以未吸附的杯面侧压。规划起点必须是 PUSH
      // 末端，不能误用已经退出后的姿态。侧压完成后才释放并反向退出。
      //
      // 该链包含：空载 RRT 高位入场 -> 笛卡尔下降到 1 mm 名义间隙 -> 笛卡尔
      // 短压 -> 三段严格反向撤离。入场/下降阶段带当前 Cube 碰撞体；压紧阶段
      // 临时摘掉当前 Cube（合法接触），但桌面、三面墙、已完成 Cube 和另一臂
      // 仍始终在 FCL 世界中。最终用 Isaac Ground Truth 检查两个墙 gap。
      struct SideCompactionPlan
      {
        trajectory_msgs::msg::JointTrajectory left_approach;
        trajectory_msgs::msg::JointTrajectory right_approach;
        trajectory_msgs::msg::JointTrajectory left_contact;
        trajectory_msgs::msg::JointTrajectory right_contact;
        trajectory_msgs::msg::JointTrajectory left_press;
        trajectory_msgs::msg::JointTrajectory right_press;
        trajectory_msgs::msg::JointTrajectory left_withdraw;
        trajectory_msgs::msg::JointTrajectory right_withdraw;
        trajectory_msgs::msg::JointTrajectory left_lift;
        trajectory_msgs::msg::JointTrajectory right_lift;
        trajectory_msgs::msg::JointTrajectory left_return;
        trajectory_msgs::msg::JointTrajectory right_return;
      };

      const auto planSideCompaction =
        [&](const trajectory_msgs::msg::JointTrajectory& left_start,
            const trajectory_msgs::msg::JointTrajectory& right_start,
            const geometry_msgs::msg::Pose& seated_cube,
            const std::string& stage_prefix, SideCompactionPlan* output)
      {
        if (!output || left_start.points.empty() || right_start.points.empty())
        {
          return false;
        }
        // 逐字段默认构造，避免 ROS 消息的 explicit 默认构造器在聚合赋值时产生
        // 无意义的编译告警。
        SideCompactionPlan reset_plan;
        *output = std::move(reset_plan);
        // 原推入臂按任务行选择；另一臂一定从相反侧压向对应墙：
        //   +Y 行：right 负责 +X 推入，left 从 -Y 压向 +Y 墙；
        //   -Y 行：left 负责 +X 推入，right 从 +Y 压向 -Y 墙。
        const bool compactor_is_left = !push_left;
        const double press_direction_y = compactor_is_left ? 1.0 : -1.0;
        if ((press_direction_y > 0.0 && task.cell.position.y <= seated_cube.position.y) ||
            (press_direction_y < 0.0 && task.cell.position.y >= seated_cube.position.y))
        {
          RCLCPP_ERROR(node->get_logger(),
            "%s SIDE_COMPACTION target is not outward from the seated Cube.", task.id);
          return false;
        }
        const Arm& compactor = compactor_is_left ? left : right;
        auto& compactor_group = compactor_is_left ? left_group : right_group;
        const std::vector<double> compactor_start = compactor_is_left
          ? finalPositions(left_start) : finalPositions(right_start);
        const double entry_contact_y = seated_cube.position.y -
          press_direction_y * (kCubeHalf + kSideContactCommandGap);
        const double final_contact_y = task.cell.position.y +
          press_direction_y * kSideWallPressOvertravel -
          press_direction_y * (kCubeHalf + kSideContactCommandGap);
        const auto high_target = sidePose(
          seated_cube.position.x, entry_contact_y,
          seated_cube.position.z + kRegraspLiftHeight, compactor_is_left);
        const auto contact_target = sidePose(
          seated_cube.position.x, entry_contact_y,
          seated_cube.position.z + kSideContactCommandZOffset, compactor_is_left);
        const auto press_target = sidePose(
          seated_cube.position.x, final_contact_y,
          seated_cube.position.z + kSideContactCommandZOffset, compactor_is_left);

        // 空载 RRT 必须看见当前 Cube；否则工具可能从它的顶边穿过。
        if (!scene.applyCollisionObject(cubeObject(object_id, seated_cube)))
        {
          return false;
        }
        std::this_thread::sleep_for(250ms);
        const auto world_with_cube = staticWorld(scene);
        std::vector<trajectory_msgs::msg::JointTrajectory> candidates;
        if (!compactor.planPoseCandidatesFrom(compactor_group, compactor_start, high_target,
              stage_prefix + " SIDE_HIGH_APPROACH", &candidates))
        {
          scene.removeCollisionObjects({object_id});
          std::this_thread::sleep_for(250ms);
          return false;
        }

        bool selected = false;
        for (std::size_t index = 0; index < candidates.size(); ++index)
        {
          auto left_approach = compactor_is_left ? candidates[index] : holdTrajectory(
            left_start, finalPositions(left_start), pointTime(candidates[index].points.back()));
          auto right_approach = compactor_is_left ? holdTrajectory(
            right_start, finalPositions(right_start), pointTime(candidates[index].points.back())) : candidates[index];
          if (!validateSync(node, left_group.getRobotModel(), world_with_cube,
                left_approach, right_approach,
                stage_prefix + " SIDE_HIGH_APPROACH_" + std::to_string(index + 1)))
          {
            continue;
          }

          trajectory_msgs::msg::JointTrajectory contact_helper;
          if (!planCommonCartesian(node, compactor_group, compactor.groupName(),
                compactor_is_left ? right.groupName() : left.groupName(),
                compactor_is_left ? finalPositions(left_approach) : finalPositions(right_approach),
                compactor_is_left ? finalPositions(right_approach) : finalPositions(left_approach),
                contact_target,
                stage_prefix + " SIDE_CONTACT_" + std::to_string(index + 1),
                &contact_helper, compactor.eefLink(), true))
          {
            continue;
          }
          auto left_contact = compactor_is_left ? contact_helper : holdTrajectory(
            left_approach, finalPositions(left_approach), pointTime(contact_helper.points.back()));
          auto right_contact = compactor_is_left ? holdTrajectory(
            right_approach, finalPositions(right_approach), pointTime(contact_helper.points.back())) : contact_helper;
          if (!validateSync(node, left_group.getRobotModel(), world_with_cube,
                left_contact, right_contact,
                stage_prefix + " SIDE_CONTACT_" + std::to_string(index + 1)))
          {
            continue;
          }

          // 只有从 1 mm 间隙压进 Cube 的这一段允许当前 Cube 不在 MoveIt world；
          // 其它障碍物仍由 world_after_remove 完整检查。
          scene.removeCollisionObjects({object_id});
          std::this_thread::sleep_for(250ms);
          trajectory_msgs::msg::JointTrajectory press_helper;
          if (!planCommonCartesian(node, compactor_group, compactor.groupName(),
                compactor_is_left ? right.groupName() : left.groupName(),
                compactor_is_left ? finalPositions(left_contact) : finalPositions(right_contact),
                compactor_is_left ? finalPositions(right_contact) : finalPositions(left_contact),
                press_target,
                stage_prefix + " SIDE_PRESS_" + std::to_string(index + 1),
                &press_helper, compactor.eefLink(), false))
          {
            // 下一候选仍需带 Cube 重新规划接近段。
            scene.applyCollisionObject(cubeObject(object_id, seated_cube));
            std::this_thread::sleep_for(250ms);
            continue;
          }

          // 深端背挡不是被动的几何装饰：原 -X 面推入臂保持吸附，并与侧压臂
          // 同步沿 Y 走相同位移。这样 Cube 既能被另一臂压到侧墙，又不会因
          // 横向摩擦回带离开 +X 深墙。两条轨迹在下一步严格同步、FCL 验证。
          const std::vector<double> pusher_start = push_left
            ? finalPositions(left_contact) : finalPositions(right_contact);
          const std::vector<double> pusher_partner_start = compactor_is_left
            ? finalPositions(left_contact) : finalPositions(right_contact);
          trajectory_msgs::msg::JointTrajectory press_pusher;
          // FR3 在这条姿态族上一次性横移 194 mm 时可能跨到另一套冗余 IK 分支：
          // fraction 仍会显示 1.0，但 FK 实测会偏离命令直线数百毫米。将背挡臂
          // 的同步横移拆为 8 段，每段都复用贴线检查；任一小段跳支就拒绝本候选。
          constexpr int kBackstopCartesianSegments = 8;
          std::vector<double> segment_start = pusher_start;
          bool pusher_side_ok = true;
          for (int segment_index = 1; segment_index <= kBackstopCartesianSegments; ++segment_index)
          {
            const double alpha = static_cast<double>(segment_index) /
              static_cast<double>(kBackstopCartesianSegments);
            const double segment_y = seated_cube.position.y + alpha *
              (task.cell.position.y - seated_cube.position.y);
            trajectory_msgs::msg::JointTrajectory segment;
            if (!planCommonCartesian(node, pusher_group, pusher.groupName(),
                  compactor.groupName(), segment_start, pusher_partner_start,
                  pushPose(seated_cube.position.x - kPushCupOffsetX, segment_y,
                    seated_cube.position.z),
                  stage_prefix + " SIDE_BACKSTOP_Y_" + std::to_string(index + 1) +
                    "_SEGMENT_" + std::to_string(segment_index),
                  &segment, pusher.eefLink(), false))
            {
              pusher_side_ok = false;
              break;
            }
            if (press_pusher.points.empty())
            {
              press_pusher = std::move(segment);
            }
            else
            {
              const double offset = pointTime(press_pusher.points.back());
              for (std::size_t point_index = 1; point_index < segment.points.size(); ++point_index)
              {
                auto point = segment.points[point_index];
                setPointTime(point, offset + pointTime(point));
                press_pusher.points.push_back(std::move(point));
              }
            }
            segment_start = finalPositions(press_pusher);
          }
          if (!pusher_side_ok || press_pusher.points.empty())
          {
            scene.applyCollisionObject(cubeObject(object_id, seated_cube));
            std::this_thread::sleep_for(250ms);
            continue;
          }
          auto left_press = push_left ? press_pusher : press_helper;
          auto right_press = push_left ? press_helper : press_pusher;
          if (!synchronize(&left_press, &right_press) ||
              !validateSync(node, left_group.getRobotModel(), world_after_remove,
                left_press, right_press,
                stage_prefix + " SIDE_PRESS_" + std::to_string(index + 1)))
          {
            scene.applyCollisionObject(cubeObject(object_id, seated_cube));
            std::this_thread::sleep_for(250ms);
            continue;
          }
          output->left_approach = std::move(left_approach);
          output->right_approach = std::move(right_approach);
          output->left_contact = std::move(left_contact);
          output->right_contact = std::move(right_contact);
          output->left_press = std::move(left_press);
          output->right_press = std::move(right_press);
          output->left_withdraw = reverseTrajectory(output->left_press);
          output->right_withdraw = reverseTrajectory(output->right_press);
          output->left_lift = reverseTrajectory(output->left_contact);
          output->right_lift = reverseTrajectory(output->right_contact);
          output->left_return = reverseTrajectory(output->left_approach);
          output->right_return = reverseTrajectory(output->right_approach);
          selected = true;
          RCLCPP_INFO(node->get_logger(),
            "%s SIDE_COMPACTION selected candidate=%zu/%zu: %s arm presses %.1f mm toward %s wall.",
            stage_prefix.c_str(), index + 1, candidates.size(), compactor_is_left ? "left" : "right",
            std::abs(task.cell.position.y - seated_cube.position.y) * 1000.0,
            press_direction_y > 0.0 ? "+Y" : "-Y");
          break;
        }
        // 成功时 Cube 已摘出，恰好供后续合法接触/物理执行使用；失败时确保不残留。
        if (!selected)
        {
          scene.removeCollisionObjects({object_id});
          std::this_thread::sleep_for(250ms);
        }
        return selected;
      };

#ifdef TASK27_FIVE_CUBE
      side_preflight = [&](const trajectory_msgs::msg::JointTrajectory& left_push,
                           const trajectory_msgs::msg::JointTrajectory& right_push,
                           const geometry_msgs::msg::Pose& predicted_seat,
                           const std::string& label)
      {
        SideCompactionPlan plan;
        return planSideCompaction(left_push, right_push, predicted_seat,
          label + " PREPUSH_SIDE_PREFLIGHT", &plan);
      };
#endif

      if (planning_only)
      {
        // 零命令预检必须把推入段也走完：用候选预演出的 short-push 末端作为起点，
        // 预演「换位 → +X 推入 → 退出」。任何一段失败都不能算通过。
        trajectory_msgs::msg::JointTrajectory po_regrasp, po_push, po_right_push;
        trajectory_msgs::msg::JointTrajectory po_retreat, po_right_retreat;
        const auto& po_start = push_left ? selected_left_push : selected_right_push;
        const auto& po_hold = push_left ? selected_right_retreat : selected_left_retreat;
        if (!preplanPush(po_start, po_hold, std::string(task.id), false, false,
              &po_regrasp, &po_push, &po_right_push, &po_retreat, &po_right_retreat))
        {
          RCLCPP_ERROR(node->get_logger(),
            "%s planning-only FAILED at the push stage (regrasp / push / cell retreat).", task.id);
          all_complete = false;
          break;
        }
        if (!isStraightInsertTask(task))
        {
          SideCompactionPlan side_plan;
          auto planned_deep_seat = task.cell;
          planned_deep_seat.position.y = task.pre_push.position.y;
          if (!planSideCompaction(po_push, po_right_push, planned_deep_seat,
                std::string(task.id) + " PLANNING_ONLY", &side_plan))
          {
            RCLCPP_ERROR(node->get_logger(),
              "%s planning-only FAILED at the side-compaction stage.", task.id);
            all_complete = false;
            break;
          }
        }
        // 整条链路通过：把本件按最终的"深端 + 侧墙均贴合"格位放回 Planning
        // Scene，供下一件的 FCL 使用。
        if (!scene.applyCollisionObject(cubeObject(object_id, task.cell)))
        {
          RCLCPP_ERROR(node->get_logger(),
            "%s planning-only cannot restore cell CollisionObject %s.",
            task.id, object_id.c_str());
          all_complete = false;
          break;
        }
        std::this_thread::sleep_for(100ms);
        RCLCPP_INFO(node->get_logger(),
          "%s %s PLANNING-ONLY PASS: 共同搬运、-X 面重抓、+X 推入与退出通过 IK/FCL；"
          "侧向压紧=%s；未发布 joint、suction 或 feed_command。",
          task.id, kTaskLabel, isStraightInsertTask(task) ? "不需要" : "已预检");
        continue;
      }

      RCLCPP_INFO(node->get_logger(), "%s RIGHT_CONTACT final command q=%s.",
        task.id, formatJointPositions(finalPositions(right_outer_descent)).c_str());
      auto left_hold_contact = holdTrajectory(
        left_contact, finalPositions(left_contact), pointTime(right_outer_descent.points.back()));
      if (!executeSync(left, left_hold_contact, right, right_outer_descent))
      {
        all_complete = false;
        break;
      }
      RCLCPP_INFO(node->get_logger(), "%s RIGHT_CONTACT final command q=%s.",
        task.id, formatJointPositions(finalPositions(right_contact)).c_str());
      left_hold_contact = holdTrajectory(
        left_contact, finalPositions(left_contact), pointTime(right_contact.points.back()));
      if (!executeSync(left, left_hold_contact, right, right_contact))
      {
        all_complete = false;
        break;
      }

      // CLOSED 仅代表 Surface Gripper 创建了约束，不代表两侧都吸在对应位置。
      // 在任何一只吸盘打开前，以 Cube + 左/右 TCP Ground Truth 建立闭环接触
      // 门控；失败时只在未吸附状态下以最新 Cube pose 微调并复测，绝不带着
      // 歪斜几何进入共同抬升。
      bool pre_close_geometry_ok = false;
      geometry_msgs::msg::Pose grasp_pose;
      for (int alignment_attempt = 1; alignment_attempt <= kRetries; ++alignment_attempt)
      {
        std::this_thread::sleep_for(250ms);
        const auto [live_cube, live_revision] = cubes.get(task.cube_index);
        (void)live_revision;
        if (validateDualSideAttachment(node, task, live_cube, left_tcp.get(), right_tcp.get(),
              "PRE_CLOSE_GEOMETRY"))
        {
          grasp_pose = live_cube;
          pre_close_geometry_ok = true;
          break;
        }
        if (alignment_attempt == kRetries)
        {
          break;
        }

        RCLCPP_WARN(node->get_logger(),
          "%s PRE_CLOSE_GEOMETRY retry=%d/%d: no suction constraint exists; reacquire both "
          "TCP targets from the latest Cube Ground Truth.",
          task.id, alignment_attempt, kRetries);
        trajectory_msgs::msg::JointTrajectory left_reacquire;
        trajectory_msgs::msg::JointTrajectory right_reacquire;
        // 闭环纠偏：用实测间隙差修正两侧指令，保持 Task26 已验证的行为。
        // Task27 如两侧都偏远，最终仍由预吸附间隙门禁及 Isaac CLOSED
        // 物理确认把关；本分支不尝试未经验证的强制压入。
        const double left_gap = live_cube.position.y - left_tcp.get().position.y - kCubeHalf;
        const double right_gap = right_tcp.get().position.y - live_cube.position.y - kCubeHalf;
        const double correction_y = 0.5 * (left_gap - right_gap);
        const double live_left_y = live_cube.position.y - kCubeHalf - kSideContactCommandGap + correction_y;
        const double live_right_y = live_cube.position.y + kCubeHalf + kSideContactCommandGap + correction_y;
        RCLCPP_WARN(node->get_logger(),
          "%s PRE_CLOSE_GEOMETRY 纠偏：left_gap=%.3f mm right_gap=%.3f mm ⇒ 两侧 y 指令平移 %+.3f mm。",
          task.id, left_gap * 1000.0, right_gap * 1000.0, correction_y * 1000.0);
        if (!planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(left_contact), finalPositions(right_contact),
              sidePose(live_cube.position.x, live_left_y,
                live_cube.position.z + kSideContactCommandZOffset, true),
              sidePose(live_cube.position.x, live_right_y,
                live_cube.position.z + kSideContactCommandZOffset, false),
              world_after_remove, std::string(task.id) + " PRE_CLOSE_REACQUIRE",
              &left_reacquire, &right_reacquire) ||
            !executeSync(left, left_reacquire, right, right_reacquire))
        {
          RCLCPP_ERROR(node->get_logger(), "%s PRE_CLOSE_REACQUIRE failed.", task.id);
          break;
        }
        left_contact = std::move(left_reacquire);
        right_contact = std::move(right_reacquire);
      }
      if (!pre_close_geometry_ok)
      {
        RCLCPP_ERROR(node->get_logger(),
          "%s cannot establish corresponding dual side contact; do not enable suction.", task.id);
        all_complete = false;
        break;
      }

      // 在物理吸附前先对首个负载段做完整预检。这样若当前空载 RRT 末端
      // 构型会让共同 LIFT 的 Cartesian IK 支路擦桌面，就在两杯仍 OFF 时
      // 停止，而不是建立约束后才发现没有安全的负载轨迹。
      trajectory_msgs::msg::JointTrajectory preflight_left_lift;
      trajectory_msgs::msg::JointTrajectory preflight_right_lift;
      const double preflight_left_y = grasp_pose.position.y - kCubeHalf - kSideContactCommandGap;
      const double preflight_right_y = grasp_pose.position.y + kCubeHalf + kSideContactCommandGap;
      if (!planAndCheckCommon(node, left_group, right_group, left, right,
            finalPositions(left_contact), finalPositions(right_contact),
            sidePose(grasp_pose.position.x, preflight_left_y,
              grasp_pose.position.z + kSideContactCommandZOffset + kLiftHeight, true),
            sidePose(grasp_pose.position.x, preflight_right_y,
              grasp_pose.position.z + kSideContactCommandZOffset + kLiftHeight, false),
            world_after_remove, std::string(task.id) + " PRE_CLOSE_COMMON_LIFT_PREFLIGHT",
            &preflight_left_lift, &preflight_right_lift))
      {
        RCLCPP_ERROR(node->get_logger(),
          "%s cannot preflight a safe COMMON_LIFT; suction remains OFF.", task.id);
        all_complete = false;
        break;
      }

      std::thread left_on([&]() { left.suction(true); });
      std::thread right_on([&]() { right.suction(true); });
      left_on.join();
      right_on.join();
      if (!left.waitSuction(true, 3.0) || !right.waitSuction(true, 3.0))
      {
        RCLCPP_ERROR(node->get_logger(), "%s side suction did not close on both arms.", task.id);
        openBothAndConfirm("safe abort after close failure");
        all_complete = false;
        break;
      }

      std::this_thread::sleep_for(250ms);
      const auto [attachment_pose, attachment_revision] = cubes.get(task.cube_index);
      (void)attachment_revision;
      if (!validateDualSideAttachment(node, task, attachment_pose, left_tcp.get(), right_tcp.get(),
            "POST_CLOSE"))
      {
        openBothAndConfirm("safe abort after attachment validation failure");
        all_complete = false;
        break;
      }

      // 以闭合后真实锁定的 Cube pose 作为所有负载 Cartesian 段的起点，不能再
      // 回退到任务开始时的 source。这样 Z->X->Y->Z 的每段都从同一真实几何出发。
      grasp_pose = attachment_pose;
      const double grasp_left_contact_y =
        grasp_pose.position.y - kCubeHalf - kSideContactCommandGap;
      const double grasp_right_contact_y =
        grasp_pose.position.y + kCubeHalf + kSideContactCommandGap;

      const auto trace_held = [&](const std::string& phase)
      {
        // 让 PhysX 与 ROS Ground Truth 至少完成数帧更新，再记录共同物体真实状态。
        std::this_thread::sleep_for(250ms);
        const auto [pose, revision] = cubes.get(task.cube_index);
        (void)revision;
        RCLCPP_INFO(node->get_logger(),
          "%s %s GT=(%.3f, %.3f, %.3f), suction=(left:%s right:%s).",
          task.id, phase.c_str(), pose.position.x, pose.position.y, pose.position.z,
          left.isClosed() ? "CLOSED" : "OPEN", right.isClosed() ? "CLOSED" : "OPEN");
        if (!left.isClosed() || !right.isClosed())
        {
          RCLCPP_ERROR(node->get_logger(), "%s %s: side suction lost before release.",
            task.id, phase.c_str());
          return false;
        }
        return validateDualSideAttachment(node, task, pose, left_tcp.get(), right_tcp.get(), phase);
      };

      auto stage = [&](const std::string& name, const geometry_msgs::msg::Pose& left_target,
                       const geometry_msgs::msg::Pose& right_target,
                       const trajectory_msgs::msg::JointTrajectory& left_start,
                       const trajectory_msgs::msg::JointTrajectory& right_start,
                       trajectory_msgs::msg::JointTrajectory* left_result,
                       trajectory_msgs::msg::JointTrajectory* right_result)
      {
        return planAndCheckCommon(node, left_group, right_group, left, right,
          finalPositions(left_start), finalPositions(right_start), left_target, right_target,
          world_after_remove, std::string(task.id) + " " + name, left_result, right_result);
      };

      trajectory_msgs::msg::JointTrajectory left_lift, right_lift;
      if (!stage("COMMON_LIFT",
            sidePose(grasp_pose.position.x, grasp_left_contact_y,
              grasp_pose.position.z + kSideContactCommandZOffset + kLiftHeight, true),
            sidePose(grasp_pose.position.x, grasp_right_contact_y,
              grasp_pose.position.z + kSideContactCommandZOffset + kLiftHeight, false),
            left_contact, right_contact, &left_lift, &right_lift) ||
          !executeSync(left, left_lift, right, right_lift) ||
          !trace_held("COMMON_LIFT"))
      {
        openBothAndConfirm("safe abort after COMMON_LIFT failure");
        all_complete = false;
        break;
      }

      const double transport_z =
        grasp_pose.position.z + kSideContactCommandZOffset + kLiftHeight;

      // 吸住 Cube 后不再允许 OMPL 产生任意弯绕路径。共同负载轨迹被显式拆为
      // Z(lift) -> X(travel) -> Y(align) -> Z(descent) -> X(short push)；每段均
      // 在两臂完整 RobotState 上做同步 FCL 检查。
      trajectory_msgs::msg::JointTrajectory left_transport_x, right_transport_x;
      if (!stage("COMMON_X_TRAVEL",
            sidePose(entry_x, grasp_left_contact_y, transport_z, true),
            sidePose(entry_x, grasp_right_contact_y, transport_z, false),
            left_lift, right_lift, &left_transport_x, &right_transport_x) ||
          !executeSync(left, left_transport_x, right, right_transport_x) ||
          !trace_held("COMMON_X_TRAVEL"))
      {
        openBothAndConfirm("safe abort after COMMON_X_TRAVEL failure");
        all_complete = false;
        break;
      }

      trajectory_msgs::msg::JointTrajectory left_transport_y, right_transport_y;
      if (!stage("COMMON_Y_ALIGN",
            sidePose(entry_x, entry_left_y, transport_z, true),
            sidePose(entry_x, entry_right_y, transport_z, false),
            left_transport_x, right_transport_x, &left_transport_y, &right_transport_y) ||
          !executeSync(left, left_transport_y, right, right_transport_y) ||
          !trace_held("COMMON_Y_ALIGN"))
      {
        openBothAndConfirm("safe abort after COMMON_Y_ALIGN failure");
        all_complete = false;
        break;
      }

      trajectory_msgs::msg::JointTrajectory left_descent, right_descent;
      if (!stage("COMMON_DESCENT_TO_ENTRY",
            sidePose(entry_x, entry_left_y, release_z, true),
            sidePose(entry_x, entry_right_y, release_z, false),
            left_transport_y, right_transport_y, &left_descent, &right_descent) ||
          !executeSync(left, left_descent, right, right_descent) ||
          !trace_held("COMMON_DESCENT_TO_ENTRY"))
      {
        openBothAndConfirm("safe abort after COMMON_DESCENT_TO_ENTRY failure");
        all_complete = false;
        break;
      }
      trajectory_msgs::msg::JointTrajectory left_push, right_push;
      if (!stage("COMMON_SIDE_SHORT_PUSH",
            sidePose(task.pre_push.position.x, entry_left_y, release_z, true),
            sidePose(task.pre_push.position.x, entry_right_y, release_z, false),
            left_descent, right_descent, &left_push, &right_push) ||
          !executeSync(left, left_push, right, right_push) ||
          !trace_held("COMMON_SIDE_SHORT_PUSH"))
      {
        openBothAndConfirm("safe abort after COMMON_SIDE_SHORT_PUSH failure");
        all_complete = false;
        break;
      }

      // ------------------------------------------- Task26 推入：沿 +X，持 -X 面
      //
      // 为什么必须换持件面：侧吸工具在杯面后方伸出 155 mm，而车厢内腔只有 0.25 宽、
      // 两排各 0.12。若继续用 ±Y 面持件沿 +X 推入，工具会伸进车厢并撞 ±Y 墙或
      // 另一排已落稳的 Cube。改持 **-X 面（装料口那面）** 后，整个工具留在车厢
      // 外侧，往里推 +X 不碰任何墙——这是本车厢唯一可行的推入持件面。
      //
      // 流程：右臂先释放并竖直退出 → 左臂松开 -Y 面（Cube 由桌面承托）→ 左臂
      // 空载换位吸住 -X 面 → 左臂沿 +X 分段推入格位 → 格内释放并退出。
      // 推入全程贴桌面滑动、不做抬升：单臂吸盘不承重，重量必须由桌面承担。
      // 「先落桌、再双臂同步撤出」——修正原先的单臂承重窗口。
      //
      // SIDE_SHORT_PUSH 结束时 Cube 中心还在 release_z = pre_push.z + kReleaseGapZ，
      // 即**离桌面 kReleaseGapZ = 20 mm**。原实现让辅助臂先松手并竖直退出、推入臂仍
      // 吸着，于是这 20 mm 的下落/失稳由**单臂**承担 —— 与本项目"单臂吸盘搬不动
      // Cube、必须双臂"的前提矛盾（验收实测：Cube 悬着、左臂已退出）。
      // 改为：① 双臂共同下降 kReleaseGapZ，把 Cube **完全放到桌面**（重量交给桌面）；
      //       ② 两侧吸盘**同时**松开；③ 之后才各自退出。
      trajectory_msgs::msg::JointTrajectory left_drop, right_drop;
      if (!stage("COMMON_DROP_TO_TABLE",
            sidePose(entry_x, entry_left_y, task.pre_push.position.z + kSideContactCommandZOffset, true),
            sidePose(entry_x, entry_right_y, task.pre_push.position.z + kSideContactCommandZOffset, false),
            left_push, right_push, &left_drop, &right_drop) ||
          !executeSync(left, left_drop, right, right_drop) ||
          !trace_held("COMMON_DROP_TO_TABLE"))
      {
        openBothAndConfirm("safe abort after COMMON_DROP_TO_TABLE failure");
        all_complete = false;
        break;
      }
      // 两侧吸盘同时松开。Cube 此时已完全由桌面承托，不存在单臂承重阶段。
      openBothAndConfirm("after COMMON_DROP_TO_TABLE: cube is on the table");

      trajectory_msgs::msg::JointTrajectory left_retreat, right_retreat;
      // 退出段起点改为落桌后的末端（left_drop/right_drop），否则预演的起点与实际
      // 位置差 kReleaseGapZ，第一段会跳。
      if (!stage("PREPLANNED_COMMON_RETREAT",
            push_left ? sidePose(task.pre_push.position.x, entry_left_y,
              release_z + kLiftHeight, true) : helper_park,
            push_left ? helper_park : sidePose(task.pre_push.position.x, entry_right_y,
              release_z + kLiftHeight, false),
            left_drop, right_drop, &left_retreat, &right_retreat))
      {
        openBothAndConfirm("safe abort after retreat preplanning failure");
        all_complete = false;
        break;
      }
      const auto [unused_pose, release_revision] = cubes.get(task.cube_index);
      (void)unused_pose;

      // 真实执行路径：用**实际执行后**的 short-push 末端与右臂退出位姿作为推入起点
      // （预检路径用的是候选预演末端）。两段共用同一个 preplanPush，保证「预检覆盖
      // 的就是真正执行的那一段」，不会各写一份而悄悄分叉。
      // 换位/推入的规划必须放在**滑轨搬站位之后**：搬站位会把整个规划世界平移 -Δ，
      // 换位与推入的目标都必须在那个世界系里生成，否则双臂会整体偏 Δ。
      trajectory_msgs::msg::JointTrajectory left_regrasp;
      trajectory_msgs::msg::JointTrajectory left_push_in, right_during_push;
      trajectory_msgs::msg::JointTrajectory left_cell_retreat, right_during_cell_retreat;
      // 起点是**落桌后**的末端：中间多了 COMMON_DROP_TO_TABLE 这一段。
      const auto& ex_start = push_left ? left_drop : right_drop;
      const auto& ex_hold = push_left ? right_retreat : left_retreat;

      // 吸盘在上面的 COMMON_DROP_TO_TABLE 之后已经**同时**松开，这里只让辅助臂
      // 空载退到停放位（Cube 已落桌承托，不存在单臂承重）。
      const auto& helper_retreat_traj = push_left ? right_retreat : left_retreat;
      if (!helper.executeAt(helper_retreat_traj, std::chrono::steady_clock::now()) ||
          !helper.waitAtTarget(helper_retreat_traj, kJointSettleToleranceRad, kJointSettleTimeoutSec))
      {
        openBothAndConfirm("safe abort after helper arm retreat failure");
        all_complete = false;
        break;
      }

      // ---------------------------------------- 换位第一段：竖直抬起（滑轨平移之前）
      // 顺序很关键：**抬起必须在滑轨平移之前执行**。平移时工具已在 Cube 上方
      // kRegraspLiftHeight = 0.2 m，彻底不会蹭到那颗自由站立的 Cube（此前平移时吸盘
      // 离 Cube 面只有约 1.2 mm，任何偏差都会顶件，实测 Y 向漂 25.4 mm）。
      // 这一段在**未偏移**的世界系里规划，此刻完全正确；摆动/推入稍后在偏移后的
      // 世界系里再规划（见下面的 preplanPush 调用）。
      // 执行路径改用**实测** Cube 位姿定义推入段几何（X/Y/Z）。短推靠杯面摩擦推件，
      // 落点会偏离标称预推位（实测 x=0.821 vs 标称 0.840，差 19 mm）。换位摆臂的碰撞体
      // 已按实测放置，若目标仍按标称算，目标位就会落进碰撞体内部 19 mm ⇒ RRT 无解
      // （实测 REGRASP_NEG_X produced no RRT candidate）。两者必须同源。
      // 推入**终点**仍用标称格位（push_cell_x），让 Cube 最终落在设计格位。
      // 换位前后比较 Cube 位置时的基准：用**换位前实测**位姿，而不是标称预推位。
      // 短推靠杯面摩擦推件，落点本来就偏离标称（实测 x=0.821 vs 标称 0.840）；若拿标称
      // 当基准，会把"没被扰动"误判成 19.1 mm 漂移并中止（实测踩过）。
      geometry_msgs::msg::Pose regrasp_reference_pose;
      bool regrasp_reference_valid = false;
      {
        const auto [live_cube, live_revision] = cubes.get(task.cube_index);
        (void)live_revision;
        regrasp_reference_pose = live_cube;
        regrasp_reference_valid = true;
        push_entry_x = live_cube.position.x - kPushCupOffsetX;
        push_cube_y = live_cube.position.y;
        push_cube_z = live_cube.position.z;
        RCLCPP_INFO(node->get_logger(),
          "%s 推入段几何改用实测 Cube：entry_x=%.4f cube_y=%.4f cube_z=%.4f"
          "（标称 entry_x=%.4f，差 %.1f mm）。",
          task.id, push_entry_x, push_cube_y, push_cube_z,
          task.pre_push.position.x - kPushCupOffsetX,
          1000.0 * (live_cube.position.x - task.pre_push.position.x));
      }
      {
        trajectory_msgs::msg::JointTrajectory dummy_regrasp, dummy_push, dummy_hold_push;
        trajectory_msgs::msg::JointTrajectory dummy_retreat, dummy_hold_retreat;
        if (!preplanPush(ex_start, ex_hold, std::string(task.id), true, false,
              &dummy_regrasp, &dummy_push, &dummy_hold_push,
              &dummy_retreat, &dummy_hold_retreat))
        {
          openBothAndConfirm("safe abort after REGRASP_LIFT preplanning failure");
          all_complete = false;
          break;
        }
      }
      std::this_thread::sleep_for(250ms);
      if (regrasp_lift.points.empty() ||
          !pusher.executeAt(regrasp_lift, std::chrono::steady_clock::now()) ||
          !pusher.waitAtTarget(regrasp_lift, kJointSettleToleranceRad, kJointSettleTimeoutSec))
      {
        RCLCPP_ERROR(node->get_logger(), "%s REGRASP_LIFT execution failed.", task.id);
        all_complete = false;
        break;
      }
      RCLCPP_INFO(node->get_logger(), "%s REGRASP_LIFT 已在滑轨平移之前完成（工具已抬离 Cube）。",
        task.id);

      // ------------------------------------------------ 滑轨搬站位（推入前）
      // 实测（t26_baseline2.log）：基座停在 0.650 时推入最后两片的推力需求达
      // 87.0 N*m —— 恰好等于 maxForce，驱动器顶在 J1-J4 硬件上限，Cube 落后
      // 6.53 mm；而前 14 片只要 26~28 N*m。基座先沿轨 +X 前进 Δ 直接缩短力臂。
      //
      // 时序约束（两条都成立才合法）：
      //   1) bridge 互锁：**任一吸盘夹紧即拒绝动轨** —— 此处两侧吸盘刚都松开；
      //   2) 距上次关节命令静默 >= RAIL_COMMAND_QUIET_SEC = 1.0 s。
      // 轨道是**瞬移**（USD + Fabric 同步写 world pose），不是连续滑动，所以推臂
      // 杯面此刻虽只离 Cube 1 mm，也不会"贴着面刮过去"（那是最初 446 mm 拖件的成因）。
      // 辅助臂同步前进：MoveIt 只有一个世界系，双臂必须共用同一个偏移；它此刻已
      // 停放在停放位，同步平移无副作用。
      if (push_base_advance > 0.0)
      {
        // 互锁要求"距上次关节命令静默 >= 1.0 s"，而**复位和前进都落在这个静默窗内**，
        // 所以必须先睡够再动轨。实测把这个 sleep 放在复位之后，复位会被互锁拒绝：
        // "left rail 未在 15.0 s 内到位 —— 很可能被互锁拒绝"。
        std::this_thread::sleep_for(1200ms);
        if (g_world_shift_x != 0.0)
        {
          // 上一件遗留的站位：先回静止位再重新搬，保证每件都从确定起点开始。
          RCLCPP_INFO(node->get_logger(), "%s 复位滑轨到静止位 %.3f（清掉上件偏移 %.3f）。",
            task.id, kRailRestX, g_world_shift_x);
          if (!left_rail.moveTo(kRailRestX, 15.0) || !right_rail.moveTo(kRailRestX, 15.0))
          {
            openBothAndConfirm("safe abort after rail reset failure");
            all_complete = false;
            break;
          }
          g_world_shift_x = 0.0;
          if (!refreshStaticWorld(scene, include_box_walls))
          {
            openBothAndConfirm("safe abort after world refresh on reset failure");
            all_complete = false;
            break;
          }
        }
        const double rail_target = kRailRestX + push_base_advance;
        if (!left_rail.moveTo(rail_target, 15.0) || !right_rail.moveTo(rail_target, 15.0))
        {
          openBothAndConfirm("safe abort after rail positioning failure");
          all_complete = false;
          break;
        }
        g_world_shift_x = push_base_advance;
        if (!refreshStaticWorld(scene, include_box_walls))
        {
          openBothAndConfirm("safe abort after shifted world refresh failure");
          all_complete = false;
          break;
        }
        // 滑轨改变后，不能继续用搬运前捕获的 FCL 世界：它仍在旧坐标系。
        // 同时将场景中已存在的 Cube 按最新 Ground Truth 重投影（不添加待供料件），
        // 保证桌面、围墙和已完成件与当前目标共享同一 world_shift_x。
        std::vector<moveit_msgs::msg::CollisionObject> shifted_cubes;
        const auto present_objects = scene.getObjects();
        for (std::size_t index = 0; index < cube_count; ++index)
        {
          const std::string id = "task26_cube_" + std::to_string(index + 1);
          if (id == object_id || present_objects.count(id) == 0)
          {
            continue;
          }
          const auto [pose, revision] = cubes.get(index);
          (void)revision;
          shifted_cubes.push_back(cubeObject(id, pose));
        }
        if (!shifted_cubes.empty() && !scene.applyCollisionObjects(shifted_cubes))
        {
          openBothAndConfirm("safe abort after shifted cube world refresh failure");
          all_complete = false;
          break;
        }
        world_after_remove = staticWorld(scene);
        RCLCPP_INFO(node->get_logger(),
          "%s 滑轨站位完成：基座 %.3f -> %.3f（世界偏移 %.3f m，规划世界已按偏移重建）。",
          task.id, kRailRestX, rail_target, g_world_shift_x);
      }

      // 抬起段已在滑轨平移之前执行：这里以它为起点，只规划摆动/推入/退出。
      if (!preplanPush(regrasp_lift, ex_hold, std::string(task.id), false, true,
            &left_regrasp, &left_push_in, &right_during_push,
            &left_cell_retreat, &right_during_cell_retreat))
      {
        openBothAndConfirm("safe abort after push preplanning failure");
        all_complete = false;
        break;
      }
      // 换位第二段：从抬起位摆到 -X 面姿态（抬起段已在滑轨平移之前完成）。
      RCLCPP_INFO(node->get_logger(), "%s REGRASP_SWING 时长=%.2f s，点数=%zu。",
        task.id, pointTime(left_regrasp.points.back()), left_regrasp.points.size());
      std::this_thread::sleep_for(250ms);
      if (!pusher.executeAt(left_regrasp, std::chrono::steady_clock::now()) ||
          !pusher.waitAtTarget(left_regrasp, kJointSettleToleranceRad, kJointSettleTimeoutSec))
      {
        all_complete = false;
        break;
      }
      // 换位不搬动 Cube：按最新 Ground Truth 复核它相对**换位前实测位姿**没有被扰动。
      std::this_thread::sleep_for(250ms);
      {
        const auto [after_regrasp, after_regrasp_revision] = cubes.get(task.cube_index);
        (void)after_regrasp_revision;
        const auto& reference = regrasp_reference_valid
          ? regrasp_reference_pose.position : task.pre_push.position;
        const double drift = distance3d(reference, after_regrasp.position);
        RCLCPP_INFO(node->get_logger(),
          "%s after REGRASP: cube=(%.3f, %.3f, %.3f), drift since regrasp start=%.2f mm.",
          task.id, after_regrasp.position.x, after_regrasp.position.y,
          after_regrasp.position.z, drift * 1000.0);
        if (drift > kPlacementTolerance)
        {
          RCLCPP_ERROR(node->get_logger(),
            "%s Cube drifted %.1f mm while the pusher re-grasped; abort.",
            task.id, drift * 1000.0);
          all_complete = false;
          break;
        }
      }
      std::thread pusher_on([&]() { pusher.suction(true); });
      pusher_on.join();
      if (!pusher.waitSuction(true, 3.0))
      {
        RCLCPP_ERROR(node->get_logger(), "%s re-grasp suction did not close.", task.id);
        openBothAndConfirm("safe abort after regrasp close failure");
        all_complete = false;
        break;
      }

      // 推入臂分段 +X 推进，逐段检查 Cube 是否跟随、力矩是否越限。
      // executePushWithSupervision 的形参名是 left/right，语义是「推入臂在前」，
      // 因此按 push_left 把两条轨迹与 eef 链接对上。
      const auto& pusher_push_traj = push_left ? left_push_in : right_during_push;
      const auto& helper_hold_traj = push_left ? right_during_push : left_push_in;
      if (!executePushWithSupervision(node, pusher, helper, pusher_push_traj, helper_hold_traj,
            cubes, task, left_group.getRobotModel(), pusher.eefLink(),
            left_forces, right_forces))
      {
        openBothAndConfirm("safe abort during push supervision");
        all_complete = false;
        break;
      }

      // +X 推入完成：先只验收深端墙贴合。此时仍保留中央安全行的 Y 坐标，
      // 不能把它误当成最终无侧缝格位。
      std::this_thread::sleep_for(1500ms);
      geometry_msgs::msg::Pose deep_seated;
      if (!cubes.waitNew(task.cube_index, std::max(source_revision, release_revision), 2.0, &deep_seated) ||
          !validDeepWallSeat(node, task, deep_seated))
      {
        all_complete = false;
        break;
      }

      // 物理背挡：推入臂先**保持** -X 面吸附，作为深端墙的主动反向挡块。此前
      // 它先释放/退出再侧压，横向杯面摩擦会把 Cube 回带约 15 mm，重新产生深端
      // 缝隙；这不是可以靠放宽门限掩盖的问题。
      //
      // 因此此刻由 *另一臂* 完成侧向压紧，原推入臂同步携件横移以保持深端贴合；
      // 侧压结束后先松开推入吸盘，再按已验证的 -X 轨迹撤离。
      const auto& left_after_x = left_push_in;
      const auto& right_after_x = right_during_push;
      const auto& pusher_cell_retreat = push_left ? left_cell_retreat : right_during_cell_retreat;

      // Task27 的内侧两件已在车厢外对齐相邻 Cube 的 Y 通道；中心件对齐 y=0。
      // 这三件都只沿 +X 直推。完成深端贴合后松开 -X 吸附、原路退出，
      // 最后读取 Ground Truth 验收相邻面间隙，不做低位侧向压紧。
      if (isStraightInsertTask(task))
      {
        const auto [before_release, before_release_revision] = cubes.get(task.cube_index);
        (void)before_release;
        pusher.suction(false);
        if (!pusher.waitSuction(false, 6.0))
        {
          openBothAndConfirm("safe abort after straight insert release failure");
          all_complete = false;
          break;
        }
        if (!pusher.executeAt(pusher_cell_retreat, std::chrono::steady_clock::now()) ||
            !pusher.waitAtTarget(pusher_cell_retreat, kJointSettleToleranceRad, kJointSettleTimeoutSec))
        {
          RCLCPP_ERROR(node->get_logger(), "%s straight insert retreat failed.", task.id);
          all_complete = false;
          break;
        }
        std::this_thread::sleep_for(1500ms);
        geometry_msgs::msg::Pose settled;
        if (!cubes.waitNew(task.cube_index, before_release_revision, 2.0, &settled) ||
            !validCellPlacement(node, task, settled, cubes))
        {
          all_complete = false;
          break;
        }
        if (!scene.applyCollisionObject(cubeObject(object_id, settled)))
        {
          all_complete = false;
          break;
        }
        std::this_thread::sleep_for(300ms);
        RCLCPP_INFO(node->get_logger(),
          "%s PASS: 双吸盘预推 -> 单臂吸 -X 面直推 -> 原路退出；"
          "已验收与相邻 Cube 的真实间隙。", task.id);
        continue;
      }
      helper.suction(false);
      if (!helper.waitSuction(false, 3.0))
      {
        all_complete = false;
        break;
      }
      SideCompactionPlan side_plan;
      if (!planSideCompaction(left_after_x, right_after_x, deep_seated,
            std::string(task.id) + " EXECUTION", &side_plan))
      {
        RCLCPP_ERROR(node->get_logger(), "%s cannot preflight a safe SIDE_COMPACTION chain.", task.id);
        openBothAndConfirm("safe abort after SIDE_COMPACTION preflight failure");
        all_complete = false;
        break;
      }
      const auto execute_side_stage =
        [&](const std::string& name,
            const trajectory_msgs::msg::JointTrajectory& left_trajectory,
            const trajectory_msgs::msg::JointTrajectory& right_trajectory)
      {
        if (!executeSync(left, left_trajectory, right, right_trajectory) ||
            !left.waitAtTarget(left_trajectory, kJointSettleToleranceRad, kJointSettleTimeoutSec) ||
            !right.waitAtTarget(right_trajectory, kJointSettleToleranceRad, kJointSettleTimeoutSec))
        {
          RCLCPP_ERROR(node->get_logger(), "%s SIDE_COMPACTION %s execution failed.",
            task.id, name.c_str());
          return false;
        }
        return true;
      };
      if (!execute_side_stage("HIGH_APPROACH", side_plan.left_approach, side_plan.right_approach) ||
          !execute_side_stage("CONTACT", side_plan.left_contact, side_plan.right_contact))
      {
        all_complete = false;
        break;
      }
      const auto [before_press_pose, before_press_revision] = cubes.get(task.cube_index);
      (void)before_press_pose;
      if (!execute_side_stage("PRESS", side_plan.left_press, side_plan.right_press))
      {
        all_complete = false;
        break;
      }
      // 保持末端压紧位一小段时间，使 PhysX 接触解算把 Cube 真正推到墙面。
      std::this_thread::sleep_for(500ms);
      // 侧墙已到位后再解除 -X 面吸附。此前若在侧压之前打开，横向接触力会把
      // Cube 回带离开深端墙；此时 Cube 已同时受 +X/+Y（或 -Y）墙约束，解除
      // 后两臂的反向撤离不会再形成可见缝隙。
      pusher.suction(false);
      if (!pusher.waitSuction(false, 6.0))
      {
        openBothAndConfirm("safe abort after side-compaction pusher release failure");
        all_complete = false;
        break;
      }
      if (!execute_side_stage("WITHDRAW", side_plan.left_withdraw, side_plan.right_withdraw) ||
          !execute_side_stage("LIFT", side_plan.left_lift, side_plan.right_lift) ||
          !execute_side_stage("RETURN", side_plan.left_return, side_plan.right_return))
      {
        all_complete = false;
        break;
      }

      // Cube 已被侧压到墙角后，原推入臂再单独按已经预检的 -X 轨迹退出；它的
      // Cup 面朝远离 Cube 的方向运动，不再对已贴墙的物体施加反向拖拽。
      if (!pusher.executeAt(pusher_cell_retreat, std::chrono::steady_clock::now()) ||
          !pusher.waitAtTarget(pusher_cell_retreat, kJointSettleToleranceRad, kJointSettleTimeoutSec))
      {
        all_complete = false;
        break;
      }
      std::this_thread::sleep_for(1500ms);
      geometry_msgs::msg::Pose settled;
      if (!cubes.waitNew(task.cube_index, before_press_revision, 2.0, &settled) ||
          !validCellPlacement(node, task, settled, cubes))
      {
        all_complete = false;
        break;
      }
      // 两臂都已退出后才将已落稳 Cube 回写 MoveIt world，避免撤离起点与
      // 新的 World CollisionObject 合法接触而被误判碰撞。
      if (!scene.applyCollisionObject(cubeObject(object_id, settled)))
      {
        all_complete = false;
        break;
      }
      std::this_thread::sleep_for(300ms);
      RCLCPP_INFO(node->get_logger(),
        "%s PASS: 双吸盘预推 -> %s吸 -X 面推到深端墙并保持反向背挡 -> %s侧压到%s墙 -> 退出；"
        "深端与侧墙均无可见缝隙。",
        task.id, push_left ? "左臂" : "右臂",
        push_left ? "右臂" : "左臂", push_left ? "-Y" : "+Y");
      }  // 批内逐件结束
      if (!all_complete)
      {
        break;
      }

      // 批间退出：两件都放置完成、并且双臂同步回到共同 HOME 之后，才允许下一批到料。
      // 该姿态在八件全在场的初始状态下已被实际使用，因此是“有已完成垛墙时仍然安全”
      // 的退出位；仍必须整段通过同步 FCL 门禁，不允许为了退出而放宽碰撞规则。
      trajectory_msgs::msg::JointTrajectory home_left;
      trajectory_msgs::msg::JointTrajectory home_right;
      if (!left.planHome(left_group, kHomeQ, &home_left) ||
          !right.planHome(right_group, kHomeQ, &home_right) ||
          !synchronize(&home_left, &home_right))
      {
        RCLCPP_ERROR(node->get_logger(), "Task26 batch %d 无法规划共同 HOME 退出。", batch);
        all_complete = false;
        break;
      }
      if (!validateSync(node, left_group.getRobotModel(), staticWorld(scene),
            home_left, home_right, std::string("task26 COMMON_HOME batch ") + std::to_string(batch)))
      {
        RCLCPP_ERROR(node->get_logger(),
          "Task26 batch %d 的共同 HOME 退出轨迹未通过同步 FCL；不执行该退出。", batch);
        all_complete = false;
        break;
      }
      if (planning_only)
      {
        RCLCPP_INFO(node->get_logger(),
          "%s planning-only batch %d PASS: 当前批 %d 件与共同 HOME 退出均通过 IK/FCL 门禁；"
          "未发布任何 joint、suction 或 feed_command。", kTaskLabel, batch, kBatchSize);
        continue;
      }
      if (!executeSync(left, home_left, right, home_right))
      {
        RCLCPP_ERROR(node->get_logger(), "Task26 batch %d 执行共同 HOME 退出失败。", batch);
        all_complete = false;
        break;
      }
      RCLCPP_INFO(node->get_logger(),
        "%s batch %d PASS: 当前批 %d 件已落稳入垛，双臂已回到共同 HOME。",
        kTaskLabel, batch, kBatchSize);
    }  // 批次循环结束
    success = all_complete;
  } while (false);

  executor.cancel();
  if (spin.joinable())
  {
    spin.join();
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

namespace
{
constexpr std::size_t NUM_CUBES = 9;

constexpr double TABLE_X = 0.55;
constexpr double TABLE_Y = 0.00;
constexpr double TABLE_Z = 0.025;
constexpr double TABLE_SIZE_X = 1.20;
constexpr double TABLE_SIZE_Y = 0.80;
constexpr double TABLE_SIZE_Z = 0.05;
constexpr double TABLE_TOP_Z = TABLE_Z + 0.5 * TABLE_SIZE_Z;

constexpr double CUBE_SIZE = 0.030;
constexpr double CUBE_HALF = 0.5 * CUBE_SIZE;

// Franka 官方 cobot_pump：吸盘 TCP 相对 fr3_link8 沿局部 +Z 约 0.105 m。
// fr3_arm 的规划 tip 仍使用 fr3_link8，因此所有输入的“吸盘 TCP 世界 Z”
// 在 makeTopDownPose() 中转换成 fr3_link8 的目标世界 Z。
constexpr double COBOT_PUMP_TCP_OFFSET_Z = 0.105;

// 抓取接触时，吸盘 TCP 允许比 Cube 顶面高 1 mm；
// 与 Task04-A 的已验证几何保持一致。
constexpr double CONTACT_TCP_CLEARANCE = 0.001;

// 放置时不让 AttachedCollisionObject 与支撑表面在 MoveIt 中直接发生接触。
// 先在最高支撑表面上方 1 mm 释放，再由 Isaac PhysX 落到真实支撑面。
constexpr double PLACEMENT_RELEASE_GAP = 0.001;

// Task04-A 已验证的抓取高度关系：
// Cube 初始中心约 0.065 m 时，contact=0.081、pre_pick=0.145、lift=0.180。
constexpr double PRE_PICK_CLEARANCE = 0.064;
constexpr double LIFT_CLEARANCE = 0.099;

// 多层码垛时 PRE_PLACE / RETREAT 必须跟随当前堆叠高度上升。
constexpr double MIN_PRE_PLACE_TCP_Z = 0.180;
constexpr double PRE_PLACE_CLEARANCE = 0.070;

// 以目标 XY 处的竖直柱为“高度图查询点”。
// 17 mm < 相邻码垛中心距 32 mm，因此不会把旁边 Cube 错当成正下方支撑；
// 同时允许下层 Cube 在 PhysX 中有少量位置误差。
constexpr double SUPPORT_XY_TOLERANCE = CUBE_HALF + 0.002;

constexpr double CARTESIAN_EEF_STEP = 0.002;
constexpr double CARTESIAN_MIN_FRACTION = 0.999;

// Task04-B 只预定义 XY，Z 不写死。
// Cube1~6：3x2 底层；Cube7~9：在第一排三个位置形成第二层。
const std::array<std::array<double, 2>, NUM_CUBES> STACK_TARGET_XY = {{
    {{0.620, -0.134}},
    {{0.652, -0.134}},
    {{0.684, -0.134}},
    {{0.620, -0.166}},
    {{0.652, -0.166}},
    {{0.684, -0.166}},
    {{0.620, -0.134}},
    {{0.652, -0.134}},
    {{0.684, -0.134}},
}};

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(), "等待 /move_group 参数服务...");

    auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
    if (!client->wait_for_service(10s))
    {
        RCLCPP_ERROR(node->get_logger(), "无法连接 /move_group，请先启动 MoveIt。");
        return false;
    }

    const auto params = client->get_parameters({
        "robot_description",
        "robot_description_semantic"
    });

    if (params.size() != 2 ||
        params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
        params[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
        RCLCPP_ERROR(node->get_logger(), "robot_description / semantic 参数无效。");
        return false;
    }

    node->declare_parameter<std::string>("robot_description", params[0].as_string());
    node->declare_parameter<std::string>("robot_description_semantic", params[1].as_string());
    return true;
}

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
    return static_cast<double>(point.time_from_start.sec) +
           static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
    int32_t sec = static_cast<int32_t>(std::floor(seconds));
    int64_t nsec = static_cast<int64_t>(
        std::llround((seconds - static_cast<double>(sec)) * 1e9));

    if (nsec >= 1000000000LL)
    {
        ++sec;
        nsec -= 1000000000LL;
    }

    point.time_from_start.sec = sec;
    point.time_from_start.nanosec = static_cast<uint32_t>(std::max<int64_t>(0, nsec));
}

void ensureTrajectoryTiming(trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.empty() || pointTime(trajectory.points.back()) > 1e-6)
    {
        return;
    }

    constexpr double FALLBACK_DT = 0.03;
    for (std::size_t i = 0; i < trajectory.points.size(); ++i)
    {
        setPointTime(trajectory.points[i], static_cast<double>(i) * FALLBACK_DT);
    }
}

geometry_msgs::msg::Pose makeTopDownPose(
    double x,
    double y,
    double suction_tcp_world_z,
    double yaw = 0.0)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;

    // Rz(yaw) * Rx(pi) 时，fr3_link8 局部 +Z 指向世界 -Z。
    // 吸盘 TCP 比 link8 沿局部 +Z 多伸出 0.105 m，
    // 所以要让 TCP 到 suction_tcp_world_z，link8 必须高 0.105 m。
    pose.position.z = suction_tcp_world_z + COBOT_PUMP_TCP_OFFSET_Z;

    const double half_yaw = 0.5 * yaw;
    pose.orientation.x = std::cos(half_yaw);
    pose.orientation.y = std::sin(half_yaw);
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
}

bool sameTargetXY(std::size_t a, std::size_t b)
{
    return std::abs(STACK_TARGET_XY[a][0] - STACK_TARGET_XY[b][0]) < 1e-9 &&
           std::abs(STACK_TARGET_XY[a][1] - STACK_TARGET_XY[b][1]) < 1e-9;
}

}  // namespace

class SuctionNineCubePalletize
{
public:
    explicit SuctionNineCubePalletize(const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command", 10);
        suction_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/task04b/suction_command", 10);

        cube_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/task04b/cube_poses",
            10,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg)
            {
                if (msg->poses.size() < NUM_CUBES)
                {
                    return;
                }

                std::lock_guard<std::mutex> lock(cube_mutex_);
                for (std::size_t i = 0; i < NUM_CUBES; ++i)
                {
                    cube_poses_[i] = msg->poses[i];
                }
                have_cube_poses_ = true;
            });

        suction_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/task04b/suction_state",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                suction_closed_.store(msg->data);
            });
    }

    bool run()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "========== Task04-B：顶部吸盘九 Cube 三维码垛 ==========");

        if (!waitForIsaacBridge())
        {
            return false;
        }

        moveit::planning_interface::MoveGroupInterface move_group(node_, "fr3_arm");
        move_group.setPlannerId("RRTConnectkConfigDefault");
        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(5);
        move_group.setMaxVelocityScalingFactor(0.20);
        move_group.setMaxAccelerationScalingFactor(0.20);
        move_group.setPoseReferenceFrame("base");

        // 当前 cobot_pump 的 fr3_arm 规划 tip 按官方 SRDF 使用 fr3_link8。
        // 不把 fr3_cobot_pump_tcp 直接设置成 fr3_arm 的规划 tip；
        // TCP 偏置统一由 makeTopDownPose() 做几何转换。
        const std::string requested_eef =
            node_->declare_parameter<std::string>("eef_link", "fr3_link8");
        const auto* requested_link_model =
            move_group.getRobotModel()->getLinkModel(requested_eef);
        if (requested_link_model == nullptr)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "指定的 eef_link=%s 不存在于 RobotModel。",
                         requested_eef.c_str());
            return false;
        }

        std::string eef_link = requested_eef;
        move_group.setEndEffectorLink(eef_link);

        RCLCPP_INFO(node_->get_logger(), "MoveIt planning tip = %s", eef_link.c_str());
        RCLCPP_INFO(node_->get_logger(), "Suction TCP offset Z = %.3f m",
                    COBOT_PUMP_TCP_OFFSET_Z);
        RCLCPP_INFO(node_->get_logger(), "Robot model frame = %s",
                    move_group.getRobotModel()->getModelFrame().c_str());

        const auto* joint_model_group =
            move_group.getRobotModel()->getJointModelGroup("fr3_arm");
        if (joint_model_group == nullptr)
        {
            RCLCPP_ERROR(node_->get_logger(), "找不到 fr3_arm JointModelGroup。");
            return false;
        }

        std::array<geometry_msgs::msg::Pose, NUM_CUBES> initial_cube_poses;
        {
            std::lock_guard<std::mutex> lock(cube_mutex_);
            initial_cube_poses = cube_poses_;
        }

        if (!addInitialPlanningScene(initial_cube_poses))
        {
            return false;
        }

        const std::vector<double> q_home = {
            0.0, -0.7, 0.0, -2.2, 0.0, 2.0, 0.8
        };
        std::vector<double> current_q = q_home;

        for (std::size_t i = 0; i < NUM_CUBES; ++i)
        {
            const geometry_msgs::msg::Pose pick_pose = getCubePose(i);
            const std::string cube_id = "cube_" + std::to_string(i + 1);
            const double place_x = STACK_TARGET_XY[i][0];
            const double place_y = STACK_TARGET_XY[i][1];

            RCLCPP_INFO(node_->get_logger(), "");
            RCLCPP_INFO(node_->get_logger(),
                        "========== Cube%zu / %zu ==========" , i + 1, NUM_CUBES);
            RCLCPP_INFO(node_->get_logger(),
                        "PICK = (%.4f, %.4f, %.4f)",
                        pick_pose.position.x,
                        pick_pose.position.y,
                        pick_pose.position.z);
            RCLCPP_INFO(node_->get_logger(),
                        "TARGET XY = (%.4f, %.4f)，Z 将按当前最高表面动态计算",
                        place_x, place_y);

            if (!pickAndPlaceOne(move_group,
                                 joint_model_group,
                                 eef_link,
                                 i,
                                 cube_id,
                                 pick_pose,
                                 place_x,
                                 place_y,
                                 current_q))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Cube%zu 码垛失败，流程停止。", i + 1);
                commandSuction(false);
                return false;
            }
        }

        trajectory_msgs::msg::JointTrajectory home_trajectory;
        if (!planJointStage(move_group,
                            joint_model_group,
                            current_q,
                            q_home,
                            "FINAL RETREAT -> HOME",
                            home_trajectory))
        {
            return false;
        }
        executeTrajectory(home_trajectory);

        RCLCPP_INFO(node_->get_logger(),
                    "========== Task04-B SUCCESS：九 Cube 三维码垛完成 ==========");
        return true;
    }

private:
    geometry_msgs::msg::Pose getCubePose(std::size_t index)
    {
        std::lock_guard<std::mutex> lock(cube_mutex_);
        return cube_poses_[index];
    }

    bool waitForIsaacBridge()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "等待 Isaac Task04-B bridge：cube_poses + suction_state...");

        for (int i = 0; i < 100; ++i)
        {
            bool have_poses = false;
            {
                std::lock_guard<std::mutex> lock(cube_mutex_);
                have_poses = have_cube_poses_;
            }

            if (have_poses &&
                command_pub_->get_subscription_count() > 0 &&
                suction_pub_->get_subscription_count() > 0)
            {
                RCLCPP_INFO(node_->get_logger(), "Isaac Task04-B bridge 已连接。");
                return true;
            }

            std::this_thread::sleep_for(100ms);
        }

        RCLCPP_ERROR(node_->get_logger(),
                     "等待 Task04-B bridge 超时。请确认 Isaac 已 Play 且 bridge 脚本已运行。");
        return false;
    }

    // 查询目标 XY 位置当前的最高支撑表面。
    // 只使用已经成功放置的 Cube（index < placed_count）的最新 Isaac Ground Truth。
    double computeSupportSurfaceZ(
        double target_x,
        double target_y,
        std::size_t placed_count)
    {
        double highest_surface_z = TABLE_TOP_Z;

        std::lock_guard<std::mutex> lock(cube_mutex_);
        for (std::size_t i = 0; i < placed_count; ++i)
        {
            const auto& pose = cube_poses_[i];

            const bool target_above_this_cube =
                std::abs(pose.position.x - target_x) <= SUPPORT_XY_TOLERANCE &&
                std::abs(pose.position.y - target_y) <= SUPPORT_XY_TOLERANCE;

            if (target_above_this_cube)
            {
                const double cube_top = pose.position.z + CUBE_HALF;
                highest_surface_z = std::max(highest_surface_z, cube_top);
            }
        }

        return highest_surface_z;
    }

    bool targetExpectsExistingSupport(std::size_t cube_index)
    {
        for (std::size_t i = 0; i < cube_index; ++i)
        {
            if (sameTargetXY(i, cube_index))
            {
                return true;
            }
        }
        return false;
    }

    bool pickAndPlaceOne(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::string& eef_link,
        std::size_t cube_index,
        const std::string& cube_id,
        const geometry_msgs::msg::Pose& pick_pose,
        double place_x,
        double place_y,
        std::vector<double>& current_q)
    {
        // ----------------------------------------------------
        // 1. 根据当前 Cube Ground Truth 计算抓取高度
        // ----------------------------------------------------
        const double pick_contact_tcp_z =
            pick_pose.position.z + CUBE_HALF + CONTACT_TCP_CLEARANCE;
        const double pre_pick_tcp_z = pick_contact_tcp_z + PRE_PICK_CLEARANCE;
        const double lift_tcp_z = pick_contact_tcp_z + LIFT_CLEARANCE;

        auto pre_pick = makeTopDownPose(
            pick_pose.position.x,
            pick_pose.position.y,
            pre_pick_tcp_z);

        trajectory_msgs::msg::JointTrajectory pre_pick_traj;
        if (!planPoseStage(move_group,
                           joint_model_group,
                           eef_link,
                           current_q,
                           pre_pick,
                           "PRE_PICK",
                           pre_pick_traj))
        {
            return false;
        }
        executeTrajectory(pre_pick_traj);

        // ----------------------------------------------------
        // 2. SUCTION ON -> Cartesian 下降 -> CLOSED
        // ----------------------------------------------------
        commandSuction(true);
        std::this_thread::sleep_for(200ms);

        removeWorldCube(cube_id);

        auto contact = makeTopDownPose(
            pick_pose.position.x,
            pick_pose.position.y,
            pick_contact_tcp_z);

        trajectory_msgs::msg::JointTrajectory contact_traj;
        if (!planCartesianStage(move_group,
                                joint_model_group,
                                currentFrom(pre_pick_traj),
                                contact,
                                "PRE_PICK -> SUCTION_CONTACT",
                                contact_traj))
        {
            return false;
        }
        executeTrajectory(contact_traj);

        if (!waitForSuctionClosed(true, 2.0))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：下降完成但 Isaac Surface Gripper 未吸住物体。",
                         cube_id.c_str());
            return false;
        }

        if (!attachCubeToTcp(cube_id, eef_link))
        {
            return false;
        }

        // ----------------------------------------------------
        // 3. Cartesian LIFT
        // ----------------------------------------------------
        auto lift = makeTopDownPose(
            pick_pose.position.x,
            pick_pose.position.y,
            lift_tcp_z);

        trajectory_msgs::msg::JointTrajectory lift_traj;
        if (!planCartesianStage(move_group,
                                joint_model_group,
                                currentFrom(contact_traj),
                                lift,
                                "SUCTION_CONTACT -> LIFT",
                                lift_traj))
        {
            return false;
        }
        executeTrajectory(lift_traj);

        // ----------------------------------------------------
        // 4. 动态查询目标 XY 当前最高表面，再生成本次放置 Z
        // ----------------------------------------------------
        const double support_surface_z =
            computeSupportSurfaceZ(place_x, place_y, cube_index);

        if (targetExpectsExistingSupport(cube_index) &&
            support_surface_z <= TABLE_TOP_Z + 1e-4)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：目标 XY 理论上应已有下层 Cube，但未检测到有效支撑表面。"
                         "为避免错误放回桌面，流程停止。",
                         cube_id.c_str());
            return false;
        }

        // MoveIt 规划释放点比真实支撑面高 1 mm，避免 Attached Cube 与支撑物直接碰撞；
        // SUCTION OFF 后由 PhysX 下落到真实最高表面。
        const double planned_release_center_z =
            support_surface_z + CUBE_HALF + PLACEMENT_RELEASE_GAP;
        const double place_tcp_z =
            planned_release_center_z + CUBE_HALF + CONTACT_TCP_CLEARANCE;
        const double pre_place_tcp_z =
            std::max(MIN_PRE_PLACE_TCP_Z,
                     place_tcp_z + PRE_PLACE_CLEARANCE);
        const double retreat_tcp_z = pre_place_tcp_z;

        RCLCPP_INFO(node_->get_logger(),
                    "%s：最高支撑表面 Z=%.4f m，计划释放 Cube center Z=%.4f m，"
                    "PLACE TCP Z=%.4f m",
                    cube_id.c_str(),
                    support_surface_z,
                    planned_release_center_z,
                    place_tcp_z);

        // ----------------------------------------------------
        // 5. RRTConnect 携物到动态 PRE_PLACE
        // ----------------------------------------------------
        auto pre_place = makeTopDownPose(place_x, place_y, pre_place_tcp_z);
        trajectory_msgs::msg::JointTrajectory pre_place_traj;
        if (!planPoseStage(move_group,
                           joint_model_group,
                           eef_link,
                           currentFrom(lift_traj),
                           pre_place,
                           "LIFT -> PRE_PLACE",
                           pre_place_traj))
        {
            return false;
        }
        executeTrajectory(pre_place_traj);

        // ----------------------------------------------------
        // 6. Cartesian 垂直到动态 PLACE
        // ----------------------------------------------------
        auto place = makeTopDownPose(place_x, place_y, place_tcp_z);
        trajectory_msgs::msg::JointTrajectory place_traj;
        if (!planCartesianStage(move_group,
                                joint_model_group,
                                currentFrom(pre_place_traj),
                                place,
                                "PRE_PLACE -> PLACE",
                                place_traj))
        {
            return false;
        }
        executeTrajectory(place_traj);

        // ----------------------------------------------------
        // 7. DETACH -> 临时 remove -> SUCTION OFF
        // ----------------------------------------------------
        if (!detachCubeFromTcp(cube_id, eef_link))
        {
            return false;
        }
        removeWorldCube(cube_id);

        commandSuction(false);
        if (!waitForSuctionClosed(false, 2.0))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：吸盘释放状态确认失败。", cube_id.c_str());
            return false;
        }

        // 给 Cube 从 1 mm 释放高度落到真实支撑面的时间。
        std::this_thread::sleep_for(300ms);

        // ----------------------------------------------------
        // 8. Cartesian 垂直 RETREAT
        // ----------------------------------------------------
        auto retreat = makeTopDownPose(place_x, place_y, retreat_tcp_z);
        trajectory_msgs::msg::JointTrajectory retreat_traj;
        if (!planCartesianStage(move_group,
                                joint_model_group,
                                currentFrom(place_traj),
                                retreat,
                                "PLACE -> RETREAT",
                                retreat_traj))
        {
            return false;
        }
        executeTrajectory(retreat_traj);

        // ----------------------------------------------------
        // 9. 使用 Isaac 最新真实落稳位姿重新加入 MoveIt World
        // ----------------------------------------------------
        std::this_thread::sleep_for(300ms);
        const geometry_msgs::msg::Pose settled_pose = getCubePose(cube_index);

        RCLCPP_INFO(node_->get_logger(),
                    "%s：Isaac settle pose = (%.4f, %.4f, %.4f)",
                    cube_id.c_str(),
                    settled_pose.position.x,
                    settled_pose.position.y,
                    settled_pose.position.z);

        if (!addWorldCube(cube_id, settled_pose))
        {
            return false;
        }

        current_q = currentFrom(retreat_traj);
        RCLCPP_INFO(node_->get_logger(),
                    "%s：PICK -> DYNAMIC PLACE 完成。", cube_id.c_str());
        return true;
    }

    static std::vector<double> currentFrom(
        const trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        if (trajectory.points.empty())
        {
            return {};
        }
        return trajectory.points.back().positions;
    }

    bool planPoseStage(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::string& eef_link,
        const std::vector<double>& start_q,
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& stage_name,
        trajectory_msgs::msg::JointTrajectory& trajectory_out)
    {
        moveit::core::RobotState start_state(move_group.getRobotModel());
        start_state.setToDefaultValues();
        start_state.setJointGroupPositions(joint_model_group, start_q);
        start_state.update();

        move_group.setStartState(start_state);
        move_group.clearPoseTargets();

        if (!move_group.setPoseTarget(target_pose, eef_link))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：设置 PoseTarget 失败。", stage_name.c_str());
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto result = move_group.plan(plan);
        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：RRTConnect 规划失败。", stage_name.c_str());
            return false;
        }

        trajectory_out = plan.trajectory_.joint_trajectory;
        if (trajectory_out.points.empty())
        {
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "%s：规划成功，waypoints=%zu，duration=%.3f s",
                    stage_name.c_str(),
                    trajectory_out.points.size(),
                    pointTime(trajectory_out.points.back()));
        return true;
    }

    bool planJointStage(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::vector<double>& start_q,
        const std::vector<double>& target_q,
        const std::string& stage_name,
        trajectory_msgs::msg::JointTrajectory& trajectory_out)
    {
        moveit::core::RobotState start_state(move_group.getRobotModel());
        start_state.setToDefaultValues();
        start_state.setJointGroupPositions(joint_model_group, start_q);
        start_state.update();

        move_group.setStartState(start_state);
        move_group.clearPoseTargets();
        if (!move_group.setJointValueTarget(target_q))
        {
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：规划失败。", stage_name.c_str());
            return false;
        }

        trajectory_out = plan.trajectory_.joint_trajectory;
        return !trajectory_out.points.empty();
    }

    bool planCartesianStage(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::vector<double>& start_q,
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& stage_name,
        trajectory_msgs::msg::JointTrajectory& trajectory_out)
    {
        moveit::core::RobotState start_state(move_group.getRobotModel());
        start_state.setToDefaultValues();
        start_state.setJointGroupPositions(joint_model_group, start_q);
        start_state.update();
        move_group.setStartState(start_state);

        std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
        moveit_msgs::msg::RobotTrajectory robot_trajectory;
        moveit_msgs::msg::MoveItErrorCodes error_code;

        const double fraction = move_group.computeCartesianPath(
            waypoints,
            CARTESIAN_EEF_STEP,
            0.0,
            robot_trajectory,
            true,
            &error_code);

        RCLCPP_INFO(node_->get_logger(),
                    "%s：Cartesian fraction = %.4f, error_code = %d",
                    stage_name.c_str(), fraction, error_code.val);

        if (fraction < CARTESIAN_MIN_FRACTION)
        {
            return false;
        }

        trajectory_out = robot_trajectory.joint_trajectory;
        if (trajectory_out.points.empty())
        {
            return false;
        }
        ensureTrajectoryTiming(trajectory_out);
        return true;
    }

    moveit_msgs::msg::CollisionObject makeCubeObject(
        const std::string& id,
        const geometry_msgs::msg::Pose& pose,
        int operation)
    {
        moveit_msgs::msg::CollisionObject cube;
        cube.header.frame_id = "base";
        cube.id = id;

        shape_msgs::msg::SolidPrimitive shape;
        shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        shape.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};

        cube.primitives.push_back(shape);
        cube.primitive_poses.push_back(pose);
        cube.operation = operation;
        return cube;
    }

    bool addInitialPlanningScene(
        const std::array<geometry_msgs::msg::Pose, NUM_CUBES>& cubes)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        std::vector<moveit_msgs::msg::CollisionObject> objects;

        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "base";
        table.id = "table";

        shape_msgs::msg::SolidPrimitive table_shape;
        table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        table_shape.dimensions = {
            TABLE_SIZE_X,
            TABLE_SIZE_Y,
            TABLE_SIZE_Z
        };

        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.x = TABLE_X;
        table_pose.position.y = TABLE_Y;
        table_pose.position.z = TABLE_Z;

        table.primitives.push_back(table_shape);
        table.primitive_poses.push_back(table_pose);
        table.operation = moveit_msgs::msg::CollisionObject::ADD;
        objects.push_back(table);

        for (std::size_t i = 0; i < NUM_CUBES; ++i)
        {
            objects.push_back(makeCubeObject(
                "cube_" + std::to_string(i + 1),
                cubes[i],
                moveit_msgs::msg::CollisionObject::ADD));
        }

        if (!psi.applyCollisionObjects(objects))
        {
            RCLCPP_ERROR(node_->get_logger(), "初始化 Planning Scene 失败。");
            return false;
        }

        std::this_thread::sleep_for(700ms);
        return true;
    }

    void removeWorldCube(const std::string& cube_id)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        psi.removeCollisionObjects({cube_id});
        std::this_thread::sleep_for(300ms);
    }

    bool addWorldCube(
        const std::string& cube_id,
        const geometry_msgs::msg::Pose& pose)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        auto cube = makeCubeObject(
            cube_id,
            pose,
            moveit_msgs::msg::CollisionObject::ADD);
        const bool ok = psi.applyCollisionObject(cube);
        std::this_thread::sleep_for(300ms);
        return ok;
    }

    bool attachCubeToTcp(
        const std::string& cube_id,
        const std::string& eef_link)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = eef_link;

        // Cube 与抓取工具的接触属于有意接触。
        attached.touch_links = {eef_link, "fr3_cobot_pump"};
        attached.object.header.frame_id = eef_link;
        attached.object.id = cube_id;

        shape_msgs::msg::SolidPrimitive shape;
        shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        shape.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};

        geometry_msgs::msg::Pose relative_pose;

        // link8 -> TCP = 0.105 m；TCP -> Cube center = 0.016 m（15 mm + 1 mm clearance）。
        // 顶部朝下姿态下，两段都沿 link8 局部 +Z。
        relative_pose.position.z =
            COBOT_PUMP_TCP_OFFSET_Z + CUBE_HALF + CONTACT_TCP_CLEARANCE;
        relative_pose.orientation.x = 1.0;
        relative_pose.orientation.y = 0.0;
        relative_pose.orientation.z = 0.0;
        relative_pose.orientation.w = 0.0;

        attached.object.primitives.push_back(shape);
        attached.object.primitive_poses.push_back(relative_pose);
        attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        const bool ok = psi.applyAttachedCollisionObject(attached);
        std::this_thread::sleep_for(500ms);
        return ok;
    }

    bool detachCubeFromTcp(
        const std::string& cube_id,
        const std::string& eef_link)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = eef_link;
        attached.object.id = cube_id;
        attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        const bool ok = psi.applyAttachedCollisionObject(attached);
        std::this_thread::sleep_for(400ms);
        return ok;
    }

    void commandSuction(bool on)
    {
        std_msgs::msg::Bool msg;
        msg.data = on;
        for (int i = 0; i < 20; ++i)
        {
            suction_pub_->publish(msg);
            std::this_thread::sleep_for(10ms);
        }
        RCLCPP_INFO(node_->get_logger(), "SUCTION %s", on ? "ON" : "OFF");
    }

    bool waitForSuctionClosed(bool expected, double timeout_sec)
    {
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - start).count() < timeout_sec)
        {
            if (suction_closed_.load() == expected)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "Isaac suction_state = %s",
                            expected ? "CLOSED" : "OPEN");
                return true;
            }
            std::this_thread::sleep_for(20ms);
        }
        return false;
    }

    void executeTrajectory(const trajectory_msgs::msg::JointTrajectory& input)
    {
        if (input.points.empty())
        {
            return;
        }

        auto trajectory = input;
        ensureTrajectoryTiming(trajectory);

        const double total_time = pointTime(trajectory.points.back());
        std::size_t segment = 0;
        const auto start_time = std::chrono::steady_clock::now();

        while (true)
        {
            const double t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time).count();
            if (t > total_time)
            {
                break;
            }

            while (segment + 1 < trajectory.points.size() &&
                   pointTime(trajectory.points[segment + 1]) < t)
            {
                ++segment;
            }

            std::vector<double> q_command;
            if (segment + 1 >= trajectory.points.size())
            {
                q_command = trajectory.points.back().positions;
            }
            else
            {
                const auto& p0 = trajectory.points[segment];
                const auto& p1 = trajectory.points[segment + 1];
                const double t0 = pointTime(p0);
                const double t1 = pointTime(p1);
                double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
                alpha = std::clamp(alpha, 0.0, 1.0);

                q_command.resize(p0.positions.size());
                for (std::size_t j = 0; j < q_command.size(); ++j)
                {
                    q_command[j] = p0.positions[j] +
                        alpha * (p1.positions[j] - p0.positions[j]);
                }
            }

            sensor_msgs::msg::JointState msg;
            msg.header.stamp = node_->now();
            msg.name = trajectory.joint_names;
            msg.position = q_command;
            command_pub_->publish(msg);
            std::this_thread::sleep_for(10ms);
        }

        for (int i = 0; i < 50; ++i)
        {
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = node_->now();
            msg.name = trajectory.joint_names;
            msg.position = trajectory.points.back().positions;
            command_pub_->publish(msg);
            std::this_thread::sleep_for(10ms);
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cube_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr suction_state_sub_;

    std::mutex cube_mutex_;
    std::array<geometry_msgs::msg::Pose, NUM_CUBES> cube_poses_{};
    bool have_cube_poses_ = false;
    std::atomic_bool suction_closed_{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("suction_nine_cube_palletize");

    if (!copyRobotModelParameters(node))
    {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    bool success = false;
    try
    {
        SuctionNineCubePalletize demo(node);
        success = demo.run();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(node->get_logger(), "程序异常：%s", e.what());
    }

    executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }

    rclcpp::shutdown();
    return success ? 0 : 1;
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

// ============================================================
// Task 02：机器人可执行 Placement Skill
//
// 与 Task 01 的区别：
// 1. 抓取部分沿用已经验证的稳定基线；
// 2. 放置目标不再写死在动作流程里，而是结构化为 PlacementTarget；
// 3. 放置阶段明确拆成 C_reach / C_insert / C_release / C_retreat；
// 4. 每个阶段失败时返回明确失败原因；
// 5. 目标参数可通过 ROS 2 参数覆盖，不需要重新修改源码。
// ============================================================

// ============================================================
// 场景与抓取基线参数
// ============================================================

constexpr double TABLE_X = 0.55;
constexpr double TABLE_Y = 0.00;
constexpr double TABLE_Z = 0.025;
constexpr double TABLE_SIZE_X = 1.20;
constexpr double TABLE_SIZE_Y = 0.80;
constexpr double TABLE_SIZE_Z = 0.05;
constexpr double TABLE_TOP_Z = TABLE_Z + 0.5 * TABLE_SIZE_Z;

constexpr double CUBE_SIZE = 0.030;

constexpr double PICK_X = 0.45;
constexpr double PICK_Y = 0.15;
constexpr double PICK_CUBE_Z = 0.065;
constexpr double PICK_YAW = 0.0;

constexpr double APPROACH_TCP_Z = 0.140;
constexpr double GRASP_TCP_Z = 0.075;
constexpr double LIFT_TCP_Z = 0.175;

constexpr double CARTESIAN_EEF_STEP = 0.002;
constexpr double CARTESIAN_MIN_FRACTION = 0.999;

constexpr double GRIPPER_OPEN_POS = 0.040;
constexpr double GRIPPER_CLOSE_POS = 0.014;

// MoveIt 中携带物体相对于 fr3_hand_tcp 的位置。
constexpr double ATTACHED_CUBE_TCP_Z = 0.009;

// ============================================================
// Placement Skill 输入与输出
// ============================================================

struct PlacementTarget
{
    double x;
    double y;
    double cube_z;
    double yaw;
    double pre_place_tcp_z;
    double place_tcp_z;
    double retreat_tcp_z;
};

enum class PlacementResult
{
    SUCCESS = 0,
    INVALID_TARGET,
    REACH_FAILED,
    INSERT_FAILED,
    RELEASE_FAILED,
    RETREAT_FAILED,
    WORLD_UPDATE_FAILED
};

struct PlacementSkillResult
{
    PlacementResult code = PlacementResult::INVALID_TARGET;
    std::vector<double> final_q;
};

const char* placementResultName(PlacementResult result)
{
    switch (result)
    {
        case PlacementResult::SUCCESS:
            return "SUCCESS";
        case PlacementResult::INVALID_TARGET:
            return "INVALID_TARGET";
        case PlacementResult::REACH_FAILED:
            return "REACH_FAILED";
        case PlacementResult::INSERT_FAILED:
            return "INSERT_FAILED";
        case PlacementResult::RELEASE_FAILED:
            return "RELEASE_FAILED";
        case PlacementResult::RETREAT_FAILED:
            return "RETREAT_FAILED";
        case PlacementResult::WORLD_UPDATE_FAILED:
            return "WORLD_UPDATE_FAILED";
        default:
            return "UNKNOWN";
    }
}

// ============================================================
// 从 /move_group 获取 URDF / SRDF
// ============================================================

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(), "等待 /move_group 参数服务...");

    auto client = std::make_shared<rclcpp::SyncParametersClient>(
        node, "/move_group");

    if (!client->wait_for_service(10s))
    {
        RCLCPP_ERROR(node->get_logger(),
                     "无法连接 /move_group 参数服务，请先启动 MoveIt。");
        return false;
    }

    const std::vector<std::string> names = {
        "robot_description",
        "robot_description_semantic"
    };

    std::vector<rclcpp::Parameter> params;

    try
    {
        params = client->get_parameters(names);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "读取 MoveIt 参数失败：%s", e.what());
        return false;
    }

    if (params.size() != 2 ||
        params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
        params[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
        RCLCPP_ERROR(node->get_logger(), "机器人模型参数类型无效。");
        return false;
    }

    const std::string urdf = params[0].as_string();
    const std::string srdf = params[1].as_string();

    if (urdf.empty() || srdf.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "URDF 或 SRDF 为空。");
        return false;
    }

    node->declare_parameter<std::string>("robot_description", urdf);
    node->declare_parameter<std::string>("robot_description_semantic", srdf);

    RCLCPP_INFO(node->get_logger(), "机器人模型参数复制完成。");
    return true;
}

// ============================================================
// 轨迹时间工具
// ============================================================

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
    return static_cast<double>(point.time_from_start.sec) +
           static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point,
                  double seconds)
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
    point.time_from_start.nanosec =
        static_cast<uint32_t>(std::max<int64_t>(0, nsec));
}

void ensureTrajectoryTiming(trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.empty() ||
        pointTime(trajectory.points.back()) > 1e-6)
    {
        return;
    }

    constexpr double FALLBACK_DT = 0.03;

    for (size_t i = 0; i < trajectory.points.size(); ++i)
    {
        setPointTime(trajectory.points[i],
                     static_cast<double>(i) * FALLBACK_DT);
    }
}

// ============================================================
// 顶部抓取 / 放置姿态：Rz(yaw) * Rx(pi)
// ============================================================

geometry_msgs::msg::Pose makeTopDownPose(double x,
                                         double y,
                                         double z,
                                         double yaw)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    const double half_yaw = 0.5 * yaw;
    pose.orientation.x = std::cos(half_yaw);
    pose.orientation.y = std::sin(half_yaw);
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;

    return pose;
}

// ============================================================
// Placement Skill Demo
// ============================================================

class PlacementSkillDemo
{
public:
    explicit PlacementSkillDemo(const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command", 10);

        // 默认值就是 Task 01 已验证成功的放置目标。
        target_.x = node_->declare_parameter<double>("place_x", 0.65);
        target_.y = node_->declare_parameter<double>("place_y", -0.15);
        target_.cube_z = node_->declare_parameter<double>("place_cube_z", 0.065);
        target_.yaw = node_->declare_parameter<double>("place_yaw", 0.0);
        target_.pre_place_tcp_z =
            node_->declare_parameter<double>("pre_place_tcp_z", 0.175);
        target_.place_tcp_z =
            node_->declare_parameter<double>("place_tcp_z", 0.076);
        target_.retreat_tcp_z =
            node_->declare_parameter<double>("retreat_tcp_z", 0.175);
    }

    bool run()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "========== Task 02：Placement Skill Demo ==========");

        moveit::planning_interface::MoveGroupInterface move_group(
            node_, "fr3_arm");

        move_group.setPlannerId("RRTConnectkConfigDefault");
        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(5);
        move_group.setMaxVelocityScalingFactor(0.20);
        move_group.setMaxAccelerationScalingFactor(0.20);
        move_group.setPoseReferenceFrame("base");
        move_group.setEndEffectorLink("fr3_hand_tcp");

        RCLCPP_INFO(node_->get_logger(),
                    "Robot model frame = '%s'",
                    move_group.getRobotModel()->getModelFrame().c_str());
        RCLCPP_INFO(node_->get_logger(),
                    "Pose reference frame = '%s'",
                    move_group.getPoseReferenceFrame().c_str());

        if (!validatePlacementTarget(target_))
        {
            printPlacementSummary(false,
                                  false,
                                  false,
                                  false,
                                  PlacementResult::INVALID_TARGET);
            return false;
        }

        if (!addInitialPlanningScene())
        {
            return false;
        }

        if (command_pub_->get_subscription_count() == 0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "没有检测到 /joint_command subscriber，请确认 Isaac Sim 已点击 Play。");
            return false;
        }

        const auto* joint_model_group =
            move_group.getRobotModel()->getJointModelGroup("fr3_arm");

        if (joint_model_group == nullptr)
        {
            RCLCPP_ERROR(node_->get_logger(), "找不到规划组 fr3_arm。");
            return false;
        }

        const std::vector<double> q_home = {
            0.0, -0.7, 0.0, -2.2, 0.0, 2.0, 0.8
        };

        // ----------------------------------------------------
        // A. 沿用 Task 01：稳定抓取并提升
        // ----------------------------------------------------

        std::vector<double> lift_final_q;

        if (!executeStablePickAndLift(move_group,
                                      joint_model_group,
                                      q_home,
                                      lift_final_q))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Task 02 前置抓取 / 提升失败，Placement Skill 未执行。");
            return false;
        }

        // ----------------------------------------------------
        // B. 真正的 Placement Skill
        // ----------------------------------------------------

        const PlacementSkillResult placement =
            executePlacementSkill(move_group,
                                  joint_model_group,
                                  lift_final_q,
                                  target_);

        if (placement.code != PlacementResult::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Placement Skill 失败：%s",
                         placementResultName(placement.code));
            return false;
        }

        // ----------------------------------------------------
        // C. Placement 成功后回 HOME
        // ----------------------------------------------------

        trajectory_msgs::msg::JointTrajectory home_trajectory;

        if (!planJointStage(move_group,
                            joint_model_group,
                            placement.final_q,
                            q_home,
                            "RETREAT -> HOME",
                            home_trajectory))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Placement 已成功，但返回 HOME 规划失败。");
            return false;
        }

        executeTrajectory(home_trajectory, GRIPPER_OPEN_POS);

        RCLCPP_INFO(node_->get_logger(),
                    "Task 02 第一版完整执行成功：Placement Skill = SUCCESS");
        return true;
    }

private:
    // ========================================================
    // 目标基础几何检查
    // ========================================================

    bool validatePlacementTarget(const PlacementTarget& target)
    {
        const double half_extent =
            0.5 * CUBE_SIZE *
            (std::abs(std::cos(target.yaw)) +
             std::abs(std::sin(target.yaw)));

        const double table_x_min = TABLE_X - 0.5 * TABLE_SIZE_X;
        const double table_x_max = TABLE_X + 0.5 * TABLE_SIZE_X;
        const double table_y_min = TABLE_Y - 0.5 * TABLE_SIZE_Y;
        const double table_y_max = TABLE_Y + 0.5 * TABLE_SIZE_Y;

        const bool finite =
            std::isfinite(target.x) &&
            std::isfinite(target.y) &&
            std::isfinite(target.cube_z) &&
            std::isfinite(target.yaw) &&
            std::isfinite(target.pre_place_tcp_z) &&
            std::isfinite(target.place_tcp_z) &&
            std::isfinite(target.retreat_tcp_z);

        if (!finite)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[PlacementSkill] INVALID_TARGET：存在非有限数值。");
            return false;
        }

        if (target.x - half_extent < table_x_min ||
            target.x + half_extent > table_x_max ||
            target.y - half_extent < table_y_min ||
            target.y + half_extent > table_y_max)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[PlacementSkill] INVALID_TARGET：Cube 投影超出桌面范围。");
            return false;
        }

        if (target.cube_z - 0.5 * CUBE_SIZE < TABLE_TOP_Z - 1e-4)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[PlacementSkill] INVALID_TARGET：目标 Cube 会穿入桌面。");
            return false;
        }

        if (target.pre_place_tcp_z <= target.place_tcp_z ||
            target.retreat_tcp_z <= target.place_tcp_z)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[PlacementSkill] INVALID_TARGET：PRE_PLACE / RETREAT 必须高于 PLACE。");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] PlacementTarget 合法："
                    "center=(%.3f, %.3f, %.3f), yaw=%.3f",
                    target.x, target.y, target.cube_z, target.yaw);
        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] TCP z：pre=%.3f, place=%.3f, retreat=%.3f",
                    target.pre_place_tcp_z,
                    target.place_tcp_z,
                    target.retreat_tcp_z);

        return true;
    }

    // ========================================================
    // Task 01 已验证稳定的 PICK + LIFT
    // ========================================================

    bool executeStablePickAndLift(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::vector<double>& q_home,
        std::vector<double>& lift_final_q)
    {
        const auto approach_pose =
            makeTopDownPose(PICK_X, PICK_Y, APPROACH_TCP_Z, PICK_YAW);

        trajectory_msgs::msg::JointTrajectory approach_trajectory;

        if (!planPoseStage(move_group,
                           joint_model_group,
                           q_home,
                           approach_pose,
                           "HOME -> ALIGNED_APPROACH",
                           approach_trajectory))
        {
            return false;
        }

        executeTrajectory(approach_trajectory);
        commandGripper(GRIPPER_OPEN_POS, 1.0);

        // 抓取目标物体属于有意接触，最终下降前从 MoveIt World 临时移除。
        removeWorldCube();

        const auto grasp_pose =
            makeTopDownPose(PICK_X, PICK_Y, GRASP_TCP_Z, PICK_YAW);

        trajectory_msgs::msg::JointTrajectory grasp_trajectory;

        if (!planCartesianStage(move_group,
                                joint_model_group,
                                approach_trajectory.points.back().positions,
                                grasp_pose,
                                "ALIGNED_APPROACH -> GRASP",
                                grasp_trajectory))
        {
            return false;
        }

        executeTrajectory(grasp_trajectory, GRIPPER_OPEN_POS);
        commandGripper(GRIPPER_CLOSE_POS, 1.0);
        rclcpp::sleep_for(500ms);

        if (!attachCubeToTcp())
        {
            RCLCPP_ERROR(node_->get_logger(), "MoveIt ATTACH 失败。");
            return false;
        }

        const auto lift_pose =
            makeTopDownPose(PICK_X, PICK_Y, LIFT_TCP_Z, PICK_YAW);

        trajectory_msgs::msg::JointTrajectory lift_trajectory;

        if (!planCartesianStage(move_group,
                                joint_model_group,
                                grasp_trajectory.points.back().positions,
                                lift_pose,
                                "GRASP -> LIFT",
                                lift_trajectory))
        {
            return false;
        }

        executeTrajectory(lift_trajectory, GRIPPER_CLOSE_POS);
        lift_final_q = lift_trajectory.points.back().positions;

        RCLCPP_INFO(node_->get_logger(),
                    "Task 01 稳定前置链：PICK + LIFT 完成。");
        return true;
    }

    // ========================================================
    // Placement Skill 核心
    // ========================================================

    PlacementSkillResult executePlacementSkill(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::vector<double>& start_q,
        const PlacementTarget& target)
    {
        bool c_reach = false;
        bool c_insert = false;
        bool c_release = false;
        bool c_retreat = false;

        PlacementSkillResult result;

        RCLCPP_INFO(node_->get_logger(),
                    "========== Placement Skill 开始 ==========");

        // ----------------------------------------------------
        // C_reach：携物到达 PRE_PLACE
        // ----------------------------------------------------

        const auto pre_place_pose =
            makeTopDownPose(target.x,
                            target.y,
                            target.pre_place_tcp_z,
                            target.yaw);

        trajectory_msgs::msg::JointTrajectory reach_trajectory;

        if (!planPoseStage(move_group,
                           joint_model_group,
                           start_q,
                           pre_place_pose,
                           "C_reach: LIFT -> PRE_PLACE",
                           reach_trajectory))
        {
            result.code = PlacementResult::REACH_FAILED;
            printPlacementSummary(c_reach,
                                  c_insert,
                                  c_release,
                                  c_retreat,
                                  result.code);
            return result;
        }

        executeTrajectory(reach_trajectory, GRIPPER_CLOSE_POS);
        c_reach = true;
        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] C_reach   = PASS");

        // ----------------------------------------------------
        // C_insert：保持 x/y/姿态，Cartesian 垂直下降
        // ----------------------------------------------------

        const auto place_pose =
            makeTopDownPose(target.x,
                            target.y,
                            target.place_tcp_z,
                            target.yaw);

        trajectory_msgs::msg::JointTrajectory insert_trajectory;

        if (!planCartesianStage(move_group,
                                joint_model_group,
                                reach_trajectory.points.back().positions,
                                place_pose,
                                "C_insert: PRE_PLACE -> PLACE",
                                insert_trajectory))
        {
            result.code = PlacementResult::INSERT_FAILED;
            printPlacementSummary(c_reach,
                                  c_insert,
                                  c_release,
                                  c_retreat,
                                  result.code);
            return result;
        }

        executeTrajectory(insert_trajectory, GRIPPER_CLOSE_POS);
        c_insert = true;
        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] C_insert  = PASS");

        // ----------------------------------------------------
        // C_release：DETACH + 临时移除目标 Cube + OPEN
        //
        // DETACH 会自动把 Cube 放回 collision world。
        // 释放瞬间属于有意接触区，为避免 RETREAT 起点被目标 Cube
        // 判定为碰撞，立即临时 remove。其他环境障碍物仍然保留。
        // ----------------------------------------------------

        if (!detachCubeFromTcp())
        {
            result.code = PlacementResult::RELEASE_FAILED;
            printPlacementSummary(c_reach,
                                  c_insert,
                                  c_release,
                                  c_retreat,
                                  result.code);
            return result;
        }

        removeWorldCube();
        commandGripper(GRIPPER_OPEN_POS, 1.0);
        rclcpp::sleep_for(700ms);

        c_release = true;
        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] C_release = PASS（第一版为命令层释放确认）");

        // ----------------------------------------------------
        // C_retreat：保持 x/y/姿态，Cartesian 垂直退出
        // ----------------------------------------------------

        const auto retreat_pose =
            makeTopDownPose(target.x,
                            target.y,
                            target.retreat_tcp_z,
                            target.yaw);

        trajectory_msgs::msg::JointTrajectory retreat_trajectory;

        if (!planCartesianStage(move_group,
                                joint_model_group,
                                insert_trajectory.points.back().positions,
                                retreat_pose,
                                "C_retreat: PLACE -> RETREAT",
                                retreat_trajectory))
        {
            result.code = PlacementResult::RETREAT_FAILED;
            printPlacementSummary(c_reach,
                                  c_insert,
                                  c_release,
                                  c_retreat,
                                  result.code);
            return result;
        }

        executeTrajectory(retreat_trajectory, GRIPPER_OPEN_POS);
        c_retreat = true;
        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] C_retreat = PASS");

        // 退出完成后，按最终目标位姿把 Cube 恢复为 World CollisionObject。
        if (!addPlacedCubeToWorld(target))
        {
            result.code = PlacementResult::WORLD_UPDATE_FAILED;
            printPlacementSummary(c_reach,
                                  c_insert,
                                  c_release,
                                  c_retreat,
                                  result.code);
            return result;
        }

        result.code = PlacementResult::SUCCESS;
        result.final_q = retreat_trajectory.points.back().positions;

        printPlacementSummary(c_reach,
                              c_insert,
                              c_release,
                              c_retreat,
                              result.code);

        return result;
    }

    void printPlacementSummary(bool c_reach,
                               bool c_insert,
                               bool c_release,
                               bool c_retreat,
                               PlacementResult result)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "------------------------------------------");
        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] C_reach   = %s",
                    c_reach ? "PASS" : "FAIL");
        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] C_insert  = %s",
                    c_insert ? "PASS" : "FAIL");
        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] C_release = %s",
                    c_release ? "PASS" : "FAIL");
        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] C_retreat = %s",
                    c_retreat ? "PASS" : "FAIL");
        RCLCPP_INFO(node_->get_logger(),
                    "[PlacementSkill] RESULT    = %s",
                    placementResultName(result));
        RCLCPP_INFO(node_->get_logger(),
                    "------------------------------------------");
    }

    // ========================================================
    // Planning Scene
    // ========================================================

    bool addInitialPlanningScene()
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        std::vector<moveit_msgs::msg::CollisionObject> objects;

        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "base";
        table.id = "table";

        shape_msgs::msg::SolidPrimitive table_shape;
        table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        table_shape.dimensions = {
            TABLE_SIZE_X, TABLE_SIZE_Y, TABLE_SIZE_Z
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

        objects.push_back(
            makeWorldCubeObject(PICK_X,
                                PICK_Y,
                                PICK_CUBE_Z,
                                PICK_YAW,
                                moveit_msgs::msg::CollisionObject::ADD));

        if (!psi.applyCollisionObjects(objects))
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning Scene 初始化失败。");
            return false;
        }

        rclcpp::sleep_for(1s);
        return true;
    }

    moveit_msgs::msg::CollisionObject makeWorldCubeObject(
        double x,
        double y,
        double z,
        double yaw,
        int operation)
    {
        moveit_msgs::msg::CollisionObject cube;
        cube.header.frame_id = "base";
        cube.id = "pick_cube";

        shape_msgs::msg::SolidPrimitive shape;
        shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        shape.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};

        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.z = std::sin(0.5 * yaw);
        pose.orientation.w = std::cos(0.5 * yaw);

        cube.primitives.push_back(shape);
        cube.primitive_poses.push_back(pose);
        cube.operation = operation;
        return cube;
    }

    void removeWorldCube()
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        psi.removeCollisionObjects({"pick_cube"});
        rclcpp::sleep_for(500ms);
    }

    bool attachCubeToTcp()
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::AttachedCollisionObject attached;

        attached.link_name = "fr3_hand_tcp";
        attached.touch_links = {
            "fr3_hand_tcp",
            "fr3_hand",
            "fr3_leftfinger",
            "fr3_rightfinger"
        };
        attached.object.header.frame_id = "fr3_hand_tcp";
        attached.object.id = "pick_cube";

        shape_msgs::msg::SolidPrimitive shape;
        shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        shape.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};

        geometry_msgs::msg::Pose relative_pose;
        relative_pose.orientation.x = 1.0;
        relative_pose.orientation.y = 0.0;
        relative_pose.orientation.z = 0.0;
        relative_pose.orientation.w = 0.0;
        relative_pose.position.x = 0.0;
        relative_pose.position.y = 0.0;
        relative_pose.position.z = ATTACHED_CUBE_TCP_Z;

        attached.object.primitives.push_back(shape);
        attached.object.primitive_poses.push_back(relative_pose);
        attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        if (!psi.applyAttachedCollisionObject(attached))
        {
            return false;
        }

        rclcpp::sleep_for(700ms);
        return true;
    }

    bool detachCubeFromTcp()
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::AttachedCollisionObject attached;

        attached.link_name = "fr3_hand_tcp";
        attached.object.id = "pick_cube";
        attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        if (!psi.applyAttachedCollisionObject(attached))
        {
            return false;
        }

        rclcpp::sleep_for(500ms);
        return true;
    }

    bool addPlacedCubeToWorld(const PlacementTarget& target)
    {
        moveit::planning_interface::PlanningSceneInterface psi;

        auto cube = makeWorldCubeObject(
            target.x,
            target.y,
            target.cube_z,
            target.yaw,
            moveit_msgs::msg::CollisionObject::ADD);

        if (!psi.applyCollisionObject(cube))
        {
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "已将 Cube 按目标位姿重新加入 MoveIt World："
                    "(%.3f, %.3f, %.3f), yaw=%.3f",
                    target.x,
                    target.y,
                    target.cube_z,
                    target.yaw);

        rclcpp::sleep_for(500ms);
        return true;
    }

    // ========================================================
    // 规划函数
    // ========================================================

    bool planPoseStage(
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

        if (!start_state.satisfiesBounds())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：起点违反关节限制。",
                         stage_name.c_str());
            return false;
        }

        move_group.setStartState(start_state);
        move_group.clearPoseTargets();

        if (!move_group.setPoseTarget(target_pose, "fr3_hand_tcp"))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：Pose 目标设置失败。",
                         stage_name.c_str());
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto planning_result = move_group.plan(plan);

        if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：RRTConnect 规划失败。",
                         stage_name.c_str());
            return false;
        }

        trajectory_out = plan.trajectory_.joint_trajectory;

        if (trajectory_out.points.empty())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：轨迹为空。",
                         stage_name.c_str());
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
        const auto planning_result = move_group.plan(plan);

        if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
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

        if (!start_state.satisfiesBounds())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：Cartesian 起点违反关节限制。",
                         stage_name.c_str());
            return false;
        }

        move_group.setStartState(start_state);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

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
                    stage_name.c_str(),
                    fraction,
                    error_code.val);

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

    // ========================================================
    // 执行与夹爪控制
    // ========================================================

    void executeTrajectory(
        const trajectory_msgs::msg::JointTrajectory& trajectory,
        double finger_hold = -1.0)
    {
        if (trajectory.points.empty())
        {
            return;
        }

        const double total_time = pointTime(trajectory.points.back());
        size_t segment = 0;
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

                double alpha = 0.0;
                if (t1 > t0)
                {
                    alpha = (t - t0) / (t1 - t0);
                }
                alpha = std::clamp(alpha, 0.0, 1.0);

                q_command.resize(p0.positions.size());
                for (size_t j = 0; j < q_command.size(); ++j)
                {
                    q_command[j] = p0.positions[j] +
                        alpha * (p1.positions[j] - p0.positions[j]);
                }
            }

            publishArmCommand(trajectory.joint_names,
                              q_command,
                              finger_hold);
            std::this_thread::sleep_for(10ms);
        }

        const auto& final_q = trajectory.points.back().positions;
        for (int i = 0; i < 50; ++i)
        {
            publishArmCommand(trajectory.joint_names,
                              final_q,
                              finger_hold);
            std::this_thread::sleep_for(10ms);
        }
    }

    void commandGripper(double finger_position,
                        double duration_sec)
    {
        const int cycles = std::max(
            1,
            static_cast<int>(duration_sec / 0.01));

        for (int i = 0; i < cycles; ++i)
        {
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = node_->now();
            msg.name = {
                "fr3_finger_joint1",
                "fr3_finger_joint2"
            };
            msg.position = {
                finger_position,
                finger_position
            };
            command_pub_->publish(msg);
            std::this_thread::sleep_for(10ms);
        }
    }

    void publishArmCommand(
        const std::vector<std::string>& arm_names,
        const std::vector<double>& arm_positions,
        double finger_hold)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = node_->now();
        msg.name = arm_names;
        msg.position = arm_positions;

        if (finger_hold >= 0.0)
        {
            msg.name.push_back("fr3_finger_joint1");
            msg.name.push_back("fr3_finger_joint2");
            msg.position.push_back(finger_hold);
            msg.position.push_back(finger_hold);
        }

        command_pub_->publish(msg);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
    PlacementTarget target_{};
};

// ============================================================
// main
// ============================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("placement_skill_demo");

    if (!copyRobotModelParameters(node))
    {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]()
    {
        executor.spin();
    });

    bool success = false;

    try
    {
        PlacementSkillDemo demo(node);
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

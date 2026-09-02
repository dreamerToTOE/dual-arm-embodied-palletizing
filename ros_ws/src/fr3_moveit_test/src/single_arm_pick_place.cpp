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
// Task 01：单机械臂完整 Pick & Place
//
// 当前流程：
// HOME
//   -> ALIGNED_APPROACH
//   -> OPEN
//   -> Cartesian GRASP
//   -> CLOSE
//   -> MoveIt ATTACH
//   -> Cartesian LIFT
//   -> Cartesian TRANSFER
//   -> Cartesian PLACE
//   -> MoveIt DETACH
//   -> OPEN / RELEASE
//   -> Cartesian RETREAT
//   -> 把 Cube 重新加入 MoveIt World
//   -> HOME
// ============================================================

// ============================================================
// 场景参数
// ============================================================

constexpr double TABLE_X = 0.55;
constexpr double TABLE_Y = 0.00;
constexpr double TABLE_Z = 0.025;

constexpr double TABLE_SIZE_X = 1.20;
constexpr double TABLE_SIZE_Y = 0.80;
constexpr double TABLE_SIZE_Z = 0.05;

// ------------------------------------------------------------
// PickCube：30 mm
// ------------------------------------------------------------

constexpr double CUBE_SIZE = 0.030;

constexpr double PICK_X = 0.45;
constexpr double PICK_Y = 0.15;
constexpr double PICK_CUBE_Z = 0.065;
constexpr double PICK_YAW = 0.0;

// ------------------------------------------------------------
// Place 目标
// ------------------------------------------------------------

constexpr double PLACE_X = 0.65;
constexpr double PLACE_Y = -0.15;
constexpr double PLACE_CUBE_Z = 0.065;
constexpr double PLACE_YAW = 0.0;

// ============================================================
// TCP 高度
// ============================================================

// 抓取前对齐高度
constexpr double APPROACH_TCP_Z = 0.140;

// 已经多次验证成功的抓取高度
constexpr double GRASP_TCP_Z = 0.075;

// 抓取后提升高度，同时也是搬运高度
constexpr double LIFT_TCP_Z = 0.175;

// 放置时让真实 Cube 底面比桌面高约 1 mm，
// 打开夹爪后自由落下约 1 mm，避免下降时压桌面。
constexpr double PLACE_TCP_Z = 0.076;

// 释放后的退出高度
constexpr double RETREAT_TCP_Z = 0.175;

// ============================================================
// Cartesian Path 参数
// ============================================================

constexpr double CARTESIAN_EEF_STEP = 0.002;
constexpr double CARTESIAN_MIN_FRACTION = 0.999;

// ============================================================
// 夹爪参数
// ============================================================

// 两个 finger joint 各 0.040 -> 总开口约 80 mm
constexpr double GRIPPER_OPEN_POS = 0.040;

// 两个 finger joint 各 0.014 -> 总目标开口约 28 mm
// 对 30 mm Cube 形成轻微预紧。
constexpr double GRIPPER_CLOSE_POS = 0.014;

// ============================================================
// MoveIt Attached Cube 参数
// ============================================================

// 当前抓取时：
// TCP z = 0.075
// Cube center z = 0.065
// 真实 Cube 中心比 TCP 低 10 mm。
//
// MoveIt 内部使用 9 mm，给桌面留约 1 mm 安全间隙。
// TCP +Z 在当前抓取姿态下指向世界 -Z，
// 所以 Cube 在 TCP 坐标系中位于 +Z。
constexpr double ATTACHED_CUBE_TCP_Z = 0.009;

// ============================================================
// 从 /move_group 获取 URDF + SRDF
// ============================================================

bool copyRobotModelParameters(
    const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(
        node->get_logger(),
        "等待 /move_group 参数服务..."
    );

    auto client =
        std::make_shared<rclcpp::SyncParametersClient>(
            node,
            "/move_group"
        );

    if (!client->wait_for_service(10s))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "无法连接 /move_group 参数服务，请先启动 MoveIt。"
        );
        return false;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "已连接 /move_group 参数服务。"
    );

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
        RCLCPP_ERROR(
            node->get_logger(),
            "读取 MoveIt 参数失败：%s",
            e.what()
        );
        return false;
    }

    if (
        params.size() != 2 ||
        params[0].get_type() !=
            rclcpp::ParameterType::PARAMETER_STRING ||
        params[1].get_type() !=
            rclcpp::ParameterType::PARAMETER_STRING)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "robot_description / SRDF 参数类型无效。"
        );
        return false;
    }

    const std::string urdf = params[0].as_string();
    const std::string srdf = params[1].as_string();

    if (urdf.empty() || srdf.empty())
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "URDF 或 SRDF 为空。"
        );
        return false;
    }

    node->declare_parameter<std::string>(
        "robot_description",
        urdf
    );

    node->declare_parameter<std::string>(
        "robot_description_semantic",
        srdf
    );

    RCLCPP_INFO(
        node->get_logger(),
        "机器人模型参数复制完成。"
    );

    return true;
}

// ============================================================
// trajectory 时间工具
// ============================================================

double pointTime(
    const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
    return
        static_cast<double>(point.time_from_start.sec) +
        static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void setPointTime(
    trajectory_msgs::msg::JointTrajectoryPoint& point,
    double seconds)
{
    int32_t sec =
        static_cast<int32_t>(std::floor(seconds));

    int64_t nsec =
        static_cast<int64_t>(
            std::llround(
                (seconds - static_cast<double>(sec)) * 1e9
            )
        );

    if (nsec >= 1000000000LL)
    {
        ++sec;
        nsec -= 1000000000LL;
    }

    point.time_from_start.sec = sec;
    point.time_from_start.nanosec =
        static_cast<uint32_t>(
            std::max<int64_t>(0, nsec)
        );
}

void ensureTrajectoryTiming(
    trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (
        trajectory.points.empty() ||
        pointTime(trajectory.points.back()) > 1e-6)
    {
        return;
    }

    // 正常情况下 Cartesian Path 服务会进行时间参数化。
    // 如果某些配置只返回位置点，则使用 30 ms / point 兜底。
    constexpr double FALLBACK_DT = 0.03;

    for (size_t i = 0; i < trajectory.points.size(); ++i)
    {
        setPointTime(
            trajectory.points[i],
            static_cast<double>(i) * FALLBACK_DT
        );
    }
}

// ============================================================
// 构造顶部抓取姿态
//
// R = Rz(yaw) * Rx(pi)
//
// yaw = 0 时：
// quaternion = (1, 0, 0, 0)
//
// 结果：
// TCP +Z -> 世界 -Z
// finger 局部 Y -> 世界 -Y
//
// 因此末端朝下，并且两指方向与轴对齐 Cube 侧面平齐。
// ============================================================

geometry_msgs::msg::Pose makeTopDownPose(
    double x,
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
// 单机械臂 Pick & Place
// ============================================================

class SingleArmPickPlace
{
public:
    explicit SingleArmPickPlace(
        const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        command_pub_ =
            node_->create_publisher<sensor_msgs::msg::JointState>(
                "/joint_command",
                10
            );
    }

    bool run()
    {
        // ====================================================
        // MoveGroupInterface
        // ====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "创建 MoveGroupInterface..."
        );

        moveit::planning_interface::MoveGroupInterface move_group(
            node_,
            "fr3_arm"
        );

        move_group.setPlannerId(
            "RRTConnectkConfigDefault"
        );

        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(5);

        move_group.setMaxVelocityScalingFactor(0.20);
        move_group.setMaxAccelerationScalingFactor(0.20);

        // 关键修复：统一使用 RobotModel frame = base。
        move_group.setPoseReferenceFrame("base");
        move_group.setEndEffectorLink("fr3_hand_tcp");

        RCLCPP_INFO(
            node_->get_logger(),
            "MoveGroupInterface 创建完成。"
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "Robot model frame = '%s'",
            move_group.getRobotModel()->getModelFrame().c_str()
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "Pose reference frame = '%s'",
            move_group.getPoseReferenceFrame().c_str()
        );

        // ====================================================
        // Planning Scene
        // ====================================================

        if (!addInitialPlanningScene())
        {
            return false;
        }

        // ====================================================
        // 检查 Isaac /joint_command subscriber
        // ====================================================

        if (command_pub_->get_subscription_count() == 0)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "没有检测到 /joint_command subscriber。请确认 Isaac Sim 已点击 Play。"
            );
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "已检测到 /joint_command subscriber。"
        );

        // ====================================================
        // fr3_arm
        // ====================================================

        const auto* joint_model_group =
            move_group
                .getRobotModel()
                ->getJointModelGroup("fr3_arm");

        if (joint_model_group == nullptr)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "找不到规划组 fr3_arm。"
            );
            return false;
        }

        // ====================================================
        // HOME
        // ====================================================

        const std::vector<double> q_home = {
             0.0,
            -0.7,
             0.0,
            -2.2,
             0.0,
             2.0,
             0.8
        };

        // ====================================================
        // 1. HOME -> ALIGNED_APPROACH
        //
        // 只有从 HOME 到抓取区域使用 RRTConnect。
        // 到达后末端已经完成完整姿态对齐。
        // ====================================================

        const auto approach_pose =
            makeTopDownPose(
                PICK_X,
                PICK_Y,
                APPROACH_TCP_Z,
                PICK_YAW
            );

        trajectory_msgs::msg::JointTrajectory approach_trajectory;

        if (!planPoseStage(
                move_group,
                joint_model_group,
                q_home,
                approach_pose,
                "HOME -> ALIGNED_APPROACH",
                approach_trajectory))
        {
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "执行 ALIGNED_APPROACH..."
        );

        executeTrajectory(approach_trajectory);

        RCLCPP_INFO(
            node_->get_logger(),
            "ALIGNED_APPROACH 执行完成。"
        );

        // ====================================================
        // 2. OPEN_GRIPPER
        // ====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "打开夹爪..."
        );

        commandGripper(
            GRIPPER_OPEN_POS,
            1.0
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "夹爪已打开。"
        );

        // ====================================================
        // 3. 从 MoveIt World 删除 Cube
        //
        // 这是为了允许夹爪和 Cube 有意接触。
        // Isaac 中真实 Cube 不会被删除。
        // ====================================================

        removeWorldCube();

        // ====================================================
        // 4. ALIGNED_APPROACH -> GRASP
        //
        // X、Y、orientation 不变，只沿世界 Z 轴下降。
        // ====================================================

        const std::vector<double> approach_final_q =
            approach_trajectory.points.back().positions;

        const auto grasp_pose =
            makeTopDownPose(
                PICK_X,
                PICK_Y,
                GRASP_TCP_Z,
                PICK_YAW
            );

        trajectory_msgs::msg::JointTrajectory grasp_trajectory;

        if (!planCartesianStage(
                move_group,
                joint_model_group,
                approach_final_q,
                grasp_pose,
                "ALIGNED_APPROACH -> GRASP",
                grasp_trajectory))
        {
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "执行 Cartesian 垂直抓取下降..."
        );

        executeTrajectory(
            grasp_trajectory,
            GRIPPER_OPEN_POS
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "抓取下降执行完成。"
        );

        // ====================================================
        // 5. CLOSE_GRIPPER
        // ====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "闭合夹爪..."
        );

        commandGripper(
            GRIPPER_CLOSE_POS,
            1.0
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "夹爪闭合完成。"
        );

        // 等待 PhysX 接触稳定
        rclcpp::sleep_for(500ms);

        // ====================================================
        // 6. MoveIt ATTACH
        //
        // Isaac 仍然依靠真实 finger / Cube 接触。
        // MoveIt 中使用 AttachedCollisionObject 做携物碰撞检测。
        // ====================================================

        if (!attachCubeToTcp())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MoveIt ATTACH 失败。"
            );
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "MoveIt ATTACH 完成。"
        );

        // ====================================================
        // 7. GRASP -> LIFT
        //
        // 保持抓取姿态，只沿 Z 轴上升。
        // ====================================================

        const std::vector<double> grasp_final_q =
            grasp_trajectory.points.back().positions;

        const auto lift_pose =
            makeTopDownPose(
                PICK_X,
                PICK_Y,
                LIFT_TCP_Z,
                PICK_YAW
            );

        trajectory_msgs::msg::JointTrajectory lift_trajectory;

        if (!planCartesianStage(
                move_group,
                joint_model_group,
                grasp_final_q,
                lift_pose,
                "GRASP -> LIFT",
                lift_trajectory))
        {
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "执行 Cartesian LIFT..."
        );

        executeTrajectory(
            lift_trajectory,
            GRIPPER_CLOSE_POS
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "LIFT 执行完成。"
        );

        // ====================================================
        // 8. LIFT -> PRE_PLACE / TRANSFER
        //
        // 第一版使用 Cartesian 水平搬运：
        // - Z 不变
        // - orientation 不变
        // - Cube 始终保持竖直
        //
        // 这样优先保证真实接触夹持稳定性。
        // ====================================================

        const std::vector<double> lift_final_q =
            lift_trajectory.points.back().positions;

        const auto pre_place_pose =
            makeTopDownPose(
                PLACE_X,
                PLACE_Y,
                LIFT_TCP_Z,
                PLACE_YAW
            );

        trajectory_msgs::msg::JointTrajectory transfer_trajectory;

        if (!planCartesianStage(
                move_group,
                joint_model_group,
                lift_final_q,
                pre_place_pose,
                "LIFT -> PRE_PLACE / TRANSFER",
                transfer_trajectory))
        {
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "执行 Cartesian TRANSFER..."
        );

        executeTrajectory(
            transfer_trajectory,
            GRIPPER_CLOSE_POS
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "TRANSFER 执行完成。"
        );

        // ====================================================
        // 9. PRE_PLACE -> PLACE
        //
        // 保持 X / Y / orientation，只沿 Z 轴下降。
        // 最终真实 Cube 底面理论上距离桌面约 1 mm。
        // ====================================================

        const std::vector<double> transfer_final_q =
            transfer_trajectory.points.back().positions;

        const auto place_pose =
            makeTopDownPose(
                PLACE_X,
                PLACE_Y,
                PLACE_TCP_Z,
                PLACE_YAW
            );

        trajectory_msgs::msg::JointTrajectory place_trajectory;

        if (!planCartesianStage(
                move_group,
                joint_model_group,
                transfer_final_q,
                place_pose,
                "PRE_PLACE -> PLACE",
                place_trajectory))
        {
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "执行 Cartesian PLACE 下降..."
        );

        executeTrajectory(
            place_trajectory,
            GRIPPER_CLOSE_POS
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "PLACE 下降执行完成。"
        );

        // ====================================================
        // 10. MoveIt DETACH
        //
        // 先在 MoveIt 中解除 AttachedCollisionObject。
        // 暂时不立即把 Cube 加回 World，避免退出规划时
        // 因 finger 状态与 Cube 接触产生不必要碰撞判断。
        // ====================================================

        if (!detachCubeFromTcp())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MoveIt DETACH 失败。"
            );
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "MoveIt DETACH 完成。"
        );

        // ====================================================
        // 11. OPEN / RELEASE
        // ====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "打开夹爪并释放 Cube..."
        );

        commandGripper(
            GRIPPER_OPEN_POS,
            1.0
        );

        // 让 Cube 落到桌面并稳定
        rclcpp::sleep_for(700ms);

        RCLCPP_INFO(
            node_->get_logger(),
            "Cube 已释放。"
        );

        // ====================================================
        // 12. PLACE -> RETREAT
        //
        // 保持放置姿态，只沿 Z 轴退出。
        // ====================================================

        const std::vector<double> place_final_q =
            place_trajectory.points.back().positions;

        const auto retreat_pose =
            makeTopDownPose(
                PLACE_X,
                PLACE_Y,
                RETREAT_TCP_Z,
                PLACE_YAW
            );

        trajectory_msgs::msg::JointTrajectory retreat_trajectory;

        if (!planCartesianStage(
                move_group,
                joint_model_group,
                place_final_q,
                retreat_pose,
                "PLACE -> RETREAT",
                retreat_trajectory))
        {
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "执行 Cartesian RETREAT..."
        );

        executeTrajectory(
            retreat_trajectory,
            GRIPPER_OPEN_POS
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "RETREAT 执行完成。"
        );

        // ====================================================
        // 13. 把已经放置的 Cube 重新加入 MoveIt World
        //
        // 以后返回 HOME 或继续规划时，MoveIt 会把新位置的 Cube
        // 当作环境障碍物。
        // ====================================================

        if (!addPlacedCubeToWorld())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "将已放置 Cube 加回 MoveIt World 失败。"
            );
            return false;
        }

        // ====================================================
        // 14. RETREAT -> HOME
        // ====================================================

        const std::vector<double> retreat_final_q =
            retreat_trajectory.points.back().positions;

        trajectory_msgs::msg::JointTrajectory home_trajectory;

        if (!planJointStage(
                move_group,
                joint_model_group,
                retreat_final_q,
                q_home,
                "RETREAT -> HOME",
                home_trajectory))
        {
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "执行返回 HOME..."
        );

        executeTrajectory(
            home_trajectory,
            GRIPPER_OPEN_POS
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "HOME 执行完成。"
        );

        // ====================================================
        // Task 01 当前完整链完成
        // ====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "============================================================"
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "Task 01 完整 Pick & Place 链执行完成："
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "HOME -> APPROACH -> GRASP -> LIFT -> TRANSFER -> PLACE -> RELEASE -> RETREAT -> HOME"
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "Pick  Cube center = (%.3f, %.3f, %.3f)",
            PICK_X,
            PICK_Y,
            PICK_CUBE_Z
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "Place Cube center = (%.3f, %.3f, %.3f)",
            PLACE_X,
            PLACE_Y,
            PLACE_CUBE_Z
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "============================================================"
        );

        return true;
    }

private:
    // ========================================================
    // 初始 Planning Scene：Table + PickCube
    // ========================================================

    bool addInitialPlanningScene()
    {
        moveit::planning_interface::PlanningSceneInterface psi;

        std::vector<moveit_msgs::msg::CollisionObject> objects;

        // ----------------------------------------------------
        // Table
        // ----------------------------------------------------

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

        // ----------------------------------------------------
        // PickCube
        // ----------------------------------------------------

        objects.push_back(
            makeWorldCubeObject(
                PICK_X,
                PICK_Y,
                PICK_CUBE_Z,
                PICK_YAW,
                moveit_msgs::msg::CollisionObject::ADD
            )
        );

        if (!psi.applyCollisionObjects(objects))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Planning Scene 初始化失败。"
            );
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Planning Scene 初始化完成。"
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "PickCube = %.3f m, center=(%.3f, %.3f, %.3f), yaw=%.3f",
            CUBE_SIZE,
            PICK_X,
            PICK_Y,
            PICK_CUBE_Z,
            PICK_YAW
        );

        rclcpp::sleep_for(1s);
        return true;
    }

    // ========================================================
    // 构造 MoveIt World 中的 Cube CollisionObject
    // ========================================================

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

        shape_msgs::msg::SolidPrimitive cube_shape;
        cube_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        cube_shape.dimensions = {
            CUBE_SIZE,
            CUBE_SIZE,
            CUBE_SIZE
        };

        geometry_msgs::msg::Pose cube_pose;
        cube_pose.position.x = x;
        cube_pose.position.y = y;
        cube_pose.position.z = z;
        cube_pose.orientation.z = std::sin(0.5 * yaw);
        cube_pose.orientation.w = std::cos(0.5 * yaw);

        cube.primitives.push_back(cube_shape);
        cube.primitive_poses.push_back(cube_pose);
        cube.operation = operation;

        return cube;
    }

    // ========================================================
    // 从 MoveIt World 删除 Cube
    // ========================================================

    void removeWorldCube()
    {
        moveit::planning_interface::PlanningSceneInterface psi;

        psi.removeCollisionObjects({"pick_cube"});

        RCLCPP_INFO(
            node_->get_logger(),
            "pick_cube 已从 MoveIt World 临时删除，允许抓取接触。"
        );

        rclcpp::sleep_for(500ms);
    }

    // ========================================================
    // MoveIt ATTACH
    // ========================================================

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

        shape_msgs::msg::SolidPrimitive cube_shape;
        cube_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        cube_shape.dimensions = {
            CUBE_SIZE,
            CUBE_SIZE,
            CUBE_SIZE
        };

        geometry_msgs::msg::Pose relative_pose;

        // Cube 世界姿态与抓取 yaw 对齐时，
        // Cube 相对 TCP 的方向保持 Rx(pi)。
        relative_pose.orientation.x = 1.0;
        relative_pose.orientation.y = 0.0;
        relative_pose.orientation.z = 0.0;
        relative_pose.orientation.w = 0.0;

        relative_pose.position.x = 0.0;
        relative_pose.position.y = 0.0;
        relative_pose.position.z = ATTACHED_CUBE_TCP_Z;

        attached.object.primitives.push_back(cube_shape);
        attached.object.primitive_poses.push_back(relative_pose);
        attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        if (!psi.applyAttachedCollisionObject(attached))
        {
            return false;
        }

        rclcpp::sleep_for(700ms);
        return true;
    }

    // ========================================================
    // MoveIt DETACH
    // ========================================================

    bool detachCubeFromTcp()
    {
        moveit::planning_interface::PlanningSceneInterface psi;

        moveit_msgs::msg::AttachedCollisionObject attached;

        attached.link_name = "fr3_hand_tcp";
        attached.object.id = "pick_cube";
        attached.object.operation =
            moveit_msgs::msg::CollisionObject::REMOVE;

        if (!psi.applyAttachedCollisionObject(attached))
        {
            return false;
        }

        rclcpp::sleep_for(500ms);
        return true;
    }

    // ========================================================
    // 退出后把 Cube 按目标位置重新加入 MoveIt World
    // ========================================================

    bool addPlacedCubeToWorld()
    {
        moveit::planning_interface::PlanningSceneInterface psi;

        auto cube =
            makeWorldCubeObject(
                PLACE_X,
                PLACE_Y,
                PLACE_CUBE_Z,
                PLACE_YAW,
                moveit_msgs::msg::CollisionObject::ADD
            );

        if (!psi.applyCollisionObject(cube))
        {
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "已将 Cube 作为 World CollisionObject 加到放置位置："
            "(%.3f, %.3f, %.3f)",
            PLACE_X,
            PLACE_Y,
            PLACE_CUBE_Z
        );

        rclcpp::sleep_for(500ms);
        return true;
    }

    // ========================================================
    // RRTConnect：显式起点 -> Pose 目标
    // ========================================================

    bool planPoseStage(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::vector<double>& start_q,
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& stage_name,
        trajectory_msgs::msg::JointTrajectory& trajectory_out)
    {
        moveit::core::RobotState start_state(
            move_group.getRobotModel()
        );

        start_state.setToDefaultValues();
        start_state.setJointGroupPositions(
            joint_model_group,
            start_q
        );
        start_state.update();

        if (!start_state.satisfiesBounds())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：起点违反关节限制。",
                stage_name.c_str()
            );
            return false;
        }

        move_group.setStartState(start_state);
        move_group.clearPoseTargets();

        if (!move_group.setPoseTarget(
                target_pose,
                "fr3_hand_tcp"))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：Pose 目标设置失败。",
                stage_name.c_str()
            );
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "%s 目标：p=(%.3f, %.3f, %.3f), q=(%.4f, %.4f, %.4f, %.4f)",
            stage_name.c_str(),
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        );

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        const auto result = move_group.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：规划失败。",
                stage_name.c_str()
            );
            return false;
        }

        trajectory_out = plan.trajectory_.joint_trajectory;

        if (trajectory_out.points.empty())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：轨迹为空。",
                stage_name.c_str()
            );
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "%s：规划成功，waypoints=%zu，duration=%.3f s",
            stage_name.c_str(),
            trajectory_out.points.size(),
            pointTime(trajectory_out.points.back())
        );

        return true;
    }

    // ========================================================
    // RRTConnect：显式起点 -> 关节目标
    // 用于最终返回 HOME
    // ========================================================

    bool planJointStage(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::vector<double>& start_q,
        const std::vector<double>& target_q,
        const std::string& stage_name,
        trajectory_msgs::msg::JointTrajectory& trajectory_out)
    {
        moveit::core::RobotState start_state(
            move_group.getRobotModel()
        );

        start_state.setToDefaultValues();
        start_state.setJointGroupPositions(
            joint_model_group,
            start_q
        );
        start_state.update();

        if (!start_state.satisfiesBounds())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：起点违反关节限制。",
                stage_name.c_str()
            );
            return false;
        }

        move_group.setStartState(start_state);
        move_group.clearPoseTargets();

        if (!move_group.setJointValueTarget(target_q))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：HOME 关节目标设置失败。",
                stage_name.c_str()
            );
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        const auto result = move_group.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：规划失败。",
                stage_name.c_str()
            );
            return false;
        }

        trajectory_out = plan.trajectory_.joint_trajectory;

        if (trajectory_out.points.empty())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：轨迹为空。",
                stage_name.c_str()
            );
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "%s：规划成功，waypoints=%zu，duration=%.3f s",
            stage_name.c_str(),
            trajectory_out.points.size(),
            pointTime(trajectory_out.points.back())
        );

        return true;
    }

    // ========================================================
    // Cartesian Path：显式起点 -> 单个 Pose 目标
    //
    // 目前用于：
    // 1. 抓取下降
    // 2. 抓取提升
    // 3. 水平搬运
    // 4. 放置下降
    // 5. 释放退出
    // ========================================================

    bool planCartesianStage(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::vector<double>& start_q,
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& stage_name,
        trajectory_msgs::msg::JointTrajectory& trajectory_out)
    {
        moveit::core::RobotState start_state(
            move_group.getRobotModel()
        );

        start_state.setToDefaultValues();
        start_state.setJointGroupPositions(
            joint_model_group,
            start_q
        );
        start_state.update();

        if (!start_state.satisfiesBounds())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：Cartesian 起点违反关节限制。",
                stage_name.c_str()
            );
            return false;
        }

        move_group.setStartState(start_state);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory robot_trajectory;
        moveit_msgs::msg::MoveItErrorCodes cartesian_error_code;

        const double fraction =
            move_group.computeCartesianPath(
                waypoints,
                CARTESIAN_EEF_STEP,
                0.0,
                robot_trajectory,
                true,
                &cartesian_error_code
            );

        RCLCPP_INFO(
            node_->get_logger(),
            "%s：Cartesian fraction = %.4f",
            stage_name.c_str(),
            fraction
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "%s：MoveIt error_code = %d",
            stage_name.c_str(),
            cartesian_error_code.val
        );

        if (fraction < CARTESIAN_MIN_FRACTION)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：Cartesian Path 不完整，仅完成 %.2f%%。",
                stage_name.c_str(),
                fraction * 100.0
            );
            return false;
        }

        trajectory_out = robot_trajectory.joint_trajectory;

        if (trajectory_out.points.empty())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s：Cartesian 轨迹为空。",
                stage_name.c_str()
            );
            return false;
        }

        ensureTrajectoryTiming(trajectory_out);

        RCLCPP_INFO(
            node_->get_logger(),
            "%s：Cartesian 规划成功，waypoints=%zu，duration=%.3f s",
            stage_name.c_str(),
            trajectory_out.points.size(),
            pointTime(trajectory_out.points.back())
        );

        return true;
    }

    // ========================================================
    // 轨迹执行：100 Hz 插值 -> /joint_command
    //
    // finger_hold < 0：只发布机械臂关节
    // finger_hold >= 0：同时持续保持夹爪指令
    // ========================================================

    void executeTrajectory(
        const trajectory_msgs::msg::JointTrajectory& trajectory,
        double finger_hold = -1.0)
    {
        if (trajectory.points.empty())
        {
            return;
        }

        const double total_time =
            pointTime(trajectory.points.back());

        size_t segment = 0;

        const auto start_time =
            std::chrono::steady_clock::now();

        while (true)
        {
            const double t =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start_time
                ).count();

            if (t > total_time)
            {
                break;
            }

            while (
                segment + 1 < trajectory.points.size() &&
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
                    q_command[j] =
                        p0.positions[j] +
                        alpha * (p1.positions[j] - p0.positions[j]);
                }
            }

            publishArmCommand(
                trajectory.joint_names,
                q_command,
                finger_hold
            );

            std::this_thread::sleep_for(10ms);
        }

        // 最终位置保持 0.5 秒
        const auto& final_q =
            trajectory.points.back().positions;

        for (int i = 0; i < 50; ++i)
        {
            publishArmCommand(
                trajectory.joint_names,
                final_q,
                finger_hold
            );

            std::this_thread::sleep_for(10ms);
        }
    }

    // ========================================================
    // 夹爪控制
    // ========================================================

    void commandGripper(
        double finger_position,
        double duration_sec)
    {
        const std::vector<std::string> names = {
            "fr3_finger_joint1",
            "fr3_finger_joint2"
        };

        const std::vector<double> positions = {
            finger_position,
            finger_position
        };

        const int cycles =
            std::max(
                1,
                static_cast<int>(duration_sec / 0.01)
            );

        for (int i = 0; i < cycles; ++i)
        {
            sensor_msgs::msg::JointState msg;

            msg.header.stamp = node_->now();
            msg.name = names;
            msg.position = positions;

            command_pub_->publish(msg);

            std::this_thread::sleep_for(10ms);
        }
    }

    // ========================================================
    // 发布机械臂 + 可选夹爪保持命令
    // ========================================================

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

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        command_pub_;
};

// ============================================================
// main
// ============================================================

int main(
    int argc,
    char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>(
            "single_arm_pick_place"
        );

    if (!copyRobotModelParameters(node))
    {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread(
        [&executor]()
        {
            executor.spin();
        }
    );

    bool success = false;

    try
    {
        SingleArmPickPlace pick_place(node);
        success = pick_place.run();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "程序异常：%s",
            e.what()
        );
    }

    executor.cancel();

    if (spin_thread.joinable())
    {
        spin_thread.join();
    }

    rclcpp::shutdown();

    return success ? 0 : 1;
}

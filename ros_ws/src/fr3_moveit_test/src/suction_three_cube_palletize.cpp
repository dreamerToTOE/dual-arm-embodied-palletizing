#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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
constexpr double TABLE_X = 0.55;
constexpr double TABLE_Y = 0.00;
constexpr double TABLE_Z = 0.025;
constexpr double TABLE_SIZE_X = 1.20;
constexpr double TABLE_SIZE_Y = 0.80;
constexpr double TABLE_SIZE_Z = 0.05;

constexpr double CUBE_SIZE = 0.030;
constexpr double CUBE_CENTER_Z = 0.065;
constexpr double CUBE_TOP_Z = CUBE_CENTER_Z + 0.5 * CUBE_SIZE;

// 吸盘 TCP 接触面与箱体顶面相接。
constexpr double PRE_PICK_TCP_Z = 0.145;
constexpr double CONTACT_TCP_Z = CUBE_TOP_Z + 0.001;
constexpr double LIFT_TCP_Z = 0.180;
constexpr double PRE_PLACE_TCP_Z = 0.180;
constexpr double PLACE_TCP_Z = CUBE_TOP_Z + 0.001;
constexpr double RETREAT_TCP_Z = 0.180;

constexpr double CARTESIAN_EEF_STEP = 0.002;
constexpr double CARTESIAN_MIN_FRACTION = 0.999;

const std::array<std::array<double, 3>, 3> STACK_TARGETS = {{
    {{0.620, -0.150, CUBE_CENTER_Z}},
    {{0.652, -0.150, CUBE_CENTER_Z}},
    {{0.684, -0.150, CUBE_CENTER_Z}},
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
    for (size_t i = 0; i < trajectory.points.size(); ++i)
    {
        setPointTime(trajectory.points[i], static_cast<double>(i) * FALLBACK_DT);
    }
}

geometry_msgs::msg::Pose makeTopDownPose(double x, double y, double z, double yaw = 0.0)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // Rz(yaw) * Rx(pi)
    const double half_yaw = 0.5 * yaw;
    pose.orientation.x = std::cos(half_yaw);
    pose.orientation.y = std::sin(half_yaw);
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
}

}  // namespace

class SuctionThreeCubePalletize
{
public:
    explicit SuctionThreeCubePalletize(const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command", 10);
        suction_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/task04/suction_command", 10);

        cube_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/task04/cube_poses",
            10,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg)
            {
                if (msg->poses.size() < 3)
                {
                    return;
                }
                std::lock_guard<std::mutex> lock(cube_mutex_);
                for (size_t i = 0; i < 3; ++i)
                {
                    cube_poses_[i] = msg->poses[i];
                }
                have_cube_poses_ = true;
            });

        suction_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/task04/suction_state",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                suction_closed_.store(msg->data);
            });
    }

    bool run()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "========== Task04：顶部吸盘三 Cube 依次码垛 ==========");

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

        std::string eef_link = move_group.getEndEffectorLink();
        const std::string requested_eef =
            node_->declare_parameter<std::string>("eef_link", "");
        if (!requested_eef.empty())
        {
            eef_link = requested_eef;
            move_group.setEndEffectorLink(eef_link);
        }

        if (eef_link.empty())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "MoveIt 没有给出末端执行器链接。请用 cobot_pump 配置启动 MoveIt，"
                         "或通过 -p eef_link:=... 指定 TCP link。");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "MoveIt EEF link = %s", eef_link.c_str());
        RCLCPP_INFO(node_->get_logger(), "Robot model frame = %s",
                    move_group.getRobotModel()->getModelFrame().c_str());

        const auto* joint_model_group =
            move_group.getRobotModel()->getJointModelGroup("fr3_arm");
        if (joint_model_group == nullptr)
        {
            RCLCPP_ERROR(node_->get_logger(), "找不到 fr3_arm JointModelGroup。");
            return false;
        }

        std::array<geometry_msgs::msg::Pose, 3> initial_cube_poses;
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

        for (size_t i = 0; i < 3; ++i)
        {
            geometry_msgs::msg::Pose pick_pose;
            {
                std::lock_guard<std::mutex> lock(cube_mutex_);
                pick_pose = cube_poses_[i];
            }

            const std::string cube_id = "cube_" + std::to_string(i + 1);
            const auto& target = STACK_TARGETS[i];

            RCLCPP_INFO(node_->get_logger(), "");
            RCLCPP_INFO(node_->get_logger(),
                        "========== Cube%zu / 3 ==========" , i + 1);
            RCLCPP_INFO(node_->get_logger(),
                        "PICK  = (%.4f, %.4f, %.4f)",
                        pick_pose.position.x,
                        pick_pose.position.y,
                        pick_pose.position.z);
            RCLCPP_INFO(node_->get_logger(),
                        "PLACE = (%.4f, %.4f, %.4f)",
                        target[0], target[1], target[2]);

            if (!pickAndPlaceOne(move_group,
                                 joint_model_group,
                                 eef_link,
                                 cube_id,
                                 pick_pose.position.x,
                                 pick_pose.position.y,
                                 target[0],
                                 target[1],
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
                    "========== Task04 SUCCESS：三个 Cube 依次码垛完成 ==========");
        return true;
    }

private:
    bool waitForIsaacBridge()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "等待 Isaac Task04 bridge：cube_poses + suction_state...");

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
                RCLCPP_INFO(node_->get_logger(), "Isaac bridge 已连接。");
                return true;
            }

            std::this_thread::sleep_for(100ms);
        }

        RCLCPP_ERROR(node_->get_logger(),
                     "等待 Isaac bridge 超时。请确认 Isaac 已 Play 且 bridge 脚本已运行。");
        return false;
    }

    bool pickAndPlaceOne(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::string& eef_link,
        const std::string& cube_id,
        double pick_x,
        double pick_y,
        double place_x,
        double place_y,
        std::vector<double>& current_q)
    {
        // ----------------------------------------------------
        // 1. RRTConnect 到 PRE_PICK
        // ----------------------------------------------------
        auto pre_pick = makeTopDownPose(pick_x, pick_y, PRE_PICK_TCP_Z);
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
        // 2. 按用户指定顺序：先开启吸盘，再做直线下降
        // Surface Gripper retryClose=True，会在下降过程中持续等待接近物体。
        // ----------------------------------------------------
        commandSuction(true);
        std::this_thread::sleep_for(200ms);

        // Cube 是有意接触目标，最终下降前从 MoveIt World 临时删除。
        removeWorldCube(cube_id);

        auto contact = makeTopDownPose(pick_x, pick_y, CONTACT_TCP_Z);
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
        // 3. Cartesian 垂直 LIFT
        // ----------------------------------------------------
        auto lift = makeTopDownPose(pick_x, pick_y, LIFT_TCP_Z);
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
        // 4. RRTConnect 携物到 PRE_PLACE
        // ----------------------------------------------------
        auto pre_place = makeTopDownPose(place_x, place_y, PRE_PLACE_TCP_Z);
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
        // 5. Cartesian 垂直 PLACE
        // ----------------------------------------------------
        auto place = makeTopDownPose(place_x, place_y, PLACE_TCP_Z);
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
        // 6. MoveIt DETACH -> 临时 remove -> Isaac SUCTION OFF
        // ----------------------------------------------------
        if (!detachCubeFromTcp(cube_id, eef_link))
        {
            return false;
        }
        removeWorldCube(cube_id);

        commandSuction(false);
        if (!waitForSuctionClosed(false, 2.0))
        {
            RCLCPP_ERROR(node_->get_logger(), "%s：吸盘释放状态确认失败。", cube_id.c_str());
            return false;
        }
        std::this_thread::sleep_for(300ms);

        // ----------------------------------------------------
        // 7. Cartesian 垂直 RETREAT
        // ----------------------------------------------------
        auto retreat = makeTopDownPose(place_x, place_y, RETREAT_TCP_Z);
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

        // 退出后按最终目标重新加入 MoveIt World。
        if (!addWorldCube(cube_id, place_x, place_y, CUBE_CENTER_Z))
        {
            return false;
        }

        current_q = currentFrom(retreat_traj);
        RCLCPP_INFO(node_->get_logger(), "%s：PICK -> PLACE 完成。", cube_id.c_str());
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
            RCLCPP_ERROR(node_->get_logger(), "%s：设置 PoseTarget 失败。", stage_name.c_str());
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto result = move_group.plan(plan);
        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "%s：RRTConnect 规划失败。", stage_name.c_str());
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
            RCLCPP_ERROR(node_->get_logger(), "%s：规划失败。", stage_name.c_str());
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
        double x,
        double y,
        double z,
        int operation)
    {
        moveit_msgs::msg::CollisionObject cube;
        cube.header.frame_id = "base";
        cube.id = id;

        shape_msgs::msg::SolidPrimitive shape;
        shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        shape.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        cube.primitives.push_back(shape);
        cube.primitive_poses.push_back(pose);
        cube.operation = operation;
        return cube;
    }

    bool addInitialPlanningScene(const std::array<geometry_msgs::msg::Pose, 3>& cubes)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        std::vector<moveit_msgs::msg::CollisionObject> objects;

        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "base";
        table.id = "table";

        shape_msgs::msg::SolidPrimitive table_shape;
        table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        table_shape.dimensions = {TABLE_SIZE_X, TABLE_SIZE_Y, TABLE_SIZE_Z};

        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.x = TABLE_X;
        table_pose.position.y = TABLE_Y;
        table_pose.position.z = TABLE_Z;

        table.primitives.push_back(table_shape);
        table.primitive_poses.push_back(table_pose);
        table.operation = moveit_msgs::msg::CollisionObject::ADD;
        objects.push_back(table);

        for (size_t i = 0; i < 3; ++i)
        {
            objects.push_back(makeCubeObject(
                "cube_" + std::to_string(i + 1),
                cubes[i].position.x,
                cubes[i].position.y,
                cubes[i].position.z,
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

    bool addWorldCube(const std::string& cube_id, double x, double y, double z)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        auto cube = makeCubeObject(
            cube_id, x, y, z, moveit_msgs::msg::CollisionObject::ADD);
        const bool ok = psi.applyCollisionObject(cube);
        std::this_thread::sleep_for(300ms);
        return ok;
    }

    bool attachCubeToTcp(const std::string& cube_id, const std::string& eef_link)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = eef_link;
        attached.touch_links = {eef_link};
        attached.object.header.frame_id = eef_link;
        attached.object.id = cube_id;

        shape_msgs::msg::SolidPrimitive shape;
        shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        shape.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};

        geometry_msgs::msg::Pose relative_pose;
        relative_pose.position.z = 0.5 * CUBE_SIZE;
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

    bool detachCubeFromTcp(const std::string& cube_id, const std::string& eef_link)
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
                double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
                alpha = std::clamp(alpha, 0.0, 1.0);

                q_command.resize(p0.positions.size());
                for (size_t j = 0; j < q_command.size(); ++j)
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
    std::array<geometry_msgs::msg::Pose, 3> cube_poses_{};
    bool have_cube_poses_ = false;
    std::atomic_bool suction_closed_{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("suction_three_cube_palletize");

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
        SuctionThreeCubePalletize demo(node);
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

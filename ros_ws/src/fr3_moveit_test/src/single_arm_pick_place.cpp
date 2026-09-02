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
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

// ============================================================
// Scene parameters
// ============================================================

constexpr double TABLE_X = 0.55;
constexpr double TABLE_Y = 0.00;
constexpr double TABLE_Z = 0.025;
constexpr double TABLE_SIZE_X = 1.20;
constexpr double TABLE_SIZE_Y = 0.80;
constexpr double TABLE_SIZE_Z = 0.05;

// 30 mm cube, axis-aligned in the current Isaac scene.
constexpr double CUBE_X = 0.45;
constexpr double CUBE_Y = 0.15;
constexpr double CUBE_Z = 0.065;
constexpr double CUBE_SIZE = 0.030;
constexpr double CUBE_YAW = 0.0;  // rad

// TCP targets.
constexpr double APPROACH_TCP_Z = 0.140;
constexpr double GRASP_TCP_Z = 0.075;
constexpr double LIFT_TCP_Z = 0.175;

// Cartesian interpolation.
constexpr double CARTESIAN_EEF_STEP = 0.002;  // 2 mm
constexpr double CARTESIAN_MIN_FRACTION = 0.999;

// Gripper commands. Each Franka finger joint is half of the total opening.
constexpr double GRIPPER_OPEN_POS = 0.040;   // ~80 mm total opening
constexpr double GRIPPER_CLOSE_POS = 0.014;  // ~28 mm total opening, slight preload on 30 mm cube

// At the grasp pose, the real cube center is 10 mm below the TCP.
// The MoveIt attached model uses 9 mm so its bottom starts about 1 mm
// above the table, avoiding an exact initial table-contact state.
constexpr double ATTACHED_CUBE_TCP_Z = 0.009;

// ============================================================
// Utilities
// ============================================================

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameter service...");

    auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");

    if (!client->wait_for_service(10s))
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot connect to /move_group parameter service.");
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
        RCLCPP_ERROR(node->get_logger(), "Failed to read MoveIt parameters: %s", e.what());
        return false;
    }

    if (params.size() != 2 ||
        params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
        params[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
        RCLCPP_ERROR(node->get_logger(), "Invalid robot model parameters.");
        return false;
    }

    const std::string urdf = params[0].as_string();
    const std::string srdf = params[1].as_string();

    if (urdf.empty() || srdf.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "URDF or SRDF is empty.");
        return false;
    }

    node->declare_parameter<std::string>("robot_description", urdf);
    node->declare_parameter<std::string>("robot_description_semantic", srdf);

    RCLCPP_INFO(node->get_logger(), "Robot model parameters copied successfully.");
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
    int64_t nsec = static_cast<int64_t>(std::llround((seconds - static_cast<double>(sec)) * 1e9));

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
        return;

    // Cartesian-path service normally returns timing. This is only a safe
    // fallback for versions/configurations that return position-only points.
    constexpr double FALLBACK_DT = 0.03;
    for (size_t i = 0; i < trajectory.points.size(); ++i)
        setPointTime(trajectory.points[i], static_cast<double>(i) * FALLBACK_DT);
}

// R_grasp = Rz(cube_yaw) * Rx(pi)
// For yaw=0: q=(1,0,0,0), so TCP +Z points down and the finger
// prismatic Y-axis is parallel to the world/cube Y-axis.
geometry_msgs::msg::Pose makeTopDownAlignedPose(double z)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = CUBE_X;
    pose.position.y = CUBE_Y;
    pose.position.z = z;

    const double half_yaw = 0.5 * CUBE_YAW;
    pose.orientation.x = std::cos(half_yaw);
    pose.orientation.y = std::sin(half_yaw);
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;

    return pose;
}

// ============================================================
// Single-arm Pick & Place
// ============================================================

class SingleArmPickPlace
{
public:
    explicit SingleArmPickPlace(const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command", 10);
    }

    bool run()
    {
        RCLCPP_INFO(node_->get_logger(), "Creating MoveGroupInterface...");

        moveit::planning_interface::MoveGroupInterface move_group(node_, "fr3_arm");
        move_group.setPlannerId("RRTConnectkConfigDefault");
        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(5);
        move_group.setMaxVelocityScalingFactor(0.20);
        move_group.setMaxAccelerationScalingFactor(0.20);
        move_group.setPoseReferenceFrame("fr3_link0");
        move_group.setEndEffectorLink("fr3_hand_tcp");

        if (!addPlanningScene())
            return false;

        if (command_pub_->get_subscription_count() == 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "No /joint_command subscriber detected.");
            return false;
        }

        const auto* joint_model_group =
            move_group.getRobotModel()->getJointModelGroup("fr3_arm");

        if (joint_model_group == nullptr)
        {
            RCLCPP_ERROR(node_->get_logger(), "Cannot find planning group fr3_arm.");
            return false;
        }

        const std::vector<double> q_home = {
            0.0, -0.7, 0.0, -2.2, 0.0, 2.0, 0.8
        };

        // ----------------------------------------------------
        // 1. HOME -> APPROACH_ALIGNED with RRTConnect
        // ----------------------------------------------------
        const geometry_msgs::msg::Pose approach_pose =
            makeTopDownAlignedPose(APPROACH_TCP_Z);

        trajectory_msgs::msg::JointTrajectory approach_trajectory;

        if (!planPoseStage(move_group,
                           joint_model_group,
                           q_home,
                           approach_pose,
                           "HOME -> APPROACH_ALIGNED",
                           approach_trajectory))
            return false;

        RCLCPP_INFO(node_->get_logger(), "Executing APPROACH_ALIGNED...");
        executeTrajectory(approach_trajectory);
        RCLCPP_INFO(node_->get_logger(), "APPROACH_ALIGNED execution COMPLETE.");

        // ----------------------------------------------------
        // 2. OPEN_GRIPPER
        // ----------------------------------------------------
        RCLCPP_INFO(node_->get_logger(), "Opening gripper...");
        commandGripper(GRIPPER_OPEN_POS, 1.0);
        RCLCPP_INFO(node_->get_logger(), "OPEN_GRIPPER COMPLETE.");

        // ----------------------------------------------------
        // 3. Intentional object contact: remove cube from MoveIt world.
        //    The physical Isaac cube is unchanged.
        // ----------------------------------------------------
        removePickCubeFromPlanningScene();

        // ----------------------------------------------------
        // 4. APPROACH -> GRASP: Cartesian, X/Y/orientation unchanged.
        // ----------------------------------------------------
        const std::vector<double> approach_final_q =
            approach_trajectory.points.back().positions;

        const geometry_msgs::msg::Pose grasp_pose =
            makeTopDownAlignedPose(GRASP_TCP_Z);

        trajectory_msgs::msg::JointTrajectory grasp_trajectory;

        if (!planCartesianStage(move_group,
                                joint_model_group,
                                approach_final_q,
                                grasp_pose,
                                "APPROACH -> GRASP [Cartesian Z-only]",
                                grasp_trajectory))
            return false;

        RCLCPP_INFO(node_->get_logger(), "Executing Cartesian GRASP descent...");
        executeTrajectory(grasp_trajectory, GRIPPER_OPEN_POS);
        RCLCPP_INFO(node_->get_logger(), "Cartesian GRASP descent COMPLETE.");

        // ----------------------------------------------------
        // 5. CLOSE_GRIPPER
        // ----------------------------------------------------
        RCLCPP_INFO(node_->get_logger(), "Closing gripper...");
        commandGripper(GRIPPER_CLOSE_POS, 1.0);
        RCLCPP_INFO(node_->get_logger(), "CLOSE_GRIPPER COMPLETE.");

        rclcpp::sleep_for(500ms);

        // ----------------------------------------------------
        // 6. MoveIt attach for carried-object collision checking.
        //    No Isaac FixedJoint yet: physical contact is tested first.
        // ----------------------------------------------------
        if (!attachPickCubeToTcp())
        {
            RCLCPP_ERROR(node_->get_logger(), "MoveIt ATTACH failed.");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "MOVEIT ATTACH COMPLETE.");

        // ----------------------------------------------------
        // 7. GRASP -> LIFT: Cartesian Z-only, hold gripper closed.
        // ----------------------------------------------------
        const std::vector<double> grasp_final_q =
            grasp_trajectory.points.back().positions;

        const geometry_msgs::msg::Pose lift_pose =
            makeTopDownAlignedPose(LIFT_TCP_Z);

        trajectory_msgs::msg::JointTrajectory lift_trajectory;

        if (!planCartesianStage(move_group,
                                joint_model_group,
                                grasp_final_q,
                                lift_pose,
                                "GRASP -> LIFT [Cartesian Z-only]",
                                lift_trajectory))
            return false;

        RCLCPP_INFO(node_->get_logger(), "Executing Cartesian LIFT...");
        executeTrajectory(lift_trajectory, GRIPPER_CLOSE_POS);
        RCLCPP_INFO(node_->get_logger(), "Cartesian LIFT execution COMPLETE.");

        RCLCPP_INFO(node_->get_logger(), "========================================");
        RCLCPP_INFO(node_->get_logger(), "Current Task 01 chain COMPLETE:");
        RCLCPP_INFO(node_->get_logger(),
                    "HOME -> ALIGNED_APPROACH -> OPEN -> CARTESIAN_GRASP -> CLOSE -> ATTACH -> CARTESIAN_LIFT");
        RCLCPP_INFO(node_->get_logger(), "Cube = %.0f mm, yaw = %.3f rad", CUBE_SIZE * 1000.0, CUBE_YAW);
        RCLCPP_INFO(node_->get_logger(), "TCP: approach=%.3f grasp=%.3f lift=%.3f m",
                    APPROACH_TCP_Z, GRASP_TCP_Z, LIFT_TCP_Z);
        RCLCPP_INFO(node_->get_logger(), "========================================");

        return true;
    }

private:
    bool addPlanningScene()
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        std::vector<moveit_msgs::msg::CollisionObject> objects;

        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "fr3_link0";
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

        moveit_msgs::msg::CollisionObject cube;
        cube.header.frame_id = "fr3_link0";
        cube.id = "pick_cube";

        shape_msgs::msg::SolidPrimitive cube_shape;
        cube_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        cube_shape.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};

        geometry_msgs::msg::Pose cube_pose;
        cube_pose.position.x = CUBE_X;
        cube_pose.position.y = CUBE_Y;
        cube_pose.position.z = CUBE_Z;
        cube_pose.orientation.z = std::sin(CUBE_YAW * 0.5);
        cube_pose.orientation.w = std::cos(CUBE_YAW * 0.5);

        cube.primitives.push_back(cube_shape);
        cube.primitive_poses.push_back(cube_pose);
        cube.operation = moveit_msgs::msg::CollisionObject::ADD;
        objects.push_back(cube);

        if (!psi.applyCollisionObjects(objects))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to update Planning Scene.");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Planning Scene updated: cube=%.3f m center=(%.3f, %.3f, %.3f) yaw=%.3f",
                    CUBE_SIZE, CUBE_X, CUBE_Y, CUBE_Z, CUBE_YAW);

        rclcpp::sleep_for(1s);
        return true;
    }

    void removePickCubeFromPlanningScene()
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        psi.removeCollisionObjects({"pick_cube"});

        RCLCPP_INFO(node_->get_logger(),
                    "pick_cube removed from MoveIt world for intentional grasp contact.");
        rclcpp::sleep_for(500ms);
    }

    bool attachPickCubeToTcp()
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
        cube_shape.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};

        geometry_msgs::msg::Pose relative_pose;
        // With R_grasp = Rz(yaw)*Rx(pi), an axis-aligned cube relative to
        // TCP has a constant Rx(pi) orientation. Positive TCP Z points down.
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
            return false;

        RCLCPP_INFO(node_->get_logger(),
                    "pick_cube attached to fr3_hand_tcp in MoveIt, relative z=%.3f m",
                    ATTACHED_CUBE_TCP_Z);

        rclcpp::sleep_for(1s);
        return true;
    }

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
            RCLCPP_ERROR(node_->get_logger(), "%s: start state violates joint limits.", stage_name.c_str());
            return false;
        }

        move_group.setStartState(start_state);
        move_group.clearPoseTargets();
        move_group.setPoseTarget(target_pose, "fr3_hand_tcp");

        RCLCPP_INFO(node_->get_logger(),
                    "%s target: p=(%.3f, %.3f, %.3f), q=(%.4f, %.4f, %.4f, %.4f)",
                    stage_name.c_str(),
                    target_pose.position.x, target_pose.position.y, target_pose.position.z,
                    target_pose.orientation.x, target_pose.orientation.y,
                    target_pose.orientation.z, target_pose.orientation.w);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto result = move_group.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "%s planning FAILED.", stage_name.c_str());
            return false;
        }

        trajectory_out = plan.trajectory_.joint_trajectory;
        if (trajectory_out.points.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "%s trajectory is empty.", stage_name.c_str());
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "%s planning SUCCESS. waypoints=%zu duration=%.3f s",
                    stage_name.c_str(), trajectory_out.points.size(),
                    pointTime(trajectory_out.points.back()));
        return true;
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
            RCLCPP_ERROR(node_->get_logger(), "%s: start state violates joint limits.", stage_name.c_str());
            return false;
        }

        move_group.setStartState(start_state);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory robot_trajectory;

        const double fraction = move_group.computeCartesianPath(
            waypoints,
            CARTESIAN_EEF_STEP,
            0.0,
            robot_trajectory,
            true);

        RCLCPP_INFO(node_->get_logger(),
                    "%s Cartesian fraction = %.4f",
                    stage_name.c_str(), fraction);

        if (fraction < CARTESIAN_MIN_FRACTION)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s Cartesian path incomplete: %.2f%%",
                         stage_name.c_str(), fraction * 100.0);
            return false;
        }

        trajectory_out = robot_trajectory.joint_trajectory;
        if (trajectory_out.points.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "%s Cartesian trajectory is empty.", stage_name.c_str());
            return false;
        }

        ensureTrajectoryTiming(trajectory_out);

        RCLCPP_INFO(node_->get_logger(),
                    "%s Cartesian planning SUCCESS. waypoints=%zu duration=%.3f s",
                    stage_name.c_str(), trajectory_out.points.size(),
                    pointTime(trajectory_out.points.back()));
        return true;
    }

    void executeTrajectory(
        const trajectory_msgs::msg::JointTrajectory& trajectory,
        double finger_hold = -1.0)
    {
        if (trajectory.points.empty())
            return;

        const double total_time = pointTime(trajectory.points.back());
        size_t segment = 0;
        const auto start_time = std::chrono::steady_clock::now();

        while (true)
        {
            const double t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time).count();

            if (t > total_time)
                break;

            while (segment + 1 < trajectory.points.size() &&
                   pointTime(trajectory.points[segment + 1]) < t)
                ++segment;

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
                    alpha = (t - t0) / (t1 - t0);

                alpha = std::clamp(alpha, 0.0, 1.0);
                q_command.resize(p0.positions.size());

                for (size_t j = 0; j < q_command.size(); ++j)
                    q_command[j] = p0.positions[j] +
                                   alpha * (p1.positions[j] - p0.positions[j]);
            }

            publishArmCommand(trajectory.joint_names, q_command, finger_hold);
            std::this_thread::sleep_for(10ms);
        }

        const auto& final_q = trajectory.points.back().positions;
        for (int i = 0; i < 50; ++i)
        {
            publishArmCommand(trajectory.joint_names, final_q, finger_hold);
            std::this_thread::sleep_for(10ms);
        }
    }

    void commandGripper(double finger_position, double duration_sec)
    {
        const std::vector<std::string> names = {
            "fr3_finger_joint1",
            "fr3_finger_joint2"
        };

        const std::vector<double> positions = {
            finger_position,
            finger_position
        };

        const int cycles = std::max(1, static_cast<int>(duration_sec / 0.01));

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

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("single_arm_pick_place");

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
        SingleArmPickPlace pick_place(node);
        success = pick_place.run();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(node->get_logger(), "Exception: %s", e.what());
    }

    executor.cancel();

    if (spin_thread.joinable())
        spin_thread.join();

    rclcpp::shutdown();
    return success ? 0 : 1;
}

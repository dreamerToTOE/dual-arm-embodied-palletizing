#include <algorithm>
#include <chrono>
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
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

// ============================================================
// Isaac Sim scene parameters
// ============================================================

constexpr double TABLE_X = 0.55;
constexpr double TABLE_Y = 0.00;
constexpr double TABLE_Z = 0.025;
constexpr double TABLE_SIZE_X = 1.20;
constexpr double TABLE_SIZE_Y = 0.80;
constexpr double TABLE_SIZE_Z = 0.05;

constexpr double CUBE_X = 0.45;
constexpr double CUBE_Y = 0.15;
constexpr double CUBE_Z = 0.09;
constexpr double CUBE_SIZE = 0.08;

constexpr double PRE_GRASP_Z_OFFSET = 0.15;   // TCP z = 0.24 m
constexpr double APPROACH_Z_OFFSET = 0.08;    // TCP z = 0.17 m
constexpr double GRASP_Z_OFFSET = 0.06;       // TCP z = 0.15 m

constexpr double GRIPPER_OPEN_POS = 0.04;
// Current cube is 0.08 m wide, equal to the nominal full gripper opening.
// Keep the first close command conservative. Logical attach will be used later.
constexpr double GRIPPER_CLOSE_POS = 0.039;

// ============================================================
// Copy URDF/SRDF from /move_group
// ============================================================

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameter service...");

    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(
        node, "/move_group");

    if (!parameter_client->wait_for_service(10s))
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Cannot connect to /move_group parameter service.");
        RCLCPP_ERROR(node->get_logger(), "Make sure MoveIt2 is running first.");
        return false;
    }

    const std::vector<std::string> names = {
        "robot_description",
        "robot_description_semantic"
    };

    std::vector<rclcpp::Parameter> params;

    try
    {
        params = parameter_client->get_parameters(names);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to read /move_group parameters: %s", e.what());
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
    RCLCPP_INFO(node->get_logger(), "URDF size = %zu bytes", urdf.size());
    RCLCPP_INFO(node->get_logger(), "SRDF size = %zu bytes", srdf.size());

    return true;
}

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
    return static_cast<double>(point.time_from_start.sec) +
           static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

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
        move_group.setPoseReferenceFrame("fr3_link0");
        move_group.setEndEffectorLink("fr3_hand_tcp");

        RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface created.");

        if (!addPlanningScene())
        {
            return false;
        }

        if (command_pub_->get_subscription_count() == 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "No /joint_command subscriber detected.");
            RCLCPP_ERROR(node_->get_logger(), "Check Isaac Sim Action Graph and press Play.");
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

        if (joint_model_group->getVariableCount() != q_home.size())
        {
            RCLCPP_ERROR(node_->get_logger(), "Unexpected fr3_arm joint count.");
            return false;
        }

        // ----------------------------------------------------
        // Stage 1: HOME -> PRE_GRASP
        // ----------------------------------------------------

        geometry_msgs::msg::Pose pre_grasp_pose;
        pre_grasp_pose.position.x = CUBE_X;
        pre_grasp_pose.position.y = CUBE_Y;
        pre_grasp_pose.position.z = CUBE_Z + PRE_GRASP_Z_OFFSET;
        pre_grasp_pose.orientation.x = 1.0;
        pre_grasp_pose.orientation.y = 0.0;
        pre_grasp_pose.orientation.z = 0.0;
        pre_grasp_pose.orientation.w = 0.0;

        trajectory_msgs::msg::JointTrajectory pre_grasp_trajectory;

        if (!planPoseStage(move_group,
                           joint_model_group,
                           q_home,
                           pre_grasp_pose,
                           "HOME -> PRE_GRASP",
                           pre_grasp_trajectory))
        {
            return false;
        }

        executeTrajectory(pre_grasp_trajectory);
        RCLCPP_INFO(node_->get_logger(), "PRE_GRASP execution COMPLETE.");

        // ----------------------------------------------------
        // Stage 2: PRE_GRASP -> APPROACH
        // ----------------------------------------------------

        const std::vector<double> pre_grasp_final_q =
            pre_grasp_trajectory.points.back().positions;

        geometry_msgs::msg::Pose approach_pose = pre_grasp_pose;
        approach_pose.position.z = CUBE_Z + APPROACH_Z_OFFSET;

        trajectory_msgs::msg::JointTrajectory approach_trajectory;

        if (!planPoseStage(move_group,
                           joint_model_group,
                           pre_grasp_final_q,
                           approach_pose,
                           "PRE_GRASP -> APPROACH",
                           approach_trajectory))
        {
            return false;
        }

        executeTrajectory(approach_trajectory);
        RCLCPP_INFO(node_->get_logger(), "APPROACH execution COMPLETE.");

        // ----------------------------------------------------
        // Stage 3: OPEN_GRIPPER at the near-object pose
        // ----------------------------------------------------

        RCLCPP_INFO(node_->get_logger(), "Opening gripper...");
        commandGripper(GRIPPER_OPEN_POS, 1.0);
        RCLCPP_INFO(node_->get_logger(), "OPEN_GRIPPER COMPLETE.");

        // ----------------------------------------------------
        // Stage 4: one final descent to grasp height
        // ----------------------------------------------------

        const std::vector<double> approach_final_q =
            approach_trajectory.points.back().positions;

        geometry_msgs::msg::Pose grasp_pose = approach_pose;
        grasp_pose.position.z = CUBE_Z + GRASP_Z_OFFSET;

        trajectory_msgs::msg::JointTrajectory grasp_trajectory;

        if (!planPoseStage(move_group,
                           joint_model_group,
                           approach_final_q,
                           grasp_pose,
                           "APPROACH -> FINAL_DESCENT_TO_GRASP",
                           grasp_trajectory))
        {
            return false;
        }

        executeTrajectory(grasp_trajectory);
        RCLCPP_INFO(node_->get_logger(), "FINAL_DESCENT_TO_GRASP execution COMPLETE.");

        // ----------------------------------------------------
        // Stage 5: CLOSE_GRIPPER
        // ----------------------------------------------------

        RCLCPP_INFO(node_->get_logger(), "Closing gripper conservatively...");
        commandGripper(GRIPPER_CLOSE_POS, 1.0);
        RCLCPP_INFO(node_->get_logger(), "CLOSE_GRIPPER COMPLETE.");

        RCLCPP_INFO(node_->get_logger(), "========================================");
        RCLCPP_INFO(node_->get_logger(), "Current Task 01 stage COMPLETE:");
        RCLCPP_INFO(node_->get_logger(),
                    "HOME -> PRE_GRASP -> APPROACH -> OPEN_GRIPPER -> FINAL_DESCENT -> CLOSE_GRIPPER");
        RCLCPP_INFO(node_->get_logger(), "Next stage: ISAAC + MOVEIT ATTACH");
        RCLCPP_INFO(node_->get_logger(), "========================================");

        return true;
    }

private:
    bool addPlanningScene()
    {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<moveit_msgs::msg::CollisionObject> objects;

        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "fr3_link0";
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

        moveit_msgs::msg::CollisionObject cube;
        cube.header.frame_id = "fr3_link0";
        cube.id = "pick_cube";

        shape_msgs::msg::SolidPrimitive cube_shape;
        cube_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        cube_shape.dimensions = {
            CUBE_SIZE, CUBE_SIZE, CUBE_SIZE
        };

        geometry_msgs::msg::Pose cube_pose;
        cube_pose.orientation.w = 1.0;
        cube_pose.position.x = CUBE_X;
        cube_pose.position.y = CUBE_Y;
        cube_pose.position.z = CUBE_Z;

        cube.primitives.push_back(cube_shape);
        cube.primitive_poses.push_back(cube_pose);
        cube.operation = moveit_msgs::msg::CollisionObject::ADD;
        objects.push_back(cube);

        if (!planning_scene_interface.applyCollisionObjects(objects))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to update Planning Scene.");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "Planning scene updated.");
        RCLCPP_INFO(node_->get_logger(),
                    "Table: center=(%.3f, %.3f, %.3f), size=(%.2f, %.2f, %.2f)",
                    TABLE_X, TABLE_Y, TABLE_Z,
                    TABLE_SIZE_X, TABLE_SIZE_Y, TABLE_SIZE_Z);
        RCLCPP_INFO(node_->get_logger(),
                    "PickCube: center=(%.3f, %.3f, %.3f), size=%.2f",
                    CUBE_X, CUBE_Y, CUBE_Z, CUBE_SIZE);

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
            RCLCPP_ERROR(node_->get_logger(),
                         "%s: start state violates joint limits.",
                         stage_name.c_str());
            return false;
        }

        move_group.setStartState(start_state);
        move_group.clearPoseTargets();

        if (!move_group.setPoseTarget(target_pose, "fr3_hand_tcp"))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s: invalid pose target.", stage_name.c_str());
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "%s target:", stage_name.c_str());
        RCLCPP_INFO(node_->get_logger(), "  x = %.3f", target_pose.position.x);
        RCLCPP_INFO(node_->get_logger(), "  y = %.3f", target_pose.position.y);
        RCLCPP_INFO(node_->get_logger(), "  z = %.3f", target_pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "Planning %s...", stage_name.c_str());

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto result = move_group.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s planning FAILED.", stage_name.c_str());
            return false;
        }

        trajectory_out = plan.trajectory_.joint_trajectory;

        if (trajectory_out.points.empty())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s trajectory is empty.", stage_name.c_str());
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "%s planning SUCCESS.", stage_name.c_str());
        RCLCPP_INFO(node_->get_logger(),
                    "Trajectory waypoints = %zu", trajectory_out.points.size());
        RCLCPP_INFO(node_->get_logger(),
                    "Trajectory duration = %.3f s",
                    pointTime(trajectory_out.points.back()));

        return true;
    }

    void executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
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
            const auto now = std::chrono::steady_clock::now();
            const double t = std::chrono::duration<double>(now - start_time).count();

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

            publishJointCommand(trajectory.joint_names, q_command);
            std::this_thread::sleep_for(10ms);
        }

        const auto& final_q = trajectory.points.back().positions;
        for (int i = 0; i < 100; ++i)
        {
            publishJointCommand(trajectory.joint_names, final_q);
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

        const int cycles = static_cast<int>(duration_sec / 0.01);

        for (int i = 0; i < cycles; ++i)
        {
            publishJointCommand(names, positions);
            std::this_thread::sleep_for(10ms);
        }
    }

    void publishJointCommand(
        const std::vector<std::string>& names,
        const std::vector<double>& positions)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = node_->now();
        msg.name = names;
        msg.position = positions;
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
        RCLCPP_FATAL(node->get_logger(), "Failed to initialize robot model.");
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
        success = false;
    }

    executor.cancel();

    if (spin_thread.joinable())
    {
        spin_thread.join();
    }

    rclcpp::shutdown();
    return success ? 0 : 1;
}

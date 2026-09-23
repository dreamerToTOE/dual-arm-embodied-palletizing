#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameter service...");

    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(
        node,
        "/move_group"
    );

    if (!parameter_client->wait_for_service(10s))
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot connect to /move_group parameter service.");
        RCLCPP_ERROR(node->get_logger(), "Make sure MoveIt2 is running first.");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Connected to /move_group parameter service.");

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
        RCLCPP_ERROR(node->get_logger(), "Failed to read /move_group parameters: %s", e.what());
        return false;
    }

    if (params.size() != 2)
    {
        RCLCPP_ERROR(node->get_logger(), "Expected 2 parameters, received %zu.", params.size());
        return false;
    }

    if (params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
        RCLCPP_ERROR(node->get_logger(), "robot_description is not available as a string.");
        return false;
    }

    const std::string urdf = params[0].as_string();

    if (urdf.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "robot_description is empty.");
        return false;
    }

    if (params[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
        RCLCPP_ERROR(node->get_logger(), "robot_description_semantic is not available as a string.");
        return false;
    }

    const std::string srdf = params[1].as_string();

    if (srdf.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "robot_description_semantic is empty.");
        return false;
    }

    node->declare_parameter<std::string>("robot_description", urdf);
    node->declare_parameter<std::string>("robot_description_semantic", srdf);

    RCLCPP_INFO(node->get_logger(), "Robot model parameters copied successfully.");
    RCLCPP_INFO(node->get_logger(), "URDF size = %zu bytes", urdf.size());
    RCLCPP_INFO(node->get_logger(), "SRDF size = %zu bytes", srdf.size());

    return true;
}

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& p)
{
    return static_cast<double>(p.time_from_start.sec)
        + static_cast<double>(p.time_from_start.nanosec) * 1e-9;
}

class MoveItToIsaac
{
public:
    explicit MoveItToIsaac(const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command",
            10
        );
    }

    bool run()
    {
        RCLCPP_INFO(node_->get_logger(), "Creating MoveGroupInterface...");

        moveit::planning_interface::MoveGroupInterface move_group(
            node_,
            "fr3_arm"
        );

        RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface created.");

        move_group.setPlannerId("RRTConnectkConfigDefault");
        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(5);

        const std::vector<double> q_start = {
             0.0,
            -0.7,
             0.0,
            -2.2,
             0.0,
             2.0,
             0.8
        };

        const std::vector<double> q_goal = {
             0.3,
            -0.8,
             0.2,
            -2.3,
             0.2,
             2.1,
             0.6
        };

        moveit::core::RobotState start_state(move_group.getRobotModel());
        start_state.setToDefaultValues();

        const moveit::core::JointModelGroup* joint_model_group =
            start_state.getJointModelGroup("fr3_arm");

        if (joint_model_group == nullptr)
        {
            RCLCPP_ERROR(node_->get_logger(), "Cannot find MoveIt planning group: fr3_arm");
            return false;
        }

        const std::vector<std::string>& joint_names =
            joint_model_group->getVariableNames();

        RCLCPP_INFO(node_->get_logger(), "fr3_arm has %zu joints:", joint_names.size());

        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            RCLCPP_INFO(node_->get_logger(), "  joint[%zu] = %s", i, joint_names[i].c_str());
        }

        if (joint_names.size() != q_start.size())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Joint count mismatch: MoveIt=%zu, q_start=%zu",
                joint_names.size(),
                q_start.size()
            );
            return false;
        }

        start_state.setJointGroupPositions(joint_model_group, q_start);
        start_state.update();

        if (!start_state.satisfiesBounds())
        {
            RCLCPP_ERROR(node_->get_logger(), "HOME violates MoveIt joint limits.");
            return false;
        }

        move_group.setStartState(start_state);

        if (!move_group.setJointValueTarget(q_goal))
        {
            RCLCPP_ERROR(node_->get_logger(), "Pose A is not a valid MoveIt joint target.");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "Planning HOME -> Pose A ...");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto result = move_group.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "MoveIt planning FAILED.");
            return false;
        }

        const auto& trajectory = plan.trajectory_.joint_trajectory;

        if (trajectory.points.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Trajectory is empty.");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "Planning SUCCESS.");
        RCLCPP_INFO(node_->get_logger(), "Trajectory contains %zu waypoints.", trajectory.points.size());
        RCLCPP_INFO(node_->get_logger(), "Trajectory joint order:");

        for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
        {
            RCLCPP_INFO(node_->get_logger(), "  [%zu] %s", i, trajectory.joint_names[i].c_str());
        }

        for (size_t i = 0; i < trajectory.points.size(); ++i)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Waypoint %zu | t = %.3f s",
                i,
                pointTime(trajectory.points[i])
            );
        }

        rclcpp::sleep_for(1s);

        if (command_pub_->get_subscription_count() == 0)
        {
            RCLCPP_WARN(node_->get_logger(), "No subscriber detected on /joint_command.");
            RCLCPP_WARN(node_->get_logger(), "Check Isaac Sim Action Graph and press Play.");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "/joint_command subscriber detected.");
        }

        constexpr double CONTROL_DT = 0.01;
        const double total_time = pointTime(trajectory.points.back());

        RCLCPP_INFO(node_->get_logger(), "Starting Isaac trajectory execution.");
        RCLCPP_INFO(node_->get_logger(), "Trajectory duration = %.3f s", total_time);

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

            while (
                segment + 1 < trajectory.points.size() &&
                pointTime(trajectory.points[segment + 1]) < t
            )
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

                alpha = std::max(0.0, std::min(1.0, alpha));

                q_command.resize(p0.positions.size());

                for (size_t j = 0; j < q_command.size(); ++j)
                {
                    q_command[j] = p0.positions[j]
                        + alpha * (p1.positions[j] - p0.positions[j]);
                }
            }

            publishCommand(trajectory.joint_names, q_command);

            std::this_thread::sleep_for(
                std::chrono::duration<double>(CONTROL_DT)
            );
        }

        RCLCPP_INFO(node_->get_logger(), "Holding final pose...");

        const std::vector<double> final_q = trajectory.points.back().positions;

        for (int i = 0; i < 100; ++i)
        {
            publishCommand(trajectory.joint_names, final_q);
            std::this_thread::sleep_for(10ms);
        }

        RCLCPP_INFO(node_->get_logger(), "Trajectory execution COMPLETE.");
        return true;
    }

private:
    void publishCommand(
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

    auto node = std::make_shared<rclcpp::Node>("moveit_to_isaac");

    if (!copyRobotModelParameters(node))
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialize robot model parameters.");
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
        MoveItToIsaac bridge(node);
        success = bridge.run();
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

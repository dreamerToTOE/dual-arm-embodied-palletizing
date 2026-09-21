// Task24-E：Isaac 中 L 型侧吸阵列的最小运动演示。
//
// 仅执行 left_arm 从当前 HOME 状态到 Cube_01 左侧 10 mm 预接触位姿的已验证
// MoveIt 轨迹。不会发布 suction 命令、不会 attach/detach、不会搬运或放置 Cube。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kCubeSize = 0.120;
constexpr double kCubeHalf = 0.060;
constexpr double kContactGap = 0.010;

bool copyRobotDescriptions(const rclcpp::Node::SharedPtr& node)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "无法连接 /move_group。" );
    return false;
  }
  const auto values = client->get_parameters({"robot_description", "robot_description_semantic"});
  if (values.size() != 2 ||
      values[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      values[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(node->get_logger(), "未读取到有效 RobotModel。" );
    return false;
  }
  node->declare_parameter<std::string>("robot_description", values[0].as_string());
  node->declare_parameter<std::string>("robot_description_semantic", values[1].as_string());
  return true;
}

geometry_msgs::msg::Pose cubePose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.650;
  pose.position.y = 0.000;
  pose.position.z = 0.110;
  pose.orientation.w = 1.0;
  return pose;
}

moveit_msgs::msg::CollisionObject cubeObject()
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = "task24_side_contact_demo_cube";
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {kCubeSize, kCubeSize, kCubeSize};
  object.primitives.push_back(shape);
  object.primitive_poses.push_back(cubePose());
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

geometry_msgs::msg::Pose sidePreContactPose()
{
  auto pose = cubePose();
  pose.position.y -= kCubeHalf + kContactGap;
  // side_suction_tcp 的 local +X 指向世界 +Y，Cup 面朝向 Cube 左侧。
  constexpr double root_half = 0.7071067811865476;
  pose.orientation.x = root_half;
  pose.orientation.y = root_half;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  return pose;
}

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
    static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

std::vector<double> interpolate(const trajectory_msgs::msg::JointTrajectory& trajectory, double time_sec)
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
    const double duration = pointTime(second) - pointTime(first);
    const double ratio = duration > 1e-9 ?
      std::clamp((time_sec - pointTime(first)) / duration, 0.0, 1.0) : 0.0;
    std::vector<double> result(first.positions.size());
    for (std::size_t joint = 0; joint < result.size(); ++joint)
    {
      result[joint] = first.positions[joint] +
        ratio * (second.positions[joint] - first.positions[joint]);
    }
    return result;
  }
  return trajectory.points.back().positions;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task24_side_contact_demo");
  const double time_scale = node->declare_parameter<double>("time_scale", 2.0);
  if (time_scale < 1.0 || !copyRobotDescriptions(node))
  {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin([&executor]() { executor.spin(); });
  bool success = false;
  moveit::planning_interface::PlanningSceneInterface scene;

  try
  {
    auto command_pub = node->create_publisher<sensor_msgs::msg::JointState>("/left/joint_command", 10);
    for (int attempt = 0; attempt < 100 && command_pub->get_subscription_count() == 0; ++attempt)
    {
      std::this_thread::sleep_for(100ms);
    }
    if (command_pub->get_subscription_count() == 0)
    {
      RCLCPP_ERROR(node->get_logger(), "Isaac /left/joint_command 订阅端未就绪。" );
    }
    else
    {
      moveit::planning_interface::MoveGroupInterface group(node, "left_arm");
      const std::string eef = "left_fr3_side_suction_tcp";
      group.setPoseReferenceFrame("world");
      group.setEndEffectorLink(eef);
      group.setPlannerId("RRTConnectkConfigDefault");
      group.setPlanningTime(5.0);
      group.setNumPlanningAttempts(1);
      group.setMaxVelocityScalingFactor(0.15);
      group.setMaxAccelerationScalingFactor(0.15);

      const auto initial_state = group.getCurrentState(10.0);
      if (!initial_state)
      {
        RCLCPP_ERROR(node->get_logger(), "未收到 Isaac 的当前 joint_states，停止演示。" );
      }
      else
      {
        scene.removeCollisionObjects({"task24_side_contact_demo_cube"});
        std::this_thread::sleep_for(200ms);
        if (!scene.applyCollisionObject(cubeObject()))
        {
          RCLCPP_ERROR(node->get_logger(), "无法加入演示 Cube CollisionObject。" );
        }
        else
        {
          std::this_thread::sleep_for(500ms);
          const auto target = sidePreContactPose();
          group.clearPoseTargets();
          group.setStartState(*initial_state);
          group.setPoseTarget(target, eef);
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          RCLCPP_INFO(node->get_logger(),
            "========== Task24 ISAAC L-SIDE CUP DEMO: HOME/current -> Cube_01 pre-contact ==========");
          RCLCPP_INFO(node->get_logger(),
            "Target Cup face: (%.3f, %.3f, %.3f), gap=10 mm; no suction, no attach, no transport.",
            target.position.x, target.position.y, target.position.z);
          if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS ||
              plan.trajectory_.joint_trajectory.points.empty())
          {
            RCLCPP_ERROR(node->get_logger(), "L 型侧吸演示规划失败；未发布任何运动命令。" );
          }
          else
          {
            const auto& trajectory = plan.trajectory_.joint_trajectory;
            std::vector<std::string> isaac_names;
            isaac_names.reserve(trajectory.joint_names.size());
            for (const auto& name : trajectory.joint_names)
            {
              constexpr const char* kPrefix = "left_";
              isaac_names.push_back(name.rfind(kPrefix, 0) == 0 ? name.substr(5) : name);
            }
            const double duration = pointTime(trajectory.points.back());
            const auto start = std::chrono::steady_clock::now() + 500ms;
            RCLCPP_INFO(node->get_logger(),
              "PLAN PASS: points=%zu, logical_duration=%.3f s, Isaac time_scale=%.2f.",
              trajectory.points.size(), duration, time_scale);
            while (true)
            {
              const double logical = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count() / time_scale;
              if (logical > duration)
              {
                break;
              }
              sensor_msgs::msg::JointState command;
              command.header.stamp = node->now();
              command.name = isaac_names;
              command.position = interpolate(trajectory, std::max(0.0, logical));
              command_pub->publish(command);
              std::this_thread::sleep_for(10ms);
            }
            for (int hold = 0; hold < 50; ++hold)
            {
              sensor_msgs::msg::JointState command;
              command.header.stamp = node->now();
              command.name = isaac_names;
              command.position = trajectory.points.back().positions;
              command_pub->publish(command);
              std::this_thread::sleep_for(10ms);
            }
            RCLCPP_INFO(node->get_logger(),
              "Task24 ISAAC DEMO PASS: L 型阵列已到达 Cube_01 左侧预接触位姿；未启动吸盘。" );
            success = true;
          }
        }
      }
    }
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Task24 Isaac demo exception: %s", error.what());
  }

  scene.removeCollisionObjects({"task24_side_contact_demo_cube"});
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}

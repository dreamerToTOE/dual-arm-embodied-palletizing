#include <array>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "fr3_dual_palletize/palletize_primitive.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr std::size_t NUM_BOXES = 2;
constexpr double BOX_SIZE = 0.030;

constexpr double LEFT_TARGET_X = 0.820;
constexpr double LEFT_TARGET_Y = -0.250;
constexpr double RIGHT_TARGET_X = 0.820;
constexpr double RIGHT_TARGET_Y = 0.250;

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "无法连接 /move_group，请先启动 Task06 双臂 MoveIt2。");
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
    return false;
  }

  if (!node->has_parameter("robot_description"))
  {
    node->declare_parameter<std::string>(
      "robot_description", params[0].as_string());
  }
  if (!node->has_parameter("robot_description_semantic"))
  {
    node->declare_parameter<std::string>(
      "robot_description_semantic", params[1].as_string());
  }
  return true;
}

moveit_msgs::msg::CollisionObject makeBoxObject(
  const std::string& id,
  const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;

  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {BOX_SIZE, BOX_SIZE, BOX_SIZE};

  object.primitives.push_back(shape);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

class BoxPoseBuffer
{
public:
  explicit BoxPoseBuffer(const rclcpp::Node::SharedPtr& node)
  {
    sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "/task07/box_poses",
      10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg)
      {
        if (msg->poses.size() < NUM_BOXES)
        {
          return;
        }

        {
          std::lock_guard<std::mutex> lock(mutex_);
          poses_[0] = msg->poses[0];
          poses_[1] = msg->poses[1];
          ready_ = true;
        }
        cv_.notify_all();
      });
  }

  bool wait(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(
      lock,
      std::chrono::duration<double>(timeout_sec),
      [this]() { return ready_; });
  }

  geometry_msgs::msg::Pose get(std::size_t index) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return poses_.at(index);
  }

  std::array<geometry_msgs::msg::Pose, NUM_BOXES> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return poses_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::array<geometry_msgs::msg::Pose, NUM_BOXES> poses_{};
  bool ready_{false};
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
};

bool addInitialBoxesToPlanningScene(
  const std::array<geometry_msgs::msg::Pose, NUM_BOXES>& poses)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  // 只管理 Task07 自己的箱体。Task06 已加载并维护桌面，不在这里重复创建环境。
  psi.removeCollisionObjects({"task07_box_a", "task07_box_b"});
  std::this_thread::sleep_for(200ms);

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.push_back(makeBoxObject("task07_box_a", poses[0]));
  objects.push_back(makeBoxObject("task07_box_b", poses[1]));

  if (!psi.applyCollisionObjects(objects))
  {
    return false;
  }

  std::this_thread::sleep_for(500ms);
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto pose_node = std::make_shared<rclcpp::Node>("task07_box_pose_buffer");
  auto left_node = std::make_shared<rclcpp::Node>("task07_left_primitive");
  auto right_node = std::make_shared<rclcpp::Node>("task07_right_primitive");

  if (!copyRobotModelParameters(left_node) ||
      !copyRobotModelParameters(right_node))
  {
    rclcpp::shutdown();
    return 1;
  }

  auto pose_buffer = std::make_shared<BoxPoseBuffer>(pose_node);

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(pose_node);
  executor.add_node(left_node);
  executor.add_node(right_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  RCLCPP_INFO(
    pose_node->get_logger(),
    "========== Task07：松协调双臂并行基线 ==========");
  RCLCPP_INFO(
    pose_node->get_logger(),
    "重点：复用 Task04 primitive，只验证两个完整抓放任务能否真正并发。"
  );

  if (!pose_buffer->wait(10.0))
  {
    RCLCPP_ERROR(
      pose_node->get_logger(),
      "等待 /task07/box_poses 超时。请先运行 Isaac Task07 bridge。"
    );
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto initial_poses = pose_buffer->snapshot();
  if (!addInitialBoxesToPlanningScene(initial_poses))
  {
    RCLCPP_ERROR(pose_node->get_logger(), "Task07 初始箱体加入 Planning Scene 失败。");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto scene_mutex = std::make_shared<std::mutex>();

  fr3_dual_palletize::PrimitiveConfig left_config;
  left_config.label = "LEFT / BoxA";
  left_config.planning_group = "left_arm";
  left_config.eef_link = "left_fr3_link8";
  left_config.tool_link = "left_fr3_compact_suction";
  left_config.moveit_joint_prefix = "left_";
  left_config.joint_command_topic = "/left/joint_command";
  left_config.suction_command_topic = "/task07/left/suction_command";
  left_config.suction_state_topic = "/task07/left/suction_state";
  left_config.object_id = "task07_box_a";
  left_config.pose_index = 0;
  left_config.target_x = LEFT_TARGET_X;
  left_config.target_y = LEFT_TARGET_Y;

  fr3_dual_palletize::PrimitiveConfig right_config;
  right_config.label = "RIGHT / BoxB";
  right_config.planning_group = "right_arm";
  right_config.eef_link = "right_fr3_link8";
  right_config.tool_link = "right_fr3_compact_suction";
  right_config.moveit_joint_prefix = "right_";
  right_config.joint_command_topic = "/right/joint_command";
  right_config.suction_command_topic = "/task07/right/suction_command";
  right_config.suction_state_topic = "/task07/right/suction_state";
  right_config.object_id = "task07_box_b";
  right_config.pose_index = 1;
  right_config.target_x = RIGHT_TARGET_X;
  right_config.target_y = RIGHT_TARGET_Y;

  auto provider = [pose_buffer](std::size_t index)
  {
    return pose_buffer->get(index);
  };

  fr3_dual_palletize::PalletizePrimitive left(
    left_node, left_config, provider, scene_mutex);
  fr3_dual_palletize::PalletizePrimitive right(
    right_node, right_config, provider, scene_mutex);

  // 第一段 PRE_PICK 先各自规划，随后通过 gate 同时开始真实运动。
  // Task08 才增加“时空冲突检测”，Task07 不提前掺入避障调度算法。
  fr3_dual_palletize::StartGate start_gate(2);

  bool left_ok = false;
  bool right_ok = false;
  const auto wall_start = std::chrono::steady_clock::now();

  std::thread left_thread([&]() { left_ok = left.runOnce(start_gate); });
  std::thread right_thread([&]() { right_ok = right.runOnce(start_gate); });

  left_thread.join();
  right_thread.join();

  const double wall_time = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - wall_start).count();

  if (left_ok && right_ok)
  {
    RCLCPP_INFO(
      pose_node->get_logger(),
      "========== Task07 SUCCESS ==========");
    RCLCPP_INFO(
      pose_node->get_logger(),
      "Left primitive=PASS, Right primitive=PASS, wall_time=%.3f s",
      wall_time);
    RCLCPP_INFO(
      pose_node->get_logger(),
      "Task07 只证明无冲突并行执行；下一步 Task08 主动制造共同工作区冲突。"
    );
  }
  else
  {
    RCLCPP_ERROR(
      pose_node->get_logger(),
      "Task07 FAIL: left=%s, right=%s",
      left_ok ? "PASS" : "FAIL",
      right_ok ? "PASS" : "FAIL");
  }

  executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return (left_ok && right_ok) ? 0 : 1;
}

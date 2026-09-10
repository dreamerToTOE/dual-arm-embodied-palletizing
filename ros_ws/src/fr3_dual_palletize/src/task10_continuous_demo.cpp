#include <array>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "fr3_dual_palletize/coordinated_task_executor.hpp"
#include "fr3_dual_palletize/local_wait_coordinator.hpp"
#include "fr3_dual_palletize/palletize_primitive.hpp"
#include "fr3_dual_palletize/spatiotemporal_conflict_detector.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr std::size_t NUM_BOXES = 4;
constexpr double BOX_SIZE = 0.030;

struct BatchConfig
{
  std::string name;
  std::string left_label;
  std::string right_label;
  std::string left_object_id;
  std::string right_object_id;
  std::size_t left_pose_index{0};
  std::size_t right_pose_index{0};
  double left_target_x{0.0};
  double left_target_y{0.0};
  double right_target_x{0.0};
  double right_target_y{0.0};
};

const std::array<BatchConfig, 2> BATCHES{{
  {
    "BATCH_1_CROSS",
    "TASK10 BATCH1 LEFT / BoxA",
    "TASK10 BATCH1 RIGHT / BoxB",
    "task10_box_a",
    "task10_box_b",
    0,
    1,
    0.820,
    +0.120,
    0.820,
    -0.120,
  },
  {
    "BATCH_2_WITH_PLACED_OBSTACLES",
    "TASK10 BATCH2 LEFT / BoxC",
    "TASK10 BATCH2 RIGHT / BoxD",
    "task10_box_c",
    "task10_box_d",
    2,
    3,
    0.740,
    +0.120,
    0.740,
    -0.120,
  },
}};

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "无法连接 /move_group。请先启动 Task06 双臂 MoveIt2。");
    return false;
  }

  const auto parameters = client->get_parameters({
    "robot_description",
    "robot_description_semantic",
  });
  if (parameters.size() != 2 ||
      parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(node->get_logger(), "无法从 /move_group 读取 robot_description 参数。");
    return false;
  }

  if (!node->has_parameter("robot_description"))
  {
    node->declare_parameter<std::string>(
      "robot_description", parameters[0].as_string());
  }
  if (!node->has_parameter("robot_description_semantic"))
  {
    node->declare_parameter<std::string>(
      "robot_description_semantic", parameters[1].as_string());
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
    subscription_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "/task10/box_poses",
      10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message)
      {
        if (message->poses.size() < NUM_BOXES)
        {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          for (std::size_t index = 0; index < NUM_BOXES; ++index)
          {
            poses_[index] = message->poses[index];
          }
          ready_ = true;
        }
        condition_.notify_all();
      });
  }

  bool wait(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
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
  std::condition_variable condition_;
  std::array<geometry_msgs::msg::Pose, NUM_BOXES> poses_{};
  bool ready_{false};
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

bool initializeTask10World(
  const std::array<geometry_msgs::msg::Pose, NUM_BOXES>& poses)
{
  moveit::planning_interface::PlanningSceneInterface scene_interface;
  scene_interface.removeCollisionObjects({
    "task07_box_a", "task07_box_b",
    "task08_box_a", "task08_box_b",
    "task10_box_a", "task10_box_b", "task10_box_c", "task10_box_d",
  });
  std::this_thread::sleep_for(300ms);

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.reserve(NUM_BOXES);
  objects.push_back(makeBoxObject("task10_box_a", poses[0]));
  objects.push_back(makeBoxObject("task10_box_b", poses[1]));
  objects.push_back(makeBoxObject("task10_box_c", poses[2]));
  objects.push_back(makeBoxObject("task10_box_d", poses[3]));
  if (!scene_interface.applyCollisionObjects(objects))
  {
    return false;
  }
  std::this_thread::sleep_for(500ms);
  return true;
}

void logConflictReport(
  const rclcpp::Logger& logger,
  const fr3_dual_palletize::ConflictReport& report)
{
  if (!report.valid)
  {
    RCLCPP_ERROR(logger, "Task10 FCL 检查无效：%s", report.error.c_str());
    return;
  }
  if (!report.conflict)
  {
    RCLCPP_INFO(logger, "Task10 FCL: SAFE / NO_CONFLICT");
    return;
  }
  RCLCPP_WARN(
    logger,
    "Task10 FCL: CONFLICT first=%.3f s, windows=%zu",
    report.first_conflict_time_sec,
    report.conflict_windows.size());
  for (const auto& window : report.conflict_windows)
  {
    RCLCPP_WARN(
      logger,
      "  window=[%.3f, %.3f] %s <-> %s (%s)",
      window.start_time_sec,
      window.end_time_sec,
      window.body_a.c_str(),
      window.body_b.c_str(),
      window.type.c_str());
  }
}

fr3_dual_palletize::PrimitiveConfig makePrimitiveConfig(
  const BatchConfig& batch,
  bool left)
{
  fr3_dual_palletize::PrimitiveConfig config;
  config.label = left ? batch.left_label : batch.right_label;
  config.planning_group = left ? "left_arm" : "right_arm";
  config.eef_link = left ? "left_fr3_link8" : "right_fr3_link8";
  config.tool_link = left ? "left_fr3_compact_suction" : "right_fr3_compact_suction";
  config.moveit_joint_prefix = left ? "left_" : "right_";
  config.joint_command_topic = left ? "/left/joint_command" : "/right/joint_command";
  config.suction_command_topic = left ?
    "/task10/left/suction_command" : "/task10/right/suction_command";
  config.suction_state_topic = left ?
    "/task10/left/suction_state" : "/task10/right/suction_state";
  config.object_id = left ? batch.left_object_id : batch.right_object_id;
  config.pose_index = left ? batch.left_pose_index : batch.right_pose_index;
  config.target_x = left ? batch.left_target_x : batch.right_target_x;
  config.target_y = left ? batch.left_target_y : batch.right_target_y;
  config.include_safe_egress = true;
  return config;
}

bool runBatch(
  const BatchConfig& batch,
  const rclcpp::Node::SharedPtr& coordinator_node,
  const rclcpp::Node::SharedPtr& left_node,
  const rclcpp::Node::SharedPtr& right_node,
  const std::shared_ptr<BoxPoseBuffer>& pose_buffer,
  const std::shared_ptr<std::mutex>& scene_mutex,
  bool execute,
  double wait_step_sec,
  double max_wait_sec)
{
  RCLCPP_INFO(
    coordinator_node->get_logger(),
    "========== Task10 %s START ==========" , batch.name.c_str());

  auto provider = [pose_buffer](std::size_t index)
  {
    return pose_buffer->get(index);
  };
  fr3_dual_palletize::PalletizePrimitive left(
    left_node, makePrimitiveConfig(batch, true), provider, scene_mutex);
  fr3_dual_palletize::PalletizePrimitive right(
    right_node, makePrimitiveConfig(batch, false), provider, scene_mutex);

  fr3_dual_palletize::TaskTrajectoryCandidate left_candidate;
  fr3_dual_palletize::TaskTrajectoryCandidate right_candidate;
  if (!left.planTaskTrajectoryCandidate(left_candidate) ||
      !right.planTaskTrajectoryCandidate(right_candidate))
  {
    RCLCPP_ERROR(coordinator_node->get_logger(), "Task10 %s：单臂候选规划失败。", batch.name.c_str());
    return false;
  }

  moveit::planning_interface::PlanningSceneInterface scene_interface;
  std::vector<moveit_msgs::msg::CollisionObject> static_objects;
  for (const auto& [id, object] : scene_interface.getObjects())
  {
    static_cast<void>(id);
    static_objects.push_back(object);
  }

  moveit::planning_interface::MoveGroupInterface detector_group(left_node, "left_arm");
  fr3_dual_palletize::SpatioTemporalConflictDetector detector(
    detector_group.getRobotModel(), std::move(static_objects));
  const auto original_report = detector.check(left_candidate, right_candidate, 0.01);
  logConflictReport(coordinator_node->get_logger(), original_report);
  if (!original_report.valid)
  {
    return false;
  }

  fr3_dual_palletize::LocalWaitCoordinationConfig coordination_config;
  coordination_config.wait_step_sec = wait_step_sec;
  coordination_config.max_wait_sec = max_wait_sec;
  coordination_config.prefer_left = true;
  fr3_dual_palletize::LocalWaitCoordinator coordinator(detector);
  const auto coordination = coordinator.solve(
    left_candidate, right_candidate, coordination_config);
  if (!coordination.valid || !coordination.coordinated ||
      coordination.verification_report.conflict)
  {
    RCLCPP_ERROR(
      coordinator_node->get_logger(),
      "Task10 %s：没有得到 FCL SAFE 的局部协调候选：%s",
      batch.name.c_str(), coordination.error.c_str());
    return false;
  }

  RCLCPP_INFO(
    coordinator_node->get_logger(),
    "Task10 %s：FCL SAFE，strategy=%s, yielding=%s, wait=%.3f s, makespan=%.3f s",
    batch.name.c_str(),
    fr3_dual_palletize::localWaitStrategyName(coordination.strategy),
    coordination.yielding_arm.c_str(),
    coordination.wait_duration_sec,
    coordination.coordinated_makespan_sec);

  if (!execute)
  {
    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "Task10 %s：仅验证模式，未发布 joint/suction command。",
      batch.name.c_str());
    return true;
  }

  fr3_dual_palletize::CoordinatedTaskExecutor executor(
    coordinator_node->get_logger(), left, right);
  const auto execution = executor.execute(
    coordination.coordinated_left, coordination.coordinated_right);
  if (!execution.valid || !execution.completed)
  {
    RCLCPP_ERROR(
      coordinator_node->get_logger(),
      "Task10 %s：真实执行失败：%s",
      batch.name.c_str(), execution.error.c_str());
    return false;
  }

  RCLCPP_INFO(
    coordinator_node->get_logger(),
    "Task10 %s PASS：实际执行完成，events=%zu, wall=%.3f s",
    batch.name.c_str(), execution.events_executed, execution.wall_duration_sec);
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto coordinator_node = std::make_shared<rclcpp::Node>("task10_continuous_coordinator");
  auto left_node = std::make_shared<rclcpp::Node>("task10_left_primitive");
  auto right_node = std::make_shared<rclcpp::Node>("task10_right_primitive");

  const bool execute = coordinator_node->declare_parameter<bool>("execute", false);
  const double wait_step_sec = coordinator_node->declare_parameter<double>("wait_step_sec", 0.20);
  const double max_wait_sec = coordinator_node->declare_parameter<double>("max_wait_sec", 15.0);
  if (wait_step_sec <= 0.0 || max_wait_sec < wait_step_sec)
  {
    RCLCPP_ERROR(coordinator_node->get_logger(), "Task10 local wait 参数无效。");
    rclcpp::shutdown();
    return 1;
  }

  if (!copyRobotModelParameters(left_node) || !copyRobotModelParameters(right_node))
  {
    rclcpp::shutdown();
    return 1;
  }

  auto pose_buffer = std::make_shared<BoxPoseBuffer>(coordinator_node);
  rclcpp::executors::MultiThreadedExecutor ros_executor(
    rclcpp::ExecutorOptions(), 4);
  ros_executor.add_node(coordinator_node);
  ros_executor.add_node(left_node);
  ros_executor.add_node(right_node);
  std::thread spin_thread([&ros_executor]() { ros_executor.spin(); });

  bool success = false;
  do
  {
    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "========== Task10 CONTINUOUS PALLETIZING ==========");
    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "四箱 / 两批次，execute=%s；每批均先经 Task08-B/FCL 验收。",
      execute ? "true" : "false");

    if (!pose_buffer->wait(10.0))
    {
      RCLCPP_ERROR(
        coordinator_node->get_logger(),
        "等待 /task10/box_poses 超时。请启动 Task10 四箱 Isaac bridge。");
      break;
    }
    if (!initializeTask10World(pose_buffer->snapshot()))
    {
      RCLCPP_ERROR(coordinator_node->get_logger(), "Task10 初始四箱加入 Planning Scene 失败。");
      break;
    }

    const auto scene_mutex = std::make_shared<std::mutex>();
    success = true;
    for (const auto& batch : BATCHES)
    {
      if (!runBatch(
            batch,
            coordinator_node,
            left_node,
            right_node,
            pose_buffer,
            scene_mutex,
            execute,
            wait_step_sec,
            max_wait_sec))
      {
        success = false;
        break;
      }
    }
  } while (false);

  if (success)
  {
    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "Task10 PASS：两批次连续松协调任务全部完成。");
  }
  else
  {
    RCLCPP_ERROR(coordinator_node->get_logger(), "Task10 FAIL：停止后续批次，未尝试恢复。" );
  }

  ros_executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}

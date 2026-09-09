#include <array>
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
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
#include "fr3_dual_palletize/spatiotemporal_conflict_detector.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr std::size_t NUM_BOXES = 2;
constexpr double BOX_SIZE = 0.030;

// Task08 交叉目标：最终目标不重叠，但 transfer 会进入公共工作区。
constexpr double LEFT_TARGET_X = 0.820;
constexpr double LEFT_TARGET_Y = 0.120;
constexpr double RIGHT_TARGET_X = 0.820;
constexpr double RIGHT_TARGET_Y = -0.120;

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
    // Task08 暂时复用 Task07 已验证双吸盘 bridge，因此 topic 保持 /task07/box_poses。
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

  // 防止上一轮 Task07/Task08 world object 残留。
  psi.removeCollisionObjects({
    "task07_box_a", "task07_box_b",
    "task08_box_a", "task08_box_b"
  });
  std::this_thread::sleep_for(200ms);

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.push_back(makeBoxObject("task08_box_a", poses[0]));
  objects.push_back(makeBoxObject("task08_box_b", poses[1]));

  if (!psi.applyCollisionObjects(objects))
  {
    return false;
  }

  std::this_thread::sleep_for(500ms);
  return true;
}

std::string formatVector(const std::vector<double>& values)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(4) << "[";
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i > 0)
    {
      out << ", ";
    }
    out << values[i];
  }
  out << "]";
  return out.str();
}

std::string formatNames(const std::vector<std::string>& names)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    if (i > 0)
    {
      out << ", ";
    }
    out << names[i];
  }
  out << "]";
  return out.str();
}

void logCandidate(
  const rclcpp::Logger& logger,
  const fr3_dual_palletize::TaskTrajectoryCandidate& candidate)
{
  const auto names = formatNames(candidate.trajectory.joint_names);
  const auto start_q = formatVector(candidate.start_q);
  const auto goal_q = formatVector(candidate.goal_q);

  RCLCPP_INFO(logger, "--------------------------------------------");
  RCLCPP_INFO(logger, "FULL TASK CANDIDATE: %s", candidate.label.c_str());
  RCLCPP_INFO(logger, "group        = %s", candidate.planning_group.c_str());
  RCLCPP_INFO(logger, "object       = %s", candidate.object_id.c_str());
  RCLCPP_INFO(logger, "eef_link     = %s", candidate.eef_link.c_str());
  RCLCPP_INFO(logger, "joint_names  = %s", names.c_str());
  RCLCPP_INFO(logger, "points       = %zu", candidate.trajectory.points.size());
  RCLCPP_INFO(logger, "duration     = %.3f s", candidate.duration_sec);
  RCLCPP_INFO(logger, "start_q      = %s", start_q.c_str());
  RCLCPP_INFO(logger, "goal_q       = %s", goal_q.c_str());
  RCLCPP_INFO(
    logger,
    "release_pose = (%.3f, %.3f, %.3f)",
    candidate.planned_release_pose.position.x,
    candidate.planned_release_pose.position.y,
    candidate.planned_release_pose.position.z);

  for (const auto& event : candidate.events)
  {
    RCLCPP_INFO(
      logger,
      "event        = t=%.3f s, %s, object=%s, link=%s",
      event.time_sec,
      fr3_dual_palletize::taskEventTypeName(event.type),
      event.object_name.c_str(),
      event.link_name.c_str());
  }
}

void logConflictReport(
  const rclcpp::Logger& logger,
  const fr3_dual_palletize::ConflictReport& report)
{
  RCLCPP_INFO(logger, "========== Task08-B CONFLICT REPORT ==========");
  RCLCPP_INFO(logger, "horizon         = %.3f s", report.horizon_sec);
  RCLCPP_INFO(logger, "sample_period   = %.3f s", report.sample_period_sec);
  RCLCPP_INFO(logger, "samples_checked = %zu", report.samples_checked);
  RCLCPP_INFO(logger, "detector_wall   = %.3f s", report.wall_time_sec);

  if (!report.valid)
  {
    RCLCPP_ERROR(logger, "Task08-B INVALID: %s", report.error.c_str());
    return;
  }

  if (!report.conflict)
  {
    RCLCPP_INFO(logger, "Task08-B SAFE: NO_CONFLICT");
    return;
  }

  RCLCPP_WARN(
    logger,
    "Task08-B CONFLICT: first_time = %.3f s, events_at_first_sample = %zu",
    report.first_conflict_time_sec,
    report.events.size());
  for (const auto& event : report.events)
  {
    RCLCPP_WARN(
      logger,
      "pair = %s <-> %s, type = %s, t = %.3f s",
      event.body_a.c_str(),
      event.body_b.c_str(),
      event.type.c_str(),
      event.time_sec);
  }
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto pose_node = std::make_shared<rclcpp::Node>("task08_candidate_coordinator");
  auto left_node = std::make_shared<rclcpp::Node>("task08_left_primitive");
  auto right_node = std::make_shared<rclcpp::Node>("task08_right_primitive");

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
    "========== Task08-A：完整 TaskTrajectoryCandidate 输出 ==========");
  RCLCPP_INFO(
    pose_node->get_logger(),
    "生成 HOME -> PRE_PICK -> CONTACT -> LIFT -> PRE_PLACE -> PLACE -> RETREAT；"
    "仅规划，不发布 joint command 或 suction command。"
  );

  if (!pose_buffer->wait(10.0))
  {
    RCLCPP_ERROR(
      pose_node->get_logger(),
      "等待 /task07/box_poses 超时。请先运行 Task07 双吸盘 bridge。"
    );
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto initial_poses = pose_buffer->snapshot();
  if (!addInitialBoxesToPlanningScene(initial_poses))
  {
    RCLCPP_ERROR(pose_node->get_logger(), "Task08 初始箱体加入 Planning Scene 失败。");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto scene_mutex = std::make_shared<std::mutex>();

  fr3_dual_palletize::PrimitiveConfig left_config;
  left_config.label = "TASK08 LEFT / BoxA";
  left_config.planning_group = "left_arm";
  left_config.eef_link = "left_fr3_link8";
  left_config.tool_link = "left_fr3_compact_suction";
  left_config.moveit_joint_prefix = "left_";
  left_config.joint_command_topic = "/left/joint_command";
  left_config.suction_command_topic = "/task07/left/suction_command";
  left_config.suction_state_topic = "/task07/left/suction_state";
  left_config.object_id = "task08_box_a";
  left_config.pose_index = 0;
  left_config.target_x = LEFT_TARGET_X;
  left_config.target_y = LEFT_TARGET_Y;

  fr3_dual_palletize::PrimitiveConfig right_config;
  right_config.label = "TASK08 RIGHT / BoxB";
  right_config.planning_group = "right_arm";
  right_config.eef_link = "right_fr3_link8";
  right_config.tool_link = "right_fr3_compact_suction";
  right_config.moveit_joint_prefix = "right_";
  right_config.joint_command_topic = "/right/joint_command";
  right_config.suction_command_topic = "/task07/right/suction_command";
  right_config.suction_state_topic = "/task07/right/suction_state";
  right_config.object_id = "task08_box_b";
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

  fr3_dual_palletize::TaskTrajectoryCandidate left_candidate;
  fr3_dual_palletize::TaskTrajectoryCandidate right_candidate;

  // 纯规划阶段会临时切换单个 Box 的 MoveIt AttachedCollisionObject，
  // 因此串行生成，避免两个候选在同一 Planning Scene 中互相污染。
  const bool left_ok = left.planTaskTrajectoryCandidate(left_candidate);
  const bool right_ok = left_ok &&
    right.planTaskTrajectoryCandidate(right_candidate);

  if (left_ok && right_ok)
  {
    RCLCPP_INFO(
      pose_node->get_logger(),
      "========== Task08-A CANDIDATES READY ==========");
    logCandidate(pose_node->get_logger(), left_candidate);
    logCandidate(pose_node->get_logger(), right_candidate);
    RCLCPP_INFO(pose_node->get_logger(), "--------------------------------------------");
    RCLCPP_INFO(
      pose_node->get_logger(),
      "PASS：两条完整任务轨迹与 ATTACH/DETACH 事件已经生成；未执行任何机器人或吸盘命令。"
    );
    RCLCPP_INFO(
      pose_node->get_logger(),
      "Task08-A PASS：开始 Task08-B，只读检查两个候选的联合时空碰撞。"
    );

    // 将 MoveIt 当前 world object 快照复制到 Detector 的私有 PlanningScene。
    // 其中两个 task08 Box 会在每个时间采样点按 ATTACH/DETACH 事件重新创建，
    // Detector 绝不会向 move_group 或 Isaac 回写任何状态。
    moveit::planning_interface::PlanningSceneInterface scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> static_world_objects;
    for (const auto& [id, object] : scene_interface.getObjects())
    {
      static_cast<void>(id);
      static_world_objects.push_back(object);
    }

    moveit::planning_interface::MoveGroupInterface detector_group(
      left_node, "left_arm");
    const auto robot_model = detector_group.getRobotModel();
    fr3_dual_palletize::SpatioTemporalConflictDetector detector(
      robot_model, std::move(static_world_objects));
    const auto report = detector.check(left_candidate, right_candidate, 0.01);
    logConflictReport(pose_node->get_logger(), report);

    if (!report.valid)
    {
      RCLCPP_ERROR(
        pose_node->get_logger(),
        "Task08-B FAIL：Detector 未生成有效 ConflictReport。"
      );
      executor.cancel();
      if (spin_thread.joinable())
      {
        spin_thread.join();
      }
      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(
      pose_node->get_logger(),
      "Task08-B PASS：已完成联合预测；无论 SAFE 或 CONFLICT，均未执行任何机器人或吸盘命令。"
    );
  }
  else
  {
    RCLCPP_ERROR(
      pose_node->get_logger(),
      "Task08-A FAIL: left_candidate=%s, right_candidate=%s",
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

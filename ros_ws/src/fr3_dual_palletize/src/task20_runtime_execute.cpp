// Task20-C/D：运行时对象的真实 Isaac 执行验收。
//
// 安全边界：本节点先复用 Task20-B 的完整 Task16 / Task08-FCL 或 SharedObjectPlanner
// preflight，只有 Router 接受后才发布命令。Task20-D 连续多对象调度在每次真实
// 落稳后重新读取 Ground Truth、重建 PlacementSpec 并重新做全局 preflight；它不会
// 将首件的规划结果静默复用于后续物体。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/coordination_router.hpp"
#include "fr3_dual_palletize/msg/box_state_array.hpp"
#include "fr3_dual_palletize/palletize_primitive.hpp"
#include "fr3_dual_palletize/placement_planner.hpp"
#include "fr3_dual_palletize/runtime_box_state.hpp"
#include "fr3_dual_palletize/shared_object_executor.hpp"
#include "fr3_dual_palletize/shared_object_planner.hpp"
#include "fr3_dual_palletize/spatiotemporal_conflict_detector.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr double FCL_SAMPLE_PERIOD_SEC = 0.010;
constexpr double SCENE_SYNC_WAIT_SEC = 0.30;
constexpr double SOURCE_POSE_STALE_TOLERANCE_M = 0.003;

enum class Arm
{
  LEFT,
  RIGHT,
};

struct ArmCandidate
{
  Arm arm{Arm::LEFT};
  fr3_dual_palletize::TaskTrajectoryCandidate candidate;
  double duration_sec{std::numeric_limits<double>::infinity()};
};

class InputBuffer
{
public:
  explicit InputBuffer(const rclcpp::Node::SharedPtr& node)
  {
    subscription_ = node->create_subscription<fr3_dual_palletize::msg::BoxStateArray>(
      "/task18/box_states", rclcpp::QoS(10),
      [this](const fr3_dual_palletize::msg::BoxStateArray::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = *message;
          received_ = true;
        }
        condition_.notify_all();
      });
  }

  bool wait(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock, std::chrono::duration<double>(timeout_sec), [this]() { return received_; });
  }

  fr3_dual_palletize::msg::BoxStateArray state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  bool poseFor(const std::string& object_id, geometry_msgs::msg::Pose& pose) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = std::find_if(
      state_.boxes.begin(), state_.boxes.end(),
      [&object_id](const auto& box) { return box.id == object_id; });
    if (it == state_.boxes.end())
    {
      return false;
    }
    pose = it->pose;
    return true;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool received_{false};
  fr3_dual_palletize::msg::BoxStateArray state_;
  rclcpp::Subscription<fr3_dual_palletize::msg::BoxStateArray>::SharedPtr subscription_;
};

const char* armName(Arm arm)
{
  return arm == Arm::LEFT ? "left" : "right";
}

Arm oppositeArm(Arm arm)
{
  return arm == Arm::LEFT ? Arm::RIGHT : Arm::LEFT;
}

bool allowsMode(const fr3_dual_palletize::BoxSpec& box, const std::string& mode)
{
  return std::find(box.allowed_modes.begin(), box.allowed_modes.end(), mode) !=
    box.allowed_modes.end();
}

double volume(const fr3_dual_palletize::BoxSpec& box)
{
  return box.dimensions[0] * box.dimensions[1] * box.dimensions[2];
}

double positionDistance(
  const geometry_msgs::msg::Point& first,
  const geometry_msgs::msg::Point& second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
  const double sin_yaw = 2.0 * (
    quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "Task20-C 无法连接 /move_group。请先启动 Task06 MoveIt2。");
    return false;
  }
  const auto parameters = client->get_parameters({
    "robot_description", "robot_description_semantic",
  });
  if (parameters.size() != 2 ||
      parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(node->get_logger(), "Task20-C 无法读取 /move_group RobotModel 参数。");
    return false;
  }
  if (!node->has_parameter("robot_description"))
  {
    node->declare_parameter<std::string>("robot_description", parameters[0].as_string());
  }
  if (!node->has_parameter("robot_description_semantic"))
  {
    node->declare_parameter<std::string>(
      "robot_description_semantic", parameters[1].as_string());
  }
  return true;
}

fr3_dual_palletize::PrimitiveConfig makeConfig(
  Arm arm,
  const fr3_dual_palletize::BoxSpec& box,
  const fr3_dual_palletize::PlacementSpec& placement,
  const std::string& task_label)
{
  const bool left = arm == Arm::LEFT;
  fr3_dual_palletize::PrimitiveConfig config;
  config.label = task_label + " " + std::string(armName(arm)) + " / " + box.id;
  config.planning_group = left ? "left_arm" : "right_arm";
  config.eef_link = left ? "left_fr3_link8" : "right_fr3_link8";
  config.tool_link = left ? "left_fr3_compact_suction" : "right_fr3_compact_suction";
  config.moveit_joint_prefix = left ? "left_" : "right_";
  config.joint_command_topic = left ? "/left/joint_command" : "/right/joint_command";
  config.suction_command_topic = left ? "/task20/left/suction_command" :
    "/task20/right/suction_command";
  config.suction_state_topic = left ? "/task20/left/suction_state" :
    "/task20/right/suction_state";
  config.suction_tcp_pose_topic = left ? "/task20/left/suction_tcp_pose" :
    "/task20/right/suction_tcp_pose";
  config.object_id = box.id;
  config.object_dimensions = box.dimensions;
  config.target_x = placement.target_pose.position.x;
  config.target_y = placement.target_pose.position.y;
  config.target_yaw = yawFromQuaternion(placement.target_pose.orientation);
  config.target_support_surface_z = placement.support_height;
  config.include_safe_egress = true;
  // 每一件已通过 Isaac 放置误差门禁的运行时物体都锁为保留 Collider 的稳定支撑面。
  // 下一件的 PlacementPlanner 和 FCL 均必须把它当作真实障碍/支撑物。
  config.freeze_after_settle = true;
  config.stable_support_lock_topic = "/task20/lock_placed_object";
  config.robust_planner.candidate_count = 3;
  config.robust_planner.redundancy_mode =
    fr3_dual_palletize::RedundancyMode::SOFT_PREFERENCE;
  config.robust_planner.preferred_redundant_joint = 0.0;
  return config;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
  point.time_from_start.sec = static_cast<std::int32_t>(std::floor(seconds));
  point.time_from_start.nanosec = static_cast<std::uint32_t>(std::llround(
    (seconds - std::floor(seconds)) * 1.0e9));
}

fr3_dual_palletize::TaskTrajectoryCandidate makeIdleCandidate(
  Arm arm,
  const std::vector<double>& q,
  double duration_sec)
{
  const bool left = arm == Arm::LEFT;
  fr3_dual_palletize::TaskTrajectoryCandidate candidate;
  candidate.label = "Task20-C passive " + std::string(armName(arm));
  candidate.arm_name = left ? "left_arm" : "right_arm";
  candidate.planning_group = candidate.arm_name;
  candidate.object_id = "__task20_passive_" + std::string(armName(arm));
  candidate.eef_link = left ? "left_fr3_link8" : "right_fr3_link8";
  candidate.object_dimensions = {{0.001, 0.001, 0.001}};
  candidate.initial_object_pose.position.x = 100.0;
  candidate.initial_object_pose.position.y = 100.0;
  candidate.initial_object_pose.position.z = 100.0;
  candidate.initial_object_pose.orientation.w = 1.0;
  candidate.planned_release_pose = candidate.initial_object_pose;
  for (int joint = 1; joint <= 7; ++joint)
  {
    candidate.trajectory.joint_names.push_back(
      std::string(left ? "left_fr3_joint" : "right_fr3_joint") + std::to_string(joint));
  }
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = q;
  setPointTime(start, 0.0);
  auto finish = start;
  setPointTime(finish, duration_sec);
  candidate.trajectory.points = {start, finish};
  candidate.start_q = q;
  candidate.goal_q = q;
  candidate.duration_sec = duration_sec;
  candidate.events.push_back({0.0, fr3_dual_palletize::TaskEventType::ATTACH,
    candidate.object_id, candidate.eef_link});
  candidate.events.push_back({0.0, fr3_dual_palletize::TaskEventType::DETACH,
    candidate.object_id, candidate.eef_link});
  return candidate;
}

std::vector<double> currentJointPositions(const rclcpp::Node::SharedPtr& node, Arm arm)
{
  moveit::planning_interface::MoveGroupInterface group(
    node, arm == Arm::LEFT ? "left_arm" : "right_arm");
  const auto* jmg = group.getRobotModel()->getJointModelGroup(
    arm == Arm::LEFT ? "left_arm" : "right_arm");
  const auto state = group.getCurrentState(3.0);
  if (!jmg || !state)
  {
    return {};
  }
  std::vector<double> q;
  state->copyJointGroupPositions(jmg, q);
  return q;
}

bool fclGate(
  const rclcpp::Node::SharedPtr& node,
  Arm active_arm,
  const fr3_dual_palletize::TaskTrajectoryCandidate& active,
  const std::vector<double>& passive_q,
  const std::string& task_label)
{
  if (passive_q.size() != 7)
  {
    return false;
  }
  moveit::planning_interface::PlanningSceneInterface scene;
  std::vector<moveit_msgs::msg::CollisionObject> static_objects;
  for (const auto& item : scene.getObjects())
  {
    static_objects.push_back(item.second);
  }
  moveit::planning_interface::MoveGroupInterface group(
    node, active_arm == Arm::LEFT ? "left_arm" : "right_arm");
  fr3_dual_palletize::SpatioTemporalConflictDetector detector(
    group.getRobotModel(), std::move(static_objects));
  const auto passive = makeIdleCandidate(oppositeArm(active_arm), passive_q, active.duration_sec);
  const auto report = active_arm == Arm::LEFT ?
    detector.check(active, passive, FCL_SAMPLE_PERIOD_SEC) :
    detector.check(passive, active, FCL_SAMPLE_PERIOD_SEC);
  if (!report.valid || report.conflict)
  {
    if (report.conflict && !report.events.empty())
    {
      const auto& event = report.events.front();
      RCLCPP_WARN(
        node->get_logger(), "%s FCL REJECT arm=%s t=%.3f pair=%s <-> %s",
        task_label.c_str(),
        armName(active_arm), report.first_conflict_time_sec,
        event.body_a.c_str(), event.body_b.c_str());
    }
    return false;
  }
  RCLCPP_INFO(
    node->get_logger(), "%s FCL SAFE arm=%s samples=%zu horizon=%.3f s",
    task_label.c_str(), armName(active_arm), report.samples_checked, report.horizon_sec);
  return true;
}

void removeExistingObjects(
  moveit::planning_interface::PlanningSceneInterface& scene,
  const std::vector<std::string>& ids)
{
  const auto existing = scene.getObjects(ids);
  std::vector<std::string> existing_ids;
  existing_ids.reserve(existing.size());
  for (const auto& item : existing)
  {
    existing_ids.push_back(item.first);
  }
  if (!existing_ids.empty())
  {
    scene.removeCollisionObjects(existing_ids);
  }
}

bool initializeRuntimeWorld(const std::vector<fr3_dual_palletize::BoxSpec>& boxes)
{
  moveit::planning_interface::PlanningSceneInterface scene;
  std::vector<std::string> ids;
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  for (const auto& box : boxes)
  {
    ids.push_back(box.id);
    objects.push_back(fr3_dual_palletize::makeCollisionObject(box, box.initial_pose));
  }
  removeExistingObjects(scene, ids);
  std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
  const bool applied = scene.applyCollisionObjects(objects);
  std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
  return applied;
}

bool planRuntimeRoute(
  const fr3_dual_palletize::BoxSpec& box,
  const fr3_dual_palletize::PlacementSpec& placement,
  const rclcpp::Node::SharedPtr& left_node,
  const rclcpp::Node::SharedPtr& right_node,
  const std::shared_ptr<std::mutex>& scene_mutex,
  double loose_payload_limit_kg,
  double tight_payload_limit_kg,
  ArmCandidate& loose,
  fr3_dual_palletize::SharedObjectPlan& tight,
  fr3_dual_palletize::CoordinationRouteDecision& route,
  const std::string& task_label)
{
  fr3_dual_palletize::CoordinationRouteRequest request;
  request.box = &box;
  request.placement = &placement;
  request.tight.allowed_by_task = allowsMode(box, "tight_shared_object");
  request.tight.dual_grasp_feasible = box.grasp_candidates.size() >= 2;
  request.tight.payload_safe = box.mass <= tight_payload_limit_kg;
  request.tight.shared_transport_planner_ready = request.tight.allowed_by_task;
  request.tight.diagnostics = "tight mode not evaluated";
  if (request.tight.allowed_by_task && request.tight.dual_grasp_feasible &&
      request.tight.payload_safe)
  {
    fr3_dual_palletize::SharedObjectPlanner planner(left_node, scene_mutex);
    if (planner.plan(box, placement, tight))
    {
      request.tight.geometry_safe = true;
      request.tight.estimated_duration_sec = tight.duration_sec;
      request.tight.planning_cost = tight.planning_cost;
      request.tight.diagnostics = tight.diagnostics;
    }
    else
    {
      request.tight.diagnostics = tight.diagnostics;
      RCLCPP_WARN(
        left_node->get_logger(),
        "%s tight object=%s: shared candidate REJECT: %s",
        task_label.c_str(), box.id.c_str(), request.tight.diagnostics.c_str());
    }
  }

  const auto evaluate_loose = [&](Arm arm, fr3_dual_palletize::LooseModeEstimate& estimate)
  {
    estimate.allowed_by_task = allowsMode(box, arm == Arm::LEFT ? "loose_left" : "loose_right");
    estimate.payload_safe = box.mass <= loose_payload_limit_kg;
    estimate.top_suction_stable = !box.grasp_candidates.empty();
    if (!estimate.allowed_by_task || !estimate.payload_safe || !estimate.top_suction_stable)
    {
      return;
    }
    const auto& node = arm == Arm::LEFT ? left_node : right_node;
    fr3_dual_palletize::PalletizePrimitive primitive(
      node, makeConfig(arm, box, placement, task_label),
      [pose = box.initial_pose](std::size_t) { return pose; }, scene_mutex);
    ArmCandidate trial;
    trial.arm = arm;
    if (!primitive.planTaskTrajectoryCandidate(trial.candidate))
    {
      estimate.diagnostics = "Task16 complete candidate failed";
      return;
    }
    const auto passive_q = currentJointPositions(
      arm == Arm::LEFT ? right_node : left_node, oppositeArm(arm));
    if (!fclGate(node, arm, trial.candidate, passive_q, task_label))
    {
      estimate.diagnostics = "Task08/FCL gate rejected complete candidate";
      return;
    }
    estimate.pick_reachable = true;
    estimate.place_reachable = true;
    estimate.fcl_safe = true;
    estimate.estimated_duration_sec = trial.candidate.duration_sec;
    estimate.planning_cost = trial.candidate.duration_sec;
    estimate.diagnostics = "Task16 + Task08/FCL preflight passed";
    trial.duration_sec = trial.candidate.duration_sec;
    if (trial.duration_sec < loose.duration_sec)
    {
      loose = std::move(trial);
    }
  };
  evaluate_loose(Arm::LEFT, request.left);
  evaluate_loose(Arm::RIGHT, request.right);

  const fr3_dual_palletize::CoordinationRouter router;
  route = router.decide(request);
  if (!route.feasible)
  {
    return false;
  }
  if (route.mode == fr3_dual_palletize::CoordinationMode::TIGHT_SHARED_OBJECT)
  {
    return tight.valid;
  }
  if (route.mode == fr3_dual_palletize::CoordinationMode::LOOSE_LEFT ||
      route.mode == fr3_dual_palletize::CoordinationMode::LOOSE_RIGHT)
  {
    const Arm selected = route.mode == fr3_dual_palletize::CoordinationMode::LOOSE_LEFT ?
      Arm::LEFT : Arm::RIGHT;
    return loose.duration_sec < std::numeric_limits<double>::infinity() && loose.arm == selected;
  }
  return false;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto coordinator = std::make_shared<rclcpp::Node>("task20_runtime_execute");
  auto left_node = std::make_shared<rclcpp::Node>("task20_runtime_left");
  auto right_node = std::make_shared<rclcpp::Node>("task20_runtime_right");

  const double timeout_sec = coordinator->declare_parameter<double>("timeout_sec", 60.0);
  const auto expected_seed = coordinator->declare_parameter<std::int64_t>("expected_seed", -1);
  const std::string execution_label = coordinator->declare_parameter<std::string>(
    "execution_label", "");
  const int max_tasks = coordinator->declare_parameter<int>("max_tasks", 1);
  const int task_plan_attempts = coordinator->declare_parameter<int>("task_plan_attempts", 3);
  const int max_placement_candidates_to_try = coordinator->declare_parameter<int>(
    "max_placement_candidates_to_try", 8);
  const double loose_payload_limit_kg = coordinator->declare_parameter<double>(
    "loose_payload_limit_kg", 0.50);
  const double tight_payload_limit_kg = coordinator->declare_parameter<double>(
    "tight_payload_limit_kg", 2.00);
  const double execution_time_scale = coordinator->declare_parameter<double>(
    "execution_time_scale", 2.0);
  fr3_dual_palletize::PalletRegion region;
  region.frame_id = "world";
  region.min_x = coordinator->declare_parameter<double>("pallet_min_x", 0.500);
  region.max_x = coordinator->declare_parameter<double>("pallet_max_x", 1.050);
  region.min_y = coordinator->declare_parameter<double>("pallet_min_y", -0.300);
  region.max_y = coordinator->declare_parameter<double>("pallet_max_y", 0.300);
  region.support_height = coordinator->declare_parameter<double>("pallet_support_height", 0.050);

  if (timeout_sec <= 0.0 || max_tasks <= 0 || task_plan_attempts <= 0 ||
      max_placement_candidates_to_try <= 0 || loose_payload_limit_kg <= 0.0 ||
      tight_payload_limit_kg <= 0.0 || execution_time_scale < 1.0 ||
      region.min_x >= region.max_x || region.min_y >= region.max_y)
  {
    RCLCPP_ERROR(
      coordinator->get_logger(),
      "Task20-C/D 参数无效：max_tasks、重试次数、候选上限与所有阈值必须为正。");
    rclcpp::shutdown();
    return 1;
  }
  if (!copyRobotModelParameters(left_node) || !copyRobotModelParameters(right_node))
  {
    rclcpp::shutdown();
    return 1;
  }

  InputBuffer input_buffer(coordinator);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(coordinator);
  executor.add_node(left_node);
  executor.add_node(right_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  bool success = false;
  do
  {
    const std::string task_label = !execution_label.empty() ? execution_label :
      (max_tasks == 1 ? "Task20-C" : "Task20-D");
    RCLCPP_INFO(
      coordinator->get_logger(),
      "%s runtime execution waiting /task18/box_states; real Isaac /joint_states required.",
      task_label.c_str());
    if (!input_buffer.wait(timeout_sec))
    {
      RCLCPP_ERROR(coordinator->get_logger(), "%s timeout waiting runtime Ground Truth.",
        task_label.c_str());
      break;
    }
    const auto input = input_buffer.state();
    if (input.boxes.empty() ||
        (expected_seed >= 0 && input.seed != static_cast<std::uint64_t>(expected_seed)))
    {
      RCLCPP_ERROR(
        coordinator->get_logger(),
        "%s input empty or seed mismatch: expected_seed=%ld received_seed=%lu boxes=%zu.",
        task_label.c_str(),
        static_cast<long>(expected_seed), static_cast<unsigned long>(input.seed),
        input.boxes.size());
      break;
    }
    std::vector<fr3_dual_palletize::BoxSpec> boxes;
    std::set<std::string> ids;
    for (const auto& state : input.boxes)
    {
      fr3_dual_palletize::BoxSpec box;
      std::string error;
      if (!ids.insert(state.id).second ||
          !fr3_dual_palletize::boxStateToBoxSpec(state, box, error))
      {
        RCLCPP_ERROR(coordinator->get_logger(), "%s invalid BoxState=%s: %s",
          task_label.c_str(), state.id.c_str(), error.c_str());
        boxes.clear();
        break;
      }
      boxes.push_back(std::move(box));
    }
    if (boxes.empty())
    {
      break;
    }
    std::stable_sort(
      boxes.begin(), boxes.end(),
      [](const auto& first, const auto& second) { return volume(first) > volume(second); });

    if (!initializeRuntimeWorld(boxes))
    {
      RCLCPP_ERROR(coordinator->get_logger(), "%s 无法初始化 MoveIt runtime world。",
        task_label.c_str());
      break;
    }

    const auto scene_mutex = std::make_shared<std::mutex>();
    fr3_dual_palletize::PlacementPlanner placement_planner;
    std::vector<fr3_dual_palletize::PlacedBox> placed;
    std::vector<fr3_dual_palletize::BoxSpec> pending = boxes;
    const int requested_tasks = std::min(max_tasks, static_cast<int>(pending.size()));
    int completed_tasks = 0;
    int completed_tight = 0;
    int completed_loose = 0;
    bool episode_ok = true;
    const auto episode_wall_start = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      coordinator->get_logger(),
      "========== %s CONTINUOUS EXECUTION: requested=%d of runtime_boxes=%zu, "
      "placement_candidates<=%d, plan_attempts=%d ==========" ,
      task_label.c_str(), requested_tasks, pending.size(), max_placement_candidates_to_try,
      task_plan_attempts);

    for (int task_index = 0; task_index < requested_tasks; ++task_index)
    {
      const auto task_wall_start = std::chrono::steady_clock::now();
      auto& box = pending.front();
      geometry_msgs::msg::Pose live_source_pose;
      if (!input_buffer.poseFor(box.id, live_source_pose))
      {
        RCLCPP_ERROR(coordinator->get_logger(), "%s missing live Ground Truth for object=%s.",
          task_label.c_str(), box.id.c_str());
        episode_ok = false;
        break;
      }
      // 每一轮都以 Isaac 最新 source pose 创建本件的碰撞物和顶部抓取参考，不能沿用
      // episode 初始 snapshot；之前已经放稳的物体只通过 placed / MoveIt World 表达。
      box.initial_pose = live_source_pose;
      const auto placement_wall_start = std::chrono::steady_clock::now();
      const auto planned = placement_planner.plan(box, region, placed);
      const double placement_wall_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - placement_wall_start).count();
      if (!planned.has_solution)
      {
        RCLCPP_ERROR(coordinator->get_logger(), "%s PlacementPlanner FAIL task=%d object=%s: %s",
          task_label.c_str(), task_index + 1, box.id.c_str(), planned.error.c_str());
        episode_ok = false;
        break;
      }

      ArmCandidate selected_loose;
      fr3_dual_palletize::SharedObjectPlan selected_tight;
      fr3_dual_palletize::CoordinationRouteDecision selected_route;
      fr3_dual_palletize::PlacementSpec selected_placement;
      bool route_ready = false;
      double route_wall_sec = 0.0;
      std::size_t route_calls = 0;
      const auto candidate_limit = std::min(
        planned.candidates.size(), static_cast<std::size_t>(max_placement_candidates_to_try));
      for (std::size_t candidate_index = 0; candidate_index < candidate_limit && !route_ready;
           ++candidate_index)
      {
        const auto& candidate = planned.candidates[candidate_index];
        for (int attempt = 1; attempt <= task_plan_attempts; ++attempt)
        {
          ArmCandidate loose;
          fr3_dual_palletize::SharedObjectPlan tight;
          fr3_dual_palletize::CoordinationRouteDecision route;
          const auto route_wall_start = std::chrono::steady_clock::now();
          const bool route_ok = planRuntimeRoute(
                box, candidate.placement, left_node, right_node, scene_mutex,
                loose_payload_limit_kg, tight_payload_limit_kg, loose, tight, route, task_label);
          route_wall_sec += std::chrono::duration<double>(
            std::chrono::steady_clock::now() - route_wall_start).count();
          ++route_calls;
          if (route_ok)
          {
            selected_loose = std::move(loose);
            selected_tight = std::move(tight);
            selected_route = std::move(route);
            selected_placement = candidate.placement;
            route_ready = true;
            RCLCPP_INFO(
              coordinator->get_logger(),
              "%s PLAN READY task=%d object=%s candidate=%zu/%zu attempt=%d/%d "
              "route=%s support=%s target=(%.3f, %.3f, %.3f)",
              task_label.c_str(), task_index + 1, box.id.c_str(), candidate_index + 1,
              candidate_limit, attempt, task_plan_attempts,
              fr3_dual_palletize::coordinationModeName(selected_route.mode),
              selected_placement.support_surface_id.c_str(),
              selected_placement.target_pose.position.x, selected_placement.target_pose.position.y,
              selected_placement.target_pose.position.z);
            break;
          }
          RCLCPP_WARN(
            coordinator->get_logger(),
            "%s PLAN RETRY task=%d object=%s candidate=%zu/%zu attempt=%d/%d rejected: %s",
            task_label.c_str(), task_index + 1, box.id.c_str(), candidate_index + 1,
            candidate_limit, attempt, task_plan_attempts, route.reason.c_str());
        }
      }
      if (!route_ready)
      {
        RCLCPP_ERROR(
          coordinator->get_logger(),
          "%s SAFE_REJECT task=%d object=%s: no route after %zu placement candidates * %d attempts.",
          task_label.c_str(), task_index + 1, box.id.c_str(), candidate_limit,
          task_plan_attempts);
        episode_ok = false;
        break;
      }

      geometry_msgs::msg::Pose latest_source_pose;
      if (!input_buffer.poseFor(box.id, latest_source_pose) ||
          positionDistance(latest_source_pose.position, box.initial_pose.position) >
          SOURCE_POSE_STALE_TOLERANCE_M)
      {
        RCLCPP_ERROR(
          coordinator->get_logger(),
          "%s source Ground Truth changed during planning for object=%s; reject stale path.",
          task_label.c_str(), box.id.c_str());
        episode_ok = false;
        break;
      }

      const auto pose_provider = [&input_buffer, id = box.id, fallback = box.initial_pose](std::size_t)
      {
        geometry_msgs::msg::Pose pose;
        return input_buffer.poseFor(id, pose) ? pose : fallback;
      };
      bool task_ok = false;
      const auto execution_wall_start = std::chrono::steady_clock::now();
      if (selected_route.mode == fr3_dual_palletize::CoordinationMode::TIGHT_SHARED_OBJECT)
      {
        fr3_dual_palletize::SharedObjectExecutionConfig config;
        config.execution_time_scale = execution_time_scale;
        fr3_dual_palletize::SharedObjectExecutor executor_instance(
          coordinator, scene_mutex, config);
        fr3_dual_palletize::SharedObjectExecutionResult result;
        task_ok = executor_instance.execute(
          box, selected_placement, selected_tight,
          [&input_buffer, id = box.id, fallback = box.initial_pose]()
          {
            geometry_msgs::msg::Pose pose;
            return input_buffer.poseFor(id, pose) ? pose : fallback;
          }, result);
        if (!task_ok)
        {
          RCLCPP_ERROR(coordinator->get_logger(), "%s TIGHT FAIL task=%d object=%s: %s",
            task_label.c_str(), task_index + 1, box.id.c_str(), result.error.c_str());
        }
        else
        {
          ++completed_tight;
          RCLCPP_INFO(
            coordinator->get_logger(),
            "%s TASK PASS task=%d route=TIGHT_SHARED_OBJECT object=%s placement=%.3f mm "
            "orientation=%.3f deg",
            task_label.c_str(), task_index + 1, box.id.c_str(),
            result.placement_position_error_m * 1000.0,
            result.placement_orientation_error_rad * 180.0 / 3.14159265358979323846);
        }
      }
      else
      {
        const Arm selected_arm = selected_route.mode ==
          fr3_dual_palletize::CoordinationMode::LOOSE_LEFT ? Arm::LEFT : Arm::RIGHT;
        if (selected_loose.arm != selected_arm)
        {
          RCLCPP_ERROR(coordinator->get_logger(), "%s Router/loose candidate arm mismatch.",
            task_label.c_str());
        }
        else
        {
          const auto& node = selected_arm == Arm::LEFT ? left_node : right_node;
          fr3_dual_palletize::PalletizePrimitive primitive(
            node, makeConfig(selected_arm, box, selected_placement, task_label),
            pose_provider, scene_mutex);
          fr3_dual_palletize::TaskTrajectoryExecutionConfig config;
          config.execution_time_scale = execution_time_scale;
          fr3_dual_palletize::TaskTrajectoryExecutionResult result;
          task_ok = primitive.executeTaskTrajectoryCandidate(selected_loose.candidate, config, result);
          if (!task_ok)
          {
            RCLCPP_ERROR(coordinator->get_logger(), "%s LOOSE FAIL task=%d object=%s: %s",
              task_label.c_str(), task_index + 1, box.id.c_str(), result.error.c_str());
          }
          else
          {
            ++completed_loose;
            RCLCPP_INFO(
              coordinator->get_logger(), "%s TASK PASS task=%d route=%s object=%s arm=%s events=%zu",
              task_label.c_str(), task_index + 1,
              fr3_dual_palletize::coordinationModeName(selected_route.mode), box.id.c_str(),
              armName(selected_arm), result.events_executed);
          }
        }
      }
      if (!task_ok)
      {
        episode_ok = false;
        break;
      }

      const double execution_wall_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - execution_wall_start).count();

      geometry_msgs::msg::Pose settled_pose;
      if (!input_buffer.poseFor(box.id, settled_pose))
      {
        RCLCPP_ERROR(coordinator->get_logger(), "%s missing settled Ground Truth for object=%s.",
          task_label.c_str(), box.id.c_str());
        episode_ok = false;
        break;
      }
      placed.push_back({box, settled_pose});
      pending.erase(pending.begin());
      ++completed_tasks;
      RCLCPP_INFO(
        coordinator->get_logger(),
        "%s WORLD COMMITTED task=%d/%d object=%s settled=(%.4f, %.4f, %.4f); "
        "next task will replan from updated support map and MoveIt World.",
        task_label.c_str(), completed_tasks, requested_tasks, placed.back().box.id.c_str(),
        settled_pose.position.x, settled_pose.position.y, settled_pose.position.z);
      RCLCPP_INFO(
        coordinator->get_logger(),
        "%s PROFILE task=%d object=%s placement=%.3f s candidates=%zu route_calls=%zu "
        "route=%.3f s execution=%.3f s task_wall=%.3f s",
        task_label.c_str(), completed_tasks, placed.back().box.id.c_str(),
        placement_wall_sec, planned.candidates.size(), route_calls, route_wall_sec,
        execution_wall_sec,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - task_wall_start).count());
    }

    success = episode_ok && completed_tasks == requested_tasks;
    if (success)
    {
      RCLCPP_INFO(
        coordinator->get_logger(),
        "========== %s CONTINUOUS EXECUTION PASS: completed=%d/%d tight=%d loose=%d ==========" ,
        task_label.c_str(), completed_tasks, requested_tasks, completed_tight, completed_loose);
      RCLCPP_INFO(
        coordinator->get_logger(), "%s PROFILE episode_wall=%.3f s",
        task_label.c_str(), std::chrono::duration<double>(
          std::chrono::steady_clock::now() - episode_wall_start).count());
    }
    else
    {
      RCLCPP_ERROR(
        coordinator->get_logger(),
        "%s CONTINUOUS EXECUTION STOPPED: completed=%d/%d tight=%d loose=%d; "
        "no following object was planned or commanded after the failed task.",
        task_label.c_str(), completed_tasks, requested_tasks, completed_tight, completed_loose);
    }
  } while (false);

  executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  if (success)
  {
    RCLCPP_INFO(coordinator->get_logger(),
      "Task20 runtime EXECUTION PASS: each completed object was physically settled, written back, "
      "and used by the next object\'s new plan.");
  }
  else
  {
    RCLCPP_ERROR(coordinator->get_logger(),
      "Task20 runtime EXECUTION FAIL/INCOMPLETE: no trajectory after the failing task was advanced.");
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}

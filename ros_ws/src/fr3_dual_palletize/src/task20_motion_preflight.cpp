// Task20-B：运行时多尺寸任务的无执行 Motion / FCL 预检。
//
// 本节点只读取 Task18 BoxStateArray。对于明确允许 loose_left / loose_right 的
// 物体，它将 Task19 自动生成的 PlacementSpec 转为 Task16 RobustPlanner 的完整
// TaskTrajectoryCandidate，并以 Task08 FCL 检查“活动臂 + 被动另一臂”的几何安全。
// 不发布 joint_command 或 suction_command；PalletizePrimitive 只在候选规划期间
// 临时写入 MoveIt Planning Scene，退出前会清理本节点写入的运行时 CollisionObject。
//
// tight_shared_object 不能被单臂流程假冒。本节点会明确报告该模式需要通用共享物
// planner，并安全拒绝该 task；Task21 Router 将据此选择紧协调 skill。

#include <algorithm>
#include <array>
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
#include <sensor_msgs/msg/joint_state.hpp>

#include "fr3_dual_palletize/local_wait_coordinator.hpp"
#include "fr3_dual_palletize/msg/box_state_array.hpp"
#include "fr3_dual_palletize/coordination_router.hpp"
#include "fr3_dual_palletize/palletize_primitive.hpp"
#include "fr3_dual_palletize/placement_planner.hpp"
#include "fr3_dual_palletize/runtime_box_state.hpp"
#include "fr3_dual_palletize/spatiotemporal_conflict_detector.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr std::array<double, 7> HOME_Q{{
  0.0001, 0.0002, -0.0002, -0.1518, 0.0000, 0.5445, 0.0000,
}};
constexpr double FCL_SAMPLE_PERIOD_SEC = 0.01;
constexpr double SCENE_SYNC_WAIT_SEC = 0.30;

enum class Arm
{
  LEFT,
  RIGHT,
};

struct PlannedTask
{
  fr3_dual_palletize::BoxSpec box;
  fr3_dual_palletize::PlacementSpec placement;
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

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
  const double sin_yaw = 2.0 * (
    quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
  auto sec = static_cast<std::int32_t>(std::floor(seconds));
  auto nanosec = static_cast<std::int64_t>(
    std::llround((seconds - static_cast<double>(sec)) * 1.0e9));
  if (nanosec >= 1000000000LL)
  {
    ++sec;
    nanosec -= 1000000000LL;
  }
  point.time_from_start.sec = sec;
  point.time_from_start.nanosec = static_cast<std::uint32_t>(nanosec);
}

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "Task20-B 无法连接 /move_group。");
    return false;
  }
  const auto parameters = client->get_parameters({
    "robot_description", "robot_description_semantic",
  });
  if (parameters.size() != 2 ||
      parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(node->get_logger(), "Task20-B 无法读取双 FR3 RobotModel 参数。");
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
  const fr3_dual_palletize::PlacementSpec& placement)
{
  const bool left = arm == Arm::LEFT;
  fr3_dual_palletize::PrimitiveConfig config;
  config.label = "TASK20-B " + std::string(armName(arm)) + " / " + box.id;
  config.planning_group = left ? "left_arm" : "right_arm";
  config.eef_link = left ? "left_fr3_link8" : "right_fr3_link8";
  config.tool_link = left ? "left_fr3_compact_suction" : "right_fr3_compact_suction";
  config.moveit_joint_prefix = left ? "left_" : "right_";
  // 只构造候选，以下 topic 不会被发布。
  config.joint_command_topic = left ? "/left/joint_command" : "/right/joint_command";
  config.suction_command_topic = left ? "/task20/left/suction_command" :
    "/task20/right/suction_command";
  config.suction_state_topic = left ? "/task20/left/suction_state" :
    "/task20/right/suction_state";
  config.object_id = box.id;
  config.object_dimensions = box.dimensions;
  config.target_x = placement.target_pose.position.x;
  config.target_y = placement.target_pose.position.y;
  config.target_yaw = yawFromQuaternion(placement.target_pose.orientation);
  config.target_support_surface_z = placement.support_height;
  config.include_safe_egress = true;
  // Task16 已验收默认：三条 RRTConnect 候选中按可解释代价选优。
  config.robust_planner.candidate_count = 3;
  config.robust_planner.redundancy_mode =
    fr3_dual_palletize::RedundancyMode::SOFT_PREFERENCE;
  config.robust_planner.preferred_redundant_joint = 0.0;
  return config;
}

fr3_dual_palletize::TaskTrajectoryCandidate makeIdleCandidate(
  Arm arm,
  const std::vector<double>& q,
  double duration_sec)
{
  const bool left = arm == Arm::LEFT;
  fr3_dual_palletize::TaskTrajectoryCandidate candidate;
  candidate.label = "TASK20-B passive " + std::string(armName(arm));
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
  candidate.trajectory.joint_names.reserve(7);
  for (int joint = 1; joint <= 7; ++joint)
  {
    candidate.trajectory.joint_names.push_back(
      std::string(left ? "left_fr3_joint" : "right_fr3_joint") + std::to_string(joint));
  }
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = q;
  setPointTime(start, 0.0);
  trajectory_msgs::msg::JointTrajectoryPoint finish = start;
  setPointTime(finish, duration_sec);
  candidate.trajectory.points = {start, finish};
  candidate.start_q = q;
  candidate.goal_q = q;
  candidate.duration_sec = duration_sec;
  // 同时刻 ATTACH/DETACH 让该虚拟 Object 始终在远端 world，且满足
  // Task08-B Candidate 的完整事件契约。
  candidate.events.push_back({
    0.0, fr3_dual_palletize::TaskEventType::ATTACH,
    candidate.object_id, candidate.eef_link,
  });
  candidate.events.push_back({
    0.0, fr3_dual_palletize::TaskEventType::DETACH,
    candidate.object_id, candidate.eef_link,
  });
  return candidate;
}

bool fclGate(
  const rclcpp::Node::SharedPtr& detector_node,
  Arm active_arm,
  const fr3_dual_palletize::TaskTrajectoryCandidate& active,
  const std::vector<double>& passive_q,
  rclcpp::Logger logger)
{
  moveit::planning_interface::PlanningSceneInterface scene;
  std::vector<moveit_msgs::msg::CollisionObject> static_objects;
  for (const auto& item : scene.getObjects())
  {
    static_objects.push_back(item.second);
  }

  moveit::planning_interface::MoveGroupInterface model_group(
    detector_node, active_arm == Arm::LEFT ? "left_arm" : "right_arm");
  fr3_dual_palletize::SpatioTemporalConflictDetector detector(
    model_group.getRobotModel(), std::move(static_objects));
  const auto passive = makeIdleCandidate(oppositeArm(active_arm), passive_q, active.duration_sec);
  const auto report = active_arm == Arm::LEFT ?
    detector.check(active, passive, FCL_SAMPLE_PERIOD_SEC) :
    detector.check(passive, active, FCL_SAMPLE_PERIOD_SEC);
  if (!report.valid)
  {
    RCLCPP_ERROR(logger, "Task20-B FCL INVALID: %s", report.error.c_str());
    return false;
  }
  if (report.conflict)
  {
    const auto& first = report.events.empty() ? fr3_dual_palletize::ConflictEvent{} :
      report.events.front();
    RCLCPP_WARN(
      logger,
      "Task20-B FCL REJECT: t=%.3f pair=%s <-> %s type=%s",
      report.first_conflict_time_sec, first.body_a.c_str(), first.body_b.c_str(),
      first.type.c_str());
    return false;
  }
  RCLCPP_INFO(
    logger,
    "Task20-B FCL SAFE: arm=%s samples=%zu horizon=%.3f s wall=%.3f s",
    armName(active_arm), report.samples_checked, report.horizon_sec, report.wall_time_sec);
  return true;
}

void removeExistingObjects(
  moveit::planning_interface::PlanningSceneInterface& scene,
  const std::vector<std::string>& ids)
{
  // PlanningSceneInterface 会对不存在的 id 打出警告。Task20 的每个 episode
  // 都可能是首次运行，因此先取得实际存在项，再删除，保持测试日志可读。
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
  ids.reserve(boxes.size());
  objects.reserve(boxes.size());
  for (const auto& box : boxes)
  {
    ids.push_back(box.id);
    objects.push_back(fr3_dual_palletize::makeCollisionObject(box, box.initial_pose));
  }
  removeExistingObjects(scene, ids);
  std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
  if (!scene.applyCollisionObjects(objects))
  {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
  return true;
}

bool replaceWorldObject(
  const fr3_dual_palletize::BoxSpec& box,
  const geometry_msgs::msg::Pose& pose)
{
  moveit::planning_interface::PlanningSceneInterface scene;
  removeExistingObjects(scene, {box.id});
  std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
  if (!scene.applyCollisionObject(fr3_dual_palletize::makeCollisionObject(box, pose)))
  {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
  return true;
}

void clearRuntimeWorld(const std::vector<fr3_dual_palletize::BoxSpec>& boxes)
{
  std::vector<std::string> ids;
  ids.reserve(boxes.size());
  for (const auto& box : boxes)
  {
    ids.push_back(box.id);
  }
  moveit::planning_interface::PlanningSceneInterface scene;
  removeExistingObjects(scene, ids);
}

std::vector<double> passiveJointPositions(
  const rclcpp::Node::SharedPtr& node,
  Arm arm)
{
  moveit::planning_interface::MoveGroupInterface group(
    node, arm == Arm::LEFT ? "left_arm" : "right_arm");
  const auto* joint_model_group = group.getRobotModel()->getJointModelGroup(
    arm == Arm::LEFT ? "left_arm" : "right_arm");
  const auto current = group.getCurrentState(3.0);
  if (!joint_model_group || !current)
  {
    return {};
  }
  std::vector<double> q;
  current->copyJointGroupPositions(joint_model_group, q);
  return q;
}

bool planLooseTask(
  const PlannedTask& task,
  const rclcpp::Node::SharedPtr& left_node,
  const rclcpp::Node::SharedPtr& right_node,
  const std::shared_ptr<std::mutex>& scene_mutex,
  double loose_payload_limit_kg,
  const fr3_dual_palletize::CoordinationRouter& router,
  ArmCandidate& selected,
  fr3_dual_palletize::CoordinationRouteDecision& route)
{
  fr3_dual_palletize::CoordinationRouteRequest request;
  request.box = &task.box;
  request.placement = &task.placement;
  request.tight.allowed_by_task = allowsMode(task.box, "tight_shared_object");
  request.tight.dual_grasp_feasible = task.box.grasp_candidates.size() >= 2;
  request.tight.payload_safe = task.box.mass > 0.0;
  // Task20-B 目前没有通用共享物 transport planner；必须让 Router 明确看到
  // 这一事实，不能静默降级为 loose。
  request.tight.shared_transport_planner_ready = false;
  request.tight.geometry_safe = false;
  request.tight.diagnostics = "generic shared-object planner is not available";

  const bool top_suction_candidate_available = !task.box.grasp_candidates.empty();
  const auto initializeEstimate =
    [&](Arm arm, fr3_dual_palletize::LooseModeEstimate& estimate)
    {
      estimate.allowed_by_task = allowsMode(
        task.box, arm == Arm::LEFT ? "loose_left" : "loose_right");
      estimate.payload_safe = task.box.mass <= loose_payload_limit_kg;
      // Task18 的 grasp contract 目前只发布已验证的 top-suction candidates；
      // 未来吸盘稳定性模型可直接替换此上游 bool，而 Router 接口无需改变。
      estimate.top_suction_stable = top_suction_candidate_available;
      estimate.diagnostics = "Task20-B runtime candidate not planned";
    };
  initializeEstimate(Arm::LEFT, request.left);
  initializeEstimate(Arm::RIGHT, request.right);

  std::vector<ArmCandidate> feasible;
  for (const Arm arm : {Arm::LEFT, Arm::RIGHT})
  {
    auto& estimate = arm == Arm::LEFT ? request.left : request.right;
    if (!estimate.allowed_by_task || !estimate.payload_safe ||
        !estimate.top_suction_stable)
    {
      continue;
    }
    const auto& node = arm == Arm::LEFT ? left_node : right_node;
    fr3_dual_palletize::PalletizePrimitive primitive(
      node, makeConfig(arm, task.box, task.placement),
      [pose = task.box.initial_pose](std::size_t) { return pose; }, scene_mutex);
    ArmCandidate trial;
    trial.arm = arm;
    if (!primitive.planTaskTrajectoryCandidate(trial.candidate))
    {
      estimate.diagnostics = "Task16 full candidate failed";
      RCLCPP_WARN(
        node->get_logger(), "Task20-B arm=%s object=%s: Task16 candidate FAIL",
        armName(arm), task.box.id.c_str());
      continue;
    }
    estimate.pick_reachable = true;
    estimate.place_reachable = true;
    const auto passive_q = passiveJointPositions(
      arm == Arm::LEFT ? right_node : left_node, oppositeArm(arm));
    if (passive_q.size() != HOME_Q.size() ||
        !fclGate(node, arm, trial.candidate, passive_q, node->get_logger()))
    {
      estimate.diagnostics = "Task08/FCL gate rejected complete candidate";
      RCLCPP_WARN(
        node->get_logger(), "Task20-B arm=%s object=%s: Task08/FCL gate REJECT",
        armName(arm), task.box.id.c_str());
      continue;
    }
    trial.duration_sec = trial.candidate.duration_sec;
    estimate.fcl_safe = true;
    estimate.estimated_duration_sec = trial.duration_sec;
    // Task16 已在自由空间阶段完成多候选评分；Task20-B 用完整候选时长作为
    // Router 的可解释 planning cost，避免回退到按尺寸或坐标分流。
    estimate.planning_cost = trial.duration_sec;
    estimate.diagnostics = "Task16 full candidate + Task08/FCL passed";
    feasible.push_back(std::move(trial));
  }
  route = router.decide(request);
  if (!route.feasible ||
      (route.mode != fr3_dual_palletize::CoordinationMode::LOOSE_LEFT &&
       route.mode != fr3_dual_palletize::CoordinationMode::LOOSE_RIGHT))
  {
    return false;
  }
  const Arm selected_arm = route.mode == fr3_dual_palletize::CoordinationMode::LOOSE_LEFT ?
    Arm::LEFT : Arm::RIGHT;
  const auto iterator = std::find_if(
    feasible.begin(), feasible.end(),
    [selected_arm](const ArmCandidate& candidate) { return candidate.arm == selected_arm; });
  if (iterator == feasible.end())
  {
    return false;
  }
  selected = *iterator;
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto coordinator_node = std::make_shared<rclcpp::Node>("task20_motion_preflight");
  auto left_node = std::make_shared<rclcpp::Node>("task20_left_primitive");
  auto right_node = std::make_shared<rclcpp::Node>("task20_right_primitive");

  const double timeout_sec = coordinator_node->declare_parameter<double>("timeout_sec", 30.0);
  const auto expected_seed = coordinator_node->declare_parameter<std::int64_t>("expected_seed", -1);
  const bool publish_home = coordinator_node->declare_parameter<bool>(
    "publish_home_joint_state", false);
  const bool allow_tight_deferred = coordinator_node->declare_parameter<bool>(
    "allow_tight_deferred", false);
  const double loose_payload_limit_kg = coordinator_node->declare_parameter<double>(
    "loose_payload_limit_kg", 0.50);
  fr3_dual_palletize::PalletRegion region;
  region.frame_id = "world";
  region.min_x = coordinator_node->declare_parameter<double>("pallet_min_x", 0.500);
  region.max_x = coordinator_node->declare_parameter<double>("pallet_max_x", 1.050);
  region.min_y = coordinator_node->declare_parameter<double>("pallet_min_y", -0.300);
  region.max_y = coordinator_node->declare_parameter<double>("pallet_max_y", 0.300);
  region.support_height = coordinator_node->declare_parameter<double>("pallet_support_height", 0.050);
  if (timeout_sec <= 0.0 || loose_payload_limit_kg <= 0.0 ||
      region.min_x >= region.max_x || region.min_y >= region.max_y)
  {
    RCLCPP_ERROR(coordinator_node->get_logger(), "Task20-B 参数无效。");
    rclcpp::shutdown();
    return 1;
  }
  if (!copyRobotModelParameters(left_node) || !copyRobotModelParameters(right_node))
  {
    rclcpp::shutdown();
    return 1;
  }

  const auto home_publisher = coordinator_node->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 20);
  const auto publishHome = [home_publisher, coordinator_node]()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = coordinator_node->now();
    for (const auto* prefix : {"left_fr3_", "right_fr3_"})
    {
      for (int joint = 1; joint <= 7; ++joint)
      {
        message.name.push_back(std::string(prefix) + "joint" + std::to_string(joint));
      }
    }
    message.position.insert(message.position.end(), HOME_Q.begin(), HOME_Q.end());
    message.position.insert(message.position.end(), HOME_Q.begin(), HOME_Q.end());
    home_publisher->publish(message);
  };
  rclcpp::TimerBase::SharedPtr home_timer;
  if (publish_home)
  {
    home_timer = coordinator_node->create_wall_timer(50ms, publishHome);
  }

  InputBuffer input_buffer(coordinator_node);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(coordinator_node);
  executor.add_node(left_node);
  executor.add_node(right_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  bool success = false;
  std::vector<fr3_dual_palletize::BoxSpec> boxes;
  do
  {
    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "Task20-B motion preflight started: waiting /task18/box_states; "
      "no joint/suction command will be published.");
    if (!input_buffer.wait(timeout_sec))
    {
      RCLCPP_ERROR(coordinator_node->get_logger(), "Task20-B timeout waiting runtime input.");
      break;
    }
    const auto input = input_buffer.state();
    if (input.boxes.empty() ||
        (expected_seed >= 0 && input.seed != static_cast<std::uint64_t>(expected_seed)))
    {
      RCLCPP_ERROR(coordinator_node->get_logger(), "Task20-B input empty or seed mismatch.");
      break;
    }
    std::set<std::string> ids;
    for (const auto& state : input.boxes)
    {
      fr3_dual_palletize::BoxSpec box;
      std::string error;
      if (!ids.insert(state.id).second ||
          !fr3_dual_palletize::boxStateToBoxSpec(state, box, error))
      {
        RCLCPP_ERROR(
          coordinator_node->get_logger(), "Task20-B invalid BoxState id=%s: %s",
          state.id.c_str(), error.c_str());
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

    fr3_dual_palletize::PlacementPlanner placement_planner;
    std::vector<fr3_dual_palletize::PlacedBox> placed;
    std::vector<PlannedTask> tasks;
    tasks.reserve(boxes.size());
    for (const auto& box : boxes)
    {
      const auto plan = placement_planner.plan(box, region, placed);
      if (!plan.has_solution)
      {
        RCLCPP_ERROR(
          coordinator_node->get_logger(), "Task20-B placement FAIL object=%s: %s",
          box.id.c_str(), plan.error.c_str());
        tasks.clear();
        break;
      }
      tasks.push_back({box, plan.chosen.placement});
      placed.push_back({box, plan.chosen.placement.target_pose});
    }
    if (tasks.empty() || !initializeRuntimeWorld(boxes))
    {
      RCLCPP_ERROR(coordinator_node->get_logger(), "Task20-B cannot initialize runtime world.");
      break;
    }

    const auto scene_mutex = std::make_shared<std::mutex>();
    const fr3_dual_palletize::CoordinationRouter router;
    std::size_t loose_safe = 0;
    std::size_t tight_deferred = 0;
    bool all_safe = true;
    for (const auto& task : tasks)
    {
      ArmCandidate chosen;
      fr3_dual_palletize::CoordinationRouteDecision route;
      if (!planLooseTask(
            task, left_node, right_node, scene_mutex, loose_payload_limit_kg,
            router, chosen, route))
      {
        const bool tight_only = allowsMode(task.box, "tight_shared_object") &&
          !allowsMode(task.box, "loose_left") && !allowsMode(task.box, "loose_right");
        if (tight_only)
        {
          ++tight_deferred;
        }
        RCLCPP_ERROR(
          coordinator_node->get_logger(),
          "Task20-B SAFE_REJECT object=%s route=%s: %s",
          task.box.id.c_str(),
          fr3_dual_palletize::coordinationModeName(route.mode), route.reason.c_str());
        if (tight_only && allow_tight_deferred)
        {
          continue;
        }
        all_safe = false;
        break;
      }
      if (!replaceWorldObject(task.box, task.placement.target_pose))
      {
        RCLCPP_ERROR(
          coordinator_node->get_logger(), "Task20-B cannot update static world for %s.",
          task.box.id.c_str());
        all_safe = false;
        break;
      }
      ++loose_safe;
      RCLCPP_INFO(
        coordinator_node->get_logger(),
        "Task20-B ACCEPT object=%s route=%s arm=%s duration=%.3f s target=(%.3f, %.3f, %.3f) "
        "[Task16 robust + Task08/FCL PASS; NOT_EXECUTED]",
        task.box.id.c_str(), fr3_dual_palletize::coordinationModeName(route.mode),
        armName(chosen.arm), chosen.duration_sec,
        task.placement.target_pose.position.x, task.placement.target_pose.position.y,
        task.placement.target_pose.position.z);
    }
    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "========== Task20-B PREFLIGHT STATS: seed=%lu tasks=%zu loose_safe=%zu "
      "tight_deferred=%zu executed=0 ==========" ,
      static_cast<unsigned long>(input.seed), tasks.size(), loose_safe, tight_deferred);
    success = all_safe && tight_deferred == 0 && loose_safe == tasks.size();
    if (!success && allow_tight_deferred && all_safe)
    {
      RCLCPP_WARN(
        coordinator_node->get_logger(),
        "Task20-B PARTIAL: loose tasks safe, tight task deliberately deferred; this is not a PASS.");
    }
  } while (false);

  if (!boxes.empty())
  {
    clearRuntimeWorld(boxes);
  }
  executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  if (success)
  {
    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "Task20-B PASS: all runtime loose tasks passed Task16 + Task08/FCL; no robot was executed.");
  }
  else
  {
    RCLCPP_ERROR(
      coordinator_node->get_logger(),
      "Task20-B FAIL/INCOMPLETE: no unsafe trajectory was allowed to advance.");
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}

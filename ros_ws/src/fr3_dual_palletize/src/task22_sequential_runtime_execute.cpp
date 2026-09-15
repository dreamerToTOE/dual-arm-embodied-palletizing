// Task22-D：稳定优先的真实串行执行器。
//
// 固定执行合同：
//   Isaac Ground Truth -> Python selector -> MoveIt/OMPL + FCL -> physical execution
//   -> suction release / settle -> MoveIt World writeback -> TaskWorldCommit -> next selection
//
// 该节点刻意不启动或读取 Task22-C sandbox，也不在一件物体还在运动时预规划下一件。
// 它复用 Task20 已验证的 loose/tight 执行器、真实 TCP release gate 与 GT World
// writeback；这里新增的只是 Task22 typed dispatch 和严格的 serial World Commit。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/msg/box_state_array.hpp"
#include "fr3_dual_palletize/msg/task_dispatch.hpp"
#include "fr3_dual_palletize/msg/task_dispatch_candidate.hpp"
#include "fr3_dual_palletize/msg/task_dispatch_candidate_array.hpp"
#include "fr3_dual_palletize/msg/task_world_commit.hpp"
#include "fr3_dual_palletize/palletize_primitive.hpp"
#include "fr3_dual_palletize/palletizing_job.hpp"
#include "fr3_dual_palletize/shared_object_executor.hpp"
#include "fr3_dual_palletize/shared_object_planner.hpp"
#include "fr3_dual_palletize/spatiotemporal_conflict_detector.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr double FCL_SAMPLE_PERIOD_SEC = 0.010;
constexpr double SCENE_SYNC_WAIT_SEC = 0.30;

enum class Arm { LEFT, RIGHT };

const char* armName(Arm arm)
{
  return arm == Arm::LEFT ? "left" : "right";
}

Arm opposite(Arm arm)
{
  return arm == Arm::LEFT ? Arm::RIGHT : Arm::LEFT;
}

bool parseArm(const std::string& value, Arm& arm)
{
  if (value == "left")
  {
    arm = Arm::LEFT;
    return true;
  }
  if (value == "right")
  {
    arm = Arm::RIGHT;
    return true;
  }
  return false;
}

double positionDistance(const geometry_msgs::msg::Point& first, const geometry_msgs::msg::Point& second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool hasMode(const fr3_dual_palletize::BoxSpec& box, const std::string& mode)
{
  return std::find(box.allowed_modes.begin(), box.allowed_modes.end(), mode) !=
    box.allowed_modes.end();
}

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "Task22-D cannot connect to /move_group parameter service.");
    return false;
  }
  const auto parameters = client->get_parameters({"robot_description", "robot_description_semantic"});
  if (parameters.size() != 2 ||
      parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(node->get_logger(), "Task22-D cannot read URDF/SRDF from /move_group.");
    return false;
  }
  const auto put = [&node](const std::string& name, const std::string& value)
  {
    if (!node->has_parameter(name))
    {
      node->declare_parameter<std::string>(name, value);
      return true;
    }
    return node->set_parameter(rclcpp::Parameter(name, value)).successful;
  };
  const auto urdf = parameters[0].as_string();
  const auto srdf = parameters[1].as_string();
  if (urdf.empty() || srdf.empty() || !put("robot_description", urdf) ||
      !put("robot_description_semantic", srdf))
  {
    RCLCPP_ERROR(node->get_logger(), "Task22-D received an empty/unset RobotModel.");
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "Task22-D RobotModel copied: URDF=%zu SRDF=%zu bytes.",
    urdf.size(), srdf.size());
  return true;
}

fr3_dual_palletize::BoxSpec boxFromCandidate(
  const fr3_dual_palletize::msg::TaskDispatchCandidate& candidate)
{
  fr3_dual_palletize::BoxSpec box;
  box.id = candidate.object_id;
  box.initial_pose = candidate.object_pose;
  box.dimensions = {candidate.size[0], candidate.size[1], candidate.size[2]};
  box.mass = candidate.mass;
  box.payload_class = candidate.payload_class;
  box.allowed_modes.assign(candidate.allowed_modes.begin(), candidate.allowed_modes.end());
  for (std::size_t i = 0; i < candidate.grasp_candidate_ids.size() &&
       i < candidate.grasp_candidates.size(); ++i)
  {
    box.grasp_candidates.push_back({candidate.grasp_candidate_ids[i], candidate.grasp_candidates[i]});
  }
  return box;
}

fr3_dual_palletize::PlacementSpec placementFrom(
  const fr3_dual_palletize::msg::TaskDispatch& dispatch,
  const fr3_dual_palletize::msg::TaskDispatchCandidate& candidate)
{
  fr3_dual_palletize::PlacementSpec placement;
  placement.id = candidate.candidate_id;
  placement.object_id = dispatch.object_id;
  placement.target_pose = dispatch.target_pose;
  placement.support_surface_id = dispatch.support_surface_id;
  placement.support_height = candidate.support_height;
  placement.orientation_tolerance_rad = 0.02;
  return placement;
}

fr3_dual_palletize::PrimitiveConfig makeLooseConfig(
  Arm arm, const fr3_dual_palletize::BoxSpec& box,
  const fr3_dual_palletize::PlacementSpec& placement, int candidate_count)
{
  const bool left = arm == Arm::LEFT;
  fr3_dual_palletize::PrimitiveConfig config;
  config.label = "Task22-D " + std::string(armName(arm)) + " / " + box.id;
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
  config.stable_support_lock_topic = "/task20/lock_placed_object";
  config.freeze_after_settle = true;
  config.object_id = box.id;
  config.object_dimensions = box.dimensions;
  config.target_x = placement.target_pose.position.x;
  config.target_y = placement.target_pose.position.y;
  config.target_yaw = yawFromQuaternion(placement.target_pose.orientation);
  config.target_support_surface_z = placement.support_height;
  config.include_safe_egress = true;
  config.robust_planner.candidate_count = static_cast<std::size_t>(candidate_count);
  config.robust_planner.planning_time_sec = 2.0;
  config.robust_planner.redundancy_mode = fr3_dual_palletize::RedundancyMode::SOFT_PREFERENCE;
  config.robust_planner.preferred_redundant_joint = 0.0;
  return config;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
  point.time_from_start.sec = static_cast<std::int32_t>(std::floor(seconds));
  point.time_from_start.nanosec = static_cast<std::uint32_t>(std::llround(
    (seconds - std::floor(seconds)) * 1.0e9));
}

std::vector<double> currentJointPositions(const rclcpp::Node::SharedPtr& node, Arm arm)
{
  const std::string group_name = arm == Arm::LEFT ? "left_arm" : "right_arm";
  moveit::planning_interface::MoveGroupInterface group(node, group_name);
  const auto* jmg = group.getRobotModel()->getJointModelGroup(group_name);
  const auto state = group.getCurrentState(3.0);
  if (!jmg || !state)
  {
    return {};
  }
  std::vector<double> positions;
  state->copyJointGroupPositions(jmg, positions);
  return positions;
}

fr3_dual_palletize::TaskTrajectoryCandidate makeIdleCandidate(
  Arm arm, const std::vector<double>& positions, double duration_sec)
{
  const bool left = arm == Arm::LEFT;
  fr3_dual_palletize::TaskTrajectoryCandidate result;
  result.label = "Task22-D passive " + std::string(armName(arm));
  result.arm_name = left ? "left_arm" : "right_arm";
  result.planning_group = result.arm_name;
  result.object_id = "__task22_passive_" + std::string(armName(arm));
  result.eef_link = left ? "left_fr3_link8" : "right_fr3_link8";
  result.object_dimensions = {{0.001, 0.001, 0.001}};
  result.initial_object_pose.position.x = 100.0;
  result.initial_object_pose.position.y = 100.0;
  result.initial_object_pose.position.z = 100.0;
  result.initial_object_pose.orientation.w = 1.0;
  result.planned_release_pose = result.initial_object_pose;
  for (int joint = 1; joint <= 7; ++joint)
  {
    result.trajectory.joint_names.push_back(
      std::string(left ? "left_fr3_joint" : "right_fr3_joint") + std::to_string(joint));
  }
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = positions;
  setPointTime(start, 0.0);
  auto finish = start;
  setPointTime(finish, duration_sec);
  result.trajectory.points = {start, finish};
  result.start_q = positions;
  result.goal_q = positions;
  result.duration_sec = duration_sec;
  result.events.push_back({0.0, fr3_dual_palletize::TaskEventType::ATTACH,
    result.object_id, result.eef_link});
  result.events.push_back({0.0, fr3_dual_palletize::TaskEventType::DETACH,
    result.object_id, result.eef_link});
  return result;
}

bool fullTaskFclGate(const rclcpp::Node::SharedPtr& node, Arm active_arm,
  const fr3_dual_palletize::TaskTrajectoryCandidate& active, const std::string& label)
{
  const auto passive_q = currentJointPositions(node, opposite(active_arm));
  if (passive_q.size() != 7)
  {
    RCLCPP_ERROR(node->get_logger(), "%s FCL FAIL: missing passive arm joint state.", label.c_str());
    return false;
  }
  moveit::planning_interface::PlanningSceneInterface scene;
  std::vector<moveit_msgs::msg::CollisionObject> world;
  for (const auto& item : scene.getObjects())
  {
    world.push_back(item.second);
  }
  moveit::planning_interface::MoveGroupInterface group(
    node, active_arm == Arm::LEFT ? "left_arm" : "right_arm");
  fr3_dual_palletize::SpatioTemporalConflictDetector detector(group.getRobotModel(), std::move(world));
  const auto passive = makeIdleCandidate(opposite(active_arm), passive_q, active.duration_sec);
  const auto report = active_arm == Arm::LEFT ?
    detector.check(active, passive, FCL_SAMPLE_PERIOD_SEC) :
    detector.check(passive, active, FCL_SAMPLE_PERIOD_SEC);
  if (!report.valid || report.conflict)
  {
    if (report.conflict && !report.events.empty())
    {
      const auto& event = report.events.front();
      RCLCPP_WARN(node->get_logger(), "%s FCL REJECT t=%.3f %s <-> %s", label.c_str(),
        report.first_conflict_time_sec, event.body_a.c_str(), event.body_b.c_str());
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "%s FCL REJECT: %s", label.c_str(), report.error.c_str());
    }
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "%s FCL PASS: samples=%zu horizon=%.3f s wall=%.3f s",
    label.c_str(), report.samples_checked, report.horizon_sec, report.wall_time_sec);
  return true;
}

class SequentialRuntimeExecutor final : public rclcpp::Node
{
public:
  SequentialRuntimeExecutor()
  : Node("task22_sequential_runtime_execute"),
    scene_mutex_(std::make_shared<std::mutex>()),
    left_node_(std::make_shared<rclcpp::Node>("task22_runtime_left")),
    right_node_(std::make_shared<rclcpp::Node>("task22_runtime_right")),
    tight_execution_node_(std::make_shared<rclcpp::Node>("task22_runtime_tight_physical")),
    box_state_node_(std::make_shared<rclcpp::Node>("task22_runtime_box_state"))
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/task18/box_states");
    candidate_topic_ = declare_parameter<std::string>("candidate_topic", "/task22/dispatch_candidates");
    dispatch_topic_ = declare_parameter<std::string>("dispatch_topic", "/task22/dispatch");
    world_commit_topic_ = declare_parameter<std::string>("world_commit_topic", "/task22/world_commit");
    expected_seed_ = declare_parameter<std::int64_t>("expected_seed", -1);
    max_tasks_ = declare_parameter<int>("max_tasks", 0);
    plan_attempts_ = declare_parameter<int>("plan_attempts", 3);
    fast_candidate_count_ = declare_parameter<int>("fast_candidate_count", 1);
    execution_time_scale_ = declare_parameter<double>("execution_time_scale", 2.0);
    source_stale_tolerance_m_ = declare_parameter<double>("source_stale_tolerance_m", 0.003);
    if (max_tasks_ < 0 || plan_attempts_ <= 0 || fast_candidate_count_ <= 0 ||
        execution_time_scale_ < 1.0 || source_stale_tolerance_m_ <= 0.0)
    {
      throw std::runtime_error("Task22-D invalid runtime parameter.");
    }
    const auto snapshot_qos = rclcpp::QoS(1).reliable().transient_local();
    // 物理执行会在本节点的 timer callback 中同步等待每个阶段结束。Ground Truth
    // 必须由独立 worker 接收，否则 timer 占用默认 mutually-exclusive callback
    // group 时，COMMON_LIFT 等阶段会读取到 CONTACT 时刻的旧 Box pose。
    boxes_subscription_ = box_state_node_->create_subscription<fr3_dual_palletize::msg::BoxStateArray>(
      input_topic_, rclcpp::QoS(10),
      [this](const fr3_dual_palletize::msg::BoxStateArray::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        boxes_ = *message;
        have_boxes_ = true;
      });
    candidates_subscription_ = create_subscription<fr3_dual_palletize::msg::TaskDispatchCandidateArray>(
      candidate_topic_, snapshot_qos,
      [this](const fr3_dual_palletize::msg::TaskDispatchCandidateArray::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        candidates_ = *message;
        have_candidates_ = true;
      });
    dispatch_subscription_ = create_subscription<fr3_dual_palletize::msg::TaskDispatch>(
      dispatch_topic_, snapshot_qos,
      [this](const fr3_dual_palletize::msg::TaskDispatch::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        dispatch_ = *message;
        have_dispatch_ = true;
      });
    commit_publisher_ = create_publisher<fr3_dual_palletize::msg::TaskWorldCommit>(
      world_commit_topic_, rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(200ms, std::bind(&SequentialRuntimeExecutor::tryExecute, this));
    RCLCPP_INFO(get_logger(), "Task22-D sequential executor ready: %s + %s -> physical execution; "
      "Task22-C sandbox is frozen.", candidate_topic_.c_str(), dispatch_topic_.c_str());
  }

  bool initialize()
  {
    return copyRobotModelParameters(left_node_) && copyRobotModelParameters(right_node_);
  }

  const rclcpp::Node::SharedPtr& leftNode() const { return left_node_; }
  const rclcpp::Node::SharedPtr& rightNode() const { return right_node_; }
  const rclcpp::Node::SharedPtr& tightExecutionNode() const { return tight_execution_node_; }
  const rclcpp::Node::SharedPtr& boxStateNode() const { return box_state_node_; }
  bool succeeded() const { return success_.load(); }
  bool failed() const { return failed_.load(); }

private:
  struct WorkItem
  {
    fr3_dual_palletize::msg::BoxStateArray boxes;
    fr3_dual_palletize::msg::TaskDispatchCandidateArray candidates;
    fr3_dual_palletize::msg::TaskDispatch dispatch;
    fr3_dual_palletize::msg::TaskDispatchCandidate candidate;
  };

  bool selectWorkItem(WorkItem& output)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!have_boxes_ || !have_candidates_)
    {
      return false;
    }
    if (candidates_.candidates.empty())
    {
      if (completed_tasks_ > 0)
      {
        finish(true, "all objects committed; candidate provider reports no pending box");
      }
      return false;
    }
    if (!have_dispatch_ || candidates_.scene_version != dispatch_.scene_version)
    {
      return false;
    }
    if (max_tasks_ > 0 && completed_tasks_ >= max_tasks_)
    {
      finish(true, "requested max_tasks committed");
      return false;
    }
    const auto it = std::find_if(candidates_.candidates.begin(), candidates_.candidates.end(),
      [this](const auto& candidate) { return candidate.candidate_id == dispatch_.candidate_id; });
    if (it == candidates_.candidates.end())
    {
      return false;
    }
    const std::string key = std::to_string(dispatch_.scene_version) + ":" + dispatch_.candidate_id;
    if (key == processed_key_)
    {
      return false;
    }
    output = {boxes_, candidates_, dispatch_, *it};
    processed_key_ = key;
    return true;
  }

  bool latestPose(const std::string& id, geometry_msgs::msg::Pose& pose) const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto it = std::find_if(boxes_.boxes.begin(), boxes_.boxes.end(),
      [&id](const auto& item) { return item.id == id; });
    if (it == boxes_.boxes.end())
    {
      return false;
    }
    pose = it->pose;
    return true;
  }

  bool synchronizeRuntimeWorld(const WorkItem& item)
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    std::set<std::string> ids;
    for (const auto& candidate : item.candidates.candidates)
    {
      if (!ids.insert(candidate.object_id).second)
      {
        continue;
      }
      objects.push_back(fr3_dual_palletize::makeCollisionObject(
        boxFromCandidate(candidate), candidate.object_pose));
    }
    if (objects.empty())
    {
      return false;
    }
    std::lock_guard<std::mutex> lock(*scene_mutex_);
    moveit::planning_interface::PlanningSceneInterface scene;
    const bool applied = scene.applyCollisionObjects(objects);
    std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
    return applied;
  }

  bool planLoose(const fr3_dual_palletize::BoxSpec& box,
    const fr3_dual_palletize::PlacementSpec& placement, Arm arm,
    fr3_dual_palletize::TaskTrajectoryCandidate& result)
  {
    const auto& node = arm == Arm::LEFT ? left_node_ : right_node_;
    fr3_dual_palletize::PalletizePrimitive primitive(node,
      makeLooseConfig(arm, box, placement, fast_candidate_count_),
      [this, id = box.id, fallback = box.initial_pose](std::size_t)
      {
        geometry_msgs::msg::Pose pose;
        return latestPose(id, pose) ? pose : fallback;
      }, scene_mutex_);
    if (!primitive.planTaskTrajectoryCandidate(result))
    {
      return false;
    }
    return fullTaskFclGate(node, arm, result, "Task22-D " + std::string(armName(arm)) + " / " + box.id);
  }

  void publishCommit(const WorkItem& item, const geometry_msgs::msg::Pose& settled,
    const std::string& mode, const std::string& arm)
  {
    fr3_dual_palletize::msg::TaskWorldCommit commit;
    commit.header.stamp = now();
    commit.source_seed = item.candidates.source_seed;
    commit.completed_scene_version = item.dispatch.scene_version;
    commit.object_id = item.candidate.object_id;
    commit.settled_pose = settled;
    commit.coordination_mode = mode;
    commit.executed_arm = arm;
    commit_publisher_->publish(commit);
  }

  void executeWorkItem(const WorkItem& item)
  {
    const auto began = std::chrono::steady_clock::now();
    if ((expected_seed_ >= 0 && item.candidates.source_seed != static_cast<std::uint64_t>(expected_seed_)) ||
        item.boxes.seed != item.candidates.source_seed ||
        item.dispatch.object_id != item.candidate.object_id ||
        item.dispatch.support_surface_id != item.candidate.support_surface_id)
    {
      finish(false, "candidate/dispatch/ground-truth episode contract mismatch");
      return;
    }
    geometry_msgs::msg::Pose live_source;
    if (!latestPose(item.candidate.object_id, live_source) ||
        positionDistance(live_source.position, item.candidate.object_pose.position) >
        source_stale_tolerance_m_)
    {
      finish(false, "source Ground Truth changed before planning; stale dispatch rejected");
      return;
    }
    if (!synchronizeRuntimeWorld(item))
    {
      finish(false, "cannot synchronize runtime collision world");
      return;
    }
    auto box = boxFromCandidate(item.candidate);
    box.initial_pose = live_source;
    const auto placement = placementFrom(item.dispatch, item.candidate);
    bool task_ok = false;
    std::string executed_arm = "none";

    RCLCPP_INFO(get_logger(), "========== Task22-D SERIAL TASK %d: object=%s mode=%s "
      "target=(%.3f, %.3f, %.3f) ==========" , completed_tasks_ + 1, box.id.c_str(),
      item.dispatch.coordination_mode.c_str(), placement.target_pose.position.x,
      placement.target_pose.position.y, placement.target_pose.position.z);

    if (item.dispatch.coordination_mode == "TIGHT_SHARED_OBJECT")
    {
      if (item.dispatch.preferred_arm != "none" || !hasMode(box, "tight_shared_object") ||
          box.grasp_candidates.size() < 2)
      {
        finish(false, "invalid tight dispatch/grasp contract");
        return;
      }
      fr3_dual_palletize::SharedObjectPlan plan;
      bool planned = false;
      for (int attempt = 1; attempt <= plan_attempts_; ++attempt)
      {
        fr3_dual_palletize::SharedObjectPlanner planner(left_node_, scene_mutex_);
        if (planner.plan(box, placement, plan))
        {
          planned = true;
          RCLCPP_INFO(get_logger(), "Task22-D tight plan OK attempt=%d/%d.", attempt, plan_attempts_);
          break;
        }
        RCLCPP_WARN(get_logger(), "Task22-D tight plan retry attempt=%d/%d: %s", attempt,
          plan_attempts_, plan.diagnostics.c_str());
      }
      if (!planned)
      {
        finish(false, "tight MoveIt/FCL planning exhausted all retries");
        return;
      }
      fr3_dual_palletize::SharedObjectExecutionConfig config;
      config.execution_time_scale = execution_time_scale_;
      // 不能将 Surface-Gripper bridge 的 state/TCP subscription 放在当前的 timer
      // callback 所属 node。execute() 会同步等待 CLOSED/OPEN；若使用同一个默认
      // mutually-exclusive callback group，即使 MultiThreadedExecutor 有空闲线程，
      // state callback 也无法在等待期间运行，从而把健康 bridge 误判为未就绪。
      // 独立 worker node 与 loose arm worker 一样持续被 executor spin；另一个
      // box_state_node_ 持续接收物体 Ground Truth，保证紧协调各阶段读到实时数据。
      fr3_dual_palletize::SharedObjectExecutor executor(
        tight_execution_node_, scene_mutex_, config);
      fr3_dual_palletize::SharedObjectExecutionResult result;
      task_ok = executor.execute(box, placement, plan,
        [this, id = box.id, fallback = box.initial_pose]()
        {
          geometry_msgs::msg::Pose pose;
          return latestPose(id, pose) ? pose : fallback;
        }, result);
      if (!task_ok)
      {
        finish(false, "tight physical execution failed: " + result.error);
        return;
      }
    }
    else if (item.dispatch.coordination_mode == "LOOSE")
    {
      Arm primary;
      if (!parseArm(item.dispatch.preferred_arm, primary))
      {
        finish(false, "loose dispatch has no valid primary arm");
        return;
      }
      std::vector<Arm> arms{primary};
      Arm fallback;
      if (parseArm(item.dispatch.fallback_arm, fallback) && fallback != primary)
      {
        arms.push_back(fallback);
      }
      fr3_dual_palletize::TaskTrajectoryCandidate trajectory;
      Arm selected = primary;
      bool planned = false;
      for (const Arm arm : arms)
      {
        const std::string allowed = arm == Arm::LEFT ? "loose_left" : "loose_right";
        if (!hasMode(box, allowed))
        {
          continue;
        }
        for (int attempt = 1; attempt <= plan_attempts_; ++attempt)
        {
          if (planLoose(box, placement, arm, trajectory))
          {
            selected = arm;
            planned = true;
            RCLCPP_INFO(get_logger(), "Task22-D loose %s plan OK attempt=%d/%d%s.", armName(arm),
              attempt, plan_attempts_, arm == primary ? "" : " (fallback)");
            break;
          }
          RCLCPP_WARN(get_logger(), "Task22-D loose %s plan retry attempt=%d/%d.", armName(arm),
            attempt, plan_attempts_);
        }
        if (planned)
        {
          break;
        }
      }
      if (!planned)
      {
        finish(false, "loose primary/fallback MoveIt/FCL planning exhausted all retries");
        return;
      }
      const auto& node = selected == Arm::LEFT ? left_node_ : right_node_;
      fr3_dual_palletize::PalletizePrimitive primitive(node,
        makeLooseConfig(selected, box, placement, fast_candidate_count_),
        [this, id = box.id, fallback_pose = box.initial_pose](std::size_t)
        {
          geometry_msgs::msg::Pose pose;
          return latestPose(id, pose) ? pose : fallback_pose;
        }, scene_mutex_);
      fr3_dual_palletize::TaskTrajectoryExecutionConfig config;
      config.execution_time_scale = execution_time_scale_;
      fr3_dual_palletize::TaskTrajectoryExecutionResult result;
      task_ok = primitive.executeTaskTrajectoryCandidate(trajectory, config, result);
      if (!task_ok)
      {
        finish(false, "loose physical execution failed: " + result.error);
        return;
      }
      executed_arm = armName(selected);
    }
    else
    {
      finish(false, "unknown coordination mode");
      return;
    }

    geometry_msgs::msg::Pose settled;
    if (!latestPose(box.id, settled))
    {
      finish(false, "execution returned success but no settled Ground Truth is available");
      return;
    }
    publishCommit(item, settled, item.dispatch.coordination_mode, executed_arm);
    ++completed_tasks_;
    RCLCPP_INFO(get_logger(), "Task22-D WORLD COMMIT published: task=%d object=%s settled="
      "(%.4f, %.4f, %.4f), wall=%.3f s. Next planning is blocked until provider publishes "
      "the committed scene_version.", completed_tasks_, box.id.c_str(), settled.position.x,
      settled.position.y, settled.position.z,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count());
  }

  void tryExecute()
  {
    if (finished_.load() || busy_.exchange(true))
    {
      return;
    }
    WorkItem item;
    if (!selectWorkItem(item))
    {
      busy_.store(false);
      return;
    }
    executeWorkItem(item);
    busy_.store(false);
  }

  void finish(bool success, const std::string& reason)
  {
    if (finished_.exchange(true))
    {
      return;
    }
    success_.store(success);
    failed_.store(!success);
    if (timer_)
    {
      timer_->cancel();
    }
    if (success)
    {
      RCLCPP_INFO(get_logger(), "Task22-D SERIAL EXECUTION PASS: completed=%d; %s.",
        completed_tasks_, reason.c_str());
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Task22-D SERIAL EXECUTION STOPPED: completed=%d; %s. "
        "No following candidate will be planned or commanded.", completed_tasks_, reason.c_str());
    }
    // 退出 launch 的执行节点；失败状态由 main 返回非零 exit code。
    rclcpp::shutdown();
  }

  std::string input_topic_;
  std::string candidate_topic_;
  std::string dispatch_topic_;
  std::string world_commit_topic_;
  std::int64_t expected_seed_{-1};
  int max_tasks_{0};
  int plan_attempts_{3};
  int fast_candidate_count_{1};
  int completed_tasks_{0};
  double execution_time_scale_{2.0};
  double source_stale_tolerance_m_{0.003};
  std::string processed_key_;
  std::shared_ptr<std::mutex> scene_mutex_;
  rclcpp::Node::SharedPtr left_node_;
  rclcpp::Node::SharedPtr right_node_;
  rclcpp::Node::SharedPtr tight_execution_node_;
  rclcpp::Node::SharedPtr box_state_node_;
  mutable std::mutex data_mutex_;
  bool have_boxes_{false};
  bool have_candidates_{false};
  bool have_dispatch_{false};
  fr3_dual_palletize::msg::BoxStateArray boxes_;
  fr3_dual_palletize::msg::TaskDispatchCandidateArray candidates_;
  fr3_dual_palletize::msg::TaskDispatch dispatch_;
  std::atomic_bool busy_{false};
  std::atomic_bool finished_{false};
  std::atomic_bool success_{false};
  std::atomic_bool failed_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<fr3_dual_palletize::msg::BoxStateArray>::SharedPtr boxes_subscription_;
  rclcpp::Subscription<fr3_dual_palletize::msg::TaskDispatchCandidateArray>::SharedPtr
    candidates_subscription_;
  rclcpp::Subscription<fr3_dual_palletize::msg::TaskDispatch>::SharedPtr dispatch_subscription_;
  rclcpp::Publisher<fr3_dual_palletize::msg::TaskWorldCommit>::SharedPtr commit_publisher_;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<SequentialRuntimeExecutor>();
    if (!node->initialize())
    {
      rclcpp::shutdown();
      return 1;
    }
    // 规划/物理执行的 timer 可以阻塞较久；left/right worker 的 /joint_states
    // 订阅必须同时被调度，避免 MoveGroupInterface 用过期起始关节规划。
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    executor.add_node(node->leftNode());
    executor.add_node(node->rightNode());
    executor.add_node(node->tightExecutionNode());
    executor.add_node(node->boxStateNode());
    executor.spin();
    return node->failed() ? 1 : 0;
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("task22_sequential_runtime_execute"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
}

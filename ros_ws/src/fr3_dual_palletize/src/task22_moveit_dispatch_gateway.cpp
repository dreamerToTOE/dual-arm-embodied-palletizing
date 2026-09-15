// Task22-B：接收 Python 的高层 TaskDispatch，做 primary-arm-first 的只读 MoveIt
// preflight。它刻意不发布 joint/suction command；只有未来执行器在这里返回 PASS 后
// 才能复用 Task20-E / Task08 / Task09 的受控执行路径。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/msg/task_dispatch.hpp"
#include "fr3_dual_palletize/msg/task_dispatch_candidate.hpp"
#include "fr3_dual_palletize/msg/task_dispatch_candidate_array.hpp"
#include "fr3_dual_palletize/palletize_primitive.hpp"
#include "fr3_dual_palletize/palletizing_job.hpp"
#include "fr3_dual_palletize/shared_object_planner.hpp"
#include "fr3_dual_palletize/spatiotemporal_conflict_detector.hpp"

namespace
{

constexpr double FCL_SAMPLE_PERIOD_SEC = 0.010;

enum class Arm
{
  LEFT,
  RIGHT,
};

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

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
  // ROS 2 参数属于节点私有；/move_group 持有的 robot_description 不会自动成为
  // Gateway 的参数。MoveGroupInterface 需要本节点有相同的 URDF/SRDF，故复用
  // Task20 已验证的参数复制方式，而不是依赖启动先后或全局参数。
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(), "Task22-B 无法连接 /move_group 参数服务。");
    return false;
  }
  const auto parameters = client->get_parameters({
    "robot_description", "robot_description_semantic",
  });
  if (parameters.size() != 2 ||
      parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(node->get_logger(), "Task22-B 无法从 /move_group 读取 URDF/SRDF 参数。");
    return false;
  }
  const auto set_or_declare = [&node](const std::string& name, const std::string& value)
  {
    if (!node->has_parameter(name))
    {
      node->declare_parameter<std::string>(name, value);
      return true;
    }
    // launch 可能预先声明了同名但为空的参数。只检查 has_parameter() 会让
    // RobotModelLoader 读到空 SRDF；这里必须无条件用 /move_group 的有效值覆盖。
    return node->set_parameter(rclcpp::Parameter(name, value)).successful;
  };
  const auto urdf = parameters[0].as_string();
  const auto srdf = parameters[1].as_string();
  if (urdf.empty() || srdf.empty() ||
      !set_or_declare("robot_description", urdf) ||
      !set_or_declare("robot_description_semantic", srdf))
  {
    RCLCPP_ERROR(node->get_logger(), "Task22-B 未能将有效 URDF/SRDF 写入 Gateway 参数。");
    return false;
  }
  RCLCPP_INFO(
    node->get_logger(), "Task22-B RobotModel copied from /move_group: URDF=%zu bytes SRDF=%zu bytes.",
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
  for (std::size_t index = 0;
       index < candidate.grasp_candidate_ids.size() && index < candidate.grasp_candidates.size();
       ++index)
  {
    box.grasp_candidates.push_back({
      candidate.grasp_candidate_ids[index], candidate.grasp_candidates[index]});
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
  Arm arm,
  const fr3_dual_palletize::BoxSpec& box,
  const fr3_dual_palletize::PlacementSpec& placement,
  int fast_candidate_count)
{
  const bool left = arm == Arm::LEFT;
  fr3_dual_palletize::PrimitiveConfig config;
  config.label = "Task22-B " + std::string(armName(arm)) + " / " + box.id;
  config.planning_group = left ? "left_arm" : "right_arm";
  config.eef_link = left ? "left_fr3_link8" : "right_fr3_link8";
  config.tool_link = left ? "left_fr3_compact_suction" : "right_fr3_compact_suction";
  config.moveit_joint_prefix = left ? "left_" : "right_";
  config.object_id = box.id;
  config.object_dimensions = box.dimensions;
  config.target_x = placement.target_pose.position.x;
  config.target_y = placement.target_pose.position.y;
  const auto& q = placement.target_pose.orientation;
  config.target_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  config.target_support_surface_z = placement.support_height;
  config.include_safe_egress = true;
  // Task22 的性能关键：Python 已经做廉价 primary-arm 选择，因此先只取一条
  // RRTConnect 可行轨迹。MoveIt/FCL 仍是硬门禁；失败时才进入 fallback arm。
  config.robust_planner.candidate_count = static_cast<std::size_t>(fast_candidate_count);
  config.robust_planner.planning_time_sec = 2.0;
  config.robust_planner.redundancy_mode =
    fr3_dual_palletize::RedundancyMode::SOFT_PREFERENCE;
  config.robust_planner.preferred_redundant_joint = 0.0;
  return config;
}

std::vector<double> currentJointPositions(
  const rclcpp::Node::SharedPtr& node,
  Arm arm)
{
  const std::string group_name = arm == Arm::LEFT ? "left_arm" : "right_arm";
  moveit::planning_interface::MoveGroupInterface group(node, group_name);
  const auto* joint_model_group = group.getRobotModel()->getJointModelGroup(group_name);
  const auto state = group.getCurrentState(3.0);
  if (!joint_model_group || !state)
  {
    return {};
  }
  std::vector<double> positions;
  state->copyJointGroupPositions(joint_model_group, positions);
  return positions;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
  point.time_from_start.sec = static_cast<std::int32_t>(std::floor(seconds));
  point.time_from_start.nanosec = static_cast<std::uint32_t>(std::llround(
    (seconds - std::floor(seconds)) * 1.0e9));
}

fr3_dual_palletize::TaskTrajectoryCandidate makeIdleCandidate(
  Arm arm,
  const std::vector<double>& positions,
  double duration_sec)
{
  const bool left = arm == Arm::LEFT;
  fr3_dual_palletize::TaskTrajectoryCandidate result;
  result.label = "Task22-B passive " + std::string(armName(arm));
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
  // Task08 detector 需要成对事件表达 passive object 从未参与碰撞世界。
  result.events.push_back({0.0, fr3_dual_palletize::TaskEventType::ATTACH,
    result.object_id, result.eef_link});
  result.events.push_back({0.0, fr3_dual_palletize::TaskEventType::DETACH,
    result.object_id, result.eef_link});
  return result;
}

double trajectoryDuration(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty())
  {
    return 0.0;
  }
  const auto& point = trajectory.points.back();
  return static_cast<double>(point.time_from_start.sec) +
    static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
}

bool fullTaskFclGate(
  const rclcpp::Node::SharedPtr& node,
  Arm active_arm,
  const fr3_dual_palletize::TaskTrajectoryCandidate& active,
  const std::string& label)
{
  const auto passive_positions = currentJointPositions(node, opposite(active_arm));
  if (passive_positions.size() != 7)
  {
    RCLCPP_ERROR(node->get_logger(), "%s FCL FAIL: 无法读取 passive arm 当前关节。", label.c_str());
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
  const auto passive = makeIdleCandidate(
    opposite(active_arm), passive_positions, trajectoryDuration(active.trajectory));
  const auto report = active_arm == Arm::LEFT ?
    detector.check(active, passive, FCL_SAMPLE_PERIOD_SEC) :
    detector.check(passive, active, FCL_SAMPLE_PERIOD_SEC);
  if (!report.valid || report.conflict)
  {
    if (report.conflict && !report.events.empty())
    {
      const auto& event = report.events.front();
      RCLCPP_WARN(
        node->get_logger(), "%s FCL REJECT: t=%.3f %s <-> %s",
        label.c_str(), report.first_conflict_time_sec,
        event.body_a.c_str(), event.body_b.c_str());
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "%s FCL REJECT: %s", label.c_str(), report.error.c_str());
    }
    return false;
  }
  RCLCPP_INFO(
    node->get_logger(), "%s FCL PASS: samples=%zu horizon=%.3f s wall=%.3f s",
    label.c_str(), report.samples_checked, report.horizon_sec, report.wall_time_sec);
  return true;
}

class MoveItDispatchGateway final : public rclcpp::Node
{
public:
  MoveItDispatchGateway()
  : Node("task22_moveit_dispatch_gateway")
  {
    // 与 Task20 一样，协调/订阅节点和真正持有 MoveGroupInterface 的工作节点分离。
    // 这样即便 Gateway 的 dispatch callback 正在执行，工作节点仍可由 executor 的
    // 其他线程接收 /joint_states，供 MoveIt 读取当前起始状态。
    planning_node_ = std::make_shared<rclcpp::Node>("task22_moveit_preflight_worker");
    candidate_topic_ = declare_parameter<std::string>(
      "candidate_topic", "/task22/dispatch_candidates");
    dispatch_topic_ = declare_parameter<std::string>("dispatch_topic", "/task22/dispatch");
    fast_candidate_count_ = declare_parameter<int>("fast_candidate_count", 1);
    if (fast_candidate_count_ <= 0)
    {
      throw std::runtime_error("fast_candidate_count 必须大于零。");
    }
    const auto snapshot_qos = rclcpp::QoS(1).reliable().transient_local();
    // preflight 会同步等待 MoveIt current state、调用 OMPL/FCL，不能占住默认
    // callback group；否则该节点内 MoveGroupInterface 的 /joint_states 监听无法被
    // executor 调度，current state 会错误地停在时间戳 0。单独使用可重入组，并由
    // MultiThreadedExecutor 驱动，保证状态监听可以并行更新。
    preflight_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = preflight_callback_group_;
    candidate_subscription_ = create_subscription<
      fr3_dual_palletize::msg::TaskDispatchCandidateArray>(
      candidate_topic_, snapshot_qos,
      std::bind(&MoveItDispatchGateway::onCandidates, this, std::placeholders::_1),
      subscription_options);
    dispatch_subscription_ = create_subscription<fr3_dual_palletize::msg::TaskDispatch>(
      dispatch_topic_, snapshot_qos,
      std::bind(&MoveItDispatchGateway::onDispatch, this, std::placeholders::_1),
      subscription_options);
    RCLCPP_INFO(
      get_logger(),
      "Task22-B MoveIt gateway ready: %s + %s; primary-arm-first, no joint/suction command.",
      candidate_topic_.c_str(), dispatch_topic_.c_str());
  }

  bool initializeRobotModelParameters()
  {
    // 在 executor spin 之前同步读取 /move_group 参数。参数属于真正创建
    // MoveGroupInterface 的 worker node，而非仅做消息协调的 Gateway node。
    // SyncParametersClient 在等待 service response 时会临时 spin 本节点；此时
    // latched candidate/dispatch 可能提前到达。因此必须先缓存消息，等模型参数真正
    // 写入完成才允许 tryPreflight() 创建 MoveGroupInterface。
    if (!copyRobotModelParameters(planning_node_))
    {
      return false;
    }
    robot_model_ready_ = true;
    tryPreflight();
    return true;
  }

  const rclcpp::Node::SharedPtr& planningNode() const
  {
    return planning_node_;
  }

private:
  void onCandidates(const fr3_dual_palletize::msg::TaskDispatchCandidateArray::SharedPtr message)
  {
    candidates_ = message;
    tryPreflight();
  }

  void onDispatch(const fr3_dual_palletize::msg::TaskDispatch::SharedPtr message)
  {
    dispatch_ = message;
    tryPreflight();
  }

  const fr3_dual_palletize::msg::TaskDispatchCandidate* selectedCandidate() const
  {
    if (!candidates_ || !dispatch_ || candidates_->scene_version != dispatch_->scene_version)
    {
      return nullptr;
    }
    const auto iterator = std::find_if(
      candidates_->candidates.begin(), candidates_->candidates.end(),
      [this](const auto& candidate) { return candidate.candidate_id == dispatch_->candidate_id; });
    return iterator == candidates_->candidates.end() ? nullptr : &(*iterator);
  }

  bool initializePlanningWorld() const
  {
    if (!candidates_)
    {
      return false;
    }
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    std::set<std::string> unique_ids;
    for (const auto& candidate : candidates_->candidates)
    {
      if (!unique_ids.insert(candidate.object_id).second)
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
    moveit::planning_interface::PlanningSceneInterface scene;
    return scene.applyCollisionObjects(objects);
  }

  bool planLooseArm(
    const fr3_dual_palletize::BoxSpec& box,
    const fr3_dual_palletize::PlacementSpec& placement,
    Arm arm,
    fr3_dual_palletize::TaskTrajectoryCandidate& output)
  {
    auto node = planning_node_;
    auto scene_mutex = std::make_shared<std::mutex>();
    fr3_dual_palletize::PalletizePrimitive primitive(
      node, makeLooseConfig(arm, box, placement, fast_candidate_count_),
      [pose = box.initial_pose](std::size_t) { return pose; }, scene_mutex);
    if (!primitive.planTaskTrajectoryCandidate(output))
    {
      RCLCPP_WARN(get_logger(), "Task22-B primary/fallback %s planning FAIL.", armName(arm));
      return false;
    }
    return fullTaskFclGate(
      node, arm, output, "Task22-B " + std::string(armName(arm)) + " / " + box.id);
  }

  void tryPreflight()
  {
    if (!robot_model_ready_)
    {
      return;
    }
    // candidate / dispatch 可能背靠背到达。只允许一个预检进入 MoveIt，避免重复
    // 读取同一份场景或并发改写 planning scene。
    std::lock_guard<std::mutex> lock(preflight_mutex_);
    const auto* candidate = selectedCandidate();
    if (!candidate || !dispatch_)
    {
      return;
    }
    const std::string key = std::to_string(dispatch_->scene_version) + ":" +
      dispatch_->candidate_id;
    if (key == completed_key_)
    {
      return;
    }
    const auto began = std::chrono::steady_clock::now();

    if (dispatch_->object_id != candidate->object_id ||
        dispatch_->support_surface_id != candidate->support_surface_id)
    {
      RCLCPP_ERROR(get_logger(), "Task22-B REJECT: dispatch 与 candidate 内容不一致。");
      return;
    }
    completed_key_ = key;
    if (!initializePlanningWorld())
    {
      RCLCPP_ERROR(get_logger(), "Task22-B REJECT: 无法初始化 MoveIt runtime collision world。");
      return;
    }

    const auto box = boxFromCandidate(*candidate);
    const auto placement = placementFrom(*dispatch_, *candidate);
    if (dispatch_->coordination_mode == "TIGHT_SHARED_OBJECT")
    {
      if (dispatch_->preferred_arm != "none" || box.grasp_candidates.size() < 2)
      {
        RCLCPP_ERROR(get_logger(), "Task22-B REJECT: tight dispatch 的 arm/grasp 契约无效。");
        return;
      }
      auto node = planning_node_;
      fr3_dual_palletize::SharedObjectPlanner planner(node, std::make_shared<std::mutex>());
      fr3_dual_palletize::SharedObjectPlan plan;
      if (!planner.plan(box, placement, plan))
      {
        RCLCPP_ERROR(get_logger(), "Task22-B TIGHT REJECT: %s", plan.diagnostics.c_str());
        return;
      }
      RCLCPP_INFO(
        get_logger(), "Task22-B TIGHT ACCEPT: object=%s stages=%zu duration=%.3f s FCL_samples=%zu "
        "wall=%.3f s [NOT_EXECUTED]",
        box.id.c_str(), plan.stages.size(), plan.duration_sec, plan.fcl_samples,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count());
      return;
    }

    if (dispatch_->coordination_mode != "LOOSE")
    {
      RCLCPP_ERROR(get_logger(), "Task22-B REJECT: 未知 coordination_mode=%s",
        dispatch_->coordination_mode.c_str());
      return;
    }
    Arm primary;
    if (!parseArm(dispatch_->preferred_arm, primary))
    {
      RCLCPP_ERROR(get_logger(), "Task22-B REJECT: loose dispatch 缺少 primary arm。");
      return;
    }
    fr3_dual_palletize::TaskTrajectoryCandidate trajectory;
    bool accepted = planLooseArm(box, placement, primary, trajectory);
    std::string selected_arm = dispatch_->preferred_arm;
    bool used_fallback = false;
    if (!accepted && dispatch_->fallback_arm != "none")
    {
      Arm fallback;
      if (parseArm(dispatch_->fallback_arm, fallback) && fallback != primary)
      {
        RCLCPP_WARN(get_logger(), "Task22-B primary=%s rejected; trying fallback=%s only now.",
          armName(primary), armName(fallback));
        accepted = planLooseArm(box, placement, fallback, trajectory);
        selected_arm = dispatch_->fallback_arm;
        used_fallback = true;
      }
    }
    if (!accepted)
    {
      RCLCPP_ERROR(
        get_logger(), "Task22-B LOOSE REJECT: object=%s primary=%s fallback=%s [NOT_EXECUTED]",
        box.id.c_str(), dispatch_->preferred_arm.c_str(), dispatch_->fallback_arm.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Task22-B LOOSE ACCEPT: object=%s arm=%s fallback_used=%s duration=%.3f s "
      "wall=%.3f s [NOT_EXECUTED]",
      box.id.c_str(), selected_arm.c_str(), used_fallback ? "true" : "false",
      trajectory.duration_sec,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count());
  }

  std::string candidate_topic_;
  std::string dispatch_topic_;
  int fast_candidate_count_{1};
  bool robot_model_ready_{false};
  std::string completed_key_;
  std::mutex preflight_mutex_;
  rclcpp::Node::SharedPtr planning_node_;
  fr3_dual_palletize::msg::TaskDispatchCandidateArray::SharedPtr candidates_;
  fr3_dual_palletize::msg::TaskDispatch::SharedPtr dispatch_;
  rclcpp::CallbackGroup::SharedPtr preflight_callback_group_;
  rclcpp::Subscription<fr3_dual_palletize::msg::TaskDispatchCandidateArray>::SharedPtr
    candidate_subscription_;
  rclcpp::Subscription<fr3_dual_palletize::msg::TaskDispatch>::SharedPtr dispatch_subscription_;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<MoveItDispatchGateway>();
    if (!node->initializeRobotModelParameters())
    {
      rclcpp::shutdown();
      return 1;
    }
    // 预检回调会等待 MoveIt state 并运行规划；必须保留其他线程接收
    // /joint_states，不能使用 rclcpp::spin() 的单线程 executor。
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.add_node(node->planningNode());
    executor.spin();
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("task22_moveit_dispatch_gateway"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

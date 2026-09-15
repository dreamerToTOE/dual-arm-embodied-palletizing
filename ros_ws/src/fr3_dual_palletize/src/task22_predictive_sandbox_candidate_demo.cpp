// Task22-C2C：在独立 MoveIt sandbox 中实际生成下一件 loose object 的完整候选。
//
// 输入只来自 Task22-A 的 Ground Truth/Task19 candidate topic；本节点先镜像执行
// /move_group 的 world CollisionObject 到 /task22_sandbox，再让已有
// PalletizePrimitive 只连接 sandbox 生成 HOME -> ... -> RETREAT 候选。
//
// 它刻意不接收或发布 joint/suction command，且 sandbox 的
// allow_trajectory_execution=false。该演示只验证“场景快照 + 真实候选生成”管线；
// 运行中 tight object 的预测 terminal state / attached body 复制将在后续 C3 接入。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/msg/task_dispatch_candidate.hpp"
#include "fr3_dual_palletize/msg/task_dispatch_candidate_array.hpp"
#include "fr3_dual_palletize/palletize_primitive.hpp"

namespace
{

bool copyRobotModelParameters(
  const rclcpp::Node::SharedPtr& node,
  const std::string& sandbox_move_group)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, sandbox_move_group);
  if (!client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(), "Task22-C2C 无法连接 %s 参数服务。", sandbox_move_group.c_str());
    return false;
  }

  const auto parameters = client->get_parameters({
    "robot_description", "robot_description_semantic",
  });
  if (parameters.size() != 2 ||
      parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[0].as_string().empty() || parameters[1].as_string().empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Task22-C2C sandbox URDF/SRDF 参数无效。");
    return false;
  }

  node->declare_parameter<std::string>("robot_description", parameters[0].as_string());
  node->declare_parameter<std::string>(
    "robot_description_semantic", parameters[1].as_string());
  RCLCPP_INFO(
    node->get_logger(), "Task22-C2C RobotModel copied: URDF=%zu bytes SRDF=%zu bytes.",
    parameters[0].as_string().size(), parameters[1].as_string().size());
  return true;
}

bool containsMode(
  const fr3_dual_palletize::msg::TaskDispatchCandidate& candidate,
  const std::string& selected_arm)
{
  const std::string required_mode = "loose_" + selected_arm;
  return std::find(
    candidate.allowed_modes.begin(), candidate.allowed_modes.end(), required_mode) !=
    candidate.allowed_modes.end();
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
  const double sin_yaw = 2.0 * (
    quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

std::vector<moveit_msgs::msg::CollisionObject> collisionObjectsFrom(
  const std::map<std::string, moveit_msgs::msg::CollisionObject>& objects)
{
  std::vector<moveit_msgs::msg::CollisionObject> result;
  result.reserve(objects.size());
  for (const auto& item : objects)
  {
    result.push_back(item.second);
  }
  return result;
}

std::set<std::string> objectIdsFrom(
  const std::map<std::string, moveit_msgs::msg::CollisionObject>& objects)
{
  std::set<std::string> ids;
  for (const auto& item : objects)
  {
    ids.insert(item.first);
  }
  return ids;
}

fr3_dual_palletize::PrimitiveConfig makeSandboxConfig(
  const fr3_dual_palletize::msg::TaskDispatchCandidate& candidate,
  const std::string& selected_arm,
  const std::string& sandbox_namespace)
{
  const bool is_left = selected_arm == "left";
  fr3_dual_palletize::PrimitiveConfig config;
  config.label = "Task22-C2C sandbox " + selected_arm + " / " + candidate.object_id;
  config.planning_group = is_left ? "left_arm" : "right_arm";
  config.eef_link = is_left ? "left_fr3_link8" : "right_fr3_link8";
  config.tool_link = is_left ? "left_fr3_compact_suction" : "right_fr3_compact_suction";
  config.moveit_joint_prefix = is_left ? "left_" : "right_";
  // planTaskTrajectoryCandidate() 不会调用以下控制主题；使用隔离名称作为额外约束，
  // 使未来错误扩展也不会意外连接 Isaac 的真实执行 bridge。
  config.joint_command_topic = "/task22/sandbox_never_joint_command";
  config.suction_command_topic = "/task22/sandbox_never_suction_command";
  config.suction_state_topic = "/task22/sandbox_never_suction_state";
  config.move_group_namespace = sandbox_namespace;
  config.planning_scene_namespace = sandbox_namespace;
  config.object_id = candidate.object_id;
  config.object_dimensions = {{candidate.size[0], candidate.size[1], candidate.size[2]}};
  config.target_x = candidate.target_pose.position.x;
  config.target_y = candidate.target_pose.position.y;
  config.target_yaw = yawFromQuaternion(candidate.target_pose.orientation);
  config.target_support_surface_z = candidate.support_height;
  config.include_safe_egress = true;
  // Python 已生成 placement candidate；本集成回归与 Task22-B 的 primary-first
  // 策略保持一致，只要求一条 sandbox RRTConnect 候选。
  config.robust_planner.candidate_count = 1;
  config.robust_planner.planning_time_sec = 2.0;
  config.robust_planner.redundancy_mode =
    fr3_dual_palletize::RedundancyMode::SOFT_PREFERENCE;
  config.robust_planner.preferred_redundant_joint = 0.0;
  return config;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task22_predictive_sandbox_candidate_demo");
  const auto sandbox_namespace = node->declare_parameter<std::string>(
    "sandbox_namespace", "/task22_sandbox");
  const auto sandbox_move_group_node = node->declare_parameter<std::string>(
    "sandbox_move_group_node", "sandbox_move_group");
  const auto candidate_topic = node->declare_parameter<std::string>(
    "candidate_topic", "/task22/dispatch_candidates");
  const auto object_id = node->declare_parameter<std::string>("object_id", "task18_box_03");
  const auto selected_arm = node->declare_parameter<std::string>("selected_arm", "left");
  const auto timeout_sec = node->declare_parameter<double>("timeout_sec", 10.0);
  const auto sandbox_move_group = sandbox_namespace + "/" + sandbox_move_group_node;
  bool success = false;
  std::thread spin_thread;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;

  do
  {
    if ((selected_arm != "left" && selected_arm != "right") || object_id.empty() ||
        timeout_sec <= 0.0 || sandbox_namespace.empty() || sandbox_namespace.front() != '/' ||
        sandbox_move_group_node.empty() ||
        !copyRobotModelParameters(node, sandbox_move_group))
    {
      RCLCPP_ERROR(node->get_logger(), "Task22-C2C 输入参数无效或 sandbox 尚未启动。");
      break;
    }

    std::mutex candidate_mutex;
    std::condition_variable candidate_cv;
    fr3_dual_palletize::msg::TaskDispatchCandidate selected_candidate;
    bool candidate_ready = false;
    const auto candidate_sub = node->create_subscription<
      fr3_dual_palletize::msg::TaskDispatchCandidateArray>(
      candidate_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [&](const fr3_dual_palletize::msg::TaskDispatchCandidateArray::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(candidate_mutex);
        if (candidate_ready)
        {
          return;
        }
        const auto iterator = std::find_if(
          message->candidates.begin(), message->candidates.end(),
          [&](const auto& candidate)
          {
            return candidate.object_id == object_id && containsMode(candidate, selected_arm);
          });
        if (iterator != message->candidates.end())
        {
          selected_candidate = *iterator;
          candidate_ready = true;
          candidate_cv.notify_all();
        }
      });

    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 2);
    executor->add_node(node);
    spin_thread = std::thread([&executor]() { executor->spin(); });

    {
      std::unique_lock<std::mutex> lock(candidate_mutex);
      if (!candidate_cv.wait_for(
            lock, std::chrono::duration<double>(timeout_sec), [&]() { return candidate_ready; }))
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Task22-C2C 未在 %.1f s 内收到 %s 的 loose_%s candidate。",
          timeout_sec, object_id.c_str(), selected_arm.c_str());
        break;
      }
    }
    (void)candidate_sub;

    // 只读 execution scene，随后先清空 sandbox 自己旧的镜像，再写入新的快照。
    // 此处所有 apply/remove 都显式指向 sandbox namespace，绝不修改 /move_group。
    moveit::planning_interface::PlanningSceneInterface execution_scene;
    moveit::planning_interface::PlanningSceneInterface sandbox_scene(sandbox_namespace);
    const auto execution_objects = execution_scene.getObjects();
    const auto execution_attached = execution_scene.getAttachedObjects();
    if (!execution_attached.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Task22-C2C 拒绝不完整快照：execution scene 含 %zu 个 AttachedCollisionObject；"
        "运行中 tight object 的预测 attachment 将在后续 C3 专门建模。",
        execution_attached.size());
      break;
    }
    const auto old_sandbox_ids = sandbox_scene.getKnownObjectNames();
    if (!old_sandbox_ids.empty())
    {
      sandbox_scene.removeCollisionObjects(old_sandbox_ids);
    }
    const auto snapshot_objects = collisionObjectsFrom(execution_objects);
    if (!snapshot_objects.empty() && !sandbox_scene.applyCollisionObjects(snapshot_objects))
    {
      RCLCPP_ERROR(node->get_logger(), "Task22-C2C 无法将 execution world 镜像到 sandbox。");
      break;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Task22-C2C SNAPSHOT READY: execution world objects=%zu -> %s; object=%s target=(%.3f, %.3f, %.3f)",
      execution_objects.size(), sandbox_namespace.c_str(), selected_candidate.object_id.c_str(),
      selected_candidate.target_pose.position.x, selected_candidate.target_pose.position.y,
      selected_candidate.target_pose.position.z);

    auto planning_scene_mutex = std::make_shared<std::mutex>();
    const auto config = makeSandboxConfig(selected_candidate, selected_arm, sandbox_namespace);
    const auto initial_pose = selected_candidate.object_pose;
    fr3_dual_palletize::PalletizePrimitive primitive(
      node,
      config,
      [initial_pose](std::size_t) { return initial_pose; },
      planning_scene_mutex);
    fr3_dual_palletize::TaskTrajectoryCandidate candidate;
    if (!primitive.planTaskTrajectoryCandidate(candidate))
    {
      RCLCPP_ERROR(node->get_logger(), "Task22-C2C sandbox 完整候选规划失败。");
      break;
    }

    // 规划 primitive 在 sandbox 中临时 attach/detach object。恢复镜像对象，令本次
    // demo 后的 sandbox world 与读取时的 execution snapshot 一致；依然不写 execution。
    const auto original = execution_objects.find(selected_candidate.object_id);
    if (original != execution_objects.end())
    {
      if (!sandbox_scene.applyCollisionObject(original->second))
      {
        RCLCPP_ERROR(node->get_logger(), "Task22-C2C 无法恢复 sandbox 中的源对象。");
        break;
      }
    }
    else
    {
      sandbox_scene.removeCollisionObjects({selected_candidate.object_id});
    }

    const auto before_ids = objectIdsFrom(execution_objects);
    const auto after_ids = objectIdsFrom(execution_scene.getObjects());
    if (before_ids != after_ids)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Task22-C2C FAIL：execution /move_group world object 集合发生变化，拒绝该候选。");
      break;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Task22-C2C PASS：sandbox 为 %s 生成完整候选 points=%zu duration=%.3f s events=%zu；"
      "execution /move_group world 未被写入，未调用 execute 或发布 joint/suction command。",
      candidate.object_id.c_str(), candidate.trajectory.points.size(), candidate.duration_sec,
      candidate.events.size());
    success = true;
  } while (false);

  if (spin_thread.joinable())
  {
    rclcpp::shutdown();
    spin_thread.join();
  }
  else
  {
    rclcpp::shutdown();
  }
  return success ? 0 : 1;
}

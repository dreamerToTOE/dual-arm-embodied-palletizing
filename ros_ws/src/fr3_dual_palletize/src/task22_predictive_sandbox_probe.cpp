// Task22-C2：验证 predictive planner 的 MoveIt sandbox 不会污染执行 /move_group。
//
// 本探针只在 sandbox planning scene 写入并删除一个临时 CollisionObject，随后确认
// execution scene 完全看不到该对象；它也只读取 /joint_states，不发送规划请求、轨迹、
// joint command 或 suction command。

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{

constexpr char PROBE_OBJECT_ID[] = "__task22_sandbox_isolation_probe";

bool copyRobotModelParameters(
  const rclcpp::Node::SharedPtr& node,
  const std::string& sandbox_move_group)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, sandbox_move_group);
  if (!client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(), "Task22-C2 无法连接 %s 参数服务。", sandbox_move_group.c_str());
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
    RCLCPP_ERROR(node->get_logger(), "Task22-C2 sandbox URDF/SRDF 参数无效。");
    return false;
  }
  node->declare_parameter<std::string>("robot_description", parameters[0].as_string());
  node->declare_parameter<std::string>(
    "robot_description_semantic", parameters[1].as_string());
  RCLCPP_INFO(
    node->get_logger(), "Task22-C2 sandbox RobotModel copied: URDF=%zu bytes SRDF=%zu bytes.",
    parameters[0].as_string().size(), parameters[1].as_string().size());
  return true;
}

moveit_msgs::msg::CollisionObject makeProbeObject()
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = PROBE_OBJECT_ID;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {0.010, 0.010, 0.010};
  geometry_msgs::msg::Pose pose;
  pose.position.x = -10.0;  // 远离有效工作空间，确保只验证 namespace 隔离。
  pose.position.y = -10.0;
  pose.position.z = -10.0;
  pose.orientation.w = 1.0;
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task22_predictive_sandbox_probe");
  const auto sandbox_namespace = node->declare_parameter<std::string>(
    "sandbox_namespace", "/task22_sandbox");
  const auto sandbox_move_group_node = node->declare_parameter<std::string>(
    "sandbox_move_group_node", "sandbox_move_group");
  const auto sandbox_move_group = sandbox_namespace + "/" + sandbox_move_group_node;
  bool success = false;
  std::thread spin_thread;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;

  do
  {
    if (sandbox_namespace.empty() || sandbox_namespace.front() != '/' ||
        sandbox_move_group_node.empty() ||
        !copyRobotModelParameters(node, sandbox_move_group))
    {
      break;
    }

    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 2);
    executor->add_node(node);
    spin_thread = std::thread([&executor]() { executor->spin(); });

    moveit::planning_interface::PlanningSceneInterface execution_scene;
    moveit::planning_interface::PlanningSceneInterface sandbox_scene(sandbox_namespace);
    if (!sandbox_scene.applyCollisionObject(makeProbeObject()))
    {
      RCLCPP_ERROR(node->get_logger(), "Task22-C2 无法写入 sandbox CollisionObject。");
      break;
    }

    // applyCollisionObject 是同步服务；这里仍做一次查询，避免把服务成功误判为
    // scene cache 已更新。execution scene 绝不应出现 sandbox 独有对象。
    const auto sandbox_objects = sandbox_scene.getObjects({PROBE_OBJECT_ID});
    const auto execution_objects = execution_scene.getObjects({PROBE_OBJECT_ID});
    if (sandbox_objects.find(PROBE_OBJECT_ID) == sandbox_objects.end() ||
        execution_objects.find(PROBE_OBJECT_ID) != execution_objects.end())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Task22-C2 scene isolation FAIL: sandbox_has=%s execution_has=%s",
        sandbox_objects.find(PROBE_OBJECT_ID) != sandbox_objects.end() ? "true" : "false",
        execution_objects.find(PROBE_OBJECT_ID) != execution_objects.end() ? "true" : "false");
      sandbox_scene.removeCollisionObjects({PROBE_OBJECT_ID});
      break;
    }

    // 只验证 sandbox MoveGroupInterface 能从全局 Isaac /joint_states 取得起始状态。
    // 不调用 plan()/execute()，从而不产生任何机器人或吸盘命令。
    const auto options = moveit::planning_interface::MoveGroupInterface::Options(
      "left_arm", "robot_description", sandbox_namespace);
    moveit::planning_interface::MoveGroupInterface group(node, options);
    const auto state = group.getCurrentState(3.0);
    const auto* joint_model_group = group.getRobotModel()->getJointModelGroup("left_arm");
    sandbox_scene.removeCollisionObjects({PROBE_OBJECT_ID});
    if (!state || !joint_model_group)
    {
      RCLCPP_ERROR(node->get_logger(), "Task22-C2 sandbox 无法读取 left_arm 当前状态。");
      break;
    }
    std::vector<double> positions;
    state->copyJointGroupPositions(joint_model_group, positions);
    if (positions.size() != 7)
    {
      RCLCPP_ERROR(node->get_logger(), "Task22-C2 sandbox left_arm joint 数量=%zu，预期 7。",
        positions.size());
      break;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Task22-C2 PASS：sandbox scene 与 execution /move_group 隔离，且已读取 7 个 "
      "left_arm joint states；未调用 plan/execute 或发布任何控制命令。");
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

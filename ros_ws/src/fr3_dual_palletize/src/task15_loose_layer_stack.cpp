// Task15 Phase B：四个独立小 Cube 的两层松协调码垛。
//
// 本节点不为每个 Cube 复制抓取状态机。每层只是两条 ArmAssignment 配置：
// PalletizePrimitive 复用 Task04 抓放语义，Task08 FCL 检查完整候选，
// Task09 LocalWaitCoordinator 选择必要的最小等待，CoordinatedTaskExecutor
// 再以共享时间轴执行。增加小件仅增加配置条目，执行代码保持不变。

#include <array>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
constexpr std::size_t NUM_SMALL_CUBES = 4;
constexpr double SMALL_CUBE_SIZE = 0.030;
constexpr double SMALL_CUBE_HALF = SMALL_CUBE_SIZE * 0.5;
constexpr double LARGE_CUBE_X = 0.220;
constexpr double LARGE_CUBE_Y = 0.320;
constexpr double LARGE_CUBE_Z = 0.080;
constexpr double LARGE_CUBE_HALF_Z = LARGE_CUBE_Z * 0.5;
constexpr double LARGE_NOMINAL_TOP_Z = 0.130;
constexpr double LOWER_NOMINAL_TOP_Z = 0.160;
constexpr double EXPECTED_LARGE_TARGET_X = 0.650;
constexpr double EXPECTED_LARGE_TARGET_Z = 0.090;

struct ArmAssignment
{
  std::string label;
  std::string object_id;
  std::size_t pose_index{0};
  double target_x{0.0};
  double target_y{0.0};
  // -1 代表以已放稳的大 Cube 顶面为支撑面；0/1 代表以相应下层小 Cube 顶面为支撑面。
  std::int32_t support_small_index{-1};
};

struct BatchConfig
{
  std::string name;
  ArmAssignment left;
  ArmAssignment right;
};

const std::array<BatchConfig, 2> BATCHES{{
  {
    "LOWER_LAYER",
    {
      "TASK15 LOWER LEFT / SmallCube1",
      "task15_small_cube_1",
      0,
      0.650,
      -0.080,
      -1,
    },
    {
      "TASK15 LOWER RIGHT / SmallCube2",
      "task15_small_cube_2",
      1,
      0.650,
      +0.080,
      -1,
    },
  },
  {
    "UPPER_LAYER",
    {
      "TASK15 UPPER LEFT / SmallCube3",
      "task15_small_cube_3",
      2,
      0.650,
      -0.080,
      0,
    },
    {
      "TASK15 UPPER RIGHT / SmallCube4",
      "task15_small_cube_4",
      3,
      0.650,
      +0.080,
      1,
    },
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
    RCLCPP_ERROR(node->get_logger(), "无法从 /move_group 读取双 FR3 RobotModel 参数。");
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
  const geometry_msgs::msg::Pose& pose,
  double x_size,
  double y_size,
  double z_size)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;

  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions.resize(3);
  shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = x_size;
  shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = y_size;
  shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = z_size;
  object.primitives.push_back(shape);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

class Task15PoseBuffer
{
public:
  explicit Task15PoseBuffer(const rclcpp::Node::SharedPtr& node)
  {
    large_subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/task15/large_cube_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          large_pose_ = message->pose;
          have_large_pose_ = true;
        }
        condition_.notify_all();
      });

    small_subscription_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "/task15/small_cube_poses", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message)
      {
        if (message->poses.size() < NUM_SMALL_CUBES)
        {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          for (std::size_t index = 0; index < NUM_SMALL_CUBES; ++index)
          {
            small_poses_[index] = message->poses[index];
          }
          have_small_poses_ = true;
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
      [this]() { return have_large_pose_ && have_small_poses_; });
  }

  geometry_msgs::msg::Pose largePose() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return large_pose_;
  }

  geometry_msgs::msg::Pose smallPose(std::size_t index) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return small_poses_.at(index);
  }

  std::array<geometry_msgs::msg::Pose, NUM_SMALL_CUBES> smallSnapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return small_poses_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  geometry_msgs::msg::Pose large_pose_;
  std::array<geometry_msgs::msg::Pose, NUM_SMALL_CUBES> small_poses_{};
  bool have_large_pose_{false};
  bool have_small_poses_{false};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr large_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr small_subscription_;
};

bool initializeTask15World(
  const geometry_msgs::msg::Pose& large_pose,
  const std::array<geometry_msgs::msg::Pose, NUM_SMALL_CUBES>& small_poses)
{
  moveit::planning_interface::PlanningSceneInterface scene;
  scene.removeCollisionObjects({
    "task15_large_cube",
    "task15_small_cube_1",
    "task15_small_cube_2",
    "task15_small_cube_3",
    "task15_small_cube_4",
  });
  std::this_thread::sleep_for(300ms);

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.reserve(1 + NUM_SMALL_CUBES);
  objects.push_back(makeBoxObject(
    "task15_large_cube", large_pose, LARGE_CUBE_X, LARGE_CUBE_Y, LARGE_CUBE_Z));
  for (std::size_t index = 0; index < NUM_SMALL_CUBES; ++index)
  {
    objects.push_back(makeBoxObject(
      "task15_small_cube_" + std::to_string(index + 1),
      small_poses[index], SMALL_CUBE_SIZE, SMALL_CUBE_SIZE, SMALL_CUBE_SIZE));
  }
  if (!scene.applyCollisionObjects(objects))
  {
    return false;
  }
  std::this_thread::sleep_for(500ms);
  return true;
}

double supportSurfaceHeight(
  const ArmAssignment& assignment,
  const geometry_msgs::msg::Pose& large_pose,
  const std::array<geometry_msgs::msg::Pose, NUM_SMALL_CUBES>& small_poses,
  bool execute)
{
  if (assignment.support_small_index < 0)
  {
    // 真实执行读取紧协调阶段回写的 Isaac Ground Truth；只读预检使用与场景一致的名义高度。
    return execute ? large_pose.position.z + LARGE_CUBE_HALF_Z : LARGE_NOMINAL_TOP_Z;
  }

  const auto support_index = static_cast<std::size_t>(assignment.support_small_index);
  // 上层严格以对应下层 Cube 的最新实际顶面为支撑面。这样即使下层落稳有毫米级偏差，
  // 上层接触/释放高度仍与 PhysX 真实几何一致。
  return execute ? small_poses.at(support_index).position.z + SMALL_CUBE_HALF :
    LOWER_NOMINAL_TOP_Z;
}

fr3_dual_palletize::PrimitiveConfig makePrimitiveConfig(
  const ArmAssignment& assignment,
  bool left,
  double target_support_surface_z,
  const fr3_dual_palletize::RobustPlannerConfig& robust_planner)
{
  fr3_dual_palletize::PrimitiveConfig config;
  config.label = assignment.label;
  config.planning_group = left ? "left_arm" : "right_arm";
  config.eef_link = left ? "left_fr3_link8" : "right_fr3_link8";
  config.tool_link = left ? "left_fr3_compact_suction" : "right_fr3_compact_suction";
  config.moveit_joint_prefix = left ? "left_" : "right_";
  config.joint_command_topic = left ? "/left/joint_command" : "/right/joint_command";
  config.suction_command_topic = left ?
    "/task15/left/suction_command" : "/task15/right/suction_command";
  config.suction_state_topic = left ?
    "/task15/left/suction_state" : "/task15/right/suction_state";
  config.object_id = assignment.object_id;
  config.pose_index = assignment.pose_index;
  config.target_x = assignment.target_x;
  config.target_y = assignment.target_y;
  config.target_support_surface_z = target_support_surface_z;
  config.include_safe_egress = true;
  // 下层/上层均只在 Ground Truth 放置误差验收通过后锁定为 kinematic 支撑块；
  // Collider 保留，后续 Cube 的规划与 PhysX 接触都不会绕过它。
  config.freeze_after_settle = true;
  config.robust_planner = robust_planner;
  return config;
}

void logConflictReport(
  const rclcpp::Logger& logger,
  const fr3_dual_palletize::ConflictReport& report)
{
  if (!report.valid)
  {
    RCLCPP_ERROR(logger, "Task15 FCL 检查无效：%s", report.error.c_str());
    return;
  }
  if (!report.conflict)
  {
    RCLCPP_INFO(logger, "Task15 FCL: SAFE / NO_CONFLICT");
    return;
  }
  RCLCPP_WARN(
    logger,
    "Task15 FCL: CONFLICT first=%.3f s, windows=%zu",
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

bool applyDryRunReleasePoses(
  const fr3_dual_palletize::TaskTrajectoryCandidate& left_candidate,
  const fr3_dual_palletize::TaskTrajectoryCandidate& right_candidate)
{
  // 只读预检也把本层的计划释放位姿写入私有任务 World，使下一层 FCL 看到
  // 已放置障碍物；绝不向 Isaac 或吸盘话题发布任何命令。
  moveit::planning_interface::PlanningSceneInterface scene;
  scene.removeCollisionObjects({left_candidate.object_id, right_candidate.object_id});
  std::this_thread::sleep_for(250ms);
  const std::vector<moveit_msgs::msg::CollisionObject> objects{
    makeBoxObject(
      left_candidate.object_id,
      left_candidate.planned_release_pose,
      SMALL_CUBE_SIZE,
      SMALL_CUBE_SIZE,
      SMALL_CUBE_SIZE),
    makeBoxObject(
      right_candidate.object_id,
      right_candidate.planned_release_pose,
      SMALL_CUBE_SIZE,
      SMALL_CUBE_SIZE,
      SMALL_CUBE_SIZE),
  };
  if (!scene.applyCollisionObjects(objects))
  {
    return false;
  }
  std::this_thread::sleep_for(250ms);
  return true;
}

bool runBatch(
  const BatchConfig& batch,
  const rclcpp::Node::SharedPtr& coordinator_node,
  const rclcpp::Node::SharedPtr& left_node,
  const rclcpp::Node::SharedPtr& right_node,
  const std::shared_ptr<Task15PoseBuffer>& pose_buffer,
  const std::shared_ptr<std::mutex>& scene_mutex,
  bool execute,
  double wait_step_sec,
  double max_wait_sec,
  const fr3_dual_palletize::RobustPlannerConfig& robust_planner)
{
  const auto large_pose = pose_buffer->largePose();
  const auto small_poses = pose_buffer->smallSnapshot();
  const double left_support = supportSurfaceHeight(
    batch.left, large_pose, small_poses, execute);
  const double right_support = supportSurfaceHeight(
    batch.right, large_pose, small_poses, execute);

  RCLCPP_INFO(
    coordinator_node->get_logger(),
    "========== Task15 %s START: left_support=%.4f m, right_support=%.4f m ==========" ,
    batch.name.c_str(), left_support, right_support);

  auto provider = [pose_buffer](std::size_t index)
  {
    return pose_buffer->smallPose(index);
  };
  fr3_dual_palletize::PalletizePrimitive left(
    left_node, makePrimitiveConfig(
      batch.left, true, left_support, robust_planner), provider, scene_mutex);
  fr3_dual_palletize::PalletizePrimitive right(
    right_node, makePrimitiveConfig(
      batch.right, false, right_support, robust_planner), provider, scene_mutex);

  fr3_dual_palletize::TaskTrajectoryCandidate left_candidate;
  fr3_dual_palletize::TaskTrajectoryCandidate right_candidate;
  if (!left.planTaskTrajectoryCandidate(left_candidate) ||
      !right.planTaskTrajectoryCandidate(right_candidate))
  {
    RCLCPP_ERROR(coordinator_node->get_logger(), "Task15 %s：单臂完整候选规划失败。", batch.name.c_str());
    return false;
  }

  moveit::planning_interface::PlanningSceneInterface scene;
  std::vector<moveit_msgs::msg::CollisionObject> static_objects;
  for (const auto& pair : scene.getObjects())
  {
    static_objects.push_back(pair.second);
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
  coordination_config.allow_task_start_delay = true;
  fr3_dual_palletize::LocalWaitCoordinator coordinator(detector);
  const auto coordination = coordinator.solve(
    left_candidate, right_candidate, coordination_config);
  if (!coordination.valid || !coordination.coordinated ||
      coordination.verification_report.conflict)
  {
    RCLCPP_ERROR(
      coordinator_node->get_logger(),
      "Task15 %s：未得到 FCL SAFE 的松协调计划：%s",
      batch.name.c_str(), coordination.error.c_str());
    return false;
  }

  RCLCPP_INFO(
    coordinator_node->get_logger(),
    "Task15 %s：FCL SAFE，strategy=%s, yielding=%s, wait_stage=%s, wait=%.3f s, makespan=%.3f s",
    batch.name.c_str(),
    fr3_dual_palletize::localWaitStrategyName(coordination.strategy),
    coordination.yielding_arm.c_str(),
    coordination.wait_stage_name.c_str(),
    coordination.wait_duration_sec,
    coordination.coordinated_makespan_sec);

  if (!execute)
  {
    if (!applyDryRunReleasePoses(left_candidate, right_candidate))
    {
      RCLCPP_ERROR(
        coordinator_node->get_logger(),
        "Task15 %s：预检 release pose 回写 MoveIt World 失败。", batch.name.c_str());
      return false;
    }
    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "Task15 %s PRECHECK PASS：未发布任何 joint/suction 命令；已为下一层写入计划碰撞物。",
      batch.name.c_str());
    return true;
  }

  fr3_dual_palletize::CoordinatedTaskExecutor executor(
    coordinator_node->get_logger(), left, right);
  fr3_dual_palletize::CoordinatedTaskExecutionConfig execution_config;
  // Task15 的 PhysX 小件搬运采用 2x 物理时间伸缩并在末点保持 2 s。这样
  // Articulation position controller 与 Surface Gripper 有足够跟随时间；
  // FCL 已验收的几何路径、LocalWait 时序相对关系和抓放事件顺序均保持不变。
  execution_config.execution_time_scale = 2.0;
  execution_config.final_hold_sec = 2.0;
  const auto execution = executor.execute(
    coordination.coordinated_left, coordination.coordinated_right, execution_config);
  if (!execution.valid || !execution.completed)
  {
    RCLCPP_ERROR(
      coordinator_node->get_logger(),
      "Task15 %s：松协调实际执行失败：%s",
      batch.name.c_str(), execution.error.c_str());
    return false;
  }

  RCLCPP_INFO(
    coordinator_node->get_logger(),
    "Task15 %s PASS：左右独立小 Cube 已完成同层松协调，events=%zu, wall=%.3f s",
    batch.name.c_str(), execution.events_executed, execution.wall_duration_sec);
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto coordinator_node = std::make_shared<rclcpp::Node>("task15_loose_layer_stack");
  auto left_node = std::make_shared<rclcpp::Node>("task15_left_primitive");
  auto right_node = std::make_shared<rclcpp::Node>("task15_right_primitive");

  const bool execute = coordinator_node->declare_parameter<bool>("execute", false);
  const double wait_step_sec = coordinator_node->declare_parameter<double>("wait_step_sec", 0.20);
  const double max_wait_sec = coordinator_node->declare_parameter<double>("max_wait_sec", 30.0);
  const int planner_candidate_count = coordinator_node->declare_parameter<int>(
    "planner_candidate_count", 3);
  const std::string redundancy_mode_name = coordinator_node->declare_parameter<std::string>(
    "redundancy_mode", "soft_preference");
  // 与 Task16 固定场景 benchmark 保持一致：soft_preference 以中性 joint7=0
  // 为参考。它只参与候选排序，不构成 path constraint；因此仍可为可达性绕开
  // 障碍或远离关节极限而使用第七自由度。
  const double preferred_redundant_joint = coordinator_node->declare_parameter<double>(
    "preferred_redundant_joint", 0.0);
  fr3_dual_palletize::RedundancyMode redundancy_mode;
  if (wait_step_sec <= 0.0 || max_wait_sec < wait_step_sec ||
      planner_candidate_count <= 0 ||
      !fr3_dual_palletize::parseRedundancyMode(redundancy_mode_name, redundancy_mode))
  {
    RCLCPP_ERROR(
      coordinator_node->get_logger(),
      "Task15 LocalWait / Task16 RobustPlanner 参数无效。");
    rclcpp::shutdown();
    return 1;
  }
  fr3_dual_palletize::RobustPlannerConfig robust_planner;
  robust_planner.candidate_count = static_cast<std::size_t>(planner_candidate_count);
  robust_planner.redundancy_mode = redundancy_mode;
  robust_planner.preferred_redundant_joint = preferred_redundant_joint;
  if (!copyRobotModelParameters(left_node) || !copyRobotModelParameters(right_node))
  {
    rclcpp::shutdown();
    return 1;
  }

  auto pose_buffer = std::make_shared<Task15PoseBuffer>(coordinator_node);
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
      "========== Task15 Phase B / LOOSE LAYER STACK: execute=%s ==========" ,
      execute ? "true" : "false");
    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "Task16 RobustPlanner: candidate_count=%zu, redundancy_mode=%s, preferred_joint7=%.4f rad",
      robust_planner.candidate_count,
      fr3_dual_palletize::redundancyModeName(robust_planner.redundancy_mode),
      robust_planner.preferred_redundant_joint);
    if (!pose_buffer->wait(10.0))
    {
      RCLCPP_ERROR(
        coordinator_node->get_logger(),
        "等待 /task15/large_cube_pose 或 /task15/small_cube_poses 超时。请在 Timeline Play 后启动 Task15 bridge。");
      break;
    }

    const auto isaac_large_pose = pose_buffer->largePose();
    if (execute &&
        (std::abs(isaac_large_pose.position.x - EXPECTED_LARGE_TARGET_X) > 0.030 ||
         std::abs(isaac_large_pose.position.z - EXPECTED_LARGE_TARGET_Z) > 0.025))
    {
      RCLCPP_ERROR(
        coordinator_node->get_logger(),
        "Task15 Phase B 拒绝启动：LargeCube=(%.4f, %.4f, %.4f)，期望先完成 Phase A 紧协调目标附近 (%.3f, 0.000, %.3f)。",
        isaac_large_pose.position.x, isaac_large_pose.position.y, isaac_large_pose.position.z,
        EXPECTED_LARGE_TARGET_X, EXPECTED_LARGE_TARGET_Z);
      break;
    }

    // 一键预检不会执行 Phase A；因此仅在 execute:=false 时把 Phase A 的
    // 名义终点写入私有 MoveIt World，供 Phase B 验证完整规划/协调链路。
    // execute:=true 必须严格读取紧协调后 Isaac Ground Truth，绝不伪造物理状态。
    auto planning_large_pose = isaac_large_pose;
    if (!execute)
    {
      planning_large_pose.position.x = EXPECTED_LARGE_TARGET_X;
      planning_large_pose.position.y = 0.0;
      planning_large_pose.position.z = EXPECTED_LARGE_TARGET_Z;
      RCLCPP_INFO(
        coordinator_node->get_logger(),
        "Task15 Phase B PRECHECK：使用 Phase A 名义目标作为 LargeCube Planning Scene 位姿；未发布物理命令。");
    }

    if (!initializeTask15World(planning_large_pose, pose_buffer->smallSnapshot()))
    {
      RCLCPP_ERROR(coordinator_node->get_logger(), "Task15 初始大/小 Cube 加入 MoveIt World 失败。");
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
            max_wait_sec,
            robust_planner))
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
      "Task15 Phase B PASS：两层四个独立小 Cube 已按配置完成松协调码垛。");
  }
  else
  {
    RCLCPP_ERROR(
      coordinator_node->get_logger(),
      "Task15 Phase B FAIL：停止后续批次；未执行未经 FCL 验收的恢复动作。");
  }

  ros_executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}

// Task16-D：不连接 Isaac 控制链的 Task15 规划鲁棒性 benchmark。
//
// 本程序只向 MoveIt 写入 Task16 私有 CollisionObject，并发布合成 HOME joint_states
// 供 MoveIt CurrentStateMonitor 建立起点；它不会发布 joint_command 或 suction_command。
// 每次 trial 让左右 PalletizePrimitive 生成完整候选，经过 Task08 FCL 与 Task09
// LocalWait，再把每个自由空间阶段的多候选评分写入 CSV/JSON。

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <ompl/util/RandomNumbers.h>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "fr3_dual_palletize/local_wait_coordinator.hpp"
#include "fr3_dual_palletize/palletize_primitive.hpp"
#include "fr3_dual_palletize/spatiotemporal_conflict_detector.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double SMALL_SIZE = 0.030;
constexpr double LARGE_X = 0.220;
constexpr double LARGE_Y = 0.320;
constexpr double LARGE_Z = 0.080;
constexpr double LARGE_TOP_Z = 0.130;

struct CandidateRow
{
  int trial_id{0};
  std::string redundancy_mode;
  std::string arm;
  std::string stage;
  fr3_dual_palletize::RobustCandidateMetrics metrics;
  std::size_t requested_candidate_count{0};
  std::size_t selected_attempt_index{0};
  bool full_primitive_success{false};
  bool task08_safe{false};
  std::string task09_strategy{"not_run"};
  double task09_wait_sec{-1.0};
};

struct TrialSummary
{
  int trial_id{0};
  std::string redundancy_mode;
  bool left_primitive_success{false};
  bool right_primitive_success{false};
  bool task08_safe{false};
  bool task09_safe{false};
  std::string task09_strategy{"not_run"};
  double task09_wait_sec{-1.0};
  std::size_t candidate_rows{0};
};

geometry_msgs::msg::Pose makePose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id,
  const geometry_msgs::msg::Pose& pose,
  double x,
  double y,
  double z)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {x, y, z};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "Task16 无法连接 /move_group。");
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
    RCLCPP_ERROR(node->get_logger(), "Task16 无法读取双 FR3 RobotModel 参数。");
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

bool initializeTask16World()
{
  moveit::planning_interface::PlanningSceneInterface scene;
  scene.removeCollisionObjects({
    "task16_large_cube",
    "task16_small_cube_left",
    "task16_small_cube_right",
  });
  std::this_thread::sleep_for(250ms);

  const std::vector<moveit_msgs::msg::CollisionObject> objects{
    makeBox("task16_large_cube", makePose(0.650, 0.000, 0.090), LARGE_X, LARGE_Y, LARGE_Z),
    makeBox("task16_small_cube_left", makePose(0.320, -0.250, 0.065), SMALL_SIZE, SMALL_SIZE, SMALL_SIZE),
    makeBox("task16_small_cube_right", makePose(0.320, 0.250, 0.065), SMALL_SIZE, SMALL_SIZE, SMALL_SIZE),
  };
  if (!scene.applyCollisionObjects(objects))
  {
    return false;
  }
  std::this_thread::sleep_for(300ms);
  return true;
}

void clearTask16World()
{
  moveit::planning_interface::PlanningSceneInterface scene;
  scene.removeCollisionObjects({
    "task16_large_cube",
    "task16_small_cube_left",
    "task16_small_cube_right",
  });
}

fr3_dual_palletize::PrimitiveConfig makeConfig(
  bool left,
  const fr3_dual_palletize::RobustPlannerConfig& robust_config)
{
  fr3_dual_palletize::PrimitiveConfig config;
  config.label = left ? "TASK16 LEFT / Task15 lower reference" :
    "TASK16 RIGHT / Task15 lower reference";
  config.planning_group = left ? "left_arm" : "right_arm";
  config.eef_link = left ? "left_fr3_link8" : "right_fr3_link8";
  config.tool_link = left ? "left_fr3_compact_suction" : "right_fr3_compact_suction";
  config.moveit_joint_prefix = left ? "left_" : "right_";
  // Benchmark 只调用 planTaskTrajectoryCandidate，下面的话题不会被发布。
  config.joint_command_topic = left ? "/left/joint_command" : "/right/joint_command";
  config.suction_command_topic = left ? "/task15/left/suction_command" :
    "/task15/right/suction_command";
  config.suction_state_topic = left ? "/task15/left/suction_state" :
    "/task15/right/suction_state";
  config.object_id = left ? "task16_small_cube_left" : "task16_small_cube_right";
  config.pose_index = left ? 0 : 1;
  config.target_x = 0.650;
  config.target_y = left ? -0.080 : 0.080;
  config.target_support_surface_z = LARGE_TOP_Z;
  config.include_safe_egress = true;
  config.robust_planner = robust_config;
  return config;
}

std::string csvEscape(const std::string& value)
{
  std::string escaped{"\""};
  for (const char character : value)
  {
    if (character == '\"')
    {
      escaped += "\"\"";
    }
    else
    {
      escaped += character;
    }
  }
  escaped += "\"";
  return escaped;
}

std::string jsonEscape(const std::string& value)
{
  std::string escaped;
  for (const char character : value)
  {
    switch (character)
    {
      case '\\': escaped += "\\\\"; break;
      case '\"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += character; break;
    }
  }
  return escaped;
}

bool writeCsv(const std::string& path, const std::vector<CandidateRow>& rows)
{
  std::ofstream output(path);
  if (!output)
  {
    return false;
  }
  output << "trial_id,redundancy_mode,arm,stage,candidate_index,selected,plan_success,"
    "candidate_count,planning_time_sec,path_length,min_joint_limit_margin,"
    "joint_limit_cost,redundancy_cost,total_cost,full_primitive_success,"
    "task08_safe,task09_strategy,task09_wait_sec,failure_reason\n";
  output << std::fixed << std::setprecision(6);
  for (const auto& row : rows)
  {
    output << row.trial_id << ',' << csvEscape(row.redundancy_mode) << ','
      << csvEscape(row.arm) << ',' << csvEscape(row.stage) << ','
      << row.metrics.attempt_index << ','
      << (row.metrics.attempt_index == row.selected_attempt_index ? "true" : "false") << ','
      << (row.metrics.plan_success ? "true" : "false") << ','
      << row.requested_candidate_count << ',' << row.metrics.planning_time_sec << ','
      << row.metrics.path_length << ',' << row.metrics.min_joint_limit_margin << ','
      << row.metrics.joint_limit_cost << ',' << row.metrics.redundancy_cost << ','
      << row.metrics.total_cost << ','
      << (row.full_primitive_success ? "true" : "false") << ','
      << (row.task08_safe ? "true" : "false") << ','
      << csvEscape(row.task09_strategy) << ',' << row.task09_wait_sec << ','
      << csvEscape(row.metrics.failure_reason) << '\n';
  }
  return static_cast<bool>(output);
}

bool writeJson(const std::string& path,
               std::uint_fast32_t random_seed,
               std::size_t candidate_count,
               const std::string& redundancy_mode,
               const std::vector<TrialSummary>& trials)
{
  std::ofstream output(path);
  if (!output)
  {
    return false;
  }
  output << "{\n"
    << "  \"scenario\": \"task15_lower_layer_reference\",\n"
    << "  \"random_seed\": " << random_seed << ",\n"
    << "  \"planner_candidate_count\": " << candidate_count << ",\n"
    << "  \"redundancy_mode\": \"" << jsonEscape(redundancy_mode) << "\",\n"
    << "  \"trials\": [\n";
  for (std::size_t index = 0; index < trials.size(); ++index)
  {
    const auto& trial = trials[index];
    output << "    {\"trial_id\": " << trial.trial_id
      << ", \"left_primitive_success\": " << (trial.left_primitive_success ? "true" : "false")
      << ", \"right_primitive_success\": " << (trial.right_primitive_success ? "true" : "false")
      << ", \"task08_safe\": " << (trial.task08_safe ? "true" : "false")
      << ", \"task09_safe\": " << (trial.task09_safe ? "true" : "false")
      << ", \"task09_strategy\": \"" << jsonEscape(trial.task09_strategy) << "\""
      << ", \"task09_wait_sec\": " << std::fixed << std::setprecision(6)
      << trial.task09_wait_sec
      << ", \"candidate_rows\": " << trial.candidate_rows << "}";
    output << (index + 1 == trials.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
  return static_cast<bool>(output);
}

void addRows(
  std::vector<CandidateRow>& rows,
  int trial_id,
  const std::string& mode,
  const std::string& arm,
  const std::string& stage,
  const fr3_dual_palletize::RobustPlanResult& result)
{
  for (const auto& metrics : result.candidates)
  {
    CandidateRow row;
    row.trial_id = trial_id;
    row.redundancy_mode = mode;
    row.arm = arm;
    row.stage = stage;
    row.metrics = metrics;
    row.requested_candidate_count = result.requested_candidate_count;
    row.selected_attempt_index = result.selected_attempt_index;
    rows.push_back(std::move(row));
  }
}

void annotateTrialRows(
  std::vector<CandidateRow>& rows,
  std::size_t start_index,
  bool primitive_success,
  bool task08_safe,
  const std::string& task09_strategy,
  double task09_wait_sec)
{
  for (std::size_t index = start_index; index < rows.size(); ++index)
  {
    rows[index].full_primitive_success = primitive_success;
    rows[index].task08_safe = task08_safe;
    rows[index].task09_strategy = task09_strategy;
    rows[index].task09_wait_sec = task09_wait_sec;
  }
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto coordinator_node = std::make_shared<rclcpp::Node>("task16_planning_benchmark");
  auto left_node = std::make_shared<rclcpp::Node>("task16_left_primitive");
  auto right_node = std::make_shared<rclcpp::Node>("task16_right_primitive");

  const int trials_parameter = coordinator_node->declare_parameter<int>("number_of_trials", 10);
  const int candidates_parameter = coordinator_node->declare_parameter<int>(
    "planner_candidate_count", 5);
  const int seed_parameter = coordinator_node->declare_parameter<int>(
    "random_seed", 20260912);
  const std::string mode_name = coordinator_node->declare_parameter<std::string>(
    "redundancy_mode", "free_7dof");
  const std::string csv_path = coordinator_node->declare_parameter<std::string>(
    "output_csv", "");
  const std::string json_path = coordinator_node->declare_parameter<std::string>(
    "output_json", "");

  fr3_dual_palletize::RedundancyMode redundancy_mode;
  if (trials_parameter <= 0 || candidates_parameter <= 0 || seed_parameter < 0 ||
      csv_path.empty() || json_path.empty() ||
      !fr3_dual_palletize::parseRedundancyMode(mode_name, redundancy_mode))
  {
    RCLCPP_ERROR(
      coordinator_node->get_logger(),
      "Task16 参数无效：number_of_trials/candidate_count 必须为正，"
      "random_seed 必须非负，输出 CSV/JSON 路径不能为空，redundancy_mode 必须有效。");
    rclcpp::shutdown();
    return 1;
  }

  // 在首个 OMPL planner 创建前设置种子：同一进程内的 trial 通过 run_index
  // 消费同一可复现实验序列；切换 redundancy_mode 应启动独立 benchmark 进程。
  ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(seed_parameter));
  if (!copyRobotModelParameters(left_node) || !copyRobotModelParameters(right_node))
  {
    rclcpp::shutdown();
    return 1;
  }

  const auto joint_state_publisher = coordinator_node->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 20);
  const auto publish_home_state = [joint_state_publisher, coordinator_node]()
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
    // 与 Task15 / Task06 已验收的 HOME 一致；只提供 MoveIt 起点，不下发 Isaac 命令。
    message.position = {
      0.0001, 0.0002, -0.0002, -0.1518, 0.0000, 0.5445, 0.0000,
      0.0001, 0.0002, -0.0002, -0.1518, 0.0000, 0.5445, 0.0000,
    };
    joint_state_publisher->publish(message);
  };
  const auto joint_state_timer = coordinator_node->create_wall_timer(50ms, publish_home_state);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(coordinator_node);
  executor.add_node(left_node);
  executor.add_node(right_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  std::vector<CandidateRow> rows;
  std::vector<TrialSummary> summaries;
  bool infrastructure_ok = true;
  do
  {
    std::this_thread::sleep_for(1200ms);
    fr3_dual_palletize::RobustPlannerConfig robust_config;
    robust_config.candidate_count = static_cast<std::size_t>(candidates_parameter);
    robust_config.redundancy_mode = redundancy_mode;
    // Group B/C 均相对中性地偏好 joint7 = 0；Group A 忽略此值。
    robust_config.preferred_redundant_joint = 0.0;

    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "========== Task16 BENCHMARK: scenario=Task15 lower, trials=%d, candidates=%d, mode=%s, seed=%d ==========" ,
      trials_parameter, candidates_parameter,
      fr3_dual_palletize::redundancyModeName(redundancy_mode), seed_parameter);

    for (int trial_id = 1; trial_id <= trials_parameter; ++trial_id)
    {
      if (!initializeTask16World())
      {
        RCLCPP_ERROR(coordinator_node->get_logger(), "Task16 无法初始化私有 Planning Scene。");
        infrastructure_ok = false;
        break;
      }

      const std::size_t row_start = rows.size();
      const auto left_provider = [](std::size_t)
      {
        return makePose(0.320, -0.250, 0.065);
      };
      const auto right_provider = [](std::size_t)
      {
        return makePose(0.320, 0.250, 0.065);
      };
      const auto scene_mutex = std::make_shared<std::mutex>();
      fr3_dual_palletize::PalletizePrimitive left(
        left_node, makeConfig(true, robust_config), left_provider, scene_mutex,
        [&rows, trial_id, mode_name](const std::string& stage,
          const fr3_dual_palletize::RobustPlanResult& result)
        {
          addRows(rows, trial_id, mode_name, "left", stage, result);
        });
      fr3_dual_palletize::PalletizePrimitive right(
        right_node, makeConfig(false, robust_config), right_provider, scene_mutex,
        [&rows, trial_id, mode_name](const std::string& stage,
          const fr3_dual_palletize::RobustPlanResult& result)
        {
          addRows(rows, trial_id, mode_name, "right", stage, result);
        });

      fr3_dual_palletize::TaskTrajectoryCandidate left_candidate;
      fr3_dual_palletize::TaskTrajectoryCandidate right_candidate;
      TrialSummary summary;
      summary.trial_id = trial_id;
      summary.redundancy_mode = mode_name;
      summary.left_primitive_success = left.planTaskTrajectoryCandidate(left_candidate);
      summary.right_primitive_success = right.planTaskTrajectoryCandidate(right_candidate);

      if (summary.left_primitive_success && summary.right_primitive_success)
      {
        moveit::planning_interface::PlanningSceneInterface scene;
        std::vector<moveit_msgs::msg::CollisionObject> static_objects;
        for (const auto& pair : scene.getObjects())
        {
          static_objects.push_back(pair.second);
        }
        moveit::planning_interface::MoveGroupInterface detector_group(left_node, "left_arm");
        fr3_dual_palletize::SpatioTemporalConflictDetector detector(
          detector_group.getRobotModel(), std::move(static_objects));
        const auto fcl_report = detector.check(left_candidate, right_candidate, 0.01);
        summary.task08_safe = fcl_report.valid && !fcl_report.conflict;

        if (fcl_report.valid)
        {
          fr3_dual_palletize::LocalWaitCoordinationConfig wait_config;
          wait_config.wait_step_sec = 0.20;
          wait_config.max_wait_sec = 30.0;
          wait_config.prefer_left = true;
          wait_config.allow_task_start_delay = true;
          fr3_dual_palletize::LocalWaitCoordinator coordinator(detector);
          const auto coordinated = coordinator.solve(left_candidate, right_candidate, wait_config);
          summary.task09_safe = coordinated.valid && coordinated.coordinated &&
            !coordinated.verification_report.conflict;
          if (coordinated.valid)
          {
            summary.task09_strategy =
              fr3_dual_palletize::localWaitStrategyName(coordinated.strategy);
            summary.task09_wait_sec = coordinated.wait_duration_sec;
          }
        }
      }

      summary.candidate_rows = rows.size() - row_start;
      annotateTrialRows(
        rows, row_start,
        summary.left_primitive_success && summary.right_primitive_success,
        summary.task08_safe, summary.task09_strategy, summary.task09_wait_sec);
      summaries.push_back(summary);
      RCLCPP_INFO(
        coordinator_node->get_logger(),
        "Task16 trial=%d: left=%s, right=%s, Task08=%s, Task09=%s, wait=%.3f s, rows=%zu",
        trial_id,
        summary.left_primitive_success ? "PASS" : "FAIL",
        summary.right_primitive_success ? "PASS" : "FAIL",
        summary.task08_safe ? "SAFE" : "UNSAFE",
        summary.task09_safe ? summary.task09_strategy.c_str() : "NO_SOLUTION",
        summary.task09_wait_sec, summary.candidate_rows);
    }
  } while (false);

  clearTask16World();
  const bool output_ok = infrastructure_ok &&
    writeCsv(csv_path, rows) &&
    writeJson(
      json_path, static_cast<std::uint_fast32_t>(seed_parameter),
      static_cast<std::size_t>(candidates_parameter), mode_name, summaries);
  if (!output_ok)
  {
    RCLCPP_ERROR(coordinator_node->get_logger(), "Task16 benchmark 输出 CSV/JSON 失败。");
  }
  else
  {
    RCLCPP_INFO(
      coordinator_node->get_logger(),
      "Task16 benchmark 完成：CSV=%s, JSON=%s。失败 trial 也已作为实验数据记录。",
      csv_path.c_str(), json_path.c_str());
  }

  executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return output_ok ? 0 : 1;
}

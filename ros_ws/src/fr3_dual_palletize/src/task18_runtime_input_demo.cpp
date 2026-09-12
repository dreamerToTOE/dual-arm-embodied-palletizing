// Task18：只读 Ground Truth 运行时输入验证器。
//
// 它不读取固定初始坐标、不连接 MoveIt action、也不发送任何机器人或吸盘命令。
// 收到 BoxStateArray 后，将每个对象转换成 Task17 BoxSpec / CollisionObject，
// 并由本次的世界 pose + 局部 grasp candidate 计算 pick pose。

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/msg/box_state_array.hpp"
#include "fr3_dual_palletize/palletizing_job.hpp"
#include "fr3_dual_palletize/runtime_box_state.hpp"

namespace
{

class RuntimeInputBuffer
{
public:
  explicit RuntimeInputBuffer(const rclcpp::Node::SharedPtr& node)
  {
    subscription_ = node->create_subscription<fr3_dual_palletize::msg::BoxStateArray>(
      "/task18/box_states", rclcpp::QoS(10),
      [this](const fr3_dual_palletize::msg::BoxStateArray::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_ = *message;
          received_ = true;
        }
        condition_.notify_all();
      });
  }

  bool waitForMessage(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock, std::chrono::duration<double>(timeout_sec), [this]() { return received_; });
  }

  fr3_dual_palletize::msg::BoxStateArray latest() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool received_{false};
  fr3_dual_palletize::msg::BoxStateArray latest_;
  rclcpp::Subscription<fr3_dual_palletize::msg::BoxStateArray>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task18_runtime_input_demo");
  const double timeout_sec = node->declare_parameter<double>("timeout_sec", 10.0);
  const std::int64_t expected_seed = node->declare_parameter<std::int64_t>("expected_seed", -1);
  const std::int64_t min_boxes = node->declare_parameter<std::int64_t>("min_boxes", 1);
  if (timeout_sec <= 0.0 || min_boxes < 1)
  {
    RCLCPP_ERROR(node->get_logger(), "timeout_sec 必须为正数，min_boxes 至少为 1。");
    rclcpp::shutdown();
    return 1;
  }

  RuntimeInputBuffer buffer(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  RCLCPP_INFO(
    node->get_logger(),
    "Task18 只读运行时输入验证已启动：等待 /task18/box_states（timeout=%.1f s）。",
    timeout_sec);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    if (buffer.waitForMessage(0.02))
    {
      break;
    }
  }
  if (!rclcpp::ok())
  {
    rclcpp::shutdown();
    return 1;
  }

  const auto states = buffer.latest();
  if (states.boxes.size() < static_cast<std::size_t>(min_boxes))
  {
    RCLCPP_ERROR(
      node->get_logger(), "Task18 FAIL：收到 boxes=%zu，小于 min_boxes=%ld。",
      states.boxes.size(), min_boxes);
    rclcpp::shutdown();
    return 1;
  }
  if (expected_seed >= 0 && states.seed != static_cast<std::uint64_t>(expected_seed))
  {
    RCLCPP_ERROR(
      node->get_logger(), "Task18 FAIL：期望 seed=%ld，实际 seed=%lu。",
      expected_seed, static_cast<unsigned long>(states.seed));
    rclcpp::shutdown();
    return 1;
  }

  std::set<std::string> ids;
  for (const auto& state : states.boxes)
  {
    fr3_dual_palletize::BoxSpec box;
    std::string error;
    if (!ids.insert(state.id).second ||
        !fr3_dual_palletize::boxStateToBoxSpec(state, box, error))
    {
      RCLCPP_ERROR(
        node->get_logger(), "Task18 FAIL：运行时 BoxState 无效 id=%s: %s",
        state.id.c_str(), error.c_str());
      rclcpp::shutdown();
      return 1;
    }

    const auto collision = fr3_dual_palletize::makeCollisionObject(box, box.initial_pose);
    const auto pick_pose = fr3_dual_palletize::composePose(
      box.initial_pose, box.grasp_candidates.front().local_pose);
    if (collision.primitives.size() != 1 || collision.primitive_poses.size() != 1)
    {
      RCLCPP_ERROR(node->get_logger(), "Task18 FAIL：CollisionObject 转换失败：%s", box.id.c_str());
      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "seed=%lu BoxSpec id=%s class=%s size=(%.3f, %.3f, %.3f) mass=%.3f "
      "pose=(%.3f, %.3f, %.3f) -> runtime pick=(%.3f, %.3f, %.3f), grasp=%s",
      static_cast<unsigned long>(states.seed), box.id.c_str(), box.payload_class.c_str(),
      box.dimensions[0], box.dimensions[1], box.dimensions[2], box.mass,
      box.initial_pose.position.x, box.initial_pose.position.y, box.initial_pose.position.z,
      pick_pose.position.x, pick_pose.position.y, pick_pose.position.z,
      box.grasp_candidates.front().id.c_str());
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Task18 PASS：已从运行时 Ground Truth 生成 %zu 个 BoxSpec、CollisionObject 与 pick pose；"
    "未读取任何固定初始坐标，也未执行机器人命令。",
    states.boxes.size());
  rclcpp::shutdown();
  return 0;
}

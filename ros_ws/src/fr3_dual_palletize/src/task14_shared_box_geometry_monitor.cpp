#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <limits>
#include <string>
#include <thread>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double PI = 3.14159265358979323846;

geometry_msgs::msg::Point subtract(
  const geometry_msgs::msg::Point& first,
  const geometry_msgs::msg::Point& second)
{
  geometry_msgs::msg::Point result;
  result.x = first.x - second.x;
  result.y = first.y - second.y;
  result.z = first.z - second.z;
  return result;
}

geometry_msgs::msg::Point midpoint(
  const geometry_msgs::msg::Point& first,
  const geometry_msgs::msg::Point& second)
{
  geometry_msgs::msg::Point result;
  result.x = 0.5 * (first.x + second.x);
  result.y = 0.5 * (first.y + second.y);
  result.z = 0.5 * (first.z + second.z);
  return result;
}

double norm(const geometry_msgs::msg::Point& point)
{
  return std::sqrt(
    point.x * point.x + point.y * point.y + point.z * point.z);
}

double quaternionAngle(
  const geometry_msgs::msg::Quaternion& first,
  const geometry_msgs::msg::Quaternion& second)
{
  const double first_norm = std::sqrt(
    first.x * first.x + first.y * first.y + first.z * first.z + first.w * first.w);
  const double second_norm = std::sqrt(
    second.x * second.x + second.y * second.y + second.z * second.z + second.w * second.w);
  if (first_norm <= 1e-9 || second_norm <= 1e-9)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double dot = std::abs(
    first.x * second.x + first.y * second.y + first.z * second.z + first.w * second.w) /
    (first_norm * second_norm);
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

class SharedBoxGeometryMonitor : public rclcpp::Node
{
public:
  SharedBoxGeometryMonitor()
    : Node("task14_shared_box_geometry_monitor")
  {
    relative_tcp_tolerance_ = declare_parameter<double>(
      "relative_tcp_tolerance_m", 0.003);
    box_midpoint_tolerance_ = declare_parameter<double>(
      "box_midpoint_tolerance_m", 0.005);
    orientation_tolerance_ = declare_parameter<double>(
      "orientation_tolerance_rad", 0.035);
    sync_tolerance_sec_ = declare_parameter<double>("sync_tolerance_sec", 0.030);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 45.0);

    box_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/task11/shared_box_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
      {
        box_pose_ = message->pose;
        box_stamp_ns_ = stampNs(message->header.stamp);
        have_box_ = true;
        sampleIfClosed();
      });
    left_tcp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/task11/left/suction_tcp_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
      {
        left_tcp_ = message->pose;
        left_tcp_stamp_ns_ = stampNs(message->header.stamp);
        have_left_tcp_ = true;
        sampleIfClosed();
      });
    right_tcp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/task11/right/suction_tcp_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
      {
        right_tcp_ = message->pose;
        right_tcp_stamp_ns_ = stampNs(message->header.stamp);
        have_right_tcp_ = true;
        sampleIfClosed();
      });
    left_state_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/task11/left/suction_state", 10,
      [this](const std_msgs::msg::Bool::SharedPtr message)
      {
        left_closed_ = message->data;
        finishIfReleased();
      });
    right_state_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/task11/right/suction_state", 10,
      [this](const std_msgs::msg::Bool::SharedPtr message)
      {
        right_closed_ = message->data;
        finishIfReleased();
      });
  }

  bool validParameters() const
  {
    return relative_tcp_tolerance_ > 0.0 &&
           box_midpoint_tolerance_ > 0.0 &&
           orientation_tolerance_ > 0.0 && sync_tolerance_sec_ > 0.0 && timeout_sec_ > 0.0;
  }

  bool finished() const { return finished_; }
  bool started() const { return started_; }

  void report() const
  {
    if (!started_)
    {
      RCLCPP_ERROR(get_logger(), "Task14 FAIL：在 timeout 内未观测到两吸盘同时 CLOSED。");
      return;
    }
    if (samples_ == 0)
    {
      RCLCPP_ERROR(get_logger(), "Task14 FAIL：双吸附窗口内没有有效位姿样本。");
      return;
    }
    const auto rms = [this](double squared_sum)
    {
      return std::sqrt(squared_sum / static_cast<double>(samples_));
    };
    const bool pass =
      max_relative_tcp_error_ <= relative_tcp_tolerance_ &&
      max_box_midpoint_error_ <= box_midpoint_tolerance_ &&
      max_orientation_error_ <= orientation_tolerance_;
    RCLCPP_INFO(get_logger(), "========== Task14 CONTINUOUS GEOMETRY REPORT ==========");
    RCLCPP_INFO(
      get_logger(), "samples=%zu, timestamp_rejected=%zu, closed_window_finished=%s",
      samples_, timestamp_rejected_, finished_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "relative_tcp: max=%.3f mm, rms=%.3f mm, limit=%.3f mm",
      max_relative_tcp_error_ * 1000.0, rms(relative_tcp_squared_sum_) * 1000.0,
      relative_tcp_tolerance_ * 1000.0);
    RCLCPP_INFO(
      get_logger(),
      "box_to_tcp_midpoint: max=%.3f mm, rms=%.3f mm, limit=%.3f mm",
      max_box_midpoint_error_ * 1000.0, rms(box_midpoint_squared_sum_) * 1000.0,
      box_midpoint_tolerance_ * 1000.0);
    RCLCPP_INFO(
      get_logger(),
      "box_orientation: max=%.3f deg, rms=%.3f deg, limit=%.3f deg",
      max_orientation_error_ * 180.0 / PI,
      rms(orientation_squared_sum_) * 180.0 / PI,
      orientation_tolerance_ * 180.0 / PI);
    if (pass)
    {
      RCLCPP_INFO(get_logger(), "Task14-A PASS：双吸附窗口内连续相对几何均在阈值内。");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Task14-A FAIL：连续相对几何误差超限。");
    }
  }

  bool passed() const
  {
    return started_ && samples_ > 0 &&
           max_relative_tcp_error_ <= relative_tcp_tolerance_ &&
           max_box_midpoint_error_ <= box_midpoint_tolerance_ &&
           max_orientation_error_ <= orientation_tolerance_;
  }

private:
  static std::int64_t stampNs(const builtin_interfaces::msg::Time& stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(stamp.nanosec);
  }

  void sampleIfClosed()
  {
    if (!left_closed_ || !right_closed_ ||
        !have_box_ || !have_left_tcp_ || !have_right_tcp_)
    {
      return;
    }
    if (box_stamp_ns_ <= last_sampled_box_stamp_ns_)
    {
      return;
    }
    const auto earliest_stamp = std::min({
      box_stamp_ns_, left_tcp_stamp_ns_, right_tcp_stamp_ns_});
    const auto latest_stamp = std::max({
      box_stamp_ns_, left_tcp_stamp_ns_, right_tcp_stamp_ns_});
    const double timestamp_skew_sec = static_cast<double>(
      latest_stamp - earliest_stamp) * 1e-9;
    if (timestamp_skew_sec > sync_tolerance_sec_)
    {
      ++timestamp_rejected_;
      return;
    }
    last_sampled_box_stamp_ns_ = box_stamp_ns_;
    const auto relative_tcp = subtract(right_tcp_.position, left_tcp_.position);
    const auto box_to_midpoint = subtract(
      box_pose_.position, midpoint(left_tcp_.position, right_tcp_.position));
    if (!started_)
    {
      initial_relative_tcp_ = relative_tcp;
      initial_box_to_midpoint_ = box_to_midpoint;
      initial_orientation_ = box_pose_.orientation;
      started_ = true;
      RCLCPP_INFO(get_logger(), "Task14：检测到双吸盘 CLOSED，开始连续几何采样。");
    }
    const double relative_error = norm(subtract(relative_tcp, initial_relative_tcp_));
    const double midpoint_error = norm(subtract(box_to_midpoint, initial_box_to_midpoint_));
    const double orientation_error = quaternionAngle(box_pose_.orientation, initial_orientation_);
    max_relative_tcp_error_ = std::max(max_relative_tcp_error_, relative_error);
    max_box_midpoint_error_ = std::max(max_box_midpoint_error_, midpoint_error);
    max_orientation_error_ = std::max(max_orientation_error_, orientation_error);
    relative_tcp_squared_sum_ += relative_error * relative_error;
    box_midpoint_squared_sum_ += midpoint_error * midpoint_error;
    orientation_squared_sum_ += orientation_error * orientation_error;
    ++samples_;
  }

  void finishIfReleased()
  {
    if (started_ && (!left_closed_ || !right_closed_))
    {
      finished_ = true;
    }
  }

  double relative_tcp_tolerance_{0.003};
  double box_midpoint_tolerance_{0.005};
  double orientation_tolerance_{0.035};
  double sync_tolerance_sec_{0.030};
  double timeout_sec_{45.0};
  bool have_box_{false};
  bool have_left_tcp_{false};
  bool have_right_tcp_{false};
  bool left_closed_{false};
  bool right_closed_{false};
  bool started_{false};
  bool finished_{false};
  std::size_t samples_{0};
  std::size_t timestamp_rejected_{0};
  double max_relative_tcp_error_{0.0};
  double max_box_midpoint_error_{0.0};
  double max_orientation_error_{0.0};
  double relative_tcp_squared_sum_{0.0};
  double box_midpoint_squared_sum_{0.0};
  double orientation_squared_sum_{0.0};
  geometry_msgs::msg::Pose box_pose_;
  geometry_msgs::msg::Pose left_tcp_;
  geometry_msgs::msg::Pose right_tcp_;
  geometry_msgs::msg::Point initial_relative_tcp_;
  geometry_msgs::msg::Point initial_box_to_midpoint_;
  geometry_msgs::msg::Quaternion initial_orientation_;
  std::int64_t box_stamp_ns_{0};
  std::int64_t left_tcp_stamp_ns_{0};
  std::int64_t right_tcp_stamp_ns_{0};
  std::int64_t last_sampled_box_stamp_ns_{0};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr box_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_tcp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_tcp_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_state_sub_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SharedBoxGeometryMonitor>();
  if (!node->validParameters())
  {
    RCLCPP_ERROR(node->get_logger(), "Task14 参数必须为正数。");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(
    node->get_logger(),
    "Task14-A 只读监测已启动：等待双吸盘 CLOSED，结束于任一吸盘 OPEN。");
  const double timeout_sec = node->get_parameter("timeout_sec").as_double();
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_sec);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline && !node->finished())
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  node->report();
  const bool pass = node->finished() && node->passed();
  executor.remove_node(node);
  rclcpp::shutdown();
  return pass ? 0 : 1;
}

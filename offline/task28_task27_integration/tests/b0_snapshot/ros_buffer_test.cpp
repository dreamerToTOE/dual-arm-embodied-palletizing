#define TASK27_FIVE_CUBE 1
#define main task28_original_executor_main
#include "task26_truck_box_push_in.cpp"
#undef main

#include <iostream>
#include <stdexcept>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task28_b0_snapshot_buffer_test");
  B0SnapshotBuffer buffer(node);
  auto publisher = node->create_publisher<std_msgs::msg::String>(taskTopic("/b0_poststep_snapshot"), 10);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&]() { executor.spin(); });
  bool ok = true;
  for (int i = 0; i < 100 && publisher->get_subscription_count() == 0; ++i)
    std::this_thread::sleep_for(20ms);
  if (publisher->get_subscription_count() == 0) ok = false;
  auto send = [&](int step, std::int64_t started, const std::string& frame = "world") {
    std_msgs::msg::String message;
    std::ostringstream out;
    out << "{version: 1, frame: " << frame << ", source: post_step_physx_tensor, handle_valid: true, step: "
        << step << ", simulation_time_s: " << step * .05 << ", sample_started_ns: " << started
        << ", sample_finished_ns: " << started
        << ", cube_xyz_m: [1.097104073, .06, .26], cube_quaternion_wxyz: [1, 0, 0, 0], "
        << "left_q_rad: [0, 0, 0, 0, 0, 0, 0], right_q_rad: [0, 0, 0, 0, 0, 0, 0]}";
    message.data = out.str();
    publisher->publish(message);
  };
  task28_b0::Snapshot result;
  auto mark = wallNowNs();
  send(1, mark - 1000000);
  ok = ok && !buffer.waitAfter(mark, .15, &result);
  mark = wallNowNs();
  std::this_thread::sleep_for(2ms);
  send(2, wallNowNs());
  ok = ok && buffer.waitAfter(mark, 2.0, &result) && result.step == 2 &&
    std::abs(result.cube[0] - 1.097104073) < 1e-9;
  mark = wallNowNs();
  std::this_thread::sleep_for(2ms);
  send(1, wallNowNs());
  ok = ok && !buffer.waitAfter(mark, .15, &result);
  mark = wallNowNs();
  std::this_thread::sleep_for(2ms);
  send(3, wallNowNs(), "map");
  ok = ok && !buffer.waitAfter(mark, .15, &result);
  mark = wallNowNs();
  std::this_thread::sleep_for(2ms);
  send(3, wallNowNs());
  ok = ok && buffer.waitAfter(mark, 2.0, &result) && result.step == 3;
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  std::cout << (ok ? "B0 ROS snapshot buffer PASS" : "B0 ROS snapshot buffer FAIL") << '\n';
  return ok ? 0 : 1;
}

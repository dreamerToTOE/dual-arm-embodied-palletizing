#include "task28_b0_snapshot.hpp"
#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>

int main(int argc, char** argv)
{
  const std::string good = R"({version: 1, frame: world, source: post_step_physx_tensor,
    handle_valid: true, step: 12, simulation_time_s: 0.2,
    sample_started_ns: 1000000000, sample_finished_ns: 1001000000,
    cube_xyz_m: [1.097104073, 0.06, 0.26], cube_quaternion_wxyz: [1, 0, 0, 0],
    left_q_rad: [0, 0, 0, 0, 0, 0, 0], right_q_rad: [0, 0, 0, 0, 0, 0, 0]})";
  task28_b0::Snapshot s;
  assert(task28_b0::decode(good, &s));
  assert(task28_b0::fresh(s, 1002000000, 999000000));
  assert(!task28_b0::fresh(s, 1052000000, 999000000));
  assert(!task28_b0::fresh(s, 1002000000, 1000000001));
  assert(!task28_b0::fresh(s, 1000000000, 0));
  assert(!task28_b0::fresh(s, 1002000000, 0, &s));
  auto prior = s;
  prior.step -= 1;
  prior.finished_ns = s.started_ns - 1;
  assert(!task28_b0::fresh(s, 1002000000, 0, &prior));  // Same simulation timestamp.
  prior.sim_s -= .05;
  assert(task28_b0::fresh(s, 1002000000, 0, &prior));
  std::vector<std::string> left_names, right_names;
  for (int i = 1; i <= 7; ++i) {
    left_names.push_back("left_fr3_joint" + std::to_string(i));
    right_names.push_back("right_fr3_joint" + std::to_string(i));
  }
  std::vector<double> left, right;
  assert(task28_b0::initialPositions(s, left_names, right_names, &left, &right));
  assert(left.size() == 7 && right.size() == 7);
  std::swap(left_names[0], left_names[1]);
  assert(!task28_b0::initialPositions(s, left_names, right_names, &left, &right));
  auto bad = good;
  bad.replace(bad.find("frame: world"), 12, "frame: other");
  assert(!task28_b0::decode(bad, &s));
  bad = good;
  bad.replace(bad.find("handle_valid: true"), 18, "handle_valid: false");
  assert(!task28_b0::decode(bad, &s));
  bad = good;
  bad.replace(bad.find("right_q_rad: [0, 0, 0, 0, 0, 0, 0]"), 34,
              "right_q_rad: [0, 0, 0, 0, 0, 0]");
  assert(!task28_b0::decode(bad, &s));
  bad = good;
  bad.replace(bad.find("right_q_rad: [0, 0, 0, 0, 0, 0, 0]"), 34,
              "right_q_rad: [.nan, 0, 0, 0, 0, 0, 0]");
  assert(!task28_b0::decode(bad, &s));
  bad = good;
  bad.replace(bad.find("sample_finished_ns: 1001000000"), 30, "sample_finished_ns: 1100000000");
  assert(!task28_b0::decode(bad, &s));
  bad = good;
  bad.replace(bad.find("1.097104073"), 11, ".nan");
  assert(!task28_b0::decode(bad, &s));
  if (argc > 1) {
    std::ifstream stream(argv[1]);
    std::stringstream bytes;
    bytes << stream.rdbuf();
    task28_b0::Snapshot captured;
    assert(stream && task28_b0::decode(bytes.str(), &captured));
    assert(task28_b0::fresh(captured, captured.finished_ns + 1, captured.started_ns - 1));
  }
  std::cout << "B0 snapshot contract PASS\n";
}

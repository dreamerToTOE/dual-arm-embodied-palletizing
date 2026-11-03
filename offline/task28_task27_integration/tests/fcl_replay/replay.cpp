#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <sstream>
#include <urdf_parser/urdf_parser.h>
std::string read(const char *p) {
  std::ifstream f(p);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}
int main(int argc, char **argv) {
  if (argc != 3)
    return 1;
  auto urdf = urdf::parseURDF(read(argv[1]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read(argv[2])))
    return 1;
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  planning_scene::PlanningScene scene(model);
  moveit_msgs::msg::CollisionObject cube;
  cube.header.frame_id = model->getModelFrame();
  cube.id = "task26_cube_1";
  cube.operation = cube.ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {.12, .12, .12};
  cube.primitives.push_back(box);
  geometry_msgs::msg::Pose pose;
  pose.position.x = .997469687461853;
  pose.position.y = .06021100655198097;
  pose.position.z = .26007622480392456;
  pose.orientation.w = .9999978542327881;
  pose.orientation.x = -1.1307383829262108e-06;
  pose.orientation.y = .0006346335867419839;
  pose.orientation.z = .001938563771545887;
  cube.primitive_poses.push_back(pose);
  if (!scene.processCollisionObjectMsg(cube))
    return 2;
  // Task27 table and three walls, shifted by the measured 0.100 m rail advance.
  const auto add_box = [&](const char *id, double x, double y, double z,
                           double sx, double sy, double sz) {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = model->getModelFrame();
    object.id = id;
    object.operation = object.ADD;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {sx, sy, sz};
    object.primitives.push_back(primitive);
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = 1;
    object.primitive_poses.push_back(p);
    return scene.processCollisionObjectMsg(object);
  };
  if (!add_box("task26_table", .45, 0, .1, 1.5, .8, .2) ||
      !add_box("task26_box_wall_deep", 1.07, 0, .275, .02, .646, .15) ||
      !add_box("task26_box_wall_minus_y", .935, -.313, .275, .29, .02, .15) ||
      !add_box("task26_box_wall_plus_y", .935, .313, .275, .29, .02, .15))
    return 2;
  // Diagnostic run physics step 13845, synchronized Cube and joint readings.
  std::array<double, 7> l = {.9971434474,  -1.250707388, .0724271536,
                             -2.384800673, .0759442002,  1.136402845,
                             1.052032351};
  std::array<double, 7> actual = {.4852465391,  -1.001407027, -1.562951922,
                                  -2.064811230, -1.055983663, 1.821666360,
                                  -2.230067492};
  std::array<double, 7> nominal = {.4881732578,  -1.002127586, -1.563268433,
                                   -2.066474094, -1.056101657, 1.825020077,
                                   -2.222478036};
  bool nominal_reproduced = false, actual_clear = false;
  for (auto [name, r] :
       {std::pair("actual", actual), std::pair("nominal", nominal)}) {
    moveit::core::RobotState s(model);
    s.setToDefaultValues();
    for (int i = 0; i < 7; i++) {
      s.setVariablePosition("left_fr3_joint" + std::to_string(i + 1), l[i]);
      s.setVariablePosition("right_fr3_joint" + std::to_string(i + 1), r[i]);
    }
    s.update();
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 30;
    req.max_contacts_per_pair = 5;
    collision_detection::CollisionResult res;
    scene.checkCollision(req, res, s);
    std::cout << name << " collision=" << res.collision << " tcp="
              << s.getGlobalLinkTransform("right_fr3_side_suction_tcp")
                     .translation()
                     .transpose()
              << "\n";
    for (auto const &[pair, cts] : res.contacts) {
      for (auto const &c : cts) {
        std::cout << pair.first << " " << pair.second << " depth=" << c.depth
                  << "\n";
        if (std::string(name) == "nominal" &&
            pair.first == "right_fr3_side_suction" &&
            pair.second == "task26_cube_1" &&
            std::abs(c.depth - .00143391) < .00001)
          nominal_reproduced = true;
      }
    }
    if (std::string(name) == "actual")
      actual_clear =
          !res.collision &&
          std::abs(s.getGlobalLinkTransform("right_fr3_side_suction_tcp")
                       .translation()
                       .x() -
                   .9366104543) < .00001;
  }
  // A synthetic transition, not the missing lateral-assistance candidate.
  int first_collision = -1;
  for (int sample = 0; sample <= 100; ++sample) {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    for (int i = 0; i < 7; ++i) {
      state.setVariablePosition("left_fr3_joint" + std::to_string(i + 1), l[i]);
      state.setVariablePosition("right_fr3_joint" + std::to_string(i + 1),
                                actual[i] +
                                    (nominal[i] - actual[i]) * sample / 100.0);
    }
    state.update();
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    scene.checkCollision(request, result, state);
    if (result.collision && first_collision < 0)
      first_collision = sample;
  }
  std::cout << "full_scene synthetic_transition_first_collision_sample="
            << first_collision << "/100\n";
  if (first_collision <= 0)
    return 4;
  if (!nominal_reproduced || !actual_clear) {
    std::cerr << "FCL replay FAILED\n";
    return 3;
  }
  std::cout << "FCL replay PASS\n";
}

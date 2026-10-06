// [EXPERIMENTAL] 名义 rear+side 接触链无命令筛选，不等价于物理可行证明。
// 不构造 Arm、不发布 joint/suction/feed/rail，不改远程 Planning Scene/ACM。
#define TASK27_FIVE_CUBE 1
#define TASK01_CUBE04_PRECISION_INSERT 1
#define TASK01_DUAL_SUCTION_FIXTURE 1
#define main task01_unexecuted_controller_main
#include "task26_truck_box_push_in.cpp"
#undef main
#include <iomanip>
#include <filesystem>

namespace
{
Eigen::Isometry3d fixturePoseTransform(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z).normalized().toRotationMatrix();
  return transform;
}

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task01_dual_fixture_probe");
  const bool diagnose_released_state = node->declare_parameter<bool>("diagnose_released_state", false);
  const bool diagnose_side_high = node->declare_parameter<bool>("diagnose_side_high", false);
  const bool diagnose_rear_search = node->declare_parameter<bool>("diagnose_rear_search", false);
  const bool diagnose_fixture_chain = node->declare_parameter<bool>("diagnose_fixture_chain", false);
  const bool diagnose_handoff = node->declare_parameter<bool>("diagnose_handoff", false);
  node->declare_parameter<bool>("empty_handoff_audit_totg",diagnose_handoff);
  const auto diagnostic_placed_cubes = node->declare_parameter<std::vector<double>>("diagnostic_placed_cubes", std::vector<double>{});
  const auto diagnostic_left_q = node->declare_parameter<std::vector<double>>("diagnostic_left_q", std::vector<double>{});
  const auto diagnostic_right_q = node->declare_parameter<std::vector<double>>("diagnostic_right_q", std::vector<double>{});
  const auto diagnostic_cube = node->declare_parameter<std::vector<double>>("diagnostic_cube", std::vector<double>{});
  const auto diagnostic_index = node->declare_parameter<int>("diagnostic_cube_index", 2);
  // [EXPERIMENTAL] 只读检查绕杯面法向的腕部转角；不改变中心吸点/法向或执行器。
  const double side_roll_world_y_deg = node->declare_parameter<double>("side_roll_world_y_deg", 0.0);
  const double rear_roll_magnitude_deg = node->declare_parameter<double>("rear_roll_magnitude_deg", 0.0);
  node->declare_parameter<double>("dual_fixture_side_roll_world_y_deg", side_roll_world_y_deg);
  const auto wrist_audit_path = node->declare_parameter<std::string>("wrist_audit_path", "");
  if ((diagnose_released_state && diagnose_side_high) || (diagnose_rear_search && !diagnose_side_high) ||
      (diagnose_fixture_chain && !diagnose_rear_search) ||
      (diagnose_handoff && (diagnose_side_high || diagnose_released_state || diagnose_rear_search || diagnose_fixture_chain)) ||
      !fr3_dual_palletize::validSideFixtureRoll(side_roll_world_y_deg) ||
      !fr3_dual_palletize::validRearFixtureRoll(rear_roll_magnitude_deg))
    { rclcpp::shutdown(); return 1; }
  std::ofstream audit_file;
  if (!wrist_audit_path.empty())
  {
    // 不覆盖已有记录；机器可读数据不能与多线程rosout/stdout日志混写。
    if (std::filesystem::exists(wrist_audit_path)) { rclcpp::shutdown(); return 1; }
    audit_file.open(wrist_audit_path);
    if (!audit_file) { rclcpp::shutdown(); return 1; }
  }
  if (!copyRobotDescriptions(node) || !copyFixtureKinematics(node) || !copyFixtureEmptyPlannerConfig(node))
    { rclcpp::shutdown(); return 1; }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&] { executor.spin(); });
  bool passed = true;
  try
  {
    moveit::planning_interface::MoveGroupInterface lg(node, "left_arm"), rg(node, "right_arm");
    const auto model = lg.getRobotModel();
    const auto* lj = model->getJointModelGroup("left_arm");
    const auto* rj = model->getJointModelGroup("right_arm");
    const std::string le = "left_fr3_side_suction_tcp", re = "right_fr3_side_suction_tcp";
    lg.setEndEffectorLink(le); rg.setEndEffectorLink(re);
    lg.setPoseReferenceFrame("world"); rg.setPoseReferenceFrame("world");
    g_world_shift_x = .100;  // 原推入阶段的双轨站位，不移动任何物理基座。
    if (diagnose_handoff)
    {
      // 实测释放后同一步14关节/落位Cube，无Arm，不写远程世界或ACM。
      if (diagnostic_index<0 || diagnostic_index>=4 ||
          diagnostic_placed_cubes.size()!=7*static_cast<std::size_t>(diagnostic_index+1) ||
          !fr3_dual_palletize::validMeasuredJointSeed(diagnostic_left_q,lj->getVariableCount()) ||
          !fr3_dual_palletize::validMeasuredJointSeed(diagnostic_right_q,rj->getVariableCount()))
        throw std::runtime_error("Invalid exact handoff snapshot");
      moveit::core::RobotState state(model); state.setToDefaultValues();
      state.setJointGroupPositions(lj,diagnostic_left_q);
      state.setJointGroupPositions(rj,diagnostic_right_q); state.update();
      moveit::planning_interface::PlanningSceneInterface remote;
      const auto original_world=remote.getObjects();
      auto world=staticWorld(remote);
      world.erase(std::remove_if(world.begin(),world.end(),[](const auto& o) {
        return o.id.rfind("task26_cube_",0)==0; }),world.end());
      for (int index=0;index<=diagnostic_index;++index)
      {
        const auto offset=7*index;
        geometry_msgs::msg::Pose p;
        p.position.x=diagnostic_placed_cubes[offset]; p.position.y=diagnostic_placed_cubes[offset+1];
        p.position.z=diagnostic_placed_cubes[offset+2]; p.orientation.x=diagnostic_placed_cubes[offset+3];
        p.orientation.y=diagnostic_placed_cubes[offset+4]; p.orientation.z=diagnostic_placed_cubes[offset+5];
        p.orientation.w=diagnostic_placed_cubes[offset+6];
        if (!fr3_dual_palletize::validFixturePose({p.position.x,p.position.y,p.position.z,
            p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w}))
          throw std::runtime_error("Invalid diagnostic placed Cube pose");
        auto object=cubeObject("task26_cube_"+std::to_string(index+1),p);
        if (index==diagnostic_index) for (auto& dim:object.primitives.front().dimensions) dim+=.006;
        world.push_back(std::move(object));
      }
      // 与执行器的槽位lambda相同：复用原A/B常量，不改供料几何。
      const auto source=(diagnostic_index+1)%2==0 ? worldPose(kSlotAx,kSlotAy,kBottomZ) :
        worldPose(kSlotBx,kSlotBy,kBottomZ);
      world.push_back(cubeObject("task26_cube_"+std::to_string(diagnostic_index+2),source));
      const double z=source.position.z+kSideContactCommandZOffset+kLiftHeight+.030;
      const auto lt=sidePose(source.position.x,source.position.y-kCubeHalf-kSideContactCommandGap-kPreContactOffsetY,z,true);
      const auto rt=sidePose(source.position.x,source.position.y+kCubeHalf+kSideContactCommandGap+kRightPreContactOffsetY,z,false);
      trajectory_msgs::msg::JointTrajectory lpath,rpath;
      const bool closed_rejected=!planFixtureEmptyHandoff(node,lg,rg,state,true,false,lt,rt,world,
        "HANDOFF_CLOSED_NEGATIVE",&lpath,&rpath);
      auto obstruction=lt; const auto physical=state.getGlobalLinkTransform(le).translation();
      obstruction.position.x=physical.x()+g_world_shift_x;
      obstruction.position.y=physical.y(); obstruction.position.z=physical.z();
      obstruction.orientation.x=obstruction.orientation.y=obstruction.orientation.z=0.; obstruction.orientation.w=1.;
      auto blocked=world; blocked.push_back(cubeObject("HANDOFF_START_OBSTRUCTION",obstruction));
      const bool collision_rejected=!planFixtureEmptyHandoff(node,lg,rg,state,false,false,lt,rt,blocked,
        "HANDOFF_COLLISION_NEGATIVE",&lpath,&rpath);
      const bool safe=planFixtureEmptyHandoff(node,lg,rg,state,false,false,lt,rt,world,
        "EXACT_CUBE03_HANDOFF",&lpath,&rpath);
      const auto final_world=remote.getObjects();
      bool unchanged=original_world.size()==final_world.size();
      for (const auto& [id,object]:original_world)
      {
        const auto found=final_world.find(id);
        unchanged=unchanged && found!=final_world.end();
        if (found!=final_world.end()) unchanged=unchanged &&
          moveit_msgs::msg::to_yaml(object)==moveit_msgs::msg::to_yaml(found->second);
      }
      RCLCPP_INFO(node->get_logger(),"HANDOFF_REPLAY safe_pair=%s CLOSED_rejected=%s collision_start_rejected=%s remote_world_unchanged=%s; NOT_EXECUTED.",
        safe ? "PASS":"FAIL",closed_rejected ? "PASS":"FAIL",collision_rejected ? "PASS":"FAIL",unchanged ? "PASS":"FAIL");
      passed=safe && closed_rejected && collision_rejected && unchanged;
    }
    else if (diagnose_side_high)
    {
      // [ENGINEERING] 实测后抓固定、侧臂不同IK种子的只读终点诊断。
      // 不发任何机器人命令；当前Cube仍在本地FCL，不扩大ACM。
      if (diagnostic_index < 0 || diagnostic_index >= 3 || diagnostic_cube.size()!=7 ||
          !fr3_dual_palletize::validMeasuredJointSeed(diagnostic_left_q, lj->getVariableCount()) ||
          !fr3_dual_palletize::validMeasuredJointSeed(diagnostic_right_q, rj->getVariableCount()) ||
          !fr3_dual_palletize::validFixturePose({diagnostic_cube[0],diagnostic_cube[1],
            diagnostic_cube[2],diagnostic_cube[3],diagnostic_cube[4],diagnostic_cube[5],diagnostic_cube[6]}))
        throw std::runtime_error("Invalid exact diagnostic joint/Cube snapshot");
      geometry_msgs::msg::Pose cube;
      cube.position.x=diagnostic_cube[0]; cube.position.y=diagnostic_cube[1]; cube.position.z=diagnostic_cube[2];
      cube.orientation.x=diagnostic_cube[3]; cube.orientation.y=diagnostic_cube[4];
      cube.orientation.z=diagnostic_cube[5]; cube.orientation.w=diagnostic_cube[6];
      moveit::core::RobotState original(model);
      original.setToDefaultValues();
      original.setJointGroupPositions(lj, diagnostic_left_q);
      original.setJointGroupPositions(rj, diagnostic_right_q); original.update();
      if (!original.satisfiesBounds()) throw std::runtime_error("Diagnostic seed outside original bounds");
      moveit::planning_interface::PlanningSceneInterface remote_scene;
      auto world=staticWorld(remote_scene);
      const auto id="task26_cube_"+std::to_string(diagnostic_index+1);
      world.erase(std::remove_if(world.begin(),world.end(),[&](const auto& o) { return o.id==id; }),world.end());
      world.push_back(cubeObject(id,cube));
      planning_scene::PlanningScene collision_scene(model);
      for (const auto& object:world) collision_scene.processCollisionObjectMsg(object);
      const bool side_left=diagnostic_index!=1;
      const auto* own=side_left ? lj:rj;
      const auto& eef=side_left ? le:re;
      const auto goal=dualSideFixturePose(cube.position.x,
        cube.position.y-(side_left ? 1.:-1.)*(kCubeHalf+kSideContactCommandGap),
        cube.position.z+kRegraspLiftHeight,side_left,side_roll_world_y_deg);
      RCLCPP_INFO(node->get_logger(),"SIDE_HIGH_DIAGNOSTIC index=%d goal_model=(%.9f,%.9f,%.9f) fixed_rear=%s; NOT_EXECUTED.",
        static_cast<int>(diagnostic_index),goal.position.x,goal.position.y,goal.position.z,side_left ? "right":"left");
      collision_detection::CollisionRequest request;
      request.contacts=true; request.max_contacts=100; request.max_contacts_per_pair=1;
      std::map<std::pair<std::string,std::string>,int> pairs;
      int solved=0,free=0;
      for (int sample=0;sample<60;++sample)
      {
        moveit::core::RobotState state(original);
        if (sample)
        {
          auto q=side_left ? diagnostic_left_q:diagnostic_right_q;
          const auto& names=own->getVariableNames();
          for (std::size_t j=0;j<q.size();++j)
          {
            const auto& bound=model->getVariableBounds(names[j]);
            const double fraction=.01+.98*((sample*17+static_cast<int>(j)*7)%101)/100.;
            q[j]=bound.min_position_+fraction*(bound.max_position_-bound.min_position_);
          }
          state.setJointGroupPositions(own,q); state.update();
        }
        if (!state.setFromIK(own,fixturePoseTransform(goal),eef,.05)) continue;
        state.update();
        if (!state.satisfiesBounds()) continue;
        ++solved;
        collision_detection::CollisionResult result;
        collision_scene.checkCollision(request,result,state);
        if (!result.collision)
        {
          ++free; std::vector<double> q; state.copyJointGroupPositions(own,q);
          RCLCPP_INFO(node->get_logger(),"SIDE_HIGH_DIAGNOSTIC FREE sample=%d q=%s.",sample,formatJointPositions(q).c_str());
        }
        for (const auto& contact:result.contacts) ++pairs[contact.first];
      }
      for (const auto& pair:pairs)
        RCLCPP_INFO(node->get_logger(),"SIDE_HIGH_DIAGNOSTIC COLLISION %s <-> %s candidates=%d.",
          pair.first.first.c_str(),pair.first.second.c_str(),pair.second);
      RCLCPP_INFO(node->get_logger(),"SIDE_HIGH_DIAGNOSTIC samples=60 IK_solved=%d collision_free=%d; zero free is NOT an infeasibility proof.",solved,free);
      passed=free>0;
      if (diagnose_rear_search)
      {
        const auto check_chain=[&](const moveit::core::RobotState& state,const std::string& label)
        {
          std::vector<double> lq,rq;
          state.copyJointGroupPositions(lj,lq); state.copyJointGroupPositions(rj,rq);
          trajectory_msgs::msg::JointTrajectory ls,rs,lx,rx,le_path,re_path;
          ls.joint_names=lj->getVariableNames(); rs.joint_names=rj->getVariableNames();
          ls=holdTrajectory(ls,lq,.1); rs=holdTrajectory(rs,rq,.1);
          return preflightDualFixtureRearCandidate(node,lg,rg,ls,rs,cube,kTasks.at(diagnostic_index),
            world,label,&lx,&rx,&le_path,&re_path);
        };
        if (diagnose_fixture_chain && check_chain(original,"RECORDED_BLOCKED_REAR"))
          throw std::runtime_error("Recorded blocked rear unexpectedly passed coupled gate");
        // [EXPERIMENTAL] 保持同一rear TCP位姿，寻找空载重抓可选冗余构型。
        // 仅本地RobotState假设，不移动实物，不在CLOSED时变更任何关节。
        const auto* rear=side_left ? rj:lj;
        const auto& rear_eef=side_left ? re:le;
        const auto rear_goal=original.getGlobalLinkTransform(rear_eef);
        int paired_free=0,rear_solved=0;
        for (int attempt=0;attempt<40 && paired_free<3;++attempt)
        {
          moveit::core::RobotState candidate(original);
          auto q=side_left ? diagnostic_right_q:diagnostic_left_q;
          const auto& names=rear->getVariableNames();
          for (std::size_t j=0;j<q.size();++j)
          {
            const auto& bound=model->getVariableBounds(names[j]);
            const double fraction=.01+.98*((attempt*17+static_cast<int>(j)*7)%101)/100.;
            q[j]=bound.min_position_+fraction*(bound.max_position_-bound.min_position_);
          }
          candidate.setJointGroupPositions(rear,q); candidate.update();
          if (!candidate.setFromIK(rear,rear_goal,rear_eef,.05)) continue;
          candidate.update();
          if (!candidate.satisfiesBounds()) continue;
          ++rear_solved;
          collision_detection::CollisionResult rear_at_park;
          collision_scene.checkCollision(request,rear_at_park,candidate);
          if (rear_at_park.collision) continue;
          auto at_high=candidate;
          if (!at_high.setFromIK(own,fixturePoseTransform(goal),eef,.05)) continue;
          at_high.update();
          collision_detection::CollisionResult high_collision;
          collision_scene.checkCollision(request,high_collision,at_high);
          if (high_collision.collision || !at_high.satisfiesBounds()) continue;
          auto at_contact=at_high;
          auto contact=goal; contact.position.z=cube.position.z;
          if (!at_contact.setFromIK(own,fixturePoseTransform(contact),eef,.05)) continue;
          at_contact.update();
          collision_detection::CollisionResult contact_collision;
          collision_scene.checkCollision(request,contact_collision,at_contact);
          if (contact_collision.collision || !at_contact.satisfiesBounds()) continue;
          if (diagnose_fixture_chain && !check_chain(candidate,"ALTERNATIVE_REAR attempt="+std::to_string(attempt))) continue;
          ++paired_free;
          candidate.copyJointGroupPositions(rear,q);
          std::ostringstream exact; exact<<std::setprecision(17)<<'[';
          for (std::size_t j=0;j<q.size();++j) exact<<(j ? ",":"")<<q[j];
          exact<<']';
          RCLCPP_INFO(node->get_logger(),"REAR_REDUNDANCY_DIAGNOSTIC FREE attempt=%d rear_q=%s; same rear TCP, side HIGH+CONTACT full FCL; NOT_EXECUTED.",attempt,exact.str().c_str());
        }
        RCLCPP_INFO(node->get_logger(),"REAR_REDUNDANCY_DIAGNOSTIC rear_solved=%d paired_free=%d/3 within40seeds; %s, no HIGH RRT connection or physics proof.",
          rear_solved,paired_free,diagnose_fixture_chain ? "continuous CONTACT/X/Y/short exit checked":"endpoint-only");
        passed=paired_free>0;
      }
    }
    else if (diagnose_released_state)
    {
      // [EXPERIMENTAL] 当前释放后静态姿态；不能倒推为互锁触发那一物理步。
      // 只在本地检查桌/墙/其余Cube与机器人干涉，忽略当前合法接触Cube01。
      const auto state = lg.getCurrentState(2.0);
      moveit::planning_interface::PlanningSceneInterface scene;
      auto world = staticWorld(scene);
      world.erase(std::remove_if(world.begin(), world.end(), [](const auto& object) {
        return object.id == "task26_cube_1";
      }), world.end());
      if (!state) throw std::runtime_error("No current joint-state snapshot");
      std::vector<double> lq, rq;
      state->copyJointGroupPositions(lj, lq); state->copyJointGroupPositions(rj, rq);
      RCLCPP_INFO(node->get_logger(), "RELEASED_STATE_ONLY q left=%s right=%s.",
        formatJointPositions(lq).c_str(), formatJointPositions(rq).c_str());
      trajectory_msgs::msg::JointTrajectory lp, rp;
      lp.joint_names = lj->getVariableNames(); rp.joint_names = rj->getVariableNames();
      lp = holdTrajectory(lp, lq, .1); rp = holdTrajectory(rp, rq, .1);
      passed = validateSync(node, model, world, lp, rp, "RELEASED_STATE_ONLY current measured FR3");
    }
    else
    for (std::size_t index = 0; index < 3; ++index)
    {
      const auto& task = kTasks.at(index);
      const bool side_left = index != 1;
      const double direction = side_left ? 1.0 : -1.0;
      const auto initial = task.pre_push;
      auto rear_pose = dualRearFixturePose(initial.position.x - kPushCupOffsetX,
        initial.position.y, initial.position.z, index, rear_roll_magnitude_deg);
      auto side_pose = dualSideFixturePose(initial.position.x,
        initial.position.y - direction * (kCubeHalf + kSideContactCommandGap),
        initial.position.z, side_left, side_roll_world_y_deg);
      auto world = boxWallObjects(); world.push_back(tableObject());
      for (std::size_t previous = 0; previous < index; ++previous)
        world.push_back(cubeObject("task26_cube_" + std::to_string(previous + 1), kTasks[previous].cell));
      auto contact_world = world;
      contact_world.push_back(cubeObject("task26_cube_" + std::to_string(index + 1), initial));
      planning_scene::PlanningScene collision_scene(model);
      for (const auto& object : contact_world) collision_scene.processCollisionObjectMsg(object);
      bool found = false;
      for (int attempt = 0; attempt < 40 && rclcpp::ok(); ++attempt)
      {
        moveit::core::RobotState state(model);
        state.setToDefaultValues();
        // 显式不同 seed（不修改机器人实测状态）；不能只给两个对置机械臂
        // 同一个负 joint1/home wrist seed，造成 nominal IK 分支覆盖不足。
        std::vector<double> lseed{1.1, .1, .2, -2.3, .1, 2.1, -.8};
        std::vector<double> rseed{-1.1, .1, -.2, -2.3, -.1, 2.1, -.8};
        lseed[0] += .3 * (attempt % 5 - 2);
        rseed[0] -= .3 * (attempt % 5 - 2);
        lseed[2] += .6 * ((attempt / 5) % 4 - 2);
        rseed[2] -= .6 * ((attempt / 5) % 4 - 2);
        lseed[6] += 1.0 * (attempt / 20); rseed[6] += 1.0 * (attempt / 20);
        state.setJointGroupPositions(lj, lseed); state.setJointGroupPositions(rj, rseed);
        state.update();
        if (!state.setFromIK(side_left ? rj : lj, fixturePoseTransform(rear_pose),
              side_left ? re : le, .05) ||
            !state.setFromIK(side_left ? lj : rj, fixturePoseTransform(side_pose),
              side_left ? le : re, .05)) continue;
        state.update();
        if (!state.satisfiesBounds() || collision_scene.isStateColliding(state)) continue;
        std::vector<double> lq, rq;
        state.copyJointGroupPositions(lj, lq); state.copyJointGroupPositions(rj, rq);
        trajectory_msgs::msg::JointTrajectory ls, rs, lx, rx, ly, ry, lo, ro;
        ls.joint_names = lj->getVariableNames(); rs.joint_names = rj->getVariableNames();
        ls = holdTrajectory(ls, lq, .1); rs = holdTrajectory(rs, rq, .1);
        const auto label = std::string(task.id) + " NO_COMMAND";
        if (!validateSync(node, model, contact_world, ls, rs, label + " CONTACT_WITH_CUBE") ||
            !planDualFixtureTranslation(node, lg, rg, ls, rs,
              task.cell.position.x - initial.position.x, 0.0, world, label + " X", &lx, &rx) ||
            !planDualFixtureTranslation(node, lg, rg, lx, rx, 0.0,
              dualFixturePressY(task, initial.position.y, direction) - initial.position.y,
              world, label + " Y", &ly, &ry) ||
            !planDualFixtureReleaseExit(node, model, ly, ry, side_left, world,
              label + " RELEASE_EXIT", &lo, &ro)) continue;
        RCLCPP_INFO(node->get_logger(),
          "%s NO_COMMAND_CONTACT_X_Y PASS: seed_attempt=%d; current Cube included at contact, intentional moving Cube excluded during wall push. NOT_EXECUTED.",
          task.id, attempt + 1);
        // [ENGINEERING] 导出选中轨迹的全部关节节点/腕部FK，供原Isaac网格离线复核。
        // 不包含任何 Arm、joint/suction/feed/rail publisher 或远程场景写入。
        const auto audit = [&](const char* stage,
          const trajectory_msgs::msg::JointTrajectory& lp,
          const trajectory_msgs::msg::JointTrajectory& rp)
        {
          if (!audit_file.is_open()) return;
          for (std::size_t point = 0; point < lp.points.size(); ++point)
          {
            state.setVariablePositions(lp.joint_names, lp.points.at(point).positions);
            state.setVariablePositions(rp.joint_names, rp.points.at(point).positions); state.update();
            for (const auto& arm : {std::string("left"), std::string("right")})
            {
              const auto& transform = state.getGlobalLinkTransform(arm + "_fr3_link7");
              const Eigen::Quaterniond q(transform.rotation());
              audit_file << std::setprecision(17) << "HELD_HULL_AUDIT " << task.id << ' '
                << stage << ' ' << point << ' ' << arm << ' ' << transform.translation().x()+g_world_shift_x
                << ' ' << transform.translation().y() << ' ' << transform.translation().z()
                << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
            }
          }
          audit_file.flush();
          if (!audit_file) throw std::runtime_error("Wrist audit write failed");
        };
        audit("CONTACT", ls, rs); audit("X", lx, rx); audit("Y", ly, ry);
        found = true; break;
      }
      if (!found)
      {
        RCLCPP_ERROR(node->get_logger(), "%s NO_COMMAND_CONTACT_X_Y FAIL within 40 seeds; not an infeasibility proof.", task.id);
        passed = false;
      }
    }
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "NO_COMMAND probe exception: %s", error.what());
    passed = false;
  }
  RCLCPP_INFO(node->get_logger(), "NO_COMMAND %s %s; no physical benchmark PASS implied.",
    diagnose_handoff ? "exact OPEN empty handoff replay" : (diagnose_side_high ? "measured side-high goal diagnostic" :
      (diagnose_released_state ? "released static-state diagnostic" : "three nominal contact chains")),
    passed ? "PASS" : "FAIL");
  executor.cancel(); spin.join(); rclcpp::shutdown();
  return passed ? 0 : 1;
}

// Reuse the unchanged Task28 scene, collision model, IK and acceptance checks.
#define main task28_precontact_original_main
#include "../../offline/task28_stage_b_contact/precontact.cpp"
#undef main
#include <fcl/fcl.h>
#include <chrono>
#include <iomanip>

using FclObject = fcl::CollisionObjectd;
std::shared_ptr<FclObject> fclBox(const Eigen::Vector3d& xyz, const Eigen::Vector3d& size,
                                  const Eigen::Quaterniond& rotation = Eigen::Quaterniond::Identity())
{
  fcl::Transform3d t = fcl::Transform3d::Identity();
  t.translation() = xyz; t.linear() = rotation.toRotationMatrix();
  return std::make_shared<FclObject>(std::make_shared<fcl::Boxd>(size.x(), size.y(), size.z()), t);
}
bool cubeWorldClear(const Eigen::Isometry3d& object)
{
  const auto cube = fclBox(object.translation(), {.12,.12,.12}, Eigen::Quaterniond(object.rotation()));
  const std::vector<std::shared_ptr<FclObject>> world = {
    fclBox({.45,0,.1},{1.5,.8,.2}), fclBox({1.07,0,.275},{.02,.646,.15}),
    fclBox({.935,-.313,.275},{.29,.02,.15}), fclBox({.935,.313,.275},{.29,.02,.15})};
  for (const auto& obstacle : world) {
    fcl::CollisionRequestd req; fcl::CollisionResultd res;
    fcl::collide(cube.get(), obstacle.get(), req, res);
    if (res.isCollision()) return false;
  }
  return true;
}
geometry_msgs::msg::Pose message(const Eigen::Isometry3d& target)
{
  geometry_msgs::msg::Pose p;
  p.position.x=target.translation().x(); p.position.y=target.translation().y(); p.position.z=target.translation().z();
  const Eigen::Quaterniond q(target.rotation());
  p.orientation.w=q.w(); p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z();
  return p;
}
Eigen::Isometry3d interpolatePose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double u)
{
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  t.translation()=(1-u)*a.translation()+u*b.translation();
  t.linear()=Eigen::Quaterniond(a.rotation()).slerp(u,Eigen::Quaterniond(b.rotation())).toRotationMatrix();
  return t;
}
#ifndef STAGE_B_YAW_LIBRARY
int main(int argc, char** argv)
{
  if (argc != 8) { std::cerr << "usage: yaw_planner URDF SRDF FIXTURE PRECONTACT_CSV OUTPUT_CSV DESIRED_YAW_DEG direct|correct\n"; return 2; }
  const auto started=std::chrono::steady_clock::now();
  int ik_attempts=0, samples=0; double joint_length=0;
  std::string phase="initial";
  try {
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("stage_b_yaw_offline");
    auto urdf=urdf::parseURDF(readFile(argv[1]));
    auto srdf=std::make_shared<srdf::Model>();
    require(urdf && srdf->initString(*urdf,readFile(argv[2])), "invalid robot model");
    auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    const Fixture f=fixtureFrom(YAML::LoadFile(argv[3]));
    const double desired_yaw=std::stod(argv[6])*M_PI/180.;
    require(std::isfinite(desired_yaw),"non-finite desired yaw");
    const std::string mode=argv[7]; require(mode=="direct" || mode=="correct","invalid mode");
    const auto ln=names("left"), rn=names("right");
    auto scene=std::make_shared<planning_scene::PlanningScene>(model);
    addWorld(*scene,f,true);
    lma_kinematics_plugin::LMAKinematicsPlugin ik;
    require(ik.initialize(node,*model,"left_arm","world",{"left_fr3_side_suction_tcp"},.005),"IK init failed");
    require(ik.getJointNames()==ln,"IK joint order mismatch");
    std::ifstream csv(argv[4]); require(bool(csv),"precontact csv missing");
    std::string line; std::getline(csv,line);
    std::vector<Joints> precontact;
    while (std::getline(csv,line)) {
      std::stringstream ss(line); std::string field;
      require(bool(std::getline(ss,field,',')) && bool(std::getline(ss,field,',')),"bad precontact csv");
      Joints q{}; for (double& value : q) { require(bool(std::getline(ss,field,',')),"bad precontact joints"); value=std::stod(field); }
      precontact.push_back(q);
    }
    std::size_t checked=0;
    require(validPath(*scene,model,ln,rn,precontact,f.right,&checked) && checked==precontact.size(),"precontact revalidation failed");
    for (std::size_t i=0;i<7;++i) require(std::abs(precontact.front()[i]-f.left[i])<1e-8,"precontact start mismatch");
    Eigen::Isometry3d object=Eigen::Isometry3d::Identity(); object.translation()=f.cube; object.linear()=f.orientation.toRotationMatrix();
    require(cubeWorldClear(object),"initial Cube/world collision");
    auto selftest=object; selftest.translation().x()=1.02;
    require(!cubeWorldClear(selftest),"FCL deep-wall negative control failed");
    Joints current=precontact.back();
    auto fk=[&](const Joints& q) {
      moveit::core::RobotState s(model); s.setToDefaultValues();
      s.setVariablePositions(ln,std::vector<double>(q.begin(),q.end()));
      s.setVariablePositions(rn,std::vector<double>(f.right.begin(),f.right.end())); s.update();
      return Eigen::Isometry3d(s.getGlobalLinkTransform("left_fr3_side_suction_tcp"));
    };
    auto tcp=fk(current);
    const double initial_gap=-(f.orientation.inverse()*(tcp.translation()-f.cube)).y()-.06;
    require(std::abs(initial_gap-.003)<.0001,"unexpected precontact gap");
    std::FILE* out=std::fopen(argv[5],"wx"); require(out,"output exists or cannot be created");
    std::fprintf(out,"phase,cube_x,cube_y,cube_z,cube_qw,cube_qx,cube_qy,cube_qz,q1,q2,q3,q4,q5,q6,q7\n");
    auto record=[&](const Eigen::Isometry3d& o,const Joints& q) {
      Eigen::Quaterniond rot(o.rotation());
      std::fprintf(out,"%s,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g",phase.c_str(),o.translation().x()+.1,o.translation().y(),o.translation().z(),rot.w(),rot.x(),rot.y(),rot.z());
      for(double x:q) std::fprintf(out,",%.17g",x); std::fprintf(out,"\n"); ++samples;
    };
    Eigen::Isometry3d object_to_tcp=Eigen::Isometry3d::Identity(); bool rigid=false;
    auto step=[&](const Eigen::Isometry3d& target_object,const Eigen::Isometry3d& target_tcp) {
      ++ik_attempts; std::vector<double> sol; moveit_msgs::msg::MoveItErrorCodes err;
      require(ik.getPositionIK(message(target_tcp),std::vector<double>(current.begin(),current.end()),sol,err) && sol.size()==7,"IK failed");
      Joints next{}; std::copy(sol.begin(),sol.end(),next.begin());
      double delta=0; for(std::size_t j=0;j<7;++j) delta=std::max(delta,std::abs(next[j]-current[j]));
      int n=std::max(1,static_cast<int>(std::ceil(delta/.005)));
      const auto from_tcp=fk(current);
      for(int i=1;i<=n;++i) {
        const double u=double(i)/n;
        auto o=interpolatePose(object,target_object,u);
        auto requested=rigid ? Eigen::Isometry3d(o*object_to_tcp) : interpolatePose(from_tcp,target_tcp,u);
        Joints q{}; for(std::size_t j=0;j<7;++j) q[j]=current[j]+u*(next[j]-current[j]);
        require(cubeWorldClear(o),"Cube/environment FCL collision");
        addBox(*scene,kWorldIds[0],o.translation(),{.12,.12,.12},Eigen::Quaterniond(o.rotation()));
        if (!validState(*scene,model,ln,rn,q,f.right)) {
          moveit::core::RobotState debug(model); debug.setToDefaultValues();
          debug.setVariablePositions(ln,std::vector<double>(q.begin(),q.end()));
          debug.setVariablePositions(rn,std::vector<double>(f.right.begin(),f.right.end())); debug.update();
          collision_detection::CollisionRequest req; req.contacts=true; req.max_contacts=30;
          collision_detection::CollisionResult result; scene->checkCollision(req,result,debug);
          for (const auto& [pair, contacts]:result.contacts)
            std::cerr << "ROBOT_COLLISION " << pair.first << " <-> " << pair.second << " count=" << contacts.size() << "\n";
          require(false,"robot/world FCL collision or joint bound");
        }
        const auto actual=fk(q);
        require((actual.translation()-requested.translation()).norm()<.0001 &&
                Eigen::Quaterniond(actual.rotation()).angularDistance(Eigen::Quaterniond(requested.rotation()))<.002,
                "interpolated TCP closure/path deviation");
        record(o,q);
      }
      double d2=0; for(std::size_t j=0;j<7;++j) d2+=(next[j]-current[j])*(next[j]-current[j]);
      joint_length+=std::sqrt(d2); current=next; object=target_object;
    };
    phase="approach_1mm";
    for(int i=1;i<=4;++i) { auto next=tcp; next.translation()+=f.orientation*Eigen::Vector3d(0,.002*i/4,0); step(object,next); }
    tcp=fk(current);
    const double attach_gap=-(f.orientation.inverse()*(tcp.translation()-f.cube)).y()-.06;
    require(std::abs(attach_gap-.001)<.0001,"grasp gap mismatch");
    object_to_tcp=object.inverse()*tcp; rigid=true;
    const auto initial_object=object;
    if(mode=="correct") {
    phase="lift";
    for(int i=1;i<=40;++i) { auto next=initial_object; next.translation().z()+=.04*i/40.; step(next,next*object_to_tcp); }
    auto raised=object;
    phase="yaw";
    Eigen::Isometry3d aligned=raised; aligned.linear()=Eigen::AngleAxisd(desired_yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    double angle=Eigen::Quaterniond(raised.rotation()).angularDistance(Eigen::Quaterniond(aligned.rotation()));
    int rotations=std::max(1,static_cast<int>(std::ceil(angle/(.5*M_PI/180.))));
    for(int i=1;i<=rotations;++i) { auto next=interpolatePose(raised,aligned,double(i)/rotations); step(next,next*object_to_tcp); }
    phase="place";
    for(int i=1;i<=40;++i) { auto next=aligned; next.translation().z()-=.04*i/40.; step(next,next*object_to_tcp); }
    }
    const auto placed=object;
    phase="insert";
    for(int i=1;i<=17;++i) { auto next=placed; next.translation().x()+=.017*i/17.; step(next,next*object_to_tcp); }
    phase="retreat"; rigid=false;
    tcp=fk(current);
    for(int i=1;i<=6;++i) { auto next=tcp; next.translation().y()-=.003*i/6.; step(object,next); }
    tcp=fk(current);
    for(int i=1;i<=40;++i) { auto next=tcp; next.translation().z()+=.2*i/40.; step(object,next); }
    require(std::fclose(out)==0,"output write failed");
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    std::cout << "YAW_CANDIDATE_PASS assumed_virtual_1mm_attachment=1 samples=" << samples << " ik_attempts=" << ik_attempts << " precontact_samples=" << checked << " joint_length_rad=" << joint_length << " seconds=" << seconds << "\n";
    rclcpp::shutdown(); return 0;
  } catch(const std::exception& e) {
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    std::cerr << "YAW_CANDIDATE_REJECT phase=" << phase << " reason=" << e.what() << " samples=" << samples << " ik_attempts=" << ik_attempts << " joint_length_rad=" << joint_length << " seconds=" << seconds << "\n";
    if(rclcpp::ok()) rclcpp::shutdown(); return 1;
  }
}

#endif

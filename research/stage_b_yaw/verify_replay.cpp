#define STAGE_B_YAW_LIBRARY
#include "yaw_planner.cpp"
#include <set>
int main(int argc,char** argv)
{
  if(argc!=7) { std::cerr << "usage: verify_replay URDF SRDF FIXTURE PRECONTACT CSV direct|correct\n"; return 2; }
  try {
    auto urdf=urdf::parseURDF(readFile(argv[1])); auto srdf=std::make_shared<srdf::Model>();
    require(urdf && srdf->initString(*urdf,readFile(argv[2])),"invalid model");
    auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    auto f=fixtureFrom(YAML::LoadFile(argv[3])); auto ln=names("left"),rn=names("right");
    auto scene=std::make_shared<planning_scene::PlanningScene>(model); addWorld(*scene,f,true);
    std::ifstream inpre(argv[4]); require(bool(inpre),"missing precontact"); std::string line; std::getline(inpre,line);
    std::vector<Joints> pre;
    while(std::getline(inpre,line)) { std::stringstream ss(line); std::string field; std::getline(ss,field,','); std::getline(ss,field,','); Joints q{}; for(double& v:q) { require(bool(std::getline(ss,field,',')),"bad precontact"); v=std::stod(field); } pre.push_back(q); }
    std::size_t nchecked=0; require(validPath(*scene,model,ln,rn,pre,f.right,&nchecked),"precontact invalid");
    Joints previous=pre.back(); Eigen::Isometry3d previous_object=Eigen::Isometry3d::Identity(); previous_object.translation()=f.cube; previous_object.linear()=f.orientation.toRotationMatrix();
    Eigen::Isometry3d grasp=Eigen::Isometry3d::Identity(); bool has_grasp=false; double length=0,max_pos=0,max_ang=0,max_z=previous_object.translation().z();
    std::set<std::string> phases; int count=0;
    std::ifstream csv(argv[5]); require(bool(csv),"missing path"); std::getline(csv,line);
    while(std::getline(csv,line)) {
      std::stringstream ss(line); std::string phase,field; require(bool(std::getline(ss,phase,',')),"bad path phase");
      std::array<double,14> v{}; for(double& value:v) { require(bool(std::getline(ss,field,',')),"bad path row"); value=std::stod(field); require(std::isfinite(value),"nonfinite path"); }
      Eigen::Quaterniond rot(v[3],v[4],v[5],v[6]); require(std::abs(rot.norm()-1)<1e-8,"bad object quaternion");
      Eigen::Isometry3d object=Eigen::Isometry3d::Identity(); object.translation()=Eigen::Vector3d(v[0]-.1,v[1],v[2]); object.linear()=rot.toRotationMatrix();
      Joints q{}; double d2=0; for(int j=0;j<7;++j) { q[j]=v[7+j]; double d=q[j]-previous[j]; require(std::abs(d)<=.005000001,"joint step exceeded"); d2+=d*d; }
      length+=std::sqrt(d2); require((object.translation()-previous_object.translation()).norm()<=.00100001,"object step exceeded");
      require(Eigen::Quaterniond(previous_object.rotation()).angularDistance(rot)<=.5*M_PI/180.+1e-8,"rotation step exceeded");
      require(cubeWorldClear(object),"Cube/world collision"); addBox(*scene,kWorldIds[0],object.translation(),{.12,.12,.12},rot);
      require(validState(*scene,model,ln,rn,q,f.right),"robot collision or bounds");
      moveit::core::RobotState state(model); state.setToDefaultValues(); state.setVariablePositions(ln,std::vector<double>(q.begin(),q.end())); state.setVariablePositions(rn,std::vector<double>(f.right.begin(),f.right.end())); state.update();
      Eigen::Isometry3d tcp=state.getGlobalLinkTransform("left_fr3_side_suction_tcp");
      if(phase=="approach_1mm") { grasp=object.inverse()*tcp; has_grasp=true; }
      if(phase=="lift" || phase=="yaw" || phase=="place" || phase=="insert") {
        require(has_grasp,"missing grasp");
        require(std::abs(-grasp.translation().y()-.061)<.0001,"grasp is not at documented 1 mm gap");
        auto expected=object*grasp;
        const double pos=(tcp.translation()-expected.translation()).norm();
        const double ang=Eigen::Quaterniond(tcp.rotation()).angularDistance(Eigen::Quaterniond(expected.rotation()));
        max_pos=std::max(max_pos,pos); max_ang=std::max(max_ang,ang);
        require(pos<.0001 && ang<.002,"rigid grasp closure failed");
      }
      phases.insert(phase); max_z=std::max(max_z,object.translation().z()); previous=q; previous_object=object; ++count;
    }
    require(count>0 && phases.count("approach_1mm") && phases.count("insert") && phases.count("retreat"),"missing phase");
    const bool correct=std::string(argv[6])=="correct"; require(correct || std::string(argv[6])=="direct","bad mode");
    if(correct) { require(phases.count("lift") && phases.count("yaw") && phases.count("place"),"missing correction phase"); require(std::abs(max_z-f.cube.z()-.04)<1e-7,"wrong lift height"); require(Eigen::Quaterniond(previous_object.rotation()).angularDistance(Eigen::Quaterniond::Identity())<1e-7,"wrong final orientation"); }
    else require(Eigen::Quaterniond(previous_object.rotation()).angularDistance(f.orientation)<1e-7,"direct yaw changed");
    require(std::abs(previous_object.translation().x()-f.cube.x()-.017)<1e-7 && std::abs(previous_object.translation().z()-f.cube.z())<1e-7,"wrong final position");
    std::cout<<"REPLAY_PASS samples="<<count<<" precontact_samples="<<nchecked<<" joint_length_rad="<<length<<" closure_m="<<max_pos<<" closure_rad="<<max_ang<<"\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<"REPLAY_REJECT "<<e.what()<<"\n"; return 1; }
}

#!/usr/bin/env python3
"""Generate isolated planners using the existing robot and collision checks."""
import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("prior_generator", ROOT / "research/stage_b_two_level/generate_sources.py")
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f"expected exactly one source token: {old}")
    return text.replace(old, new)


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "offline/task28_stage_b_contact/precontact.cpp").read_text()
    source = replace_once(source, "struct Fixture\n{", "double scene_width = .410, scene_depth = .300, contact_x = 0;\nstruct Fixture\n{")
    source = replace_once(source, '  addBox(scene, kWorldIds[2], {1.07, 0, .275}, {.02, .646, .15});',
                          '  addBox(scene, kWorldIds[2], {1.07, 0, .275}, {.02, scene_width + .04, .15});')
    source = replace_once(source, '  addBox(scene, kWorldIds[3], {.935, -.313, .275}, {.29, .02, .15});',
                          '  addBox(scene, kWorldIds[3], {1.06-scene_depth/2, -(scene_width/2+.01), .275}, {scene_depth+.02, .02, .15});')
    source = replace_once(source, '  addBox(scene, kWorldIds[4], {.935, .313, .275}, {.29, .02, .15});',
                          '  addBox(scene, kWorldIds[4], {1.06-scene_depth/2, (scene_width/2+.01), .275}, {scene_depth+.02, .02, .15});')
    source = prior.seeded_precontact(source)
    configure = '''
    scene_width = input["scene_width_m"].as<double>(); scene_depth = input["scene_depth_m"].as<double>();
    contact_x = input["contact_x_m"].as<double>();
    require(scene_width >= .39 && scene_width <= .43 && scene_depth >= .27 && scene_depth <= .32, "bad scene dimensions");
    require(std::isfinite(contact_x) && std::abs(contact_x) <= .026, "cup footprint outside cube face");'''
    source = replace_once(source, '    const Fixture f = fixtureFrom(input);', '    const Fixture f = fixtureFrom(input);' + configure)
    source = replace_once(source, 'Eigen::Vector3d(0, -(kHalf + kGap), 0)', 'Eigen::Vector3d(contact_x, -(kHalf + kGap), 0)')
    source = replace_once(source, 'Eigen::Vector3d(0, -(kHalf - .003), 0)', 'Eigen::Vector3d(contact_x, -(kHalf - .003), 0)')
    # Mirror the upstream version first, then apply identical scene and contact changes to both mains.
    right = source
    right = replace_once(right, 'Eigen::Vector3d(contact_x, -(kHalf + kGap), 0)', 'Eigen::Vector3d(0, -(kHalf + kGap), 0)')
    right = replace_once(right, 'Eigen::Vector3d(contact_x, -(kHalf - .003), 0)', 'Eigen::Vector3d(0, -(kHalf - .003), 0)')
    right = prior.mirror_precontact(right)
    right = replace_once(right, 'Eigen::Vector3d(0, (kHalf + kGap), 0)', 'Eigen::Vector3d(contact_x, (kHalf + kGap), 0)')
    right = replace_once(right, 'Eigen::Vector3d(0, (kHalf - .003), 0)', 'Eigen::Vector3d(contact_x, (kHalf - .003), 0)')
    (out / "precontact_left.cpp").write_text(source)
    (out / "precontact_right.cpp").write_text(right)
    yaw = (ROOT / "research/stage_b_yaw/yaw_planner.cpp").read_text()
    yaw = replace_once(yaw, '#include "../../offline/task28_stage_b_contact/precontact.cpp"', f'#include "{out / "precontact_left.cpp"}"')
    yaw = replace_once(yaw, 'fclBox({1.07,0,.275},{.02,.646,.15}),', 'fclBox({1.07,0,.275},{.02,scene_width+.04,.15}),')
    yaw = replace_once(yaw, 'fclBox({.935,-.313,.275},{.29,.02,.15}), fclBox({.935,.313,.275},{.29,.02,.15})',
                       'fclBox({1.06-scene_depth/2,-(scene_width/2+.01),.275},{scene_depth+.02,.02,.15}), fclBox({1.06-scene_depth/2,(scene_width/2+.01),.275},{scene_depth+.02,.02,.15})')
    yaw = replace_once(yaw, '    const Fixture f=fixtureFrom(YAML::LoadFile(argv[3]));', '''    const auto input = YAML::LoadFile(argv[3]);
    const Fixture f=fixtureFrom(input);
    scene_width=input["scene_width_m"].as<double>(); scene_depth=input["scene_depth_m"].as<double>(); contact_x=input["contact_x_m"].as<double>();
    require(scene_width >= .39 && scene_width <= .43 && scene_depth >= .27 && scene_depth <= .32 && std::isfinite(contact_x) && std::abs(contact_x) <= .026, "invalid scene/contact");
    std::cout << "SCENE width_m=" << scene_width << " depth_m=" << scene_depth << " contact_x_m=" << contact_x << "\\n";''')
    yaw = replace_once(yaw, '    require(std::abs(initial_gap-.003)<.0001,"unexpected precontact gap");', '''    require(std::abs(initial_gap-.003)<.0001,"unexpected precontact gap");
    const auto local_tcp=f.orientation.inverse()*(tcp.translation()-f.cube);
    require(std::abs(local_tcp.x()-contact_x)<.0001 && std::abs(local_tcp.z())<.0001, "contact location mismatch");
    moveit::core::RobotState cup_state(model); cup_state.setToDefaultValues();
    cup_state.setVariablePositions(ln,std::vector<double>(current.begin(),current.end()));
    cup_state.setVariablePositions(rn,std::vector<double>(f.right.begin(),f.right.end())); cup_state.update();
    const bool is_left=ln.front().find("left_")==0;
    const double cup_y=is_left ? -.150 : .150;
    const auto cup_link=cup_state.getGlobalLinkTransform(is_left ? "left_fr3_side_suction" : "right_fr3_side_suction");
    const auto cup_axis=f.orientation.inverse()*(cup_link.linear()*Eigen::Vector3d::UnitY());
    require(std::abs(std::abs(cup_axis.y())-1.)<.0001, "cup axis not normal to face");
    for (double x : {-.024,.024}) for(double z : {.056,.104}) {
      const Eigen::Vector3d center=f.orientation.inverse()*(cup_link*Eigen::Vector3d(x,cup_y,z)-f.cube);
      require(std::abs(center.x())+.010 <= .060 && std::abs(center.z())+.010 <= .060 &&
              std::abs(std::abs(center.y())-.068)<.0001, "cup footprint outside face or wrong gap");
    }
    std::cout << "CUP_FOOTPRINT_PASS count=4 radius_m=0.010 normal_gap_m=0.003\\n";''')
    (out / "yaw_left.cpp").write_text(yaw)
    (out / "yaw_right.cpp").write_text(prior.mirror_yaw(yaw))


if __name__ == "__main__":
    main()

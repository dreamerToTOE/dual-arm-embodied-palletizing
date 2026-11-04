#!/usr/bin/env python3
"""Add contact diagnostics to a temporary copy of the unchanged planner."""
from pathlib import Path
import sys
root = Path(__file__).resolve().parents[2]
source = (root / 'offline/task28_stage_b_contact/precontact.cpp').read_text()
old = '        require(validState(*scene, model, ln, rn, q, f.right), "alignment path collision");'
new = '''        if (!validState(*scene, model, ln, rn, q, f.right)) {
          moveit::core::RobotState debug_state(model);
          debug_state.setToDefaultValues();
          debug_state.setVariablePositions(ln, std::vector<double>(q.begin(), q.end()));
          debug_state.setVariablePositions(rn, std::vector<double>(f.right.begin(), f.right.end()));
          debug_state.update();
          collision_detection::CollisionRequest req;
          req.contacts = true; req.max_contacts = 100; req.max_contacts_per_pair = 10;
          collision_detection::CollisionResult result;
          scene->checkCollision(req, result, debug_state);
          std::cerr << "ALIGN_DEBUG segment=" << segment << " substep=" << i << "/" << count
                    << " left_bounds=" << debug_state.satisfiesBounds(model->getJointModelGroup("left_arm"))
                    << " right_bounds=" << debug_state.satisfiesBounds(model->getJointModelGroup("right_arm")) << "\\n";
          for (const auto& [pair, contacts] : result.contacts)
            for (const auto& contact : contacts)
              std::cerr << "CONTACT " << pair.first << " <-> " << pair.second << " depth_m=" << contact.depth << "\\n";
          require(false, "alignment path collision");
        }'''
assert source.count(old) == 1
Path(sys.argv[1]).write_text(source.replace(old, new))
if len(sys.argv) == 3:
    source = source.replace(old, new)
    before, main = source.split('int main(int argc, char **argv)', 1)
    replacements = [
        ('names("left"), rn = names("right")', 'names("right"), rn = names("left")'),
        ('f.left', 'f.__temporary'), ('f.right', 'f.left'), ('f.__temporary', 'f.right'),
        ('left_fr3_side_suction', 'right_fr3_side_suction'), ('"left_arm"', '"right_arm"'),
        ('left_fr3_joint', 'right_fr3_joint'),
        ('Eigen::Vector3d(0, -(kHalf + kGap), 0)', 'Eigen::Vector3d(0, (kHalf + kGap), 0)'),
        ('Eigen::Vector3d(0, -(kHalf - .003), 0)', 'Eigen::Vector3d(0, (kHalf - .003), 0)'),
        ('Eigen::Quaterniond(0, std::sqrt(.5), std::sqrt(.5), 0)', 'Eigen::Quaterniond(0, std::sqrt(.5), -std::sqrt(.5), 0)'),
        ('-(f.orientation.inverse() * (tcp.translation() - f.cube)).y() - kHalf', '(f.orientation.inverse() * (tcp.translation() - f.cube)).y() - kHalf'),
    ]
    for a, b in replacements:
        assert a in main, a
        main = main.replace(a, b)
    main = main.replace('joint3,left_"\n                                          "fr3_joint4', 'joint3,right_"\n                                          "fr3_joint4')
    main = main.replace('left_bounds=" << debug_state.satisfiesBounds(model->getJointModelGroup("right_arm"))', 'left_bounds=" << debug_state.satisfiesBounds(model->getJointModelGroup("left_arm"))')
    assert 'joint3,left_"' not in main
    Path(sys.argv[2]).write_text(before + 'int main(int argc, char **argv)' + main)

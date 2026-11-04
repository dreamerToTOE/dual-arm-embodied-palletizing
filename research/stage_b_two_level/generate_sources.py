#!/usr/bin/env python3
"""Generate the mirrored right-arm sources used by the offline candidate study."""
from pathlib import Path
import sys


def seeded_precontact(source: str) -> str:
    old = '    ompl::RNG::setSeed(28);'
    new = '    const auto seed = std::getenv("STAGE_B_SEED") ? std::stoul(std::getenv("STAGE_B_SEED")) : 28;\n    ompl::RNG::setSeed(seed);'
    assert source.count(old) == 1 and source.count('std::mt19937 rng(28);') == 1
    source = source.replace(old, new).replace('std::mt19937 rng(28);', 'std::mt19937 rng(seed);')
    world = '    addWorld(*scene, f, true);'
    assert source.count(world) == 1
    with_occupancy = world + '''
    if (input["occupied_boxes"]) {
      require(input["occupied_boxes"].IsSequence(), "occupancy must be a sequence");
      int index = 0;
      for (const auto &item : input["occupied_boxes"]) {
        std::array<double, 3> xyz{}, size{};
        require(task28_b0::finiteArray(item["xyz_m"], xyz) && task28_b0::finiteArray(item["size_m"], size), "invalid occupancy");
        const auto id = "occupied_cube_" + std::to_string(index++);
        require(item["id"] && item["id"].as<std::string>() == id, "unexpected occupancy id");
        for (double dim : size) require(std::abs(dim - .12) < 1e-9, "occupancy must use original Cube dimensions");
        addBox(*scene, id, {xyz[0], xyz[1], xyz[2]}, {size[0], size[1], size[2]});
      }
    }'''
    return source.replace(world, with_occupancy)


def mirror_precontact(source: str) -> str:
    before, main = source.split("int main(int argc, char **argv)", 1)
    replacements = [
        ('names("left"), rn = names("right")', 'names("right"), rn = names("left")'),
        ('f.left', 'f.__temporary'), ('f.right', 'f.left'), ('f.__temporary', 'f.right'),
        ('left_fr3_side_suction_tcp', 'right_fr3_side_suction_tcp'),
        ('left_fr3_side_suction', 'right_fr3_side_suction'),
        ('"left_arm"', '"right_arm"'),
        ('left_fr3_joint', 'right_fr3_joint'),
        ('Eigen::Vector3d(0, -(kHalf + kGap), 0)', 'Eigen::Vector3d(0, (kHalf + kGap), 0)'),
        ('Eigen::Vector3d(0, -(kHalf - .003), 0)', 'Eigen::Vector3d(0, (kHalf - .003), 0)'),
        ('Eigen::Quaterniond(0, std::sqrt(.5), std::sqrt(.5), 0)',
         'Eigen::Quaterniond(0, std::sqrt(.5), -std::sqrt(.5), 0)'),
        ('-(f.orientation.inverse() * (tcp.translation() - f.cube)).y() - kHalf',
         '(f.orientation.inverse() * (tcp.translation() - f.cube)).y() - kHalf'),
    ]
    for old, new in replacements:
        if old not in main:
            raise RuntimeError(f"missing precontact mirror token: {old}")
        main = main.replace(old, new)
    return before + "int main(int argc, char **argv)" + main


def mirror_yaw(source: str) -> str:
    before, main = source.split("int main(int argc, char** argv)", 1)
    replacements = [
        ('names("left"), rn=names("right")', 'names("right"), rn=names("left")'),
        ('f.left', 'f.__temporary'), ('f.right', 'f.left'), ('f.__temporary', 'f.right'),
        ('"left_arm"', '"right_arm"'),
        ('"left_fr3_side_suction_tcp"', '"right_fr3_side_suction_tcp"'),
        ('Vector3d(0,approach_distance*i/approach_steps,0)',
         'Vector3d(0,-approach_distance*i/approach_steps,0)'),
        ('next.translation().y()-=.003*i/6', 'next.translation().y()+=.003*i/6'),
        ('-(f.orientation.inverse()*(tcp.translation()-f.cube)).y()-.06',
         '(f.orientation.inverse()*(tcp.translation()-f.cube)).y()-.06'),
    ]
    for old, new in replacements:
        if old not in main:
            raise RuntimeError(f"missing yaw mirror token: {old}")
        main = main.replace(old, new)
    return before + "int main(int argc, char** argv)" + main


def main() -> None:
    if len(sys.argv) != 4:
        raise SystemExit("usage: generate_sources.py PRECONTACT_CPP YAW_CPP OUTPUT_DIR")
    precontact, yaw, output = map(Path, sys.argv[1:])
    output.mkdir(parents=True, exist_ok=True)
    left = seeded_precontact(precontact.read_text())
    (output / "precontact_left.cpp").write_text(left)
    (output / "precontact_right.cpp").write_text(mirror_precontact(left))
    (output / "yaw_planner_right.cpp").write_text(mirror_yaw(yaw.read_text()))


if __name__ == "__main__":
    main()

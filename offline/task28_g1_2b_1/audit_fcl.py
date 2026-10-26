#!/usr/bin/env python3
"""Reproduce the offline system-FCL CCD capability and motion-model audit.

Only this new audit and its JSON evidence are repository outputs.  The C++
probe is embedded here, compiled in a temporary directory, and never drives a
robot, simulator, ROS process, or GPU.  No Python FCL binding is installed.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import shlex
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[2]
BASE_HEAD = "bcc6e7d260eaa45f6f0d055d53334922df31db3a"
FCL_SOURCE = "https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/"
HEADERS = (
    "config.h",
    "common/types.h",
    "narrowphase/continuous_collision_request.h",
    "narrowphase/continuous_collision-inl.h",
    "narrowphase/continuous_collision_result-inl.h",
    "narrowphase/detail/conservative_advancement_func_matrix-inl.h",
    "narrowphase/detail/traversal/distance/shape_conservative_advancement_traversal_node-inl.h",
    "math/motion/translation_motion-inl.h",
    "math/motion/interp_motion.h",
    "math/motion/interp_motion-inl.h",
    "math/motion/screw_motion-inl.h",
    "math/motion/spline_motion-inl.h",
)


CPP_PROBE = r'''
#include <fcl/fcl.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
using T=fcl::Transform3d;
using V=fcl::Vector3d;
using G=fcl::CollisionGeometryd;
T pose(double x,double y=0,double z=0,double theta=0) {
  T t=T::Identity(); t.translation()=V(x,y,z);
  t.linear()=fcl::AngleAxisd(theta,V::UnitZ()).toRotationMatrix(); return t;
}
void matrix(const T& t) {
  std::cout<<'[';
  for(int i=0;i<4;++i) {if(i)std::cout<<',';std::cout<<'[';
    for(int j=0;j<4;++j) {if(j)std::cout<<',';std::cout<<t.matrix()(i,j);}
    std::cout<<']';}
  std::cout<<']';
}
void record(const std::string& name, std::shared_ptr<G> a, T a0,T a1,
            std::shared_ptr<G> b,T b0,T b1,int motion,int solver,
            std::size_t iterations=10,double toc_err=0.0001) {
  fcl::ContinuousCollisionRequestd q(iterations,toc_err,
      static_cast<fcl::CCDMotionType>(motion),fcl::GST_LIBCCD,
      static_cast<fcl::CCDSolverType>(solver));
  fcl::ContinuousCollisionResultd r;
  auto discrete=[&](const T& x,const T& y) {
    fcl::CollisionRequestd d;fcl::CollisionResultd z;
    return fcl::collide(a.get(),x,b.get(),y,d,z)>0;
  };
  T am=a0,bm=b0;
  am.translation()=(a0.translation()+a1.translation())*.5;
  bm.translation()=(b0.translation()+b1.translation())*.5;
  am.linear()=fcl::Quaterniond(a0.linear()).slerp(.5,fcl::Quaterniond(a1.linear())).toRotationMatrix();
  bm.linear()=fcl::Quaterniond(b0.linear()).slerp(.5,fcl::Quaterniond(b1.linear())).toRotationMatrix();
  bool start=discrete(a0,b0),mid=discrete(am,bm),end=discrete(a1,b1);
  double ret=fcl::continuousCollide(a.get(),a0,a1,b.get(),b0,b1,q,r);
  std::cout<<"{\"kind\":\"ccd\",\"name\":\""<<name
    <<"\",\"motion\":"<<motion<<",\"solver\":"<<solver
    <<",\"iterations\":"<<iterations<<",\"toc_err\":"<<toc_err
    <<",\"return_value\":"<<ret<<",\"is_collide\":"
    <<(r.is_collide?"true":"false")<<",\"time_of_contact\":"
    <<r.time_of_contact<<",\"discrete_start_collision\":"<<(start?"true":"false")
    <<",\"discrete_LINEAR_midpoint_collision\":"<<(mid?"true":"false")
    <<",\"discrete_end_collision\":"<<(end?"true":"false")<<"}\n";
}
std::shared_ptr<G> tetra(bool bvh=false) {
  std::vector<V> vs{V(-.02,-.02,-.02),V(.02,-.02,-.02),V(0,.02,-.02),V(0,0,.02)};
  std::vector<fcl::Triangle> ts{{0,2,1},{0,1,3},{0,3,2},{1,2,3}};
  if(bvh) {
    auto b=std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    b->beginModel();b->addSubModel(vs,ts);b->endModel(); return b;
  }
  auto vertices=std::make_shared<const std::vector<V>>(vs);
  auto faces=std::make_shared<const std::vector<int>>(
    std::vector<int>{3,0,2,1,3,0,1,3,3,0,3,2,3,1,2,3});
  return std::make_shared<fcl::Convexd>(vertices,4,faces,true);
}
int main() {
  std::cout<<std::setprecision(17);
  fcl::ContinuousCollisionRequestd defaults;
  std::cout<<"{\"kind\":\"defaults\",\"num_max_iterations\":"
    <<defaults.num_max_iterations<<",\"toc_err\":"<<defaults.toc_err
    <<",\"ccd_motion_type\":"<<defaults.ccd_motion_type
    <<",\"gjk_solver_type\":"<<defaults.gjk_solver_type
    <<",\"ccd_solver_type\":"<<defaults.ccd_solver_type<<"}\n";
  auto sphere=std::make_shared<fcl::Sphered>(.01);
  auto box=std::make_shared<fcl::Boxd>(.02,.02,.02);
  auto cylinder=std::make_shared<fcl::Cylinderd>(.01,.02);
  auto convex=tetra();
  // A narrow collision lies between all ten default naive sample locations.
  for(int solver=0;solver<4;++solver)
    record("thin_sphere_crossing",sphere,pose(-1),pose(1),sphere,
      pose(0),pose(0),0,solver);
  for(int motion=0;motion<3;++motion) {
    record("box_box_crossing",box,pose(-1),pose(1),box,pose(0),pose(0),motion,1);
    record("cylinder_box_crossing",cylinder,pose(-1),pose(1),box,pose(0),pose(0),motion,1);
    record("convex_box_crossing",convex,pose(-1),pose(1),box,pose(0),pose(0),motion,1);
    record("convex_convex_crossing",convex,pose(-1),pose(1),convex,pose(0),pose(0),motion,1);
    record("sphere_clear",sphere,pose(-1,1),pose(1,1),sphere,pose(0),pose(0),motion,1);
  }
  record("initial_overlap",sphere,pose(0),pose(1),sphere,pose(0),pose(0),0,1);
  record("endpoint_overlap",sphere,pose(-1),pose(0),sphere,pose(0),pose(0),0,1);
  // Request knobs are deliberately extreme: primitive CA ignores both fields.
  record("primitive_ca_request_knobs",sphere,pose(-1),pose(1),sphere,
    pose(0),pose(0),0,1,1,.5);
  // Rotation-only long box reaches obstacle at its final orientation.
  auto rod=std::make_shared<fcl::Boxd>(1,.02,.02);
  for(int motion=0;motion<3;++motion)
    record("rotation_only",rod,pose(0),pose(0,0,0,M_PI/2),box,
      pose(0,.4),pose(0,.4),motion,1);
  // FCL's advertised polynomial solver supports BVH/BVH translation only.
  for(int solver : {0,1,3})
    record("bvh_bvh_crossing",tetra(true),pose(-1),pose(1),tetra(true),
      pose(0),pose(0),0,solver);
  // Endpoint-transform interpolation of a one-revolute-joint FK arc misses
  // the sphere at its actual midpoint when LINEAR is selected.
  double theta=M_PI/3;
  auto arc_sphere=std::make_shared<fcl::Sphered>(.02);
  for(int motion=0;motion<3;++motion)
    record("fk_arc_endpoint_motion",arc_sphere,
      pose(std::cos(theta),-std::sin(theta),0,-theta),
      pose(std::cos(theta), std::sin(theta),0, theta),arc_sphere,
      pose(1),pose(1),motion,1);
  fcl::CollisionRequestd cq;fcl::CollisionResultd cr;
  std::size_t hit=fcl::collide(arc_sphere.get(),pose(1),arc_sphere.get(),pose(1),cq,cr);
  std::cout<<"{\"kind\":\"fk_arc_midpoint_collision\",\"contacts\":"<<hit<<"}\n";
  // Read frozen-robot endpoint matrices prepared by the independent Python FK.
  int n=0;std::cin>>n;
  for(int k=0;k<n;++k) {
    std::string name;std::cin>>name;T a=T::Identity(),b=T::Identity();
    for(int i=0;i<3;++i)for(int j=0;j<4;++j)std::cin>>a.matrix()(i,j);
    for(int i=0;i<3;++i)for(int j=0;j<4;++j)std::cin>>b.matrix()(i,j);
    for(int motion=0;motion<3;++motion) {
      auto m=fcl::getMotionBase<double>(a,b,static_cast<fcl::CCDMotionType>(motion));
      for(double u : {0.,.25,.5,.75,1.}) {
        m->integrate(u);T t;m->getCurrentTransform(t);
        std::cout<<"{\"kind\":\"interpolated_motion\",\"name\":\""<<name
          <<"\",\"motion\":"<<motion<<",\"u\":"<<u<<",\"transform\":";
        matrix(t);std::cout<<"}\n";
      }
    }
  }
}
'''


def command(args, **kwargs):
    return subprocess.run(args, check=True, text=True, capture_output=True,
                          timeout=120, **kwargs)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def frozen_fk_cases():
    """Use frozen G1-1 FK only to expose interpolation mismatch, never accept."""
    sys.path.insert(0, str(ROOT / "offline/task28_g1_1"))
    import numpy as np
    from task28_validator.model import RobotModel
    model = RobotModel()
    delta = np.array([.8, .3, -.25, .2, .35, -.3, .4])
    q0, q1 = model.home - delta / 2, model.home + delta / 2
    assert model.within_limits(q0) and model.within_limits(q1)
    cases = []
    for name in ("left_fr3_link7", "left_fr3_side_suction_tcp"):
        actual = {u: model.forward((1-u)*q0+u*q1, model.home)[name]
                  for u in (0., .25, .5, .75, 1.)}
        cases.append({"name": name, "q0_rad": q0.tolist(), "q1_rad": q1.tolist(),
                      "actual": actual, "source": str(Path(model.__module__.replace('.', '/')))})
    # Translation-only SCREW must also reach the requested endpoint.  Probe the
    # actual implementation, whose pure-translation branch does not normalize.
    cases.append({"name": "pure_translation_2m", "actual": {
        u: np.array([[1.,0,0,2*u],[0,1.,0,0],[0,0,1.,0],[0,0,0,1.]])
        for u in (0., .25, .5, .75, 1.)}})
    return cases


def inspect_downloaded_python_binding(wheel):
    """Optional source-only comparison; its bundled FCL is never imported."""
    if not wheel:
        return {"installed": False, "source_comparison": "not requested"}
    import ast
    import zipfile
    wheel = Path(wheel).resolve()
    with zipfile.ZipFile(wheel) as archive:
        data = archive.read("fcl/collision_data.py")
        pyx = archive.read("fcl/fcl.pyx")
        metadata_name = next(n for n in archive.namelist() if n.endswith(".dist-info/METADATA"))
        metadata = archive.read(metadata_name).decode()
        tree = ast.parse(data)
        klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ContinuousCollisionRequest")
        init = next(n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
        defaults = {arg.arg: ast.unparse(value) for arg, value in zip(init.args.args[-len(init.args.defaults):], init.args.defaults)}
    return {
        "installed": False, "executed_or_installed": False,
        "source_only_download": str(wheel), "wheel_sha256": sha(wheel),
        "version": next(s.split(": ",1)[1] for s in metadata.splitlines() if s.startswith("Version: ")),
        "source_hashes": {"fcl/collision_data.py": hashlib.sha256(data).hexdigest(),
                          "fcl/fcl.pyx": hashlib.sha256(pyx).hexdigest()},
        "ContinuousCollisionRequest_defaults": defaults,
        "continuousCollide_signature": "continuousCollide(o1, tf1_end, o2, tf2_end, request=None, result=None)",
        "wrapper_behavior": "passes only endpoint transforms to FCL; returns C++ scalar unchanged; result fields accumulate with OR(is_collide), min(time_of_contact). Use a fresh result for every query.",
        "source_links": [
            "https://github.com/BerkeleyAutomation/python-fcl/blob/v0.7.0.10/src/fcl/collision_data.py",
            "https://github.com/BerkeleyAutomation/python-fcl/blob/v0.7.0.10/src/fcl/fcl.pyx",
            "https://pypi.org/project/python-fcl/0.7.0.10/",
        ],
        "interpretation": "The downloaded binding defaults to CONSERVATIVE_ADVANCEMENT, unlike the installed C++ API's NAIVE default. This is a source comparison, not a claim that Python-FCL is the active backend.",
    }


def audit(wheel=None):
    import numpy as np
    from scipy.spatial.transform import Rotation
    flag_text = command(["pkg-config", "--cflags", "--libs", "fcl"]).stdout.strip()
    cases = frozen_fk_cases()
    words = [str(len(cases))]
    for case in cases:
        words += [case["name"]]
        for u in (0., 1.):
            words += [format(float(v), ".17g") for v in case["actual"][u][:3,:].flat]
    with tempfile.TemporaryDirectory(prefix="task28_g1_2b_1_fcl_audit_") as tmp:
        source, binary = Path(tmp)/"probe.cpp", Path(tmp)/"probe"
        source.write_text(CPP_PROBE)
        compile_args = ["g++", "-O2", "-std=c++17", str(source), "-o", str(binary), *shlex.split(flag_text)]
        compiled = command(compile_args)
        run = command([str(binary)], input=" ".join(words)+"\n")
        linked = command(["ldd", str(binary)]).stdout
    records = [json.loads(line) for line in run.stdout.splitlines() if line.startswith("{")]
    defaults = next(r for r in records if r["kind"] == "defaults")
    interpolations = []
    for case in cases:
        values = []
        for r in records:
            if r.get("kind") != "interpolated_motion" or r["name"] != case["name"]:
                continue
            actual = case["actual"][r["u"]]
            tf = np.asarray(r["transform"])
            values.append({**r, "actual_fk_transform": actual.tolist(),
                           "translation_error_m": float(np.linalg.norm(actual[:3,3]-tf[:3,3])),
                           "rotation_error_rad": float(Rotation.from_matrix(actual[:3,:3].T @ tf[:3,:3]).magnitude())})
        row = {"name": case["name"], "records": values}
        if "q0_rad" in case:
            row.update({"q0_rad": case["q0_rad"], "q1_rad": case["q1_rad"],
                        "interpolation": "q(u)=(1-u)q0+u*q1; right arm remains at frozen home; both endpoints inside frozen effective joint limits. Kinematic diagnostic only: full-state closed-chain/contact/collision validity is not asserted."})
        interpolations.append(row)
    # Header hashes identify the actual installed implementation, rather than
    # relying on a README's capabilities or an unpinned latest branch.
    provenance = []
    for relative in HEADERS:
        path = Path("/usr/include/fcl")/relative
        upstream_relative = "config.h.in" if relative == "config.h" else relative
        provenance.append({"path": str(path), "sha256": sha(path), "upstream_tag_source": FCL_SOURCE+upstream_relative})
    libraries = []
    for line in linked.splitlines():
        if "=> /" in line and any(name in line for name in ("libfcl", "libccd", "liboctomap", "liboctomath")):
            path = Path(line.split("=>",1)[1].strip().split()[0]).resolve()
            libraries.append({"path": str(path), "sha256": sha(path)})
    ccd = [r for r in records if r["kind"] == "ccd"]
    by_name = lambda n: [r for r in ccd if r["name"] == n]
    thin = by_name("thin_sphere_crossing")
    assert thin[0]["is_collide"] is False and thin[1]["is_collide"] is True
    assert all(r["return_value"] == -1 for r in thin[2:])
    arc = by_name("fk_arc_endpoint_motion")
    assert arc[1]["is_collide"] is False
    assert next(r for r in records if r["kind"] == "fk_arc_midpoint_collision")["contacts"] > 0
    robot_errors = [r["translation_error_m"] for x in interpolations[:2] for r in x["records"] if r["motion"] == 1 and r["u"] == .5]
    assert max(robot_errors) > .001
    bvh = by_name("bvh_bvh_crossing")
    assert all(not r["discrete_start_collision"] and r["discrete_LINEAR_midpoint_collision"] for r in bvh)
    assert bvh[1]["is_collide"] is False
    return {
        "schema": "task28.g1_2b_1.fcl_capabilities.v1",
        "scope": "Offline capability/motion-model audit only; FCL CCD results are not articulated-path certificates.",
        "base_head": BASE_HEAD,
        "environment": {
            "python": platform.python_version(), "python_executable": sys.executable,
            "python_fcl_importable": importlib.util.find_spec("fcl") is not None,
            "system_fcl_pkg_config_version": command(["pkg-config", "--modversion", "fcl"]).stdout.strip(),
            "packages": command(["dpkg-query", "-W", "libfcl-dev", "libfcl0.7", "libccd-dev", "libccd2", "libeigen3-dev"]).stdout.splitlines(),
            "compiler": command(["g++", "--version"]).stdout.splitlines()[0],
            "compile_flags": flag_text, "compile_stderr": compiled.stderr,
            "source_provenance": provenance, "linked_libraries": libraries,
            "probe_source_sha256": hashlib.sha256(CPP_PROBE.encode()).hexdigest(),
            "audit_script_sha256": sha(__file__),
            "frozen_fk_sources": [{"path":str(ROOT/relative),"sha256":sha(ROOT/relative)} for relative in (
                "offline/task28_g1_1/task28_validator/model.py",
                "offline/task28_g1_1/task28_validator/se3.py",
                "fixtures/task28_g1_0_new_single_cube_v0/moveit/task28_single_cube_description/urdf/task28_single_cube_frozen.urdf",
                "fixtures/task28_g1_0_new_single_cube_v0/fixture_manifest.json")],
            "upstream_header_comparison": {
                "source":FCL_SOURCE+"narrowphase/continuous_collision-inl.h",
                "downloaded_source_sha256":"c8ff31f3d110236180cc627f0e0c15319aa141ca3bf61df9ce58fdd3be2c90ce",
                "installed_header_matches_official_0_7_0_tag":sha("/usr/include/fcl/narrowphase/continuous_collision-inl.h")=="c8ff31f3d110236180cc627f0e0c15319aa141ca3bf61df9ce58fdd3be2c90ce",
                "comparison_scope":"Exact continuous_collision-inl.h bytes compared during audit. Remaining source records identify installed bytes; generated config.h need not exist upstream."},
        },
        "python_binding_source_comparison": inspect_downloaded_python_binding(wheel),
        "cpp_api": {
            "default_request_actual": defaults,
            "signature": "continuousCollide(geometry1, tf1_begin, tf1_end, geometry2, tf2_begin, tf2_end, request, result) -> double; CollisionObject overload reads begin poses from objects",
            "motion_enum": {"CCDM_TRANS":0,"CCDM_LINEAR":1,"CCDM_SCREW":2,"CCDM_SPLINE":3},
            "solver_enum": {"CCDC_NAIVE":0,"CCDC_CONSERVATIVE_ADVANCEMENT":1,"CCDC_RAY_SHOOTING":2,"CCDC_POLYNOMIAL_SOLVER":3},
            "return_semantics": "Supported queries return scalar time in [0,1]; consult fresh result.is_collide to distinguish contact from no contact. -1 means unsupported/invalid/error, never no collision. time_of_contact=1 and is_collide=false are unchanged initial defaults on unsupported calls.",
            "sources": [FCL_SOURCE+"narrowphase/continuous_collision_request.h", FCL_SOURCE+"narrowphase/continuous_collision-inl.h"],
        },
        "motion_types": [
            {"type":"CCDM_TRANS", "implemented":True, "semantics":"T(u)=T0+u(T1-T0); rotation remains R0, even at u=1. Any supplied final rotation is ignored."},
            {"type":"CCDM_LINEAR", "implemented":True, "semantics":"InterpMotion advances rotation using constant-axis principal AngleAxis(R1 R0^T). Translation depends on reference_p; the endpoint constructor does not explicitly initialize that Eigen vector. A zero reference point gives the endpoint chord, as observed in this clean probe process, but zero initialization is not a guaranteed source-level default. No joint states or FK callbacks are supplied.", "reference_initialization_caveat":"Do not infer a zero reference point from the source comment: inspected constructors do not explicitly set reference_p. Recorded transforms describe this run only; no cross-build or allocator-independent correctness guarantee is made."},
            {"type":"CCDM_SCREW", "implemented":True, "semantics":"Single screw from endpoint transforms. General articulated simultaneous-joint FK is not a single screw. Installed pure-translation branch does not normalize its axis; actual endpoint failure is recorded."},
            {"type":"CCDM_SPLINE", "implemented":False, "semantics":"Endpoint-transform constructors contain TODO and do not initialize spline controls; although enum/dispatch exist, this overload is unusable. It was not executed because using uninitialized controls is undefined behavior. Explicit multi-control-point constructor exists in C++ but is not this endpoint API."},
        ],
        "solver_capabilities": [
            {"type":"CCDC_NAIVE", "implemented":True, "semantics":"Discrete sampling only: n=min(num_max_iterations,ceil(1/toc_err)); samples u=i/(n-1). Default n=10. An actual between-sample collision is missed by the control below. It cannot certify continuous clearance."},
            {"type":"CCDC_CONSERVATIVE_ADVANCEMENT", "implemented":True, "semantics":"Shape pairs box/cylinder/convex have dispatch entries, and the recorded probes run them. Primitive wrapper ignores request (FCL_UNUSED(request)); its node has t_err=0.0001 and loop has no request iteration budget. BVH dispatch exists but the OBBRSS BVH/BVH crossing probe returns false/1 despite a discrete midpoint collision; dispatch availability alone is not correctness evidence. Useful diagnostic for its own rigid-motion model, not a joint-FK certificate."},
            {"type":"CCDC_RAY_SHOOTING", "implemented":False, "semantics":"Primitive translation branch is empty and returns -1. Actual recorded unsupported control preserves default false/1 result."},
            {"type":"CCDC_POLYNOMIAL_SOLVER", "implemented":True, "semantics":"Only BVH/BVH plus CCDM_TRANS accepted; primitive and Convex shapes are rejected with -1. BVH implementation updates vertex buffers internally; probe uses fresh geometries. Actual BVH crossing returns TOC 0 despite a discrete collision-free start, so no correct transform-aware TOC claim is made. Frozen G1-1 STL hulls are loaded as fcl::Convexd solids, not BVHModel, so this solver does not apply to those representations."},
        ],
        "actual_ccd_controls": ccd,
        "probe_stderr": run.stderr,
        "independent_mathematical_counterexample": {
            "model": "One revolute joint, unit-radius attached sphere center p(q)=(cos(q),sin(q),0), orientation Rz(q); q(u)=-pi/3+2*pi*u/3; moving and obstacle radii both 0.02 m; stationary obstacle center (1,0,0).",
            "fk_midpoint": [1.,0.,0.], "endpoint_chord_midpoint": [.5,0.,0.],
            "midpoint_deviation_m": .5,
            "derivation": "At u=1/2 FK gives cos(0)=1. Endpoint translations both have x=cos(pi/3)=1/2, so a zero-reference-point endpoint LINEAR/chord model keeps x=1/2 for every u. Its distance to the obstacle is at least 1/2 m > 0.04 m, whereas actual FK centers coincide at u=1/2. The actual endpoint API call below returned clear in this clean probe process; the separately documented reference_p initialization caveat prevents treating the chord model as an unconditional implementation guarantee.",
            "actual_FCL_endpoint_ccd": arc,
            "actual_FCL_FK_midpoint_contact_count": next(r["contacts"] for r in records if r["kind"] == "fk_arc_midpoint_collision"),
            "claim": "This exact analytic counterexample disproves treating zero-reference-point/chord LINEAR endpoint-transform CCD negatives as collision-free joint-interpolation certificates. The observed installed-API negative and independent actual-FK midpoint contact confirm a miss in this process; the analytic model mismatch itself is independent of discretization resolution and floating-point tolerances.",
        },
        "frozen_robot_and_translation_motion_comparison": interpolations,
        "method_decision": {
            "endpoint_ccd_negative_can_certify_articulated_segment": False,
            "required_for_safe_use": "Either prove actual FK motion equals the selected FCL rigid-motion model over the entire interval, or conservatively bound the mismatch and include that bound in an independently verified enclosure/clearance proof. Otherwise retain UNKNOWN and subdivide under an independent conservative bound.",
            "unacceptable_shortcuts": ["fixed dense samples as continuous certification", "motion enum existence as proof of implementation", "ret<0 interpreted as clearance", "Python-FCL and C++ defaults treated as identical", "TOC tolerance interpreted as geometry/FK mismatch bound"],
            "role_here": "FCL is a capability and collision-witness diagnostic. The continuous verifier uses independent conservative articulated motion bounds.",
        },
        "verification": {"thin_naive_miss_vs_CA_hit": True, "unsupported_returns_minus_one": True,
                         "analytic_FK_arc_collision_vs_LINEAR_clear": True,
                         "frozen_robot_midpoint_LINEAR_translation_error_exceeds_1mm": True,
                         "BVH_CA_clear_despite_independent_discrete_midpoint_collision": True},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("evidence")/"fcl_capabilities.json")
    parser.add_argument("--python-fcl-source-wheel", type=Path,
                        help="Optional downloaded wheel for source-only binding comparison; never imported or installed")
    args = parser.parse_args()
    report = audit(args.python_fcl_source_wheel)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"output":str(args.output.resolve()),"fcl_version":report["environment"]["system_fcl_pkg_config_version"],
                      "python_fcl_importable":report["environment"]["python_fcl_importable"],
                      "controls":len(report["actual_ccd_controls"]),"verification":report["verification"]},indent=2))


if __name__ == "__main__":
    main()

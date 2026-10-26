# Offline FCL state helper

`fcl_state.cpp` is a standalone, bounded single-state query helper. It has no ROS,
Isaac, actuator, suction, trajectory or planning integration. Build output goes to
`/tmp/task28_g1_1_fcl/`, outside the repository. No binary is version controlled.

`CollisionValidator` calls `pkg-config --modversion fcl` and requires `0.7.0`.
The compiled source consumes only the fixture's frozen collision geometry and
explicitly requested shape pairs. The equivalent compile command is:

```bash
g++ -O2 -std=c++14 offline/task28_g1_1/task28_validator/backend/fcl_state.cpp \
  -o /tmp/task28_g1_1_fcl_state $(pkg-config --cflags --libs fcl)
```

The Python API compiles automatically if its source/version/flags cache key is
absent. Missing headers, compiler, library, geometry or subprocess failure rejects
the query. There is no approximate fallback. Version changes require review.

Recorded local dependencies: g++ `11.4.0-1ubuntu1~22.04.3`, `libfcl0.7` and
`libfcl-dev` `0.7.0-3`, `libccd2` `2.1-2`, `liboctomap1.9`
`1.9.7+dfsg-3`, `libeigen3-dev` `3.4.0-2ubuntu2`. The loaded
`/usr/lib/x86_64-linux-gnu/libfcl.so.0.7.0` SHA-256 is
`0bb493f432f0a38ff605198937a037e1c41f1d5fb6dfc36473988da973b5dc05`.
These are measured host dependencies, not a cross-platform bitwise guarantee.

The unchanged binary STL files are the frozen proven convex conservative union
hulls. They are loaded as solid `fcl::Convexd`, not triangle surfaces, so completely
enclosed payloads are still collisions. Boxes and cylinders remain exact FCL
primitives. All 30 robot subgeometries, Cube and table participate. Same-owner
tool subgeometries form one compound body; the frozen 48 SRDF exceptions are
expanded exactly, producing 40 actual geometry-pair skips and 414 checked shape
pairs. Link8 and TCP frames have no collision geometry.

The `bilateral` attachment contact policy is separate from the unchanged SRDF.
It never skips a whole tool/Cube pair. All eight frozen cup front caps must lie
on their prescribed object-coordinate patches (X/Z +/-24 mm, Y +/-60 mm), with
cap axis error <=1e-6 rad and cap-centre/point plane error <=1e-6 m. Only contact
records of those cylinder subgeometries can be accepted, with absolute penetration
<=1e-6 m, bounded cap radius and contact-normal angle <=0.01 rad. The looser
normal bound handles ill-conditioning of GJK normals at zero-depth contact; it
does not relax the cap-axis, position or penetration gates. Reaching 128 contact
records rejects a contact exception. Detached mode, wrong face, missing opposite
tool, shifted patch, manifold collision and excessive penetration all reject.
The caller separately enforces full grasp SE(3) closure and typed attachment
consistency; a contact diagnostic alone never establishes suction or closure.

No distance margin, edge interpolation, swept volume or trajectory validity is
computed. Exact touching can round to a positive tiny separation on one side;
reported contacts are checked, and the independent geometric contact context is
checked before any exception. A nominal 1 mm grasp gap needs no contact exception.

The response also carries `shape_count` and `checked_pair_count`, written after
the query loop finishes. Python requires exact completion counts and validates
all pair/contact fields, uniqueness, finite vectors and unit normals. Missing,
incomplete, duplicate or malformed results reject instead of treating an empty
collision list as evidence of clearance. Complex or boolean transforms reject
before numeric conversion. `backend_path` is an explicitly trusted executable
injection for controlled tests/deployments; the default builds the delivered
source. A completion receipt does not authenticate an arbitrary supplied binary.

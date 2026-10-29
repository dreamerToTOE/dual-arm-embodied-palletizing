# R7 literature and source review

This review was written before any R7 Isaac experiment. The frozen source is
`fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py::build_scene`.
R6 files and evidence are read-only inputs; no R6 artifact is edited.

## Sources and technical lessons

* The Franka interactive controller examples describe joint gravity compensation
  and joint/cartesian impedance as torque-domain controllers. They also warn that
  external-tool compensation is an explicit model term; this is not evidence that
  Isaac explicit effort has FCI hardware semantics. Source:
  https://github.com/nbfigueroa/franka_interactive_controllers
* libfranka documents a 1 kHz FCI loop, external torque controllers, and torque
  rate limiting/signal filtering. The installed Isaac stage is a 60 Hz CPU TGS
  simulation with end-of-step solver velocity, so those hardware timing and
  filtering guarantees are not transferred to this experiment. Source:
  https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html
* Pinocchio documents RNEA for inverse dynamics/gravity, CRBA for joint-space
  inertia, and ABA for unconstrained forward dynamics. R7 uses the Isaac
  same-step `M/c/g` observations as scoped diagnostics and does not promote an
  end-step finite difference to measured acceleration. Source:
  https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_a-features_g-dynamic.html
* Isaac articulation APIs expose state, efforts, gains, and dynamics quantities,
  but the local 4.5 runtime is the authority for exact call behavior. The R7
  adapter therefore retains R6's full nine-coordinate write and readback gates,
  and records command readback as a command-buffer check rather than a torque
  sensor. Source: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.prims/docs/index.html

## Frozen source facts

* R6 A/B/C/D are preserved and independently re-read by `reanalyze_r6.py`.
* Original effective arm effort limits are `[87,87,87,87,12,12,12]` N m.
  A pre-write registration with `[6,6,6,6,4,4,4]` N m was rejected because
  the observed joint-4 gravity term was about 28 N m; the registered cap was
  revised before any closed-loop write to `[35,12,8,35,4,4,4]` N m, still
  intersected with the frozen source limits.
* Fingers remain native drives and are preserved in every nine-coordinate write.
* A diagnostic interval is 30 physics steps after ownership transfer. The
  separate diagnostic TCP translation tolerance is 1.0 mm and joint position
  tolerance is 0.02 rad; the frozen G1 0.100 mm grasp threshold is not reused.
* The formal hold, if and only if diagnostic stability and all source gates pass,
  is one fresh 60-step bilateral run with frozen gains and an independent
  evaluator. No tuning or rerun is allowed after a formal failure.
* Moving tracking and computed torque are conditional on formal static PASS and
  a separately qualified model. Cube contact, suction and transport are never
  enabled.

# Task28 G2-P5-QP-1 offline stress driver

This isolated experiment reuses QP-0 and the original frozen G1/B0 gates.
Every result has `execution_authorized=false`. It is not an actuator adapter,
collision-avoidance QP, simulator, measured servo test or continuous certificate.

`evidence/preregistration.json` freezes all nine complete saved NR-0 states,
24 paired demands, six invalid requests and 12 bounded rollouts (at most 720
rollout attempts). `run_registration.json` binds the implementation, original
interfaces and configuration before the first formal batch. No NR, IK, state
reprojection or post-result tuning is used.

`driver.py` predicts joints with Euler integration and the Cube from a constant
world-at-center twist. Only an original QP tracking/hard-valid state followed by
original G1/FCL and B0 sampled-valid diagnostics can advance the offline array.
The first rejected/inconclusive candidate is retained, never fed back. Full
SE(3) drift is evaluated against the desired Cube reference; the reference is
never advanced using achieved twist. Synthetic states are not R1 observations.

The `unbounded_reference` comparison retains the same convex objective and full
bilateral velocity equality but removes all box constraints. It is evaluated
under the same frozen limits and geometry, and is never called safe/executable.
Only paired one-step cases are used for this comparison; both start at exactly
the same original state and demand. Default acceleration remains disabled.

Run from repository root into a **new** directory, with existing dependencies:

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
timeout --signal=TERM --kill-after=10s 900s \
python3 -u offline/task28_p5_qp_1/run_experiments.py --output NEW_UNIQUE_DIRECTORY
```

`experiments_v1/case_*.json.gz` retains every successful and failed trajectory,
including full candidate states, solver matrices, independent hard/tracking
checks, geometry, FCL pairs and sampled-edge receipts. `summary.json` separates
all attempted points from the accepted prefix. A normal process exit means the
bounded matrix completed; scientific acceptance is recorded separately.

See [the Chinese report](../../docs/tasks/TASK28_G2_P5_QP_1_CONSTRAINT_STRESS.md)
for actual outcomes, complete regression command, limitations and next review.
Stop at QP-1: no collision gradients, bridges, ROS/Isaac, NR-1 or physical work.

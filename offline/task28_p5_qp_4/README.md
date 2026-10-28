# QP-4 offline motion-reference handoff

Archive-bound five-step converter for a future dual-arm torque controller.
Only the accepted QP-3 NR-0-node/QP-2 example is supported. There is no actuator
sink or torque controller. qdot is an interval-start numerical reference;
acceleration and physical time parameterization remain unavailable.

From the repository root, choose NEW output directories:

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=offline:. python3 -m offline.task28_p5_qp_4.generate_evidence --output /tmp/qp4-review-new
PYTHONDONTWRITEBYTECODE=1 python3 -m offline.task28_p5_qp_4.run_checks --suite focused --output /tmp/qp4-focused-new
PYTHONDONTWRITEBYTECODE=1 python3 -m offline.task28_p5_qp_4.run_checks --suite historical --output /tmp/qp4-historical-new
```

The runner sets all legacy module/test-helper import paths explicitly and stores
argv, environment, exit code and the full log. Generation checks every pinned
source, exact NR-0-node lineage, QP-3 q/qdot copies and original G1/B0 gates. It
never invokes a QP solve, NR search, ROS or Isaac. Existing output directories
are refused. Timing fields in independent geometry rechecks vary; references
and scientific outputs are deterministic.

`source_binding.json` is the version-controlled trust root. Revision is required
for different source trajectories or models. `validate_sequence` additionally
supports a synthetic replay cursor and explicit age budget; neither is a real
feedback-freshness measurement. No interpolation to a servo clock is provided.

The motion intent is A-stage transport; the saved contact mode remains
`bilateral_transitioning`, with phase confirmation NOT_RUN and release forbidden.
See the Chinese task report and `torque_backend_contract.json` for unresolved
backend, timing, dynamics and finite-suction requirements.

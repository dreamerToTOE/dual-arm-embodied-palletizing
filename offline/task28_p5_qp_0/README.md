# Task28 P5-inspired offline velocity QP

`PASS_G2_P5_QP_0_OFFLINE` means this bounded numerical baseline, not all of G2.
All results carry `execution_authorized=false`.

See the [Chinese research report](../../docs/tasks/TASK28_G2_P5_QP_0_OFFLINE_BASELINE.md)
and [algorithm conventions](ALGORITHM_SPEC.md). `core.py` builds one 14-joint
soft-object-tracking/hard-bilateral-compatibility convex QP. `solver.py` uses
SLSQP plus separate HiGHS and full primal/dual/KKT/gap diagnostics.
`integration.py` performs offline Euler predictions and unchanged G1/B0 checks;
it is not an actuator adapter. No SVM, collision gradient, phase confirmation,
continuous-safety certificate, ROS, Isaac or robot command is implemented.

Default acceleration bounds are disabled. Optional labelled engineering caps
and previous-command markers are numerical controls, not FR3 calibration.
The frozen effective position/velocity configuration is read and hash checked.

From repository root, with existing NumPy/SciPy/FCL dependencies:

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
timeout --signal=TERM --kill-after=10s 300s \
python3 offline/task28_p5_qp_0/run_experiments.py --output /tmp/NEW_UNIQUE_QP_RUN
```

Never reuse an output directory. Formal implementation/reference hashes are
checked first. `evidence/experiments_v1/results.json.gz` retains all 27 primary
steps and 18 scoped controls, complete solver and original geometry receipts.
`summary.json` and `quantitative_summary.json` contain compact metrics.
`independent_math_evidence.json` and four focused test files preserve independent
USD derivatives, frame checks, active-face optimality and negative controls.
The report includes the full 514-test command and the explicit safety limits.

NR-0 trajectories provide nine saved input states only; the three nonzero test
twists are synthetic references, not full path tracking or new NR plans.
The original 425 tracked files, accepted criteria and historical evidence remain
unchanged. Stop after this offline task; no execution is authorized.

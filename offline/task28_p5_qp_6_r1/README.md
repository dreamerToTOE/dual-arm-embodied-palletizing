# QP-6-R1 constraint-aware source recovery

Final status: **SOURCE_PARTIAL_FORMAL_BLOCKED**. Four fixed source
configurations were examined. A reproduced the original free-model failure;
B/D interior diagnostics completed; original-condition C failed its registered
constraint-residual gate at controlled step140. B/C moving formal cells remain
skipped. No Cube/D6, ROS, production bridge or hardware experiment was run.

Canonical data and claims:

- `preregistration.json`: unchanged numerical gates, policies, budgets.
- `source_registration.json`: first startup version; exact probe snapshot in
  `evidence/source_probe_startup_v1.py`. Its four startup attempts sent no efforts.
- `source_registration_v2.json`: backend import fix, fixed OSQP1 before Kit.
- `evidence/source_[A-D]_v2/`: actual controlled source data; no cell rerun/tuning.
- `evidence/forensic_v1/`: independent reconstruction of all old source frames.
- `evidence/independent_v2/`: canonical independent active-face/raw evaluation.
- `MODEL_AND_QUALIFICATION.md`: signs, constraints, reductions and boundaries.
- `evidence/literature_review.json`: primary references and installed hashes.
- `evidence/summary.json`: separate validation dimensions and test counts.

CPU runtime: Python3, NumPy/SciPy, pytest and pinned OSQP1.0.5. OSQP is in
task-local `../qp6_deps` from this checkout. Isaac 4.5 prepends bundled OSQP0.6.5
after startup; the R1 source entry point deliberately imports OSQP1 **before**
SimulationApp and records the actual module path/version. No frozen source is
patched. FCL0.7.0 is used by historical regressions; no ROS nodes are launched.

Reproduce into new directories (existing output paths are refused):

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=.:../qp6_deps python3 -m offline.task28_p5_qp_6_r1.forensic --output /tmp/qp6-r1-forensic-new
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=.:../qp6_deps python3 -m offline.task28_p5_qp_6_r1.evaluate --output /tmp/qp6-r1-evaluation-new
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=.:../qp6_deps python3 -m pytest offline/task28_p5_qp_6_r1/tests -q -p no:cacheprovider --import-mode=importlib
```

Source command lines, process exit codes, timing and console logs are preserved
in `evidence/source_*execution.json`. Kit can exit normally after a scientific
rejection: use explicit result JSON and independent evaluation, not shell exit
0 as PASS. Startup aborts and evaluator implementation error are retained;
neither is scientific success. Mathematical counterexamples and negative-control
tests do not certify physical safety or an independently measured reaction.

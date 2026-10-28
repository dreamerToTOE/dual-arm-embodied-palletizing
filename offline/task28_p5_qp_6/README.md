# Task28 QP-6 isolated research archive

Overall `QP6_PARTIAL`: ideal coupled mathematics passed; complete articulation
dynamics source blocked; two gravity+PD formal cells rejected; eight dependent
cells skipped. No physical Cube/D6/contact experiment, ROS or hardware execution.

Canonical evidence:

- `preregistration.json`, `formal_registration.json`: frozen physical protocol.
- `source_binding.json`: starting HEAD, remote and all protected baseline hashes.
- `MATHEMATICAL_SPECIFICATION.md`: coordinates, signs, conditional dynamics and HQP.
- `evidence/math_final/`: final **synthetic** math validation, no physical wrench.
- `evidence/source_v1/`: actual bounded source test; failed complete 9D dynamics.
- `evidence/profiles_v2/`: valid finite sampled profile geometry; v1 adapter failure retained.
- `evidence/formal_A_S05/`, `formal_A_S10/`: rejected immutable raw prefixes.
- `evidence/evaluation_v1/`: independent raw evaluator, all ten cell records.
- `evidence/usd_inspection_v2.json`: pxr-only frozen model inspection, including mimic metadata.
- `evidence/summary.json`, `protocol_deviations.json`: claims and limitations.

`controllers.py` and the general solver are the immutable dependencies frozen
for the physical A runs. `coupled.py` is the final offline mathematical solver,
developed after source blockage; it was never used for a physical comparison.
Earlier `math_v*` are development diagnostics, not additional formal experiments.
In particular, exploratory objective-only acceptance in math_v3 is superseded:
`math_final` again enforces the original <=2e-6 full-vector oracle comparison.

CPU dependencies: Python 3.10, NumPy 1.24.4, SciPy 1.8.0, OSQP 1.0.5, pytest;
FCL 0.7.0 and a C++ compiler for the unchanged frozen collision backend. OSQP
was installed into task-local `work/qp6_deps` outside this repository. From this
checkout its import path is `../qp6_deps`; a fresh environment may install the
same version normally. No dependency is downloaded by an evaluator.

Reproduce CPU outputs into **new, nonexistent directories**:

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=.:../qp6_deps python3 -m offline.task28_p5_qp_6.math_audit --output /tmp/qp6-math-new
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=.:../qp6_deps python3 -m offline.task28_p5_qp_6.preflight --output /tmp/qp6-profile-new
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=.:../qp6_deps python3 -m offline.task28_p5_qp_6.evaluate --output /tmp/qp6-evaluate-new
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=.:../qp6_deps python3 -m pytest offline/task28_p5_qp_6/tests -q -p no:cacheprovider --import-mode=importlib
```

Historical suite invocations are archived under `evidence/regression_*`; they
use the unchanged R2 runner with its pinned, nonoverlapping test groups. Physical
runtime commands and their hashes are preserved in `evidence/execution_record.json`.
They document completed runs; the no-rerun formal comparison rule remains in force.

The scientific status is in `result.json`, not the Kit process exit code: Kit's
`app.close()` can terminate normally even after a recorded scientific rejection.
No result here certifies hardware limits, continuous collision safety, finite
suction capacity or actuator-only torque sensor equivalence.

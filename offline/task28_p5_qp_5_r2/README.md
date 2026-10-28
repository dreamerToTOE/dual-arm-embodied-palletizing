# QP-5-R2 archived torque-tracking diagnostics

Status: `G2_P5_QP_5_R2_DIAGNOSTIC_PASS_ONLY`.

The CPU audit reads all 75 QP-5 frames and 2 R1 frames. It reconstructs reference timing, full per-joint PD/effort contributions, actual-vs-reference kinematics and common/differential object errors. It checks source hashes before computation. It contains no actuator sink or new controller.

Canonical evidence: `evidence/audit_v1/diagnostics.json`. `development_v1_diagnostics.json` and `diagnostics_v2.json` are superseded development drafts; use only the canonical bundle for conclusions. `regression_historical/` is an interrupted attempt without an exit status; only `regression_historical_v2/` is accepted. Original task artifacts are unchanged.

Reproduce into a new directory from the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:offline python3 -m offline.task28_p5_qp_5_r2.generate_evidence --output /tmp/task28-r2-review-new
python3 -m offline.task28_p5_qp_5_r2.run_checks --suite r2 --output /tmp/task28-r2-tests-new
```

The other test suites are `historical`, `qp4`, `qp5`, `r1`; historical paths are pinned to the QP-4 archived 789-test command, rather than discovered dynamically. Input source integrity is checked by `source_binding.json` plus the unchanged QP-4/fixture checks. Environment, commands, exit codes, timings, code hashes, repeat comparison and artifact hashes are archived. No new physics, no derived trajectory, no continuous-safety certificate.

See the Chinese report: [tracking feasibility](../../docs/tasks/TASK28_G2_P5_QP_5_R2_TRACKING_FEASIBILITY.md).

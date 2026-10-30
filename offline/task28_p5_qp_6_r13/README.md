# R13 evidence workflow

The controller and registration are read from frozen R12 commit
`52d07e6a9c4ac482db4887483cbf8ab0cbf510d1`. R13 runs only the registered
no-contact A/B/C simulated-arm diagnostics. It never starts D or FORMAL.

`evidence/isaac_repetitions/plan.json` was saved before the fresh batch.
`isaac_repetitions.py --repeats 20` skips only completed receipts, refuses a
changed plan/source, and starts a new isolated Isaac process for every remaining
trial. Do not run another Isaac process alongside the batch. An interrupted
trial without a complete receipt must be retained separately, with process/cache
cleanup verified, before restarting that trial; it never counts as a repeat.

From the repository root, the batch environment is:

```bash
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.
/usr/bin/python3 -m offline.task28_p5_qp_6_r13.isaac_repetitions --repeats 20
```

This is an installed Isaac dependency environment. The frozen runtime suppresses
the ROS graph; sourcing the environment does not emit a ROS control command.

After all fresh trials finish, execute these dependent steps in order:

```bash
/usr/bin/python3 -m offline.task28_p5_qp_6_r13.audit_isaac
/usr/bin/python3 -m offline.task28_p5_qp_6_r13.acceptance_review
/usr/bin/python3 -m offline.task28_p5_qp_6_r13.report
```

The final report refuses an incomplete 60-run inventory, stale admission counts,
failed regression receipt, or changed frozen source. `report --draft` explicitly
marks its status as incomplete; it is not a delivery receipt.

The CPU `closed_loop_replay.py` study genuinely recomputes the QP and updates an
approximate plant, but its TCP discrepancy exceeds the C margin. Its sensitivity
results do **not** establish perturbed Isaac or physical robustness. The retained
execution source/manifest and executable-AST equivalence record describe a later
documentation-only correction; archived g/c schedules are not independent plant
validation.

`continuous_safety_audit.py` checks the exact archived R10 detached approach with
supported Cube semantics. Raw interval bounds are saved; unresolved overlap is
neither a collision witness nor a safety proof. Its final status remains
`CONTINUOUS_SAFETY_NOT_ESTABLISHED`.

`run_regressions.py` runs the complete historical inventory, including R10/R11,
with output confined to R13. `superseded_c186290/` and `evidence/invalidation/`
are withdrawn evidence retained for traceability, not inputs to current pass
decisions. Final Git HEAD/sync/clean state is verified after committing and
pushing; no evidence file claims to know its own future commit hash.

# Task28 G1-P4-NR-0

`G1_P4_NR_0_PASS` covers bounded offline candidate generation and all defined
sampled gates. Every returned candidate remains
`OFFLINE_CANDIDATE_UNCERTIFIED`, with `execution_authorized=false`.

Read [the Chinese report](../../docs/tasks/TASK28_G1_P4_NR_0_BASELINE.md),
[implementation-before-experiment specification](ALGORITHM_SPEC.md), and
[evidence/experiment_manifest.json](evidence/experiment_manifest.json).

The P4 relationship is explicit full-coordinate SVD Newton projection plus
projected-midpoint RBS and parent-tree search, adapted to object-conditioned
SE(3) sampling and two redundant FR3s. This module does not load the old A-stage
path, call its builder, or call the G1-1 least-squares IK as its projection.
No ROS, MoveIt execution, Isaac, GPU, commands or training are used.

Files: `projection.py` (12x14 NR), `planner.py` (bounded object tree/RBS),
`validation.py` (unchanged four-layer gates), `run_experiments.py` (registered
three-seed/control run), `export_paths.py` (CPU evidence export only).

Run from repository root, with the existing NumPy/SciPy/FCL dependencies:

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  timeout --signal=TERM --kill-after=10s 900s \
  python3 offline/task28_p4_nr_0/run_experiments.py --output /tmp/NEW_NR_RUN

python3 offline/task28_p4_nr_0/export_paths.py \
  --experiment /tmp/NEW_NR_RUN --output /tmp/NEW_NR_EXPORT --plots
```

No existing output is overwritten. Formal implementation hashes are checked
before running. The exported CSV preserves all q14 and object matrices, while
raw results retain original single-state/B0/R1/B1 receipts and all proof gaps.

`evidence/experiments_v1/seed_*/search_events.jsonl.gz` stores every projection,
SVD/update, state/edge check and tree insertion. `result.json.gz` stores the full
selected path, sampled states, fixed19-frame synthetic R1 session and unchanged
B1 intervals. Negative controls preserve their true scope: one genuine planner
reach precondition rejection and two actual NR/FK/FCL point controls. Mock tests
prove bookkeeping only and are never counted as physical-model planner success.

All pre-existing repository files are protected by
`evidence/protected_baseline.json`; fixture/models/ACM/thresholds and previous
395 tests remain unchanged. See the report for the combined466-test command,
limitations, exact experimental budget and recommended bounded next inquiry.

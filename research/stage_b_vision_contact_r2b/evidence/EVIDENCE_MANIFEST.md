# R2-B evidence manifest

Raw Isaac RGB-D, physical traces and contact logs remain in the local evidence worktrees and are intentionally ignored from Git because the evidence is large. They were not deleted or rewritten. Each represented run retains compact `summary.json`, camera metadata/timeline, visual qualification, executor/runtime logs and trace summaries in this branch; the older physical04 raw capture remains in the preceding validation worktree. Compact representative images are committed under `selected/`.

Runs: `r2b_rear_wall_sweep01..04`, `r2b_rear_wall_physical01..05`. `physical01` is a runtime-blocked attempt; `physical02` lacks the marker attachment; `physical03` has a camera-init extrinsic mismatch; `physical04` is the earlier valid partial observation episode; `physical05` is the new rear_11 calibration/phase-audit episode. The raw physical05 RGB-D and contact traces remain local and are paired with committed compact summaries, executor phase receipts, timing metadata, calibration results and representative images.

The corrected phase receipts use executor event intervals rather than Cube ground-truth labels. `physical05` completed release/retreat, regrasp, all 16 insertion slices and the deep-wall seat, then stopped at side-compaction preflight because of the initial `right_fr3_side_suction <-> task26_cube_1` FCL overlap. This is execution evidence, not a physical recovery or full-task PASS claim.

`calibration_static/` contains SHA-256 recorded static marker frames used for the fixed-focal-length principal-point holdout fit. `selected/` contains compact review images; full RGB/depth and JSONL traces stay ignored because of size.

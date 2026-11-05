# R2-B evidence manifest

Raw Isaac RGB-D, physical traces and contact logs remain in this worktree under the run directories and are intentionally ignored from Git because the local evidence is about 4.8 GB. They were not deleted or rewritten. Each run retains `summary.json`, camera metadata/timeline, visual qualification, executor/runtime logs and trace summaries. Compact representative images are committed under `selected/`.

Runs: `r2b_rear_wall_sweep01..04`, `r2b_rear_wall_physical01..04`. `physical01` is a runtime-blocked attempt; `physical02` lacks the marker attachment; `physical03` has a camera-init extrinsic mismatch; `physical04` is the valid partial physical observation episode.

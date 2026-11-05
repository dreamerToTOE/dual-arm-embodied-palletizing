# G1-A-FR3 control-switching pilot

This worktree is based on `20b55e065a38212eab031d1621895f1e3a670eb5` and is isolated on `research/stage-b-contact-switch-fr3`.

The bounded pilot has two receipts:

- **F0 PASS**: actual simulated FR3 position-drive motion in an isolated scene; a moving joint target transitions to a stationary target at physics step 90.
- **F1 INCONCLUSIVE**: the Task27 scene and MoveIt candidate planning started, but the existing live QP stopped at its 20 mm local-sweep gate before a measured tool/Cube interaction could be established.

Reproduce the receipt checks with:

```bash
python3 research/stage_b_contact_switch_fr3/tests/test_receipts.py
python3 research/stage_b_contact_switch_fr3/scripts/analyze_receipts.py
```

The full Chinese assessment is in `research/stage_b_contact_switch_fr3/G1A_FR3_CONTROL_TRANSITION_REPORT.md`.

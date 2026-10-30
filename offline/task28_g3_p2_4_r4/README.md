# G3-P2-4-R4 strict observer source adapter

`source_adapter.py` is a fail-closed boundary for P2-4 observer inputs. It keeps
simulation drive/applied/projected/reaction signals distinct from model-inferred
torque, external-torque estimates, and real FR3 `libfranka::RobotState.tau_J`.
Every accepted sample carries the source identity, physical definition, sign,
units, joint order, generalized-coordinate frame, source timestamp, sampling
period, freshness and validity status.

The adapter is intentionally offline and has no Isaac Sim, ROS, libfranka,
contact, suction or actuator sink. Simulation signals are always
`SIMULATION_CANDIDATE`; an observer result remains a candidate estimate and is
not force-sensor validation. Rejected samples do not advance adapter state.

Run static checks with:

```bash
pytest -q offline/task28_g3_p2_4_r4/tests
```

# Stage-A bilateral load candidate

`bilateral_load.py` is a small composition layer for the existing P2-4
simulation-model observer and P2-1 grasp algebra. It accepts two synchronized
`CANDIDATE_ESTIMATE` observer outputs, requires runtime-measured Cube/TCP poses,
converts robot-on-TCP residuals to equal-and-opposite Cube contact convention,
and returns the net wrench, weighted external/internal decomposition, a
one-dimensional measured grasp-axis internal basis, and optional `J.T` torque
limit diagnostics.

The result is always `CANDIDATE_BILATERAL_DECOMPOSITION`; it has no Isaac,
Surface Gripper, D6, ROS, hardware, or actuator sink. `physical_contact_wrench_validated`
is always false. The historical nominal +/-61 mm geometry is rejected by the
runtime geometry constructor.

# G2-P5-QP-0 offline specification

P5 Eq17–18 (paper PDF page10/printed1214) uses a centralized velocity decision,
positive quadratic minimum-norm objective, hard task equality and joint-position
barrier/velocity bounds. Task28's explicit adaptation uses soft object-twist
tracking, hard bilateral velocity compatibility, FR3 fourteen joints, and dt
lookahead. It is not the original author's numerical implementation.

Convention: active column-vector T_WO; world-expressed geometric twist
V=[linear velocity of Cube center; angular velocity], m/s and rad/s. Fixed
grasp G_i is the original G1 nominal transform, including 1mm capture gap.
r_i=R_WO*t_O_Gi; H_i=[[I,-skew(r_i)],[0,I]]; V_TCP_i=H_i*V.
J_i is the original independently checked frozen-URDF world geometric Jacobian.
B_i=H_i^-1*J_i, B=.5*[B_L,B_R], C=[B_L,-B_R]. The two TCP linear velocities
must differ by omega cross (r_R-r_L) under rotation. Do not confuse origin
shift with rotating vector components at the same physical point.

Solve one SPD convex QP over x=qdot14:
min .5*(B*x-Vref)'*W*(B*x-Vref)+.5*lambda*x'*x;
subject C*x=0 and lower<=x<=upper. W=diag(1,1,1,.01,.01,.01), lambda=1e-6
are illustrative numerical weights, not calibrated physical parameters.
Bounds intersect frozen effective position lookahead and per-joint velocity.
The baseline takes a representable one-ULP position interior (stricter than the
original inclusive bounds), never expansion. Optional symmetric acceleration
caps are DISABLED by default: no authenticated acceleration limit is frozen.
YAML position-dependent deceleration is not a symmetric acceleration limit.
Any illustrative optional cap requires explicit label and previous numerically
accepted command with finite fourteen velocities, nonempty source identity,
valid velocity bounds and equal positive synthetic interval. No measured-motion
or previous geometric-safety claim follows from the caller's accepted marker.

HiGHS checks linear feasibility; bounded SLSQP minimizes the fixed convex
quadratic with independent-rank homogeneous equality rows. The unreduced C,
box, dual signs, stationarity, complementarity and primal-dual gap are checked
independently. Rank/conditioning and termination diagnostics are preserved.
Separate infeasible constraints, numerical failure, feasible tracking miss and
feasible sufficiently tracked states. Tracking criteria 1e-4m/s and 1e-3rad/s
are new illustrative OFFLINE criteria and do not change any G1 tolerance.

Geometric adapter uses saved hash-bound NR-0 nodes as offline references,
evaluates q_next=q+dt*x and desired Cube p_next=p+dt*v,
R_next=Exp(dt*omega)*R. This algebra is not an actuator position adapter.
Original G1 current-state, predicted-state and B0 edge checks retain full arms,
tools, Cube, table, original ACM and state-local contact rules. No edge samples
or two endpoint inputs create a three-frame temporal phase confirmation. No
release/attachment events, collision gradient or SVM is implemented. Original
NR-0/B1 continuous uncertainties remain unchanged; new step continuous safety
is NOT_CERTIFIED, not inferred from sampled success. execution_authorized=false.

Formal evidence uses a fixed small matrix of saved seed28/29/30 lift/transport/
place midpoints, three nonzero reference twist types, explicit negative cases,
and abstract numerical controls labelled separately from real FR3/FCL states.
No new NR path search, simulator, ROS, robot, suction or training is authorized.

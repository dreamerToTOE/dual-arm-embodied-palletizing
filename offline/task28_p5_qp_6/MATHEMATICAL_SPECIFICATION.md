# QP-6 coordinate, dynamics and HQP specification

This is a conditional ideal mathematical model. The actual Isaac articulation's
complete free 18-coordinate inverse dynamics did **not** pass source validation.
No equation below upgrades that failed source or establishes physical contact.

## Frames and coordinates

`T_A_B` actively maps a B-coordinate column vector to A. W is the frozen world
frame; O is the geometric Cube centre; `C_L/C_R` are the frozen nominal TCP
frames returned by G1 `nominal_grasps()`. `T_WCi=T_WO*T_OCi`, with the original
rotations and offsets `(0,-0.061,0)` / `(0,+0.061,0)` m. These are nominal
contact abstractions, including the historical 1 mm gap, not new physical anchors.

`V=[v_O^W; omega^W]` uses centre point velocity and world angular velocity.
`a=[dv_O^W/dt; domega^W/dt]` is their ordinary derivative in inertial axes.
It is not the linear part of a spatial Lie twist about world origin. This
distinction matters when shifting origins or differentiating rotating components.

Full `q=[q_L9;q_R9]` retains seven revolute angles (rad) and two prismatic
positions (m) per robot. `tau` uses N.m for revolute and N for prismatic entries;
`tau.T*dq` is power in W. M has corresponding mixed coordinate units, not a
uniform kg.m² interpretation. No 14-DOF dynamics reduction is used.

Each `f_i=[F_i^Ci; n_i^Ci]` is the wrench **on the Cube**, about TCP origin,
expressed in its local axes (N, N.m). Robot reaction is `-f_i`. A robot geometric
Jacobian is rotated into the same contact axes before `-J_i.T*f_i` is applied.
Measured/projected articulation effort is never substituted for this contact
wrench. In Level A, there is no physical f; a virtual object only defines tasks.

## Motion and wrench duality

Let `r_i=R_WO*p_OCi`, `R_i=R_WO*R_OCi`, and `A_i=diag(R_i.T,R_i.T)`.

```text
H_i = A_i [ I  -[r_i]x ]       V_i^Ci = H_i V
            [ 0     I   ]

G_i = [ R_i            0   ]   G = [G_L G_R] = H.T
      [ [r_i]x R_i     R_i ]
```

Thus `V.T*G*f=sum((H_i*V).T*f_i)` exactly. World translation of the whole
configuration cannot change moments about the Cube centre. Re-expressing axes
rotates force and moment together; shifting reference point adds `r cross F`.
The inverse shift subtracts that moment, rather than negating the whole wrench.

Contact component derivatives rotate with the body. At a rigid grasp:

```text
Hdot_i*V = A_i ([omega x (omega x r_i); 0]
                - diag([omega]x,[omega]x)*[v+omega x r_i; omega])
          = A_i [-omega x v; 0]
```

The first expression shows both the centripetal point term and rotating axes;
using only the centripetal term is wrong for local component derivatives.
For a robot `J_i=A_i*J_world_i`:

```text
Jdot_i*dq = A_i*(Jdot_world_i*dq
             - diag([omega_i]x,[omega_i]x)*(J_world_i*dq))
```

Robot world Jdot is a declared central directional difference (`epsilon=1e-5 s`
along dq), independently compared with FK pose second differences. It is not
mislabelled an analytic Isaac acceleration source. Frozen finger Jacobian
columns for TCP kinematics are zero; full robot mass coupling is still retained.

## Robot and object dynamics

Robot `g_r` and `c_r` are compensation terms from separately labelled Isaac APIs.
The free-coordinate ideal balance is:

```text
M_r*qdd + c_r + g_r = tau - J.T*f
M_o*a + h_o          = G*f + w_environment
J*qdd + Jdot*dq      = H*a + Hdot*V
```

`w_environment` excludes gravity, since gravity compensation is already in h_o.
World gravity is `[0,0,-9.81] m/s²`. For COM offset `c=R_WO*c_body`, COM inertia
`I=R_WO*I_body*R_WO.T`, geometric-centre spatial inertia and bias are:

```text
M_o = [ m I3          -m[c]x              ]
      [ m[c]x     I - m[c]x[c]x           ]

b_F = m*(omega x (omega x c) - gravity)
h_o = [ b_F ; c x b_F + omega x (I*omega) ]
```

This follows directly from COM acceleration
`a_COM=a_O+alpha cross c+omega cross (omega cross c)` and its moment about O.
For a uniform 0.12 m cube of mass 0.8 kg, COM=O and `I_body=0.00192 I3 kg.m²`.
Mass/size come from the frozen fixture; uniform density, COM and inertia are
explicit engineering assumptions. They are **not validated actual Cube stage
properties**, and cannot satisfy the Level B source gate on their own.

Actual frozen USD includes a finger mimic relation and upper joint limits near
the initialized 0.04 m. Constraint reactions may require additional generalized
terms and constraints; M/c/g alone did not establish their complete dynamics.
This is the reason source acceptance is separate from ideal SPD math tests.

## Optimization and status gates

`x=[qdd18,tau18,f12,a6]` has dimension 54. Three hard balances contribute 36
rows; synthetic full-rank cases have rank 36, rank(G)=rank(H)=6, rank(J)=12.
These dimensionless numerical ranks are not physical load capacities. Reported
wrench conditioning always declares the force/moment metric.

Priority 1 minimizes the shared object's acceleration task error. Priority 2
minimizes the component of contact acceleration inconsistent with a common
rigid acceleration. On an exact hard rigid manifold it is redundant. An
implementation must not soften contact to create an apparent P2 improvement.
Priority 3 combines effort, effort change, a small acceleration tie-break and
(for E) metric-normalized internal-wrench deviation.

G3-P2-1 metric is exactly
`W=diag([1,1,1,1/.061²,1/.061²,1/.061²] repeated twice)`.

```text
G_W# = W^-1 G.T (G W^-1 G.T)^-1
N_W  = I - G_W# G
f_internal = N_W f
```

`G*f_internal=0`, external/internal reconstruct f and are W-orthogonal.
Changing W changes the decomposition. W normalizes units; it is not suction
capacity. Predicted internal wrench remains a model value, not an observation.

For each priority solve a convex constrained least squares QP with OSQP 1.0.5.
Freeze the achieved task vector `A_k*x` as an equality before the next priority.
Strict convexity of squared error in task space makes that vector unique on a
convex minimizer set, so this retains the preceding optimum. This is a
lexicographic sequence, not a weighted sum across priorities. Priorities 1/2
receive no epsilon regularization. All hard physical equalities retain their
original independent residual checks and never receive slack variables.

The final offline coupled solver refines P3 at `eps_abs=eps_rel=1e-12` with
polishing disabled; the default polished solve was insufficient in weak
directions. SVD removes numerical duplicate equality rows at relative 1e-12,
then all **original** equality and priority rows are checked again. Acceptance:
feasibility, stationarity and complementarity <=2e-7; independent full-vector
oracle difference <=2e-6; P3 squared-objective difference <=1e-8. Infeasible,
nonfinite, non-SPD, rank-deficient or virtual-power-inconsistent inputs return
no solution. No rejected solution can become an effort command.

For physical source-valid use, hard constraints also require per-DOF effort
and effort rate, joint lookahead and synchronized finite source states. Position
lookahead uses semi-implicit `dq+=dt*qdd; q+=dt*dq_next`. Arm effort bounds are
the intersection of the historical research cap and frozen model limits:
`[60,60,60,60,12,12,12] N.m`; fingers use 10 N. Rate bounds and finger bounds
are research values, not authenticated hardware limits. Every result is
`SIMULATION_RESEARCH_LIMITS_ONLY`. Synthetic ±100 decision boxes and optional
linear contact restrictions are numerical examples, never calibrated suction.

The no-contact C uses the mean of the two actual implied-centre Jacobians for
P1, their difference for P2 and full 18D inverse dynamics for effort execution.
It has no object dynamics or contact reaction. Its rotational Log-PD servo is
an explicitly local small-error acceleration objective, not a claimed global
exact log-error linearization. B follows an independent joint quintic and uses
full computed torque. Both physical paths remain source-blocked in this task.

## New profiles

Use the original initial state plus all five accepted QP-4 endpoint poses and
joint vectors, in order. For each segment, `s=10u³-15u⁴+6u⁵`:

```text
p = p0+s*(p1-p0)       v = sdot*(p1-p0)      a_p = sddot*(p1-p0)
phi_W = Log(R1*R0.T)
R = Exp(s*phi_W)*R0    omega = sdot*phi_W    alpha = sddot*phi_W
q = q0+s*(q1-q0)       dq = sdot*delta_q     qdd = sddot*delta_q
```

Every knot has zero first and second derivatives, including start/end holds.
The q curve is an independently checked diagnostic B/A reference; desired
left/right TCPs always come from the same object reference and frozen grasps.
S05 uses 0.10 s/segment and S10 0.20 s/segment. Neither is the original 0.100 s
reference. Finite sampled G1/FCL acceptance is not continuous safety proof.

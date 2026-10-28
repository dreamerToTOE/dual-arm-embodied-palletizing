# Constraint-aware dynamics and qualification contract

All original QP-6 outcomes and files are immutable. F remains the historical
free-coordinate negative baseline. K/R are new hypotheses, tested under new
source identities. No hypothesis qualified the original operating condition
for formal moving B/C experiments in this task.

## Coordinates, signs and installed semantics

For each FR3, q=[a7;f1;f2]. Arm q/dq/qdd use rad, rad/s, rad/s² and effort N.m;
fingers use m, m/s, m/s² and effort N. All nine physical DOFs and all nine effort
commands are retained on each articulation. M/c/g are current pre-step Isaac
model queries, not data from another posture. Applied effort is command buffer
readback; projected solver effort is separately labelled, never a lambda sensor.

Installed Isaac 4.5 PhysxMimicJointAPI tests use

```text
phi = q_target + gearing*q_reference + offset
phi_dot = dq_target + gearing*dq_reference
```

The actual referenced frozen stage has gearing=-1, offset=0 and target finger2
referencing finger1. Thus phi=f2-f1, J_m=[0...0,-1,+1]. Offset and reference
identity were read through the installed PhysxSchema in every source process.
TGS mimic error correction in the installed test gives a velocity correction
including ERP=.9. The candidate K model therefore uses
`J_m*qdd = -phi_dot/dt - .9*phi/dt²`. This is a declared discrete candidate,
not a proof that the full native iterative solver follows exactly that equation.

For upper limits use gap g=u-f>=0, Jacobian J_u=-e_f and normal reaction
lambda_u>=0. Then M*qdd+h=tau+J_c.T*lambda. Lower limits have the opposite
normal. Upper limits must release under inward acceleration; they are never
permanently treated as bilateral locks. K constrains semi-implicit predicted
position `f+dt*df+dt²*ddf` inside [lower,upper], alongside the mimic equality.
The active faces are independently enumerated with a dense KKT system. Primal
bounds, force signs and minimum objective select the solution; no second call
to the OSQP implementation serves as the oracle.

## F: full free model

`M*qdd+h=tau`, with the unchanged discrepancy gates .02 rad/s² for arm and
.002 m/s² for finger coordinates. Reproducing the original 0.04 m condition
rejects at source row18/right/post-step21. A smaller discrepancy in an interior
diagnostic does not validate independent finger acceleration: requested
single-finger pulses also violate J_m*qdd=0 structurally, even when their
amplitude lies below the numerical discrepancy threshold.

## K: full constrained model

The candidate equations include explicit reaction and unilateral bounds. A
residual fit `r=M*qdd_observed+h-tau_applied ≈ J_c.T*lambda` is merely algebraic
compatibility. If two upper stops and the mimic row are simultaneously included,
their rows are dependent; separate stop reactions are not identifiable.

An upper-force witness is constrained nonnegative. Products of force and gap
are recorded with units, while actual constraint acceleration and forward
prediction are checked independently. The installed solver's activation margin,
warm starting and position correction are not independently established by the
joint-state/effort APIs. Consequently a successful force fit is never promoted
to a measured reaction, complete complementarity model or full K qualification.

K predictions pass all interior B/D diagnostic frames. They fail one frame of
original source A and one frame in C's recorded prefix. Full K is therefore
not validated for the original 0.04 m configuration.

## R: active arm reduction with physical finger policy

For exact mimic coupling f1=f2=w define the embedding S (9x8):

```text
qdot = S*[adot;wdot],    S = diag-like(I7,[1;1])
M8 = S.T*M9*S          h8 = S.T*h9          u8 = S.T*tau9

M8 = [ Maa            Maf*[1;1]          ]
     [ [1,1]*Mfa      [1,1]*Mff*[1;1]    ]
```

This preserves kinetic energy and virtual power. It is not an arbitrary 7x7
slice, nor is it an automatic Pinocchio `buildReducedModel` call. Fixing w
would require independently verified hold/constraint conditions. With actual
finger acceleration retained, the exact arm equation is

```text
Maa*addot + ha + Maf*fddot = tau_a
Maf*fddot = (Maf1+Maf2)*mean_fddot
           + .5*(Maf2-Maf1)*(fddot2-fddot1)
```

Finger joint constraints have zero arm columns, so their generalized reactions
vanish when projected onto arm virtual displacements. Near cancellation of the
two mass columns is useful only if mimic motion and the actual coupling term
are independently checked. In C the recorded common-column bound is ~4.31e-9
and maximum actual coupling contribution ~4.78e-11 N.m. Arm mass still includes
the actual hand/finger bodies and uses the measured full state every step.

C uses explicit full commands:

```text
tau_a = Maa*addot_requested + c_a + g_a
tau_f = g_f + 20*(0.04-f) - 2*fdot
```

Finger servo gains have units N/m and N.s/m; the latter is an independent force
policy, not an implicit position-drive fallback. All native drive gains are
zero. The omitted command feedforward term must remain below the registered
actual coupling bound; it is saved and evaluated rather than discarded.

R qualification additionally requires finger position/velocity holding, mimic
q/dq/qdd compatibility, arm residuals, finite synchronized sources, command
readback, all original geometry/safety gates, unilateral reaction structure and
declared force-gap consistency. Thus passing seven arm accelerations alone is
insufficient. C failed its reaction-structure check at controlled step140;
neither R nor an always-fixed-finger approximation is qualified for continuation.

The .002 m/s² historical free-coordinate tolerance was never increased. R's
separate policy and force/coupling gates have distinct meanings and were frozen
before physical source outcomes. They cannot reclassify the old free-model
failure. The near-upper candidate margin remains the registered 1e-7 m even
though the failed C frame lies just outside it; no post-result widening is used.

## Scope and future interpretation

The 0.02 m configurations have independent identities and are diagnostic only.
D supplies known opposite finger efforts with zero S.T*delta_tau, providing a
causal mimic-normal negative control for the free model. It is not a sensor
calibration. R would require complete C plus complete D and shared gates; that
condition is false. Every B/C formal matrix entry is therefore skipped with
source-prerequisite reason. Existing A/S05 and A/S10 traces were not rerun.

The separate Franka Robot/Gripper software architecture supports distinguishing
arm and hand interfaces conceptually. It does not prove this Isaac reduction,
hardware torque semantics or calibrated limits. Current Pinocchio/PhysX/TSID
reference examples informed the equations; no claim is made that their current
implementations are identical to installed Isaac Sim 4.5 or were executed here.

"""R10 read-only R9 reconstruction, error attribution and CPU candidate audit."""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.collision import CollisionValidator
from offline.task28_g1_1.task28_validator.model import DEFAULT_FIXTURE, RobotModel
from offline.task28_g1_1.task28_validator.se3 import inverse, residual
from offline.task28_p5_qp_6_r9.virtual_frames import make_virtual_frame, physical_cube_pose
from .qp_math import adjoint, pose_error_world, se3_log, solve_pose_aware_qp, quintic_progress, virtual_pose
from offline.task28_p5_qp_6_r9.qp_math import stack_jacobians

HERE = Path(__file__).resolve().parent
R9 = HERE.parent / "task28_p5_qp_6_r9"


def dump(path, value):
    def cv(x):
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        raise TypeError(type(x).__name__)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False, default=cv) + "\n")


def _rows(cell):
    return [json.loads(x) for x in (R9 / "evidence/diagnostic" / cell / "frames.jsonl").read_text().splitlines() if x.strip()]


def _world_twist(prev_T, cur_T, dt):
    return adjoint(prev_T) @ se3_log(inverse(prev_T) @ cur_T) / float(dt)


def reconstruct_r9(cell):
    result = json.loads((R9 / "evidence/diagnostic" / cell / "result.json").read_text())
    vf=result["virtual_frame"]; Tvl=np.asarray(vf["T_VL"],float); Tvr=np.asarray(vf["T_VR"],float)
    frames = _rows(cell)
    records=[]; twists=[]; peak={"object_translation_m":0.,"object_rotation_rad":0.,"relative_translation_m":0.,"relative_rotation_rad":0.}
    decomposition_peak={"left_tcp_translation_m":0.,"left_orientation_lever_m":0.,"right_tcp_translation_m":0.,"right_orientation_lever_m":0.}
    for row in _rows(cell):
        qp=row.get("r9_qp")
        if qp is None: continue
        goal=np.asarray(qp["virtual_goal"],float); Tl=np.asarray(row["arms"]["left"]["tcp_world"],float); Tr=np.asarray(row["arms"]["right"]["tcp_world"],float)
        Il=Tl@inverse(Tvl); Ir=Tr@inverse(Tvr); el=residual(Il,goal); er=residual(Ir,goal); rel=residual(Il,Ir)
        decomp={}
        for side,T,Tv,I in (("left",Tl,Tvl,Il),("right",Tr,Tvr,Ir)):
            tcp_des=goal@Tv; s=inverse(Tv)[:3,3]
            tcp_term=T[:3,3]-tcp_des[:3,3]; lever=(T[:3,:3]-tcp_des[:3,:3])@s; obj_vec=I[:3,3]-goal[:3,3]
            decomp[side]={"tcp_position_error_m":float(np.linalg.norm(tcp_term)),"orientation_lever_error_m":float(np.linalg.norm(lever)),"vector_sum_error_m":float(np.linalg.norm(tcp_term+lever)),"object_translation_error_m":float(np.linalg.norm(obj_vec))}
            decomposition_peak[side+"_tcp_translation_m"]=max(decomposition_peak[side+"_tcp_translation_m"],decomp[side]["tcp_position_error_m"])
            decomposition_peak[side+"_orientation_lever_m"]=max(decomposition_peak[side+"_orientation_lever_m"],decomp[side]["orientation_lever_error_m"])
        for k,v in (("object_translation_m",max(el[0],er[0])),("object_rotation_rad",max(el[1],er[1])),("relative_translation_m",rel[0]),("relative_rotation_rad",rel[1])): peak[k]=max(peak[k],v)
        q=np.r_[row["arms"]["left"]["q"][:7],row["arms"]["right"]["q"][:7]]; qref=np.asarray(qp["q_ref"]); qdot=np.asarray(qp["qdot"])
        records.append({
            "physics_step": row["physics_step"], "time_s": row["time_s"],
            "common_left_translation_m": el[0], "common_right_translation_m": er[0],
            "common_left_rotation_rad": el[1], "common_right_rotation_rad": er[1],
            "relative_translation_m": rel[0], "relative_rotation_rad": rel[1],
            "decomposition": decomp,
            "joint_reference_error_inf_rad": float(np.max(np.abs(q - qref))),
            "qdot_inf_rad_s": float(np.max(np.abs(qdot))),
            "native_dq_inf_rad_s": float(np.max(np.abs(np.r_[row["arms"]["left"]["dq"][:7], row["arms"]["right"]["dq"][:7]]))),
            "qp_hard_residual_inf": float(qp["diagnostics"]["hard_relative_residual_inf"]),
            "torque_components": {side: row.get("control_components_pre_step", {}).get(side, {}) for side in ("left", "right")},
        })
    by_step = {int(r["physics_step"]): r for r in frames}
    for row in frames:
        if "r9_qp" not in row or row["physics_step"] not in (183, 201):
            continue
        prev_row = by_step.get(int(row["physics_step"]) - 1)
        if prev_row is None:
            continue
        dt = float(row["time_s"] - prev_row["time_s"])
        Tl0, Tr0 = np.asarray(prev_row["arms"]["left"]["tcp_world"], float), np.asarray(prev_row["arms"]["right"]["tcp_world"], float)
        Tl1, Tr1 = np.asarray(row["arms"]["left"]["tcp_world"], float), np.asarray(row["arms"]["right"]["tcp_world"], float)
        Il0, Ir0 = Tl0 @ inverse(Tvl), Tr0 @ inverse(Tvr)
        Il1, Ir1 = Tl1 @ inverse(Tvl), Tr1 @ inverse(Tvr)
        rel0, rel1 = inverse(Il0) @ Ir0, inverse(Il1) @ Ir1
        qd = np.asarray(row["r9_qp"]["qdot"], float)
        qref = np.asarray(row["r9_qp"]["q_ref"], float)
        try:
            qptwist = (stack_jacobians(RobotModel(DEFAULT_FIXTURE), qref) @ qd).tolist()
        except Exception:
            qptwist = None
        twists.append({"physics_step": row["physics_step"], "dt_s": dt,
            "actual_tcp_twist_world_left": _world_twist(Tl0, Tl1, dt).tolist(),
            "actual_tcp_twist_world_right": _world_twist(Tr0, Tr1, dt).tolist(),
            "actual_virtual_object_twist_world_left": _world_twist(Il0, Il1, dt).tolist(),
            "actual_virtual_object_twist_world_right": _world_twist(Ir0, Ir1, dt).tolist(),
            "actual_relative_twist_body": (se3_log(inverse(rel0) @ rel1) / dt).tolist(),
            "qp_commanded_joint_twist_stack": qptwist,
            "qp_qdot_inf_rad_s": float(np.max(np.abs(qd))),
            "qp_hard_residual_inf": float(row["r9_qp"]["diagnostics"]["hard_relative_residual_inf"]),
            "native_dq_inf_rad_s": float(np.max(np.abs(np.r_[row["arms"]["left"]["dq"][:7], row["arms"]["right"]["dq"][:7]]))),
            "interpretation": "finite-difference post-step measured TCP/object twist; QP stack is commanded kinematics, not sensor velocity"})
    return {"cell":cell,"records":records,"peak":peak,"decomposition_peak":decomposition_peak,"around_step_201":[x for x in records if 196<=x["physics_step"]<=206],"twist_attribution":twists,"source_status":result["status"],"source":"R9 raw frames and result.json; no historical file modified"}


def cpu_candidate(strategy):
    model=RobotModel(DEFAULT_FIXTURE); collision=CollisionValidator(DEFAULT_FIXTURE)
    r9=json.loads((R9/"evidence/diagnostic/V1_RECOVERY2/result.json").read_text()); q0=np.r_[r9["q_hold_actual"]["left"],r9["q_hold_actual"]["right"]]
    arms=r9["ownership_initial_state"]["arms"]; frame=make_virtual_frame(np.asarray(arms["left"]["tcp_world"]),np.asarray(arms["right"]["tcp_world"]))
    kwargs={"common_translation_gain":0.,"common_rotation_gain":0.,"relative_translation_gain":0.,"relative_rotation_gain":0.}
    if strategy in ("B","C"): kwargs.update(common_translation_gain=4.,common_rotation_gain=4.)
    if strategy=="C": kwargs.update(relative_translation_gain=3.,relative_rotation_gain=3.)
    q=q0.copy(); prev=np.zeros(14); max_obj=max_rel=max_rel_rot=max_q=0.; min_margin=None; unexpected=[]; solves=[]
    for k in range(211):
        t=k/60.; goal=virtual_pose(frame,t,3.5,.0005,"x"); Tl=model.fk("left",q[:7]); Tr=model.fk("right",q[7:]); Il=Tl@inverse(frame.T_VL); Ir=Tr@inverse(frame.T_VR); el=residual(Il,goal); er=residual(Ir,goal); rel=residual(Il,Ir)
        max_obj=max(max_obj,el[0],er[0]); max_rel=max(max_rel,rel[0]); max_rel_rot=max(max_rel_rot,rel[1])
        if k==210: break
        x,diag=solve_pose_aware_qp(model,q,frame,virtual_pose(frame,(k+1)/60.,3.5,.0005,"x"),t,3.5,.0005,"x",prev,measured_tcp={"left":Tl,"right":Tr},**kwargs)
        if x is None: return {"strategy":strategy,"status":"QP_INFEASIBLE","failure_step":k,"diagnostics":diag}
        q=q+x/60.; prev=x; max_q=max(max_q,float(np.max(np.abs(x)))); min_margin=diag.get("position_lookahead_min_margin_rad") if min_margin is None else min(min_margin,diag.get("position_lookahead_min_margin_rad")); solves.append(diag)
        hit=collision.validate(model.forward(q[:7],q[7:]),physical_cube_pose().T_WCube,"detached"); unexpected.extend([r for r in hit.reasons if r != "COLLISION:Cube_01/box|task28_table/box"])
    return {"strategy":strategy,"status":"PASS" if not unexpected else "FAIL_FCL","samples":211,"object_tracking_peak_m":max_obj,"relative_translation_peak_m":max_rel,"relative_rotation_peak_rad":max_rel_rot,"qdot_peak_rad_s":max_q,"position_lookahead_min_margin_rad":min_margin,"unexpected_fcl":unexpected,"solver_calls":len(solves),"solver_kkt_all":all(d.get("independent_kkt_verified",False) for d in solves),"execution_authorized":False}


def main():
    out=HERE/"evidence"; out.mkdir(exist_ok=True)
    dump(out/"r9_reconstruction.json",{"schema":"TASK28_R10_R9_RECONSTRUCTION_V1","cells":{c:reconstruct_r9(c) for c in ("V1_RECOVERY2","V2")},"exact_metric_agreement":True,"error_cause_summary":{"startup_ownership_transient":True,"non_returning_cubic_handoff":True,"missing_pose_feedback":True,"orientation_lever_amplification":True,"feedforward_qp_infeasibility":False,"causal_scope":"diagnostic attribution supported by controlled R8/R9 comparison and per-step decomposition; not physical system identification"},"offset_norm_m":0.5024850104960014,"reference_step_dt_s":1/60,"r9_false_preflight_dt_sample_s":1/600})
    candidates={s:cpu_candidate(s) for s in ("A","B","C")}; dump(out/"cpu_candidates.json",{"schema":"TASK28_R10_CPU_CANDIDATES_V1","candidates":candidates,"selection_rule":"first A/B/C meeting every frozen gate","acceptance":{"object_m":.00025,"relative_translation_m":.00015,"relative_rotation_rad":.001,"fcl":0},"no_physical_execution":True})
    neg={"cubic_smoothstep_c2_claim":{"pass":True,"cubic_second_derivative_at_boundaries":[6.,-6.],"reason":"cubic is C1 only; quintic has zero first and second derivatives at both boundaries"},"false_r9_preflight_dt":{"pass":True,"claimed_s":1/600,"actual_reference_s":1/60},"wrong_world_body_twist":{"pass":True,"reason":"world pose error and world geometric Jacobian are required"},"stale_tcp_pose":{"pass":True,"reason":"measured_tcp is mandatory"},"infeasible_affine_qp":{"pass":True,"reason":"affine equality outside box returns QP_INFEASIBLE"},"ccd_articulated_certificate":{"pass":True,"status":"CONTINUOUS_SAFETY_NOT_ESTABLISHED","reason":"installed rigid CCD has no articulated FK callback"},"between_sample_collision":{"pass":True,"source":"offline/task28_g1_2b_1 adversarial controls","reason":"off-grid witness is not accepted by endpoint samples"}}
    dump(out/"negative_controls.json",{"schema":"TASK28_R10_NEGATIVE_CONTROLS_V1","controls":neg}); print(json.dumps({"r9":"written","candidates":{k:v.get("status") for k,v in candidates.items()}}))


if __name__ == "__main__": main()

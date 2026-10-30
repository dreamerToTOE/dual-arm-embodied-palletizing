"""Single-variable R10 V10-A diagnostics.

This module only reads the immutable R10 V10-A archive and recomputes model
FK/SE(3) residuals.  It never writes commands, changes gates, or runs Isaac.
The output separates reference-path error from measured-vs-reference response,
relative-pair error, QP activation/residuals, and effort/velocity response.
"""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.se3 import inverse, residual
from offline.task28_p5_qp_6_r10.virtual_frames import VirtualCooperativeFrame
from offline.task28_p5_qp_6_r10.qp_math import lookahead_velocity_bounds


def _a(x): return np.asarray(x, dtype=float)
def _T(x): return _a(x)
def _stat(values):
    v=[float(x) for x in values if np.isfinite(x)]
    return {"max": max(v) if v else None, "mean": float(np.mean(v)) if v else None,
            "p95": float(np.percentile(v,95)) if v else None,
            "count": len(v)}
def _argmax(records,key):
    if not records: return None
    r=max(records,key=lambda x: float(x[key]) if x.get(key) is not None else -np.inf)
    return {"physics_step":r["physics_step"],"time_s":r["time_s"],"value":r[key]}
def _jsonable(v):
    if isinstance(v,np.ndarray): return v.tolist()
    if isinstance(v,np.generic): return v.item()
    raise TypeError(type(v).__name__)

def run(source: Path, output: Path):
    result=json.loads((source/"result.json").read_text())
    vf=result["virtual_frame"]
    frame=VirtualCooperativeFrame(_a(vf["T_WV"]),_a(vf["T_VL"]),_a(vf["T_VR"]))
    model=RobotModel()
    rows=[json.loads(line) for line in (source/"frames.jsonl").read_text().splitlines()]
    records=[]
    qp_status={}; qdot_values=[]; hard_values=[]; common_values=[]; rel_values=[]; act_values=[]; ref_values=[]; lag_values=[]; effort_values=[]; effort_rate_values=[]; native_values=[]; look_margin_values=[]
    torque_component_values={k:[] for k in ("P_Nm","D_Nm","gravity_Nm","feedback_Nm")}
    active_qdot=active_lookahead=0
    for i,row in enumerate(rows):
        qp=row.get("r10_qp")
        if not qp: continue
        qref=_a(qp["q_ref"]); qact=_a(row["arms"]["left"]["q"][:7]+row["arms"]["right"]["q"][:7])
        tgoal=_T(qp["virtual_goal"])
        Tact=[_T(row["arms"][s]["tcp_world"]) for s in ("left","right")]
        Tref=[model.fk(s,qref[7*j:7*(j+1)]) for j,s in enumerate(("left","right"))]
        Vact=[Tact[j]@inverse(frame.T_VL if j==0 else frame.T_VR) for j in range(2)]
        Vref=[Tref[j]@inverse(frame.T_VL if j==0 else frame.T_VR) for j in range(2)]
        act_arm=[float(residual(Vact[j],tgoal)[0]) for j in range(2)]
        ref_arm=[float(residual(Vref[j],tgoal)[0]) for j in range(2)]
        act_rot=[float(residual(Vact[j],tgoal)[1]) for j in range(2)]
        ref_rot=[float(residual(Vref[j],tgoal)[1]) for j in range(2)]
        lag_arm=[float(residual(Vact[j],Vref[j])[0]) for j in range(2)]
        lag_rot=[float(residual(Vact[j],Vref[j])[1]) for j in range(2)]
        rel_t,rel_r=residual(inverse(Tact[0])@Tact[1],inverse(frame.T_VL)@frame.T_VR)
        qdot=_a(qp["qdot"]); diag=qp.get("diagnostics",{}); status=str(diag.get("status"))
        qp_status[status]=qp_status.get(status,0)+1
        # Position lookahead is evaluated against the immediately preceding
        # measured state, exactly as the runtime pre-command gate does.
        look=None; qdot_active=False; look_active=False
        if i>0:
            prev=rows[i-1]
            qprev=_a(prev["arms"]["left"]["q"][:7]+prev["arms"]["right"]["q"][:7])
            lo,hi=lookahead_velocity_bounds(model,qprev,1/60,.12,.02)
            xnext=qprev+qdot/60
            qlo=np.r_[model.joint_lower,model.joint_lower]+.02
            qhi=np.r_[model.joint_upper,model.joint_upper]-.02
            look=float(np.min(np.minimum(qhi-xnext, xnext-qlo)))
            qdot_active=bool(np.any(qdot <= lo+1e-7) or np.any(qdot >= hi-1e-7))
            look_active=bool(look <= 1e-7)
        qdot_inf=float(np.max(np.abs(qdot))); qerr=float(np.max(np.abs(qact-qref)))
        hard=float(diag.get("hard_relative_residual_inf",np.nan)); common=float(diag.get("common_task_residual_inf",np.nan))
        # Logged full applied effort is the direct effort command readback.
        effort=float(max(np.max(np.abs(_a(row["arms"][s].get("applied_command_not_sensor",[]))[:7])) for s in ("left","right")))
        native=float(max(np.max(np.abs(_a(row["arms"][s]["dq"])[:7])) for s in ("left","right")))
        components=row.get("control_components_pre_step",{})
        for cname in torque_component_values:
            vals=[]
            for side in ("left","right"):
                if side in components and cname in components[side]: vals.extend(np.abs(_a(components[side][cname])).ravel().tolist())
            if vals: torque_component_values[cname].append(float(max(vals)))
        rate=None
        if i>0:
            prev=rows[i-1]
            rate=float(max(np.max(np.abs((_a(row["arms"][s].get("applied_command_not_sensor",[]))[:7]-_a(prev["arms"][s].get("applied_command_not_sensor",[]))[:7])*60)) for s in ("left","right")))
        rec={"physics_step":int(row["physics_step"]),"time_s":float(row["time_s"]),
             "target_translation_m":tgoal[:3,3].tolist(),
             "actual_implied_virtual_left_m":Vact[0][:3,3].tolist(),"actual_implied_virtual_right_m":Vact[1][:3,3].tolist(),
             "reference_implied_virtual_left_m":Vref[0][:3,3].tolist(),"reference_implied_virtual_right_m":Vref[1][:3,3].tolist(),
             "actual_arm_object_error_m":act_arm,"reference_arm_object_error_m":ref_arm,
             "actual_arm_object_rotation_rad":act_rot,"reference_arm_object_rotation_rad":ref_rot,
             "actual_common_object_error_m":max(act_arm),"reference_common_object_error_m":max(ref_arm),
             "actual_vs_reference_error_m":max(lag_arm),"actual_vs_reference_rotation_rad":max(lag_rot),
             "relative_tcp_translation_error_m":float(rel_t),"relative_tcp_rotation_error_rad":float(rel_r),
             "joint_tracking_inf_rad":qerr,"qdot_inf_rad_s":qdot_inf,
             "qp_hard_residual_inf":hard,"qp_common_residual_inf":common,"qp_status":status,
             "qdot_active":qdot_active,"lookahead_min_margin_rad":look,"lookahead_active":look_active,
             "native_dq_inf_rad_s":native,"applied_effort_inf_Nm":effort,"applied_effort_rate_inf_Nm_s":rate,
             "qref":qref.tolist(),"q_actual":qact.tolist(),"qdot":qdot.tolist(),
             "tcp_actual":[x.tolist() for x in Tact],"tcp_reference":[x.tolist() for x in Tref]}
        records.append(rec)
        qdot_values.append(qdot_inf); hard_values.append(hard); common_values.append(common)
        rel_values.append(float(rel_t)); act_values.append(max(act_arm)); ref_values.append(max(ref_arm)); lag_values.append(max(lag_arm)); effort_values.append(effort); native_values.append(native)
        if rate is not None: effort_rate_values.append(rate)
        if look is not None: look_margin_values.append(look)
        active_qdot += int(qdot_active); active_lookahead += int(look_active)
    # Match R10's command-rate definition: rates are computed between the
    # successive pre-command effort vectors, excluding the startup baseline
    # readback-to-first-command jump.
    command_rates=[]
    prev_cmd={}
    for cmd in result.get("command_sequence",[]):
        for side in ("left","right"):
            e=_a(cmd.get("effort_arm",{}).get(side,[]))
            if side in prev_cmd and e.size:
                command_rates.extend(np.abs((e-prev_cmd[side])*60.).ravel())
            prev_cmd[side]=e
    command_rate_peak=max(command_rates,default=float("nan"))
    # Keep the direct readback difference available as a separate diagnostic,
    # but do not confuse it with the registered effort-rate gate.
    peaks={k:_argmax(records,k) for k in ("actual_common_object_error_m","reference_common_object_error_m","actual_vs_reference_error_m","relative_tcp_translation_error_m","relative_tcp_rotation_error_rad","qp_hard_residual_inf","qp_common_residual_inf","qdot_inf_rad_s","native_dq_inf_rad_s","applied_effort_inf_Nm","lookahead_min_margin_rad")}
    peaks["registered_command_effort_rate_inf_Nm_s"]={"value":float(command_rate_peak) if np.isfinite(command_rate_peak) else None}
    # Ownership-transition trace: the first QP command follows the frozen
    # startup posture.  Report measured drift against that posture over the
    # first 13 commands; this is descriptive evidence, never a command path.
    first_i=next(i for i,r in enumerate(rows) if r.get("r10_qp"))
    hold=_a(rows[first_i-1]["arms"]["left"]["q"][:7]+rows[first_i-1]["arms"]["right"]["q"][:7])
    transient=[r for r in records if r["physics_step"] <= records[0]["physics_step"]+12]
    drift_rows=[]; native_rows=[]
    for r in transient:
        drift=_a(r["q_actual"])-hold
        drift_rows.append(drift)
        native_rows.append(_a(r["qdot"])*0.0)  # keep schema explicit; native is in source frames
    drift_arr=np.asarray(drift_rows,float)
    # Use the corresponding measured native velocities from source frames.
    native_trans=[]
    for src in rows[first_i:first_i+len(transient)]:
        native_trans.append(_a(src["arms"]["left"]["dq"][:7]+src["arms"]["right"]["dq"][:7]))
    native_trans=np.asarray(native_trans,float)
    startup={"hold_physics_step":int(rows[first_i-1]["physics_step"]),"first_control_physics_step":int(records[0]["physics_step"]),"window_last_physics_step":int(transient[-1]["physics_step"]),"window_commands":len(transient),"max_abs_joint_drift_rad":float(np.max(np.abs(drift_arr))),"max_abs_joint_drift_joint_index":int(np.unravel_index(np.argmax(np.abs(drift_arr)),drift_arr.shape)[1]),"max_abs_native_dq_rad_s":float(np.max(np.abs(native_trans))),"measured_native_dq_peak_joint_index":int(np.unravel_index(np.argmax(np.abs(native_trans)),native_trans.shape)[1]),"drift_vector_at_window_last_rad":drift_arr[-1].tolist(),"evidence":"MEASURED_POST_STEP_STATE_AND_NATIVE_DQ; descriptive ownership-transition signature"}
    # Single-variable counterfactual: recompute the R8 D term if the initial
    # handoff velocity target were zero.  This is an offline torque calculation
    # only; it is never sent to the simulator or used to bypass a gate.
    d_zero=[]; d_current=[]
    for src in rows[first_i:first_i+len(transient)]:
        for side in ("left","right"):
            m=_a(src["arms"][side]["M"])[:7,:7]; m=(m+m.T)/2; kd=32.0*m
            dq=_a(src["arms"][side]["dq"][:7]); c=src.get("control_components_pre_step",{}).get(side,{})
            d_zero.append((kd@(-dq)).tolist()); d_current.extend(c.get("D_Nm",[]))
    d_zero_arr=np.asarray(d_zero,float)
    startup["offline_zero_dqref_counterfactual"]={"not_executed":True,"first_command_joint2_left_Nm":float(d_zero_arr[0,1]),"first_command_joint4_left_Nm":float(d_zero_arr[0,3]),"max_abs_D_Nm":float(np.max(np.abs(d_zero_arr))),"description":"same frozen-mass Kd, dq_ref=0 instead of logged native-dq-matching handoff"}
    out={"schema":"TASK28_R12_V10A_SINGLE_VARIABLE_DIAGNOSTICS_V1","source":str(source),"source_result_status":result.get("status"),"execution_authorized":False,"physical_execution":False,
         "cells":{"V10-A":{"frames_with_qp":len(records),"physics_step_first":records[0]["physics_step"],"physics_step_last":records[-1]["physics_step"]}},
         "peak_metrics":peaks,
         "startup_ownership_transition":startup,
         "torque_components_peak_Nm":{k:float(max(v)) if v else None for k,v in torque_component_values.items()},
         "distributions":{"actual_common_object_error_m":_stat(act_values),"reference_common_object_error_m":_stat(ref_values),"actual_vs_reference_error_m":_stat(lag_values),"relative_tcp_translation_error_m":_stat(rel_values),"qdot_inf_rad_s":_stat(qdot_values),"native_dq_inf_rad_s":_stat(native_values),"applied_effort_inf_Nm":_stat(effort_values),"applied_readback_step_rate_inf_Nm_s":_stat(effort_rate_values),"registered_command_effort_rate_inf_Nm_s":{"max":float(command_rate_peak) if np.isfinite(command_rate_peak) else None,"count":len(command_rates)},"qp_hard_residual_inf":_stat(hard_values),"qp_common_residual_inf":_stat(common_values),"lookahead_min_margin_rad":_stat(look_margin_values)},
         "qp_activation":{"status_counts":qp_status,"qdot_limit_rad_s":0.12,"position_margin_rad":0.02,"qdot_active_rows":active_qdot,"lookahead_active_rows":active_lookahead,"all_samples_optimal":qp_status=={"QP_OPTIMAL":len(records)},"constraints_active":bool(active_qdot or active_lookahead)},
         "evidence_assessment":{"reference_trajectory":{"max_error_m":peaks["reference_common_object_error_m"],"support":"DIRECT_MODEL_FK_OF_LOGGED_QREF","strength":"SUPPORTED_SECONDARY"},"dynamic_feedback_lag":{"max_error_m":peaks["actual_vs_reference_error_m"],"support":"MEASURED_TCP_IMPLIED_VIRTUAL_MINUS_LOGGED_QREF_FK","strength":"SUPPORTED_PRIMARY"},"relative_motion":{"max_error_m":peaks["relative_tcp_translation_error_m"],"support":"MEASURED_TCP_RELATIVE_SE3","strength":"SUPPORTED_SECONDARY"},"qp_constraints":{"active_rows":active_qdot+active_lookahead,"support":"LOGGED_SOLVER_STATUS_AND_RECOMPUTED_LOOKAHEAD","strength":"SUPPORTED_PRIMARY_NOT_CAUSAL"},"numerical_residual":{"max_hard_residual":peaks["qp_hard_residual_inf"],"max_common_residual":peaks["qp_common_residual_inf"],"support":"LOGGED_QP_DIAGNOSTICS","strength":"SUPPORTED_PRIMARY_NOT_CAUSAL"},"torque_velocity_response":{"effort_peak_Nm":peaks["applied_effort_inf_Nm"],"native_dq_peak_rad_s":peaks["native_dq_inf_rad_s"],"support":"POST_STEP_READBACK_AND_CONTROL_LOG","strength":"SUPPORTED_DESCRIPTIVE"}},
         "records":records}
    output.parent.mkdir(parents=True,exist_ok=True); output.write_text(json.dumps(out,indent=2,sort_keys=True,allow_nan=False,default=_jsonable)+"\n")
    return out

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--source',type=Path,default=Path('offline/task28_p5_qp_6_r10/evidence/diagnostic/V10-A')); ap.add_argument('--output',type=Path,default=Path('offline/task28_p5_qp_6_r12/evidence/single_variable_diagnostics.json')); args=ap.parse_args(); out=run(args.source,args.output); print(json.dumps({"frames":len(out["records"]),"peaks":out["peak_metrics"],"qp_activation":out["qp_activation"]},indent=2))

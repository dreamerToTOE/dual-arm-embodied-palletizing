"""Restricted offline input: exact robot measurements, uncertain visual-time sets."""
import bisect
import copy
import numpy as np
from research.stage_b_multisensor_r3.history_contract import SynchronizedHistory, identity
from research.stage_b_multisensor_r3.r1_time_code import decode_clock, SOURCES


def decode_birth(rgb, depth, layout):
    return decode_clock(rgb, depth, {s:layout['birth'] for s in SOURCES})['app']


def interpolate_rows(a, b, t, method, fk, max_bracket_s=.034):
    """Evaluation-only interpolation. No grant of independent accuracy or attachment."""
    if identity(a) != identity(b):
        return {'status':'UNKNOWN','reason':'epoch_boundary'}
    dt=b['time_s']-a['time_s']
    if not np.isfinite(t) or dt<=0 or dt>max_bracket_s or not a['time_s']<=t<=b['time_s']:
        return {'status':'UNKNOWN','reason':'invalid_or_out_of_range_bracket'}
    u=(t-a['time_s'])/dt; q={};dq={}
    for s in ('left','right'):
        p0,p1,v0,v1=(np.array(r[k][s]) for r,k in ((a,'q'),(b,'q'),(a,'dq'),(b,'dq')))
        if method=='linear':
            q[s]=((1-u)*p0+u*p1).tolist();dq[s]=((1-u)*v0+u*v1).tolist()
        elif method=='hermite':
            q[s]=((2*u**3-3*u**2+1)*p0+(u**3-2*u**2+u)*dt*v0+(-2*u**3+3*u**2)*p1+(u**3-u**2)*dt*v1).tolist()
            dq[s]=(((6*u**2-6*u)*p0+(-6*u**2+6*u)*p1)/dt+(3*u**2-4*u+1)*v0+(3*u**2-2*u)*v1).tolist()
        else:
            raise ValueError('unknown interpolation method')
    return dict(status='COMPUTED_UNVERIFIED',q=q,dq=dq,tcp_fk={s:fk(s,q[s]).tolist() for s in ('left','right')})


class ReadinessHistory(SynchronizedHistory):
    def exact_at(self, clock_identity, time_s):
        try:key=identity(clock_identity)
        except ValueError as exc:return {'status':'UNKNOWN','reason':str(exc)}
        if key not in self.epochs:return {'status':'UNKNOWN','reason':'incompatible_run_epoch_or_clock'}
        if not isinstance(time_s,(float,int)) or not np.isfinite(time_s):return {'status':'UNKNOWN','reason':'invalid_candidate_time'}
        rows=list(self.epochs[key].rows);times=[r['time_s'] for r in rows];i=bisect.bisect_left(times,time_s)
        if i==len(times) or abs(times[i]-time_s)>1e-9:
            return {'status':'UNKNOWN','reason':'no_exact_measured_sample_no_snapping'}
        return super().at(clock_identity,times[i])

    def associate_epoch(self, frame, physics_times, epoch, birth):
        def reject(reason):return {'status':'UNKNOWN','reason':reason,'measurement_time_s':None,'online_fusion_eligible':False}
        if identity(frame)!=identity(epoch) or frame.get('camera_stream_id')!=epoch['camera_stream_id']:
            return reject('retired_or_incompatible_camera_stream')
        if not epoch.get('fresh_render_product') or epoch['robot_first_time_s']>epoch['birth_time_s'] or epoch['render_product_created_time_s']<epoch['birth_time_s']:
            return reject('unproven_epoch_lower_bound')
        if not birth.get('paired') or birth.get('rgb_code')!=epoch['birth_code']:
            return reject('old_or_unproven_image_birth')
        output=super().associate(frame,physics_times,epoch['birth_time_s'])
        if output['status']!='UNCERTAIN':return output
        output['unverified_linear_domains_s']=output.pop('interpolation_domains_s')
        output['validated_interpolation_domains_s']=[]
        output['restricted_robot_input']='EXACT_MEASURED_SAMPLES_ONLY_NO_NEAREST_NEIGHBOR'
        history=list(self.epochs[identity(frame)].rows);lo,hi=output['admissible_interval_s']
        brackets=[[max(lo,a['time_s']),min(hi,b['time_s'])] for a,b in zip(history,history[1:]) if b['physics_step']-a['physics_step']==1 and a['time_s']<hi and b['time_s']>lo]
        covered=sum(max(0.,b-a) for a,b in brackets)
        output['robot_acquisition_unsupported_duration_s']=max(0.,hi-lo-covered)
        output['robot_bracket_domains_s']=brackets
        output['full_grid_robot_coverage']=output['unsupported_grid_candidates']==0
        output['continuous_candidate_warning']='Complete acquisition/bracketing is not validated interpolation. Non-grid candidates remain unresolved under exact-only mode.'
        return output

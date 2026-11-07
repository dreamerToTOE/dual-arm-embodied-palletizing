"""Interpretable offline baselines. No oracle imports, attachment proof, or point image time."""
import numpy as np


class ConnectionEstimator:
    def __init__(self,cfg):
        self.cfg=cfg;self.contradictions=0

    def observe(self,visual_xy,conditional_cube_candidates,suction_software=None):
        # Agreement, software feedback, and small tracking error cannot verify attachment.
        if visual_xy is None or not len(conditional_cube_candidates):
            return {'state':'UNKNOWN','reason':'insufficient_independent_attachment_observation','confidence':None}
        residual=float(np.min(np.linalg.norm(np.array(conditional_cube_candidates)-visual_xy,axis=1)))
        threshold=self.cfg['connection_residual_threshold_m']+self.cfg['unresolved_between_step_padding_m']
        self.contradictions=self.contradictions+1 if residual>threshold else 0
        if self.contradictions>=self.cfg['connection_confirmations']:
            return {'state':'DISCONNECTED','reason':'visual_motion_contradicts_initial_rigid_relation_for_all_supported_candidates','confidence':None,'residual_min_m':residual}
        return {'state':'UNKNOWN','reason':'suspected_loss' if self.contradictions else 'motion_consistency_does_not_prove_connection','confidence':None,'residual_min_m':residual}


class OfflineCubeEstimator:
    def __init__(self,method,cfg):
        if method not in cfg['method_names']+cfg['ablation_names']:raise ValueError('unknown method')
        self.method=method;self.cfg=cfg;self.x=None;self.P=None;self.anchor=None;self.last_t=None;self.last_visual=None
        self.connection=ConnectionEstimator(cfg);self.connection_state={'state':'UNKNOWN','reason':'no_anchor','confidence':None};self.last_kinematic=None

    def _predict(self,t):
        if self.last_t is not None and self.x is not None:
            dt=t-self.last_t
            if dt<0:raise ValueError('reset_requires_new_estimator')
            F=np.eye(4);F[:2,2:]=np.eye(2)*dt;G=np.vstack((np.eye(2)*dt*dt/2,np.eye(2)*dt))
            self.x=F@self.x;self.P=F@self.P@F.T+G@G.T*self.cfg['process_acceleration_sigma_m_s2']**2
        self.last_t=t

    def step(self,robot,visual_events,candidate_robot_states):
        if any('oracle' in k.lower() or k.lower().endswith('_gt') or k=='tcp_physx' for k in robot):raise ValueError('oracle_not_an_input')
        t=robot['time_s'];self._predict(t);used=0;timing_status='NO_NEW_VISUAL'
        tool=np.array(robot['tcp_fk']['left'])
        for v in visual_events:
            if v['cube_pose']['status']!='PRESENT' or not v['image_epoch_accepted']:continue
            interval=v['admissible_interval_s']
            if interval[1]>t+1e-8 or interval[0]>interval[1]:raise ValueError('future_or_invalid_visual_interval')
            z=np.array(v['cube_pose']['cube_xyz_m'][:2]);R=np.eye(2)*self.cfg['visual_sigma_m']**2
            if self.anchor is None:
                candidates=[np.array(s['tcp_fk']['left']) for s in candidate_robot_states if interval[0]-1e-8<=s['time_s']<=interval[1]+1e-8]
                if not candidates:continue
                # Initial static registration is a stated prior; never calibrate from Cube truth.
                spread=np.max(np.linalg.norm(np.array([c[:3,3] for c in candidates])-candidates[0][:3,3],axis=1))
                if spread>self.cfg['visual_sigma_m']:continue
                cube=np.eye(4);cube[:3,3]=v['cube_pose']['cube_xyz_m']
                self.anchor=np.linalg.inv(candidates[0])@cube
                self.x=np.r_[z,[0.,0.]];self.P=np.diag([self.cfg['visual_sigma_m']**2]*2+[.02**2]*2)
            if self.method=='F1_no_visual' and self.last_visual is not None:continue
            possible=[]
            if self.method!='F1_no_robot':
                possible=[(np.array(s['tcp_fk']['left'])@self.anchor)[:2,3] for s in candidate_robot_states if interval[0]-1e-8<=s['time_s']<=interval[1]+1e-8]
            self.connection_state=self.connection.observe(z,possible,robot.get('suction_observation'))
            if self.method in ('F1','F1_naive_sdk_time') and self.connection_state['state']=='DISCONNECTED':
                self.x[2:]=0.;self.P[2:,2:]+=np.eye(2)*.02**2
            if self.method=='V1':self.x[:2]=z;self.x[2:]=0.;self.P[:2,:2]=R
            elif self.method!='K1':
                lo,hi=interval
                if self.method=='F1_naive_sdk_time':lo=hi=v['sdk_time_s']
                delay_lo=max(0.,t-hi);delay_hi=max(0.,t-lo)
                # Moment approximation of a uniform delayed-measurement mixture, not a timestamp selection.
                mean_delay=(delay_lo+delay_hi)/2;variance_delay=(delay_hi-delay_lo)**2/12
                # z = position_now - delay * velocity_now; preserve position/velocity cross-covariance.
                H=np.column_stack((np.eye(2),-np.eye(2)*mean_delay))
                effective_R=R+(self.P[2:,2:]+np.outer(self.x[2:],self.x[2:]))*variance_delay
                effective_R+=np.eye(2)*self.cfg['unresolved_between_step_padding_m']**2
                S=H@self.P@H.T+effective_R;gain=np.linalg.solve(S,H@self.P).T
                self.x+=gain@(z-H@self.x);I=np.eye(4)-gain@H
                self.P=I@self.P@I.T+gain@effective_R@gain.T
            self.last_visual={'interval_s':list(interval),'arrival_s':t};used+=1;timing_status='BOUNDED_UNCERTAIN_MARGINALIZED'
        if self.x is None:return {'status':'UNKNOWN','reason':'no_valid_initial_visual_anchor','connection':self.connection_state}
        if self.method=='K1':
            xy=(tool@self.anchor)[:2,3]
            self.x[2:]=(xy-self.last_kinematic[1])/(t-self.last_kinematic[0]) if self.last_kinematic is not None and t>self.last_kinematic[0] else 0.
            self.x[:2]=xy;self.last_kinematic=(t,xy.copy())
            self.connection_state={'state':'CONNECTED','reason':'EXPLICIT_INITIAL_CONNECTION_ASSUMPTION_BASELINE_ONLY','confidence':None}
        elif not used:
            self.connection_state={'state':'UNKNOWN','reason':'no_independent_connection_measurement_during_prediction','confidence':None}
        if self.method in ('F1','F1_no_robot','F1_no_visual','F1_naive_sdk_time') and self.connection_state['state']!='CONNECTED':
            rigid_constraint='SUSPENDED'
        else:rigid_constraint='CONDITIONAL_ASSUMPTION' if self.method=='K1' else 'NONE'
        age=[t-self.last_visual['interval_s'][1],t-self.last_visual['interval_s'][0]] if self.last_visual else None
        return {'status':'ESTIMATED','xy_m':self.x[:2].tolist(),'velocity_xy_m_s':self.x[2:].tolist(),
            'covariance_xy_m2':self.P[:2,:2].tolist(),'position_uncertainty_radius_m':float(np.sqrt(5.991*max(np.linalg.eigvalsh(self.P[:2,:2])))),
            'observation_age_interval_s':age,'connection':self.connection_state,'rigid_constraint':rigid_constraint,
            'new_visual_updates':used,'measurement_time_s':None,'timing_status':timing_status,
            'timing_mixture_prior':'uniform interval; Gaussian moment approximation, not a guaranteed confidence bound',
            'online_fusion_eligible':False}

"""Post-estimation oracle using only recorded applied loads and direct statics.

This module is deliberately separate from the estimator and grasp mapping.
It computes the on-robot applied wrench about each actual TCP, the hypothetical
on-Cube net wrench about the fixed center, and a direct KKT weighted split.
"""
import numpy as np


def direct_measured_center(estimated, center):
    """Sum estimated wrenches directly at actual TCPs, independent of P2-1."""
    center=np.asarray(center,dtype=float)
    net=np.zeros(6)
    for side in ('left','right'):
        data=estimated[side]
        wr=np.asarray(data['on_robot_world_tcp'],dtype=float)
        point=np.asarray(data['actual_world_tcp'],dtype=float)[:3,3]
        cube_force=-wr[:3]
        net+=np.r_[cube_force,-wr[3:]+np.cross(point-center,cube_force)]
    return net


def independent_kkt(local_contacts, center, nominal_transforms):
    """Independent hand-assembled map and direct KKT for arbitrary contacts."""
    center = np.asarray(center,dtype=float)
    blocks = []
    for side in ('left','right'):
        t=np.asarray(nominal_transforms[side],dtype=float)
        rotation=t[:3,:3]
        x,y,z=t[:3,3]-center
        skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        blocks.append(np.block([[rotation,np.zeros((3,3))],[skew@rotation,rotation]]))
    g=np.hstack(blocks)
    metric=np.diag(np.tile([1.,1.,1.,1/.061**2,1/.061**2,1/.061**2],2))
    kkt=np.block([[metric,-g.T],[g,np.zeros((6,6))]])
    x=np.asarray(local_contacts,dtype=float)
    external=np.linalg.solve(kkt,np.r_[np.zeros(12),g@x])[:12]
    internal=x-external
    return {'net_world_center':g@x,'weighted_external_local':external,
            'weighted_internal_local':internal,'null_residual':float(np.linalg.norm(g@internal))}


def applied_oracle(raw, center, nominal_transforms):
    center = np.asarray(center, dtype=float)
    contacts = []
    per_arm = {}
    object_net = np.zeros(6)
    for side in ('left', 'right'):
        t = np.asarray(nominal_transforms[side], dtype=float)
        rotation, nominal_point = t[:3, :3], t[:3, 3]
        robot_tcp, local, centers, points = [], [], [], []
        for row in raw['observations']:
            o = row['arms'][side]['load_oracle']
            force = np.asarray(o['applied_force_world_n'], dtype=float)
            pure_moment = np.asarray(o['applied_pure_torque_world_nm'], dtype=float)
            point = np.asarray(o['application_point_world_m'], dtype=float)
            tcp = np.asarray(o['tcp_pre_step_world'], dtype=float)[:3, 3]
            robot_tcp.append(np.r_[force, pure_moment + np.cross(point - tcp, force)])
            # Reverse action; shift applied moment from actual point directly to nominal.
            local.append(np.r_[rotation.T @ (-force),
                               rotation.T @ (-pure_moment + np.cross(point - nominal_point, -force))])
            centers.append(np.r_[-force, -pure_moment + np.cross(point - center, -force)])
            points.append(point)
        per_arm[side] = {'on_robot_world_tcp': np.mean(robot_tcp, axis=0),
                         'actual_application_point_mean_world_m': np.mean(points, axis=0),
                         'nominal_contact_point_world_m': nominal_point}
        object_net += np.mean(centers, axis=0)
        contacts.extend(np.mean(local, axis=0))
    # Direct KKT oracle; all weights match the documented P2-1 default metric.
    x = np.asarray(contacts)
    kkt=independent_kkt(x,center,nominal_transforms)
    return {'per_arm': per_arm, 'net_world_center': object_net,
            'nominal_local_contacts': x, 'net_from_kkt_matrix': kkt['net_world_center'],
            'weighted_external_local': kkt['weighted_external_local'],
            'weighted_internal_local': kkt['weighted_internal_local'],
            'null_residual': kkt['null_residual'],
            'net_direct_vs_kkt_residual': float(np.linalg.norm(object_net-kkt['net_world_center'])),
            'method': 'independent direct force/moment summation and 18x18 KKT'}

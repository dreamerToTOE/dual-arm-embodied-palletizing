"""Independent RGB/depth step-code decoder and a frozen interval time interface."""
import math
import numpy as np

SOURCES=('app','pre','post')


def code_bits(value):
    if not isinstance(value,(int,np.integer)) or not 0<=value<65536:
        raise ValueError('step code must fit uint16')
    bits=np.array([(value>>i)&1 for i in range(16)])
    return np.stack((bits,1-bits))


def decode_clock(rgb,depth,layout):
    """Decode only raw image pixels. No producer state or oracle is an input."""
    output={}
    for source in SOURCES:
        colors=[];ranges=[]
        for pair in layout[source]:
            c=[];d=[]
            for u,v in pair:
                c.append(float(np.median(np.mean(rgb[v-1:v+2,u-1:u+2,:3],axis=2))))
                d.append(float(np.median(depth[v-1:v+2,u-1:u+2])))
            colors.append(c);ranges.append(d)
        colors=np.array(colors);ranges=np.array(ranges)
        color_valid=bool(np.isfinite(colors).all() and np.all(abs(colors[:,0]-colors[:,1])>=60))
        depth_valid=bool(np.isfinite(ranges).all() and np.all(abs(ranges[:,0]-ranges[:,1])>=.020)
            and np.all((ranges>.70)&(ranges<1.05)))
        rgb_code=sum(int(a>b)<<i for i,(a,b) in enumerate(colors)) if color_valid else None
        depth_code=sum(int(a>b)<<i for i,(a,b) in enumerate(ranges)) if depth_valid else None
        output[source]={'rgb_code':rgb_code,'depth_code':depth_code,'paired':rgb_code is not None and rgb_code==depth_code,
            'rgb_min_complement_contrast':float(np.min(abs(colors[:,0]-colors[:,1]))),
            'depth_min_complement_difference_m':float(np.min(abs(ranges[:,0]-ranges[:,1])))}
    return output


def timestamp_interface(observation,binding):
    """Apply a clock-calibrated model, never a Cube-trajectory-fitted offset."""
    result={'frame_id':observation['frame_id'],'measurement_time_s':None,'measurement_time_interval_s':None,
        'reported_sdk_time_s':observation.get('sdk_time_s'),'fabric_sim_time_s':observation.get('fabric_sim_time_s'),
        'arrival_physics_time_s':observation.get('arrival_physics_time_s'),'timing_status':'UNCERTAIN',
        'online_fusion_eligible':False}
    t=observation.get('fabric_sim_time_s')
    if binding.get('status')!='FROZEN_CALIBRATION' or t is None or not math.isfinite(t):return result
    if observation.get('configuration_fingerprint')!=binding['configuration_fingerprint']:return result
    lo,hi=binding['offset_interval_s'];interval=[t-hi,t-lo]
    result.update(measurement_time_s=sum(interval)/2,measurement_time_interval_s=interval,
        timing_status='CALIBRATED_INTERVAL_PENDING_HOLDOUT',binding_sha256=observation.get('binding_sha256'))
    return result

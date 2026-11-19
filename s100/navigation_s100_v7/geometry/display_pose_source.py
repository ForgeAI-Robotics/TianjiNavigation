"""Strict snapshot validation for display only. No timestamp substitution."""
import math
from pathlib import Path
import json,time

def read_source(directory):
    path=Path(directory)/'pose_source.json'
    try:
        age=time.time()-path.stat().st_mtime
        if not 0<=age<=2.5:return dict(mode='realtime')
        value=json.loads(path.read_text());value['_arrival_age']=age;return value
    except FileNotFoundError:return dict(mode='realtime')

def saved_positions(source,model_sha):
    if source.get('mode')!='saved' or source.get('schema_version')!=1 or source.get('model_sha256')!=model_sha or not 0<=source.get('_arrival_age',0)<=2.5:raise ValueError('SAVED_DISPLAY_SOURCE_INVALID')
    p=source.get('snapshot',{});q=p.get('joint_positions_rad',{});names={f'Joint{i}_{s}' for s in 'LR' for i in range(1,8)}
    if (p.get('kind')!='saved_measured_arm_pose' or p.get('live') is not False or p.get('model_sha256')!=model_sha
        or set(q)!=names or any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>2*math.pi for v in q.values())
        or type(p.get('observation_time')) not in (int,float) or not math.isfinite(p['observation_time'])):raise ValueError('SAVED_ARM_POSE_INVALID')
    return dict(q)

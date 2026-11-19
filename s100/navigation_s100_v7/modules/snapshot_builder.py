"""S100-owned URDF adapter. Callers must supply aligned world points and original times.

This module does not infer a lidar/map transform, fill missing joints or relabel old
observations as current. It has no network, ROS publisher or motion interface.
"""
import math
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

V2=Path(__file__).resolve().parents[1]/'geometry'
sys.path.insert(0,str(V2))
from local_model import LocalRobotModel,default_urdf_path
from link import JOINTS
from chassis_proxy import profile,replace_shapes

def local_model():
    return LocalRobotModel(default_urdf_path(),V2/'model_manifest.json',V2/'collision_geometry.json')

def build_snapshot(model, *, joints, pose_time, observation_time, points, snapshot_id,
                   frame_id, alignment_verified, source, simplify_chassis=True):
    if set(joints)!=set(JOINTS) or any(type(v) not in (int,float) or not math.isfinite(v) for v in joints.values()):
        raise ValueError('FULL_MEASURED_OR_EXPLICIT_SYNTHETIC_POSTURE_REQUIRED')
    if source not in ('measured','recorded','synthetic'):raise ValueError('SOURCE_REQUIRED')
    if frame_id!='nav3d_map' or alignment_verified is not True:raise ValueError('VERIFIED_WORLD_TRANSFORM_REQUIRED')
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in [pose_time,observation_time]):
        raise ValueError('ORIGINAL_TIMESTAMPS_REQUIRED')
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)>50000 or not np.isfinite(p).all():raise ValueError('OCCUPIED_POINTS_INVALID')
    shapes=model.shapes(joints)
    if simplify_chassis:shapes=replace_shapes(shapes,profile(model))
    return dict(schema_version=1,snapshot_id=snapshot_id,frame_id=frame_id,
        observation_time=observation_time,pose_time=pose_time,model_sha256=model.sha256,
        source=source,alignment_verified=True,joint_positions_rad=dict(joints),resolution_m=.03,
        geometry_source='verified_local_urdf_with_chassis_proxy' if simplify_chassis else 'verified_local_urdf',
        occupied_points=p.tolist(),shapes=[{k:s[k] for k in ('link','matrix','bounds')} for s in shapes],
        advisory_only=True,free_space_certified=False,command_path_connected=False)

def build_snapshot_replay(model, recorded):
    """Explicit immutable preview; never accepted by World::parse/live consumers."""
    saved=recorded['saved_pose'];joints=dict(saved['joint_positions_rad'])
    if saved.get('live') is not False or saved.get('model_sha256')!=model.sha256 or recorded.get('model_sha256')!=model.sha256:
        raise ValueError('SAVED_MODEL_OR_NONLIVE_CONTRACT_INVALID')
    arms=set(JOINTS[:14]);allowed=set(JOINTS)
    if not arms.issubset(joints) or not set(joints).issubset(allowed) or any(type(x) not in (int,float) or not math.isfinite(x) for x in joints.values()):
        raise ValueError('SAVED_ARM_POSTURE_INVALID')
    if recorded.get('clearance_assumed') is not True:raise ValueError('EXPLICIT_CLEARANCE_ASSUMPTION_REQUIRED')
    points=np.asarray(recorded['points'],float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)>50000 or not np.isfinite(points).all():raise ValueError('OCCUPIED_POINTS_INVALID')
    stamp=recorded['original_cloud_observation_time'];pose_stamp=saved['observation_time']
    if any(type(x) not in (int,float) or not math.isfinite(x) or x<=0 for x in (stamp,pose_stamp)):raise ValueError('ORIGINAL_TIMESTAMPS_REQUIRED')
    digest=hashlib.sha256(json.dumps(recorded,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()
    shapes=replace_shapes(model.shapes(joints),profile(model))
    return dict(schema_version=1,planning_mode='immutable_snapshot_replay',snapshot_id='recorded-'+digest[:24],
        source_packet_sha256=digest,frame_id='nav3d_map',original_source_frame=recorded.get('source_frame'),
        observation_time=stamp,pose_time=pose_stamp,source='recorded',model_sha256=model.sha256,
        alignment_verified=False,alignment_assumption='vendor_base_to_model_identity',geometry_owner='verified_local_urdf',
        joint_positions_rad=joints,bounded_missing_joints=sorted(allowed-set(joints)),resolution_m=.03,
        shapes=[{k:s[k] for k in ('link','matrix','bounds','mode')} for s in shapes],occupied_points=points.tolist(),
        clearance_assumed=True,advisory_only=True,free_space_certified=False,execution_permission=False,
        command_path_connected=False,motion_inhibit=True)

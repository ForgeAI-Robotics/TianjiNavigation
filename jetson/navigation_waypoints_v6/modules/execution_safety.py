"""Pure live admission checks and conservative whole-body swept-point guard."""
import math,time
import numpy as np
from scipy.spatial import cKDTree
from paths import ROOT
from saved_map import base_transform
from environment import self_candidates
from self_envelope import settings as envelope_settings,mask as envelope_mask
from pillar_cables import settings as cable_settings,ignore_mask

REQUIRED_JOINTS=[f'Joint{i}_{s}' for s in ['L','R'] for i in range(1,8)]+['head_pitch_joint','left_robotiq_85_left_knuckle_joint','right_robotiq_85_left_knuckle_joint']

def posture_check(saved,current,model_hash,now):
 if current.get('model_sha256')!=model_hash:return False,'URDF_VERSION_MISMATCH',999.
 age=now-current.get('observation_time',0.)
 if not 0<=age<=.35:return False,'JOINT_FEEDBACK_STALE',age
 q=current.get('joint_positions_rad',{});old=saved.get('joint_positions_rad',{})
 if any(k not in q or k not in old for k in REQUIRED_JOINTS):return False,'FULL_HEAD_ARM_GRIPPER_FEEDBACK_REQUIRED',age
 if any(not math.isfinite(q[k]) or abs(q[k]-old[k])>math.radians(3) for k in REQUIRED_JOINTS):return False,'POSTURE_CHANGED_REPLAN_REQUIRED',age
 return True,'OK',age

def clearance_check(data,model_hash,now,joints=None):
 try:
  age=max(now-data['received_at'],now-data['evaluation_received_at']);b=data['bottom_clearance']
  if not 0<=age<=.75:return False,'CAMERA_CLEARANCE_STALE',age
  if data['model_sha256']!=model_hash:return False,'CAMERA_MODEL_MISMATCH',age
  if b.get('calibration_test_mode') or b.get('assumption'):return False,'CAMERA_CLEARANCE_ASSUMED_OR_TEST',age
  if not b.get('clear') or b.get('stop_required') or b.get('state') not in ['CLEAR','clear','SAFE']:return False,'CAMERA_CLEARANCE_NOT_CLEAR',age
  evaluated=np.asarray(data['evaluation_joints_deg'],float);actual=np.array([[math.degrees(joints[f'Joint{i}_{s}']) for i in range(1,8)] for s in ['L','R']])
  if evaluated.shape!=(2,7) or not np.isfinite(evaluated).all() or np.max(abs(evaluated-actual))>3.:return False,'CAMERA_POSTURE_MISMATCH',age
  return True,'OK',age
 except (KeyError,TypeError,ValueError):return False,'CAMERA_CLEARANCE_UNAVAILABLE',999.

def operator_check(data,run_id,now):
 try:
  age=now-data['received_at'];return bool(data['run_id']==run_id and data['active'] and 0<=age<=.5),age
 except (KeyError,TypeError):return False,999.

def position_check(packet,plan,now):
 try:
  elapsed=now-packet['updated_wall_time'];age=elapsed+packet['odometry_age_s'];loc=plan['localization']
  if not packet['running'] or not packet['map_pose_valid'] or not 0<=elapsed<=.4 or not 0<=age<=.5 or packet['lidar_fix_age_s']+elapsed>3:return False,'LOCALIZATION_STALE',age
  if packet['map_voxel_sha256']!=plan['map_voxel_sha256'] or packet['source_boot_id']!=loc['source_boot_id'] or packet['odometry_epoch']!=loc['odometry_epoch'] or packet['calibration_sha256']!=loc['calibration_sha256']:return False,'LOCALIZATION_FRAME_OR_EPOCH_CHANGED',age
  return True,'OK',age
 except (KeyError,ValueError,TypeError):return False,'LOCALIZATION_UNAVAILABLE',999.

def trajectory_deviation(pose,path):
 path=np.asarray(path);return float(np.linalg.norm(path[:,:2]-np.asarray(pose)[:2],axis=1).min())

def sweep_guard(frames,transforms,reference_wheel,shapes,model_hash,floor,twist,now_mono):
 """Use full measured live returns, including small obstacles. No radius outlier deletion."""
 if set(frames)!= {'front','back'}:return False,'BOTH_LIDARS_REQUIRED',999.
 cloud_age=max(now_mono-f[2]+float(f[0].get('source_age_s',0.)) for f in frames.values())
 if not 0<=cloud_age<=1.2:return False,'LIDAR_STALE',cloud_age
 reference=base_transform(reference_wheel);parts=[]
 for sensor,(h,p,arrival) in frames.items():
  T=np.linalg.inv(reference)@base_transform(h['pose'])@transforms[sensor];p=np.asarray(p);q=p@T[:3,:3].T+T[:3,3];q=q[np.isfinite(q).all(1)]
  own,_,_=envelope_mask(q,shapes,envelope_settings(),model_hash);q=q[~own];cables,_=ignore_mask(q,shapes,cable_settings(),model_hash);q=q[~cables]
  # Floor model is expressed in the live robot frame by the caller.
  if floor is not None:q=q[np.abs(q[:,2]-np.c_[q[:,:2],np.ones(len(q))]@floor)>.04]
  parts.append(q)
 p=np.concatenate(parts);tree=cKDTree(p)
 for t in np.linspace(0,.5,6):
  vx,vy,w=twist;a=w*t;c,s=math.cos(a),math.sin(a);rot=np.array([[c,-s,0],[s,c,0],[0,0,1]])
  translation=np.array([vx*t,vy*t,0.]);moved=[]
  for shape in shapes:
   T=np.array(shape['matrix']);T[:3,3]=rot@T[:3,3]+translation;T[:3,:3]=rot@T[:3,:3];moved.append(dict(shape,matrix=T.tolist()))
  # Add unobserved motion since the oldest cloud to the 5 cm safety interval.
  margin=.05+math.hypot(vx,vy)*cloud_age
  if self_candidates(p,moved,margin,_tree=tree).any():return False,'LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY',cloud_age
 return True,'OK',cloud_age

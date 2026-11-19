"""Pure live admission checks and conservative whole-body swept-point guard."""
import math,time
import numpy as np
from scipy.spatial import cKDTree
from paths import ROOT
from saved_map import base_transform
from environment import self_candidates
from self_envelope import settings as envelope_settings,mask as envelope_mask
from pillar_cables import settings as cable_settings,ignore_mask
from navigation_self_filter import evaluate as evaluate_self_filter

REQUIRED_JOINTS=[f'Joint{i}_{s}' for s in ['L','R'] for i in range(1,8)]+['head_pitch_joint','left_robotiq_85_left_knuckle_joint','right_robotiq_85_left_knuckle_joint']

def posture_check(saved,current,model_hash,now):
 if current.get('model_sha256')!=model_hash:return False,'URDF_VERSION_MISMATCH',999.
 if saved.get('fixed_gripper_policy')!=current.get('fixed_gripper_policy'):return False,'GRIPPER_CONFIGURATION_CHANGED_REPLAN_REQUIRED',999.
 age=now-current.get('observation_time',0.)
 # Match the receiver's bounded cross-host clock skew tolerance.
 if not -.10<=age<=.35:return False,'JOINT_FEEDBACK_STALE',age
 age=max(0.,age)
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
  elapsed=now-packet['updated_wall_time'];odom_age=packet.get('odometry_age_s');age=elapsed+odom_age if isinstance(odom_age,(int,float)) else 999.;loc=plan['localization']
  if packet.get('running') and packet.get('status') in ('NEED_LIDAR_FIX','NO_ODOMETRY') and packet.get('T_map_base') is None and 0<=elapsed<=.4:return False,'LOCALIZATION_INITIALIZING',age
  if not packet['running'] or not packet['map_pose_valid'] or not 0<=elapsed<=.4 or not 0<=age<=.5 or packet['lidar_fix_age_s']+elapsed>3:return False,'LOCALIZATION_STALE',age
  if packet['map_voxel_sha256']!=plan['map_voxel_sha256'] or packet['source_boot_id']!=loc['source_boot_id'] or packet['odometry_epoch']!=loc['odometry_epoch'] or packet['calibration_sha256']!=loc['calibration_sha256']:return False,'LOCALIZATION_FRAME_OR_EPOCH_CHANGED',age
  return True,'OK',age
 except (KeyError,ValueError,TypeError):return False,'LOCALIZATION_UNAVAILABLE',999.

def trajectory_deviation(pose,path):
 path=np.asarray(path);return float(np.linalg.norm(path[:,:2]-np.asarray(pose)[:2],axis=1).min())

def translation_sweep_shapes(shapes,twist,horizon=.5):
 """OBBs containing every translated shape over the continuous horizon."""
 velocity=np.array([*twist[:2],0.]);result=[]
 for shape in shapes:
  matrix=np.asarray(shape['matrix']);lo,hi=np.asarray(shape['bounds']);delta=matrix[:3,:3].T@velocity*horizon
  result.append(dict(shape,bounds=[(lo+np.minimum(delta,0)).tolist(),(hi+np.maximum(delta,0)).tolist()]))
 return result

def translation_sweep_distance_squared(points,shape,twist,horizon):
 """Exact minimum point/OBB distance across a constant straight translation."""
 matrix=np.asarray(shape['matrix']);lo,hi=np.asarray(shape['bounds'])
 q=(np.asarray(points)-matrix[:3,3])@matrix[:3,:3]
 velocity=matrix[:3,:3].T@np.array([*twist[:2],0.])
 moving=np.abs(velocity)>1e-14
 times=[]
 for bound in (lo,hi):
  crossings=np.zeros_like(q)
  np.divide(q-bound,velocity,out=crossings,where=moving)
  times.append(np.clip(crossings,0.,horizon))
 breaks=np.sort(np.column_stack((np.zeros(len(q)),np.full(len(q),horizon),*times)),axis=1)
 lower,upper=breaks[:,:-1],breaks[:,1:]
 midpoint=(lower+upper)*.5
 local=q[:,None,:]-midpoint[:,:,None]*velocity
 active=(local<lo)|(local>hi)
 bound=np.where(local<lo,lo,hi)
 numerator=np.sum(active*velocity*(q[:,None,:]-bound),axis=2)
 denominator=np.sum(active*velocity**2,axis=2)
 optimum=lower.copy()
 np.divide(numerator,denominator,out=optimum,where=denominator>0.)
 optimum=np.clip(optimum,lower,upper)
 local=q[:,None,:]-optimum[:,:,None]*velocity
 gap=np.maximum(np.maximum(lo-local,local-hi),0.)
 return np.min(np.sum(gap*gap,axis=2),axis=1)

def precise_translation_hits(points,shapes,swept,twist,horizon,margin,tree):
 hit=np.zeros(len(points),bool)
 if not len(points) or not swept:return hit
 # Same sphere/OBB predicates as self_candidates, batched once for all links.
 matrices=np.asarray([s['matrix']for s in swept]);bounds=np.asarray([s['bounds']for s in swept])
 lo,hi=bounds[:,0],bounds[:,1]
 centers=np.einsum('sij,sj->si',matrices[:,:3,:3],(lo+hi)/2)+matrices[:,:3,3]
 radii=np.linalg.norm((hi-lo)/2,axis=1)+margin
 tree=cKDTree(points) if tree is None else tree
 groups=tree.query_ball_point(centers,radii)
 for i,(original,rows) in enumerate(zip(shapes,groups)):
  ids=np.asarray(rows,dtype=int)
  ids=ids[~hit[ids]]
  if not len(ids):continue
  q=(points[ids]-matrices[i,:3,3])@matrices[i,:3,:3]
  delta=np.maximum(np.maximum(lo[i]-q,q-hi[i]),0.)
  ids=ids[np.linalg.norm(delta,axis=1)<=margin]
  if len(ids):
   distance=translation_sweep_distance_squared(points[ids],original,twist,horizon)
   hit[ids[distance<=margin*margin+1e-12]]=True
 return hit

def curved_sweep_shapes(shapes,twist,horizon=.5):
 """Continuous conservative OBB envelope, including legacy and exact SE(2).

 Each corner's deviation from its endpoint chord is bounded by M*h*h/8,
 where M bounds its second derivative. Euclidean safety margins still apply
 outside this containing volume. This replaces six full geometry queries.
 """
 vx,vy,w=map(float,twist)
 if abs(w)<1e-8:return translation_sweep_shapes(shapes,twist,horizon)
 if not shapes:return []
 matrices=np.asarray([s['matrix'] for s in shapes]);bounds=np.asarray([s['bounds'] for s in shapes]);R=matrices[:,:3,:3];origin=matrices[:,:3,3]
 pattern=np.array([[x,y,z]for x in (0,1)for y in (0,1)for z in (0,1)])
 corners=bounds[:,0,None,:]+pattern[None,:,:]*(bounds[:,1]-bounds[:,0])[:,None,:]
 world=np.einsum('ski,sji->skj',corners,R)+origin[:,None,:]
 a=w*horizon;c,si=math.cos(a),math.sin(a);rotation=np.array([[c,-si,0],[si,c,0],[0,0,1]])
 exact=np.array([(vx*si+vy*(c-1))/w,(vx*(1-c)+vy*si)/w,0.])
 legacy=np.array([vx*horizon,vy*horizon,0.]);lo=bounds[:,0].copy();hi=bounds[:,1].copy()
 for translation in (legacy,exact):
  end=world@rotation.T+translation;local=np.einsum('skj,sji->ski',end-origin[:,None,:],R)
  lo=np.minimum(lo,local.min(1));hi=np.maximum(hi,local.max(1))
 radius=np.linalg.norm(world[:,:,:2],axis=2).max(1)
 pad=(radius*w*w+math.hypot(vx,vy)*abs(w))*horizon*horizon/8+1e-12
 return [dict(shape,bounds=[lower.tolist(),upper.tolist()]) for shape,lower,upper in zip(shapes,lo-pad[:,None],hi+pad[:,None])]

def sweep_reference_drift(old_shapes,new_shapes,old_odom,new_odom,twist):
 """Maximum corresponding OBB-corner displacement across checked sweeps."""
 from itertools import product
 if len(old_shapes)!=len(new_shapes):return float('inf')
 old=[];new=[]
 for a,b in zip(old_shapes,new_shapes):
  if a['link']!=b['link'] or not np.array_equal(a['bounds'],b['bounds']):return float('inf')
  for shape,dest in [(a,old),(b,new)]:
   lo,hi=np.asarray(shape['bounds']);p=np.array(list(product(*zip(lo,hi))));T=np.asarray(shape['matrix']);dest.append(p@T[:3,:3].T+T[:3,3])
 if not old:return float('inf')
 old=np.concatenate(old);new=np.concatenate(new);D=np.linalg.inv(base_transform(old_odom))@base_transform(new_odom);maximum=0.
 if not np.isfinite(D).all() or not np.isfinite(old).all() or not np.isfinite(new).all():return float('inf')
 vx,vy,w=twist
 for t in np.linspace(0,.5,6):
  c,s=math.cos(w*t),math.sin(w*t);R=np.array([[c,-s,0],[s,c,0],[0,0,1]]);translation=np.array([vx*t,vy*t,0.])
  a=old@R.T+translation;b=(new@R.T+translation)@D[:3,:3].T+D[:3,3]
  maximum=max(maximum,float(np.linalg.norm(a-b,axis=1).max()))
 return maximum

def sweep_guard(frames,transforms,reference_wheel,shapes,model_hash,floor,twist,now_mono,probe_nonapproach=False,probe_deferral=None,revalidation_reserve_m=0.,diagnostics=None,prepared=None,motion_advance_s=0.,directional_age_shadow=False,directional_age_compensation=False):
 """Use full measured live returns, including small obstacles. No radius outlier deletion."""
 if set(frames)!= {'front','back'}:return False,'BOTH_LIDARS_REQUIRED',999.
 cloud_age=max(now_mono-f[2]+float(f[0].get('source_age_s',0.)) for f in frames.values())
 if not 0<=cloud_age<=1.2:return False,'LIDAR_STALE',cloud_age
 reference=base_transform(reference_wheel);parts=[];filter_reports={}
 corners=[]
 from itertools import product
 for shape in shapes:
  T=np.asarray(shape['matrix']);lo,hi=np.asarray(shape['bounds']);v=np.array(list(product(*zip(lo,hi))));corners.append(v@T[:3,:3].T+T[:3,3])
 envelope=np.concatenate(corners);lower=envelope.min(0)-.60;upper=envelope.max(0)+.60
 for sensor,(h,p,arrival) in frames.items():
  if prepared is not None:
   data=prepared[sensor]
   if data['model_sha256']!=model_hash or data['frame'][0]['sequence']!=h['sequence']:raise ValueError('PREPARED_FRAME_IDENTITY_MISMATCH')
   T=np.linalg.inv(reference)@base_transform(h['pose']);q=data['points_body']@T[:3,:3].T+T[:3,3]
   if diagnostics is not None:filter_reports[sensor]=data['report']
  else:
   T=np.linalg.inv(reference)@base_transform(h['pose'])@transforms[sensor];p=np.asarray(p);q=p@T[:3,:3].T+T[:3,3];q=q[np.isfinite(q).all(1)]
  # Every possible 0.5s sweep hit lies inside this box. The extra 60cm also
  # retains independent external surface context for cable/table protection.
  q=q[((q>=lower)&(q<=upper)).all(1)]
  if prepared is None:
   self_result=evaluate_self_filter(q,shapes,model_hash,envelope_cfg=envelope_settings(),cable_cfg=cable_settings())
   if diagnostics is not None:filter_reports[sensor]=dict(points=len(q),filtered=int(self_result['filtered'].sum()),protected=int(self_result['protected'].sum()),cable_report=self_result['cable_report'])
   q=q[~self_result['filtered']]
  # Floor model is expressed in the live robot frame by the caller.
  if floor is not None:q=q[np.abs(q[:,2]-np.c_[q[:,:2],np.ones(len(q))]@floor)>.04]
  if probe_deferral is not None:probe_deferral.observe(q,shapes,model_hash,reference_wheel,sensor,h['sequence'])
  parts.append(q)
 p=np.concatenate(parts)
 if probe_deferral is not None:p=p[~probe_deferral.mask(p,shapes,model_hash,reference_wheel)]
 tree=cKDTree(p);escape=np.zeros(len(p),bool);baseline=None
 def record_collision(hit,margin,t):
  if diagnostics is not None:
   diagnostics.update(points_body=p[hit].copy(),retained_points_body=p.copy(),margin_m=margin,sweep_time_s=t,reference_wheel=list(reference_wheel),twist=list(twist),cloud_age_s=cloud_age,shapes=shapes,filter_reports=filter_reports)
 if not 0<=revalidation_reserve_m<=.005:raise ValueError('INVALID_REVALIDATION_RESERVE')
 if not math.isfinite(motion_advance_s) or not 0<=motion_advance_s<=.35:raise ValueError('INVALID_MOTION_ADVANCE_HORIZON')
 if not probe_nonapproach:
  # One conservative continuous sweep replaces six repeated whole-body
  # queries. It contains the old six samples and all positions between them.
  horizon=.5+motion_advance_s
  margin=.05+revalidation_reserve_m
  directional=(directional_age_shadow or directional_age_compensation) and abs(twist[2])<1e-8 and probe_deferral is None
  if directional:
   # Returns are aligned to current measured odometry. Cover extra base
   # travel along the commanded translation through an extended sweep.
   horizon+=cloud_age
  else:
   margin+=math.hypot(*twist[:2])*cloud_age
  swept=curved_sweep_shapes(shapes,twist,horizon=horizon)
  hit=precise_translation_hits(p,shapes,swept,twist,horizon,margin,tree) if directional else self_candidates(p,swept,margin,_tree=tree)
  if hit.any():
   record_collision(hit,margin,f'CONTINUOUS_BODY_SWEEP_0_TO_{horizon:g}')
   if diagnostics is not None:diagnostics['directional_age_compensation']=directional
   return False,'LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY',cloud_age
  return True,'PROBE_INITIAL_WIRES_DEFERRED_UNTIL_5CM' if probe_deferral is not None and probe_deferral.last_deferred else 'OK',cloud_age
 if probe_nonapproach:
  from cable_precheck import deferred_mask
  from pillar_cables import distances
  candidates,_=deferred_mask(p,shapes,model_hash);baseline=distances(p,shapes)
  escape=candidates&(baseline>=.025)&(baseline<.05+math.hypot(*twist[:2])*cloud_age)
 for t in np.linspace(0,.5,6):
  vx,vy,w=twist;a=w*t;c,s=math.cos(a),math.sin(a);rot=np.array([[c,-s,0],[s,c,0],[0,0,1]])
  translation=np.array([vx*t,vy*t,0.]);moved=[]
  for shape in shapes:
   T=np.array(shape['matrix']);T[:3,3]=rot@T[:3,3]+translation;T[:3,:3]=rot@T[:3,:3];moved.append(dict(shape,matrix=T.tolist()))
  # Add unobserved motion since the oldest cloud to the 5 cm safety interval.
  margin=.05+math.hypot(vx,vy)*cloud_age+revalidation_reserve_m
  hit=self_candidates(p,moved,margin,_tree=tree)
  if hit.any():
   record_collision(hit,margin,float(t))
   # A pre-existing near-wire ambiguity may be left behind, never approached.
   # This is a temporary retreat rule, not a learned self mask or free space.
   ids=np.flatnonzero(hit)
   if not probe_nonapproach or not escape[ids].all():return False,'LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY',cloud_age
   if (distances(p[ids],moved)<baseline[ids]-1e-7).any():return False,'CABLE_PROBE_APPROACHES_UNCONFIRMED_RETURN',cloud_age
 return True,'PROBE_INITIAL_WIRES_DEFERRED_UNTIL_5CM' if probe_deferral is not None and probe_deferral.last_deferred else 'OK_CABLE_PROBE_NONAPPROACH' if escape.any() else 'OK',cloud_age

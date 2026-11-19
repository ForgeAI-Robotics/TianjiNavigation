"""Revalidate an output lease with genuinely measured, geometrically bounded joints."""
import time,itertools
import numpy as np
from execution_safety import posture_check

def tree_motion_weights(geometry):
 """Path-length upper bound for each joint's farthest descendant vertex.

 Rotation preserves lengths. The sum of descendant origin lengths plus a
 collision corner radius bounds the lever arm in EVERY configuration.
 Mimic contributions are summed per source joint, across branches.
 """
 try:
  joints=geometry['joints'];by_child={j['child']:j for j in joints};by_name={j['name']:j for j in joints}
  if len(by_child)!=len(joints) or len(by_name)!=len(joints):return None
  levers={j['name']:0. for j in joints};weights={}
  for s in geometry['shapes']:
   radius=float(np.linalg.norm(s['xyz']))+float(np.linalg.norm(np.array(list(itertools.product(*zip(*s['bounds'])))),axis=1).max());link=s['link'];seen=set()
   while link in by_child:
    if link in seen:return None
    seen.add(link);j=by_child[link];levers[j['name']]=max(levers[j['name']],radius);radius+=float(np.linalg.norm(j['xyz']));link=j['parent']
   if not np.isfinite(radius):return None
  for j in joints:
   if j['type']=='fixed':continue
   if j['type'] not in ('revolute','continuous') or not np.isfinite(j['axis']).all() or np.linalg.norm(j['axis'])<1e-12:return None
   root=j;gain=1.;seen=set()
   while root.get('mimic'):
    if root['name'] in seen:return None
    seen.add(root['name']);gain*=abs(float(root['mimic']['multiplier']));root=by_name[root['mimic']['joint']]
   value=levers[j['name']]*gain
   if not np.isfinite(value):return None
   weights[root['name']]=weights.get(root['name'],0.)+value
  return weights
 except (KeyError,ValueError,TypeError):return None

def motion_weights(geometry):
 """Global triangle-inequality bound on vertex travel per measured radian.

 A downstream lever arm cannot exceed all joint-origin lengths plus the
 largest local collision corner/offset. Sum every dependent rotation, also
 across branches, to remain conservative. Unsupported kinematics use exact FK.
 """
 tight=tree_motion_weights(geometry)
 if tight is not None:return tight
 try:
  joints=geometry['joints'];length=sum(float(np.linalg.norm(j['xyz'])) for j in joints)
  mesh=max(float(np.linalg.norm(s['xyz']))+float(np.linalg.norm(np.array(list(itertools.product(*zip(*s['bounds'])))),axis=1).max()) for s in geometry['shapes'])
  lever=length+mesh;weights={}
  if not np.isfinite(lever) or lever<=0:return None
  for joint in joints:
   if joint['type']=='fixed':continue
   if joint['type'] not in ['revolute','continuous'] or np.linalg.norm(joint['axis'])<1e-12:return None
   mimic=joint.get('mimic');root=mimic['joint'] if mimic else joint['name'];gain=abs(float(mimic['multiplier'])) if mimic else 1.
   weights[root]=weights.get(root,0.)+lever*gain
  return weights
 except (KeyError,ValueError,TypeError):return None

def corner_shift(before,after):
 if len(before)!=len(after):return float('inf')
 maximum=0.
 for a,b in zip(before,after):
  if a['link']!=b['link'] or not np.array_equal(a['bounds'],b['bounds']):return float('inf')
  lo,hi=np.asarray(a['bounds']);p=np.array(list(itertools.product(*zip(lo,hi))));A=np.asarray(a['matrix']);B=np.asarray(b['matrix'])
  delta=p@(A[:3,:3]-B[:3,:3]).T+A[:3,3]-B[:3,3]
  if not np.isfinite(delta).all():return float('inf')
  maximum=max(maximum,float(np.linalg.norm(delta,axis=1).max()))
 return maximum

def joint_shift_bound(shapes,old,current,weights):
 """Conservative measured-joint displacement; unsupported geometry uses FK."""
 if old==current:return 0.
 if weights is None or set(old)!=set(current) or not all(k in weights for k in current) or not all(s.get('mode')=='measured_or_fixed' for s in shapes):return float('inf')
 if not all(np.isfinite([current[k],old[k],weights[k]]).all() and weights[k]>=0 for k in current):return float('inf')
 return sum(weights[k]*abs(current[k]-old[k]) for k in current)

def revalidate(reference,current,saved,model_hash,shape_function,now=None):
 clock=time.time if now is None else lambda:now
 valid,_,_=posture_check(saved,current,model_hash,clock())
 reserve=reference.get('reserve_m',-1.)
 if not valid or not np.isfinite(reserve) or not 0<=reserve<=.005:return None
 q=current['joint_positions_rad'];shift=0.
 if q!=reference['joint_positions_rad']:
  bound=joint_shift_bound(reference['shapes'],reference['joint_positions_rad'],q,reference.get('motion_weights_m'))
  shift=bound if bound<=reserve else corner_shift(reference['shapes'],shape_function(q))
 if shift>reserve:return None
 valid,_,age=posture_check(saved,current,model_hash,clock())
 if not valid:return None
 # All corresponding convex OBB vertices move by at most shift; applying
 # the same certified base twist preserves that bound over the full sweep.
 return dict(age_s=age,observation_time=current['observation_time'],joint_geometry_shift_m=shift)

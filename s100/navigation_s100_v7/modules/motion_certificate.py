"""Conservative SE(2) revalidation inside an already checked motion envelope.

No sensor timestamps or leases are renewed here. A measured base may advance
along the SAME checked constant twist; all remaining base/joint discrepancy
must still fit the original 5mm geometric reserve.
"""
import math,itertools
import numpy as np
from saved_map import base_transform

FUTURE_HORIZON_S=.5
BASE_ADVANCE_S=.35

def twist_pose(twist,t):
 vx,vy,w=map(float,twist);a=w*t
 if abs(w)<1e-10:return [vx*t,vy*t,a]
 return [(vx*math.sin(a)+vy*(math.cos(a)-1))/w,(vx*(1-math.cos(a))+vy*math.sin(a))/w,a]

def shape_radius(shapes):
 corners=[]
 for s in shapes:
  if 'world_corners' in s:p=np.asarray(s['world_corners'],float)
  else:
   lo,hi=np.asarray(s['bounds'],float);local=np.array(list(itertools.product(*zip(lo,hi))));T=np.asarray(s['matrix'],float);p=local@T[:3,:3].T+T[:3,3]
  if not np.isfinite(p).all():raise ValueError('INVALID_CERTIFICATE_GEOMETRY')
  corners.append(p)
 if not corners:raise ValueError('EMPTY_CERTIFICATE_GEOMETRY')
 return float(np.linalg.norm(np.concatenate(corners)[:,:2],axis=1).max())

def certify_motion(shapes,old_odom,new_odom,twist,joint_shift_m,max_advance_s=BASE_ADVANCE_S):
 """Bound ALL vertices at ALL future times, not just sampled poses.

 If measured D differs from checked P by translation e and yaw d, every
 future corner differs by <= |e| + 2*sin(|d|/2)*(radius+speed*horizon),
 plus the independently bounded joint motion. P followed by the same twist
 is a suffix of the extended, collision-checked SE(2) trajectory.
 """
 vx,vy,w=map(float,twist)
 if not np.isfinite([vx,vy,w,joint_shift_m,max_advance_s]).all() or joint_shift_m<0 or not 0<=max_advance_s<=BASE_ADVANCE_S:raise ValueError('INVALID_MOTION_CERTIFICATE_INPUT')
 D=np.linalg.inv(base_transform(old_odom))@base_transform(new_odom)
 if not np.isfinite(D).all():raise ValueError('INVALID_CERTIFICATE_ODOMETRY')
 displacement=D[:2,3];yaw=math.atan2(D[1,0],D[0,0]);speed=math.hypot(vx,vy);radius=shape_radius(shapes)
 def bound(t):
  p=twist_pose(twist,t);rotation_error=yaw-p[2]
  return math.hypot(displacement[0]-p[0],displacement[1]-p[1])+2*abs(math.sin(rotation_error/2))*(radius+speed*FUTURE_HORIZON_S)+1e-12
 if abs(w)<1e-10:
  t=float(np.clip((vx*displacement[0]+vy*displacement[1])/(speed*speed),0,max_advance_s)) if speed>1e-10 else 0.
 elif speed<1e-10:t=float(np.clip(yaw/w,0,max_advance_s))
 else:
  # Search only selects a candidate phase. Safety uses the analytic bound at
  # that phase, so imperfect optimization can reject but cannot false-admit.
  grid=np.linspace(0,max_advance_s,17);cost=[bound(t) for t in grid];i=int(np.argmin(cost));lo=float(grid[max(0,i-1)]);hi=float(grid[min(16,i+1)]);ratio=(math.sqrt(5)-1)/2
  a=hi-ratio*(hi-lo);b=lo+ratio*(hi-lo);fa=bound(a);fb=bound(b)
  for _ in range(24):
   if fa<fb:hi=b;b=a;fb=fa;a=hi-ratio*(hi-lo);fa=bound(a)
   else:lo=a;a=b;fa=fb;b=lo+ratio*(hi-lo);fb=bound(b)
  t=min([float(grid[i]),a,b,0.,max_advance_s],key=bound)
 base_bound=bound(t);total=base_bound+float(joint_shift_m)
 return dict(proven=total<=.005,phase_s=t,base_residual_bound_m=base_bound,joint_shift_bound_m=float(joint_shift_m),total_residual_bound_m=total,actual_base_translation_m=float(np.linalg.norm(displacement)),actual_base_yaw_rad=yaw,checked_horizon_s=FUTURE_HORIZON_S+max_advance_s,max_advance_s=max_advance_s,reference_odometry=list(map(float,old_odom)),measured_odometry=list(map(float,new_odom)),checked_twist=list(map(float,twist)),shape_radius_m=radius)

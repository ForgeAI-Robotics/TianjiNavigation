"""Bounded forward-translation constraints. Never grants general navigation clearance."""
import math,itertools
import numpy as np
from collision_geometry import hull
from clearance_model import receiver_gate,direct_table_gap

def clip_y(poly,low,high):
 result=[np.array(p) for p in poly]
 for bound,sign in [(low,1),(high,-1)]:
  if not result:break
  out=[]
  for a,b in zip(result,result[1:]+result[:1]):
   ia=sign*(a[1]-bound)>=0;ib=sign*(b[1]-bound)>=0
   if ia:out.append(a)
   if ia!=ib:out.append(a+(b-a)*(bound-a[1])/(b[1]-a[1]))
  result=out
 return np.array(result)

def plane_band_polygon(shape,plane,margin):
 a,b,c=plane;v=np.array(shape['world_corners']);dist=v[:,2]-a*v[:,0]-b*v[:,1]-c
 points=[p for p,d in zip(v,dist) if -margin<=d<=margin]
 # world_corners use itertools.product ordering, hence bit-adjacent edges.
 for i,j in itertools.combinations(range(8),2):
  if (i^j) not in (1,2,4):continue
  for h in (-margin,margin):
   if (dist[i]-h)*(dist[j]-h)<0:
    t=(h-dist[i])/(dist[j]-dist[i]);points.append(v[i]+t*(v[j]-v[i]))
 return hull(np.array(points)[:,:2]) if points else []

def table_geometry(shapes,scene,margin=.05):
 if not scene or not scene.get('valid'):return dict(valid=False,reason='TABLE_NOT_OBSERVED')
 plane=np.asarray(scene['plane'],float);k,c=scene['edge'];lo,hi=scene['y_range'];blocking=[];above=[];below=[]
 for s in shapes:
  v=np.array(s['world_corners']);dist=v[:,2]-plane[0]*v[:,0]-plane[1]*v[:,1]-plane[2]
  if dist.max()<-margin:below.append(s['link']);continue
  if dist.min()>margin:above.append(s['link']);continue
  p=plane_band_polygon(s,plane,margin);p=clip_y(p,lo,hi)
  if len(p)<3:continue
  gap=(c-np.max(p[:,0]-k*p[:,1]))/math.sqrt(1+k*k)
  blocking.append(dict(link=s['link'],shape=s['name'],surface_gap_m=float(gap),forward_remaining_m=float(gap*math.sqrt(1+k*k)-margin)))
 if not blocking:return dict(valid=False,reason='NO_BOUNDED_CROSSING_GEOMETRY')
 nearest=min(blocking,key=lambda b:b['forward_remaining_m'])
 return dict(valid=True,reason='OBSERVED_TABLE_PLANE_AND_EDGE_CONSTRAINT',table_height_m=float(plane[2]),blocking=nearest,intersecting_height_band_shapes=blocking,below_plane_links=sorted(set(below)),above_plane_links=sorted(set(above)),forward_remaining_m=max(0.,nearest['forward_remaining_m']),table_collision_risk=nearest['surface_gap_m']<=margin+1e-9,scope='observed planar tabletop edge; underside, legs and unseen volumes are NOT certified')

def upper_forward_constraint(data,now,receive_age,model_match,arm_fresh,current_q,max_horizon=.2):
 gate=receiver_gate(data,now,receive_age,model_match,arm_fresh)
 if not gate['clearance_clear']:return dict(valid=False,forward_remaining_m=0.,reasons=gate['reasons'])
 old=data.get('evaluation_joints_rad',[])
 if len(old)!=14 or len(current_q)!=14 or not all(math.isfinite(v) for v in [*old,*current_q]) or max(abs(a-b) for a,b in zip(old,current_q))>math.radians(.5):return dict(valid=False,forward_remaining_m=0.,reasons=['ARM_POSE_CHANGED_SINCE_CAMERA_EVALUATION'])
 budgets=[]
 for p in data['cameras']:
  direct=direct_table_gap(data.get('assumption'),p['role'],p.get('plane_normal_chassis'),p.get('reason',''))
  gap=p['raw_gap_m'] if direct else p['conservative_gap_m'];available=max(0.,gap-p['margin_m']);reason=p.get('reason','');n=p['plane_normal_chassis']
  if reason.startswith('FINITE_SURFACE_'):
   # Distance between a fixed observed set and a translated body is 1-Lipschitz.
   # Never extrapolate a finite surface into an infinite plane.
   budget=available;method='FINITE_SET_TRANSLATION_DISTANCE_BOUND'
  elif direct:
   if abs(n[2])<.01:return dict(valid=False,forward_remaining_m=0.,reasons=['INVALID_VERTICAL_PLANE'])
   rate=n[0]/n[2];budget=available/(-rate) if rate<0 else max_horizon;method='DIRECT_VERTICAL_PLANE_GAP'
  elif p['role']=='head':
   rate=n[0]/n[2] if abs(n[2])>.01 else float('-inf');budget=available/(-rate) if rate<0 else max_horizon;method='HEAD_VERTICAL_PLANE_GAP'
  else:
   rate=-n[0];budget=available/(-rate) if rate<0 else max_horizon;method='WRIST_ORIENTED_FORWARD_PLANE_GAP'
  budgets.append(dict(role=p['role'],forward_remaining_m=min(max_horizon,max(0.,float(budget))),method=method))
 return dict(valid=True,forward_remaining_m=min(b['forward_remaining_m'] for b in budgets),per_camera=budgets,reasons=[],scope='current observed camera geometry and existing plane assumptions; fixed arm pose, pure +X translation only')

def combined_limit(table,upper,velocity_x=0.,latency=.7,deceleration=.5,max_speed=.15):
 reasons=[]
 if not table.get('valid'):reasons.append('LIDAR_TABLE_CONSTRAINT_INVALID')
 if not upper.get('valid'):reasons.extend(upper.get('reasons') or ['UPPER_FORWARD_CONSTRAINT_INVALID'])
 remaining=min(table.get('forward_remaining_m',0.),upper.get('forward_remaining_m',0.)) if not reasons else 0.
 speed=min(max_speed,max(0.,-deceleration*latency+math.sqrt((deceleration*latency)**2+2*deceleration*remaining)))
 needed=max(0.,velocity_x)*latency+max(0.,velocity_x)**2/(2*deceleration)
 inhibit=bool(reasons or remaining<=1e-6 or needed>=remaining)
 if not reasons and needed>=remaining:reasons.append('BRAKING_DISTANCE_REACHES_LIMIT')
 return dict(valid=not reasons,forward_remaining_m=remaining,max_forward_speed_mps=0. if inhibit else speed,forward_inhibit=inhibit,braking_distance_m=needed,reasons=reasons,hardware_stop_connected=False,general_navigation_permission=False)


def filter_auto_command(values,command_age,header_age,limit):
 """Only lower an existing fresh pure-forward request. All unsupported inputs stop."""
 zero=[0.]*6
 if len(values)!=6 or not all(math.isfinite(v) for v in values):return zero,'INVALID_COMMAND'
 if command_age is None or not 0<=command_age<=.2 or header_age is None or not -.05<=header_age<=.3:return zero,'COMMAND_EXPIRED'
 x,y,z,ax,ay,az=values
 if x<0 or abs(y)>1e-6 or abs(ax)>1e-6 or abs(ay)>1e-6 or abs(az)>1e-6:return zero,'UNSUPPORTED_SIDE_REVERSE_OR_ROTATION'
 if limit.get('forward_inhibit',True):return zero,'JOINT_CLEARANCE_INHIBIT'
 return [min(x,max(0.,limit.get('max_forward_speed_mps',0.))),0.,z,0.,0.,0.],'CLAMPED_FORWARD_REQUEST'

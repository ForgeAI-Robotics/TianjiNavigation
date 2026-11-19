"""Versioned clearance wire contract; invalid or expired input never grants clearance."""
import math
from copy import deepcopy
ROLES=('head','left_wrist','right_wrist')
STATES={'UNKNOWN':0,'CLEAR_ESTIMATE':1,'BLOCKED':2}
SOURCE_TTL=.6
VIEW_TTL=1.2
POLICY='HEAD_BELOW_WRIST_FORWARD_PLANES'
VERTICAL_ASSUMPTION='TABLE_DIRECT_VERTICAL_GAP_WRIST_FORWARD_PLANES'

def direct_table_gap(assumption,role,normal,reason=''):
 if assumption!=VERTICAL_ASSUMPTION:return False
 if reason.startswith('FINITE_SURFACE_') or reason=='MEASURED_WRIST_OBSTACLE':return False
 if role=='head':return True
 return isinstance(normal,list) and len(normal)==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in normal) and abs(normal[2])>=math.cos(math.radians(15))

def finite(value,default=0.):
 try:return float(value) if math.isfinite(float(value)) else default
 except (TypeError,ValueError):return default

def age(now,stamp):
 try:return now-float(stamp)
 except (TypeError,ValueError):return None

def in_age(value,limit):return value is not None and 0<=value<=limit

def build_clearance(packet,now,arm_valid,arm_age,manifest):
 packet=packet or {};bottom=packet.get('bottom_clearance') or {};reports=packet.get('camera_reports') or {};planes=bottom.get('plane_views') or {}
 elapsed=age(now,packet.get('received_at'));evaluation_age=age(now,packet.get('evaluation_received_at'))
 fresh=in_age(elapsed,SOURCE_TTL) and in_age(evaluation_age,SOURCE_TTL)
 match=packet.get('model_sha256')==manifest['source_urdf_sha256']
 reasons=[]
 if not fresh:reasons.append('CLEARANCE_SOURCE_EXPIRED_OR_MISSING')
 if not match:reasons.append('SOURCE_MODEL_MISMATCH')
 if not arm_valid:reasons.append('ARM_FEEDBACK_INVALID_OR_EXPIRED')
 cameras=[]
 for role in ROLES:
  p=planes.get(role) or {};stamp=reports.get(role,{}).get('frame_time');elapsed_view=age(now,stamp)
  direct=direct_table_gap(bottom.get('assumption'),role,(p.get('plane_world') or {}).get('normal'),p.get('reason',''))
  gap=p.get('raw_gap_m') if direct else p.get('conservative_gap_m');gap_valid=isinstance(gap,(int,float)) and math.isfinite(gap)
  decision=p.get('decision_gap_m',gap) if direct else gap
  decision_valid=isinstance(decision,(int,float)) and math.isfinite(decision)
  view_fresh=fresh and in_age(elapsed_view,VIEW_TTL)
  plane=p.get('plane_world') or {};normal=plane.get('normal')
  plane_valid=isinstance(normal,list) and len(normal)==3 and all(isinstance(v,(float,int)) and math.isfinite(v) for v in normal) and math.isfinite(finite(plane.get('offset_m'),float('nan')))
  if plane_valid:plane_valid=abs(sum(v*v for v in normal)-1.)<.02
  margin=finite(p.get('margin_m'),.06)
  threshold=margin+(1e-9 if direct else 0.)
  hazard=bool(p.get('stop_required') or p.get('state')=='BLOCKED' or (decision_valid and decision<=threshold))
  safe_plane=gap_valid and decision_valid and plane_valid and p.get('evaluated_at_current_measured_pose',False) and .01<=margin<=.3 and gap>threshold and decision>threshold
  valid=bool(view_fresh and (hazard or (p.get('state')=='CLEAR_ESTIMATE' and safe_plane)))
  state=(2 if hazard else 1) if valid else 0
  if not valid:reasons.append(role.upper()+'_VIEW_INVALID_OR_EXPIRED')
  cameras.append({'role':role,'state':state,'valid':valid,'stop_required':bool(valid and hazard),
   'gap_valid':bool(valid and gap_valid),'conservative_gap_m':finite(p.get('conservative_gap_m')),'raw_gap_m':finite(p.get('raw_gap_m')),
   'margin_m':finite(p.get('margin_m'),.06),'uncertainty_m':finite(p.get('uncertainty_m')),
   'observation_time':finite(stamp),'observation_age_s':elapsed_view if elapsed_view is not None else -1.,
   'plane_valid':plane_valid and valid,'plane_normal_chassis':[finite(v) for v in normal] if plane_valid else [0.,0.,0.],
   'plane_offset_chassis_m':finite(plane.get('offset_m')),'support_points':int(p.get('plane',{}).get('support_points',0)),
   'inlier_fraction':finite(p.get('plane',{}).get('inlier_fraction')),'residual95_m':finite(p.get('plane',{}).get('residual95_m')),
   'reason':p.get('reason','NO_CURRENT_CAMERA_REPORT') if valid else 'VIEW_INVALID_OR_EXPIRED'})
 observed_block=any(p['stop_required'] for p in cameras) or bool(fresh and match and bottom.get('stop_required'))
 complete=all(p['valid'] and p['state']==1 for p in cameras)
 known_policy=bottom.get('policy')==POLICY
 q=packet.get('evaluation_joints_deg')
 pose_valid=isinstance(q,list) and len(q)==2 and all(isinstance(a,list) and len(a)==7 for a in q)
 pose_valid=pose_valid and all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<=360 for arm in q for v in arm)
 if not pose_valid:reasons.append('EVALUATION_POSE_MISSING_OR_INVALID')
 if not known_policy:reasons.append('UNKNOWN_CLEARANCE_POLICY')
 clear=bool(fresh and match and arm_valid and pose_valid and complete and known_policy and bottom.get('clear') and not observed_block)
 state=2 if observed_block else (1 if clear else 0)
 return {'schema_version':1,'assessment_time':finite(packet.get('received_at')),'evaluation_time':finite(packet.get('evaluation_received_at')),
  'source_id':str(packet.get('source_id','')),'sequence':int(packet.get('sequence',0)),
  'source_fresh':fresh,'model_match':match,'model_version':manifest['model_version'],'model_sha256':manifest['portable_urdf_sha256'],
  'policy':bottom.get('policy',''),'assumption':bottom.get('assumption',''),'state':state,'clear':clear,
  'stop_required':observed_block,'motion_inhibit':not clear,'hardware_stop_connected':False,
  'source_age_s':elapsed if elapsed is not None else -1.,'arm_feedback_age_s':arm_age if arm_age is not None else -1.,
  'evaluation_pose_valid':bool(pose_valid and fresh),'evaluation_joints_rad':[math.radians(v) for arm in q for v in arm] if pose_valid else [0.]*14,
  'cameras':cameras,'reasons':reasons}

class StopRequestLatch:
 def __init__(self):self.stopped=False
 def apply(self,result):
  result=deepcopy(result)
  if result['stop_required']:self.stopped=True
  elif result['clear']:self.stopped=False
  result['stop_required']=self.stopped
  if self.stopped:result.update(clear=False,motion_inhibit=True)
  return result

def receiver_gate(data,now,receive_age,model_match,arm_fresh):
 reasons=[]
 if receive_age is None or not 0<=receive_age<=.3:reasons.append('CLEARANCE_TRANSPORT_TIMEOUT')
 if data:
  elapsed=age(now,data.get('assessment_time'))
  if elapsed is None or not -.1<=elapsed<=SOURCE_TTL:reasons.append('ASSESSMENT_EXPIRED_OR_FUTURE')
  evaluation_age=age(now,data.get('evaluation_time'))
  if evaluation_age is None or not -.1<=evaluation_age<=SOURCE_TTL:reasons.append('EVALUATION_POSE_EXPIRED_OR_FUTURE')
  if data.get('schema_version')!=1:reasons.append('UNSUPPORTED_SCHEMA')
  if data.get('policy')!=POLICY:reasons.append('UNKNOWN_CLEARANCE_POLICY')
  if data.get('state')!=1:reasons.append('SOURCE_STATE_NOT_CLEAR')
  if not data.get('model_match'):reasons.append('SOURCE_MODEL_MISMATCH')
  if not data.get('source_fresh'):reasons.append('SOURCE_NOT_FRESH')
  if not data.get('clear'):reasons.append('SOURCE_NOT_CLEAR')
  if data.get('stop_required'):reasons.append('STOP_REQUEST_LATCHED')
  if data.get('motion_inhibit'):reasons.append('SOURCE_MOTION_INHIBIT')
  if not data.get('evaluation_pose_valid'):reasons.append('EVALUATION_POSE_INVALID')
  cameras=data.get('cameras',[])
  if len(cameras)!=3 or {v.get('role') for v in cameras}!=set(ROLES):reasons.append('MISSING_REQUIRED_CAMERA')
  for p in cameras:
   elapsed=age(now,p.get('observation_time'))
   if not p.get('valid') or p.get('state')!=1 or elapsed is None or not -.1<=elapsed<=VIEW_TTL:reasons.append(p.get('role','CAMERA').upper()+'_OBSERVATION_EXPIRED_OR_INVALID')
   direct=direct_table_gap(data.get('assumption'),p.get('role'),p.get('plane_normal_chassis'),p.get('reason',''))
   gap=finite(p.get('raw_gap_m') if direct else p.get('conservative_gap_m'),float('nan'));margin=finite(p.get('margin_m'),float('nan'))
   if not p.get('gap_valid') or not math.isfinite(gap) or not .01<=margin<=.3 or gap<=margin+(1e-9 if direct else 0.):reasons.append(p.get('role','CAMERA').upper()+'_GAP_INVALID_OR_TOO_SMALL')
   normal=p.get('plane_normal_chassis',[])
   if not p.get('plane_valid') or len(normal)!=3 or not all(math.isfinite(finite(v,float('nan'))) for v in normal):reasons.append(p.get('role','CAMERA').upper()+'_PLANE_INVALID')
 else:reasons.append('NO_CLEARANCE_MESSAGE')
 if not model_match:reasons.append('MODEL_MISMATCH')
 if not arm_fresh:reasons.append('ARM_FEEDBACK_NOT_FRESH')
 return {'clearance_fresh':bool(data) and not any(r in reasons for r in ('CLEARANCE_TRANSPORT_TIMEOUT','ASSESSMENT_EXPIRED_OR_FUTURE','EVALUATION_POSE_EXPIRED_OR_FUTURE','SOURCE_NOT_FRESH')),
  'clearance_clear':not reasons,'motion_inhibit':bool(reasons),'stop_required':bool(data and data.get('stop_required')),'reasons':reasons,'hardware_stop_connected':False}

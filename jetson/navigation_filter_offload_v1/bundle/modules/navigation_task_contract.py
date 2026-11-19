"""Task API contract. No sensors, ROS, SDK, or control imports."""
import copy,hashlib,json,math,re
from pathlib import Path

TERMINAL={'SUCCEEDED','FAILED','CANCELED','INTERRUPTED','PLAN_READY','TEST_COMPLETE'}
class ApiError(Exception):
 def __init__(self,status,code,message=None):
  self.status=status;self.code=code;super().__init__(message or code)

def normalize(kind,payload,default_scene='office'):
 if not isinstance(payload,dict):raise ApiError(422,'JSON_OBJECT_REQUIRED')
 allowed={'goal','scene','translation_cap_mps','timeout_s','client_id'}
 if kind=='plan':allowed={'goal','scene','options','client_id'}
 if kind=='test':allowed|={'duration_s'}
 if set(payload)-allowed:raise ApiError(422,'UNKNOWN_TASK_FIELDS',','.join(sorted(set(payload)-allowed)))
 if type(payload.get('goal'))is not int or payload['goal']not in (1,2,3):raise ApiError(422,'GOAL_MUST_BE_1_2_OR_3')
 scene=payload.get('scene',default_scene)
 if scene not in ('office','outside'):raise ApiError(422,'UNKNOWN_SCENE')
 result=dict(goal=payload['goal'],scene=scene)
 client=payload.get('client_id','agent')
 if not isinstance(client,str)or len(client)>64:raise ApiError(422,'INVALID_CLIENT_ID')
 result['client_id']=client
 if kind=='plan':
  options=dict(ignore_head_camera=True,open_grippers=False,use_saved_arms=False)
  given=payload.get('options',{})
  if not isinstance(given,dict)or set(given)-set(options)or any(type(v)is not bool for v in given.values()):raise ApiError(422,'INVALID_PREVIEW_OPTIONS')
  options.update(given);result['options']=options
 else:
  speed=payload.get('translation_cap_mps',.10);timeout=payload.get('timeout_s',0.)
  if type(speed)not in (int,float)or not math.isfinite(speed)or not .005<=speed<=.10:raise ApiError(422,'SPEED_CAP_MUST_BE_0_005_TO_0_10_MPS')
  if type(timeout)not in (int,float)or not math.isfinite(timeout)or not 0<=timeout<=3600:raise ApiError(422,'INVALID_TASK_TIMEOUT')
  result.update(translation_cap_mps=float(speed),timeout_s=float(timeout))
  if kind=='test':
   duration=payload.get('duration_s',20.)
   if type(duration)not in (int,float)or not math.isfinite(duration)or not 5<=duration<=120:raise ApiError(422,'TEST_DURATION_MUST_BE_5_TO_120_SECONDS')
   result['duration_s']=float(duration)
 return result

def fingerprint(kind,request):return hashlib.sha256(json.dumps([kind,request],sort_keys=True,allow_nan=False).encode()).hexdigest()

class Waypoints:
 def __init__(self,root):self.root=Path(root)
 def list(self,scene):
  if scene not in ('office','outside'):raise ApiError(422,'UNKNOWN_SCENE')
  file=self.root/('config/scenes/office_targets.json'if scene=='office'else'config/targets.json')
  try:rows=json.loads(file.read_text())['targets']
  except (OSError,ValueError,KeyError)as exc:raise ApiError(503,'WAYPOINTS_UNAVAILABLE',str(exc))
  return [copy.deepcopy(r)for r in rows.values()]
 def get(self,scene,goal):
  for row in self.list(scene):
   if row.get('id')==goal:return row
  raise ApiError(422,'GOAL_NOT_CONFIGURED_IN_SCENE')

def terminal_result(kind,code,final,canceled=False,timed_out=False):
 if timed_out:return 'FAILED',dict(code='TASK_TIMEOUT',arrived=False)
 if canceled:return 'CANCELED',dict(code='CANCELED_BY_CLIENT',arrived=False)
 if kind=='plan':
  if code==0 and final and (final.get('selected')or{}).get('geometry_recheck_passed'):
   return 'PLAN_READY',dict(code='CHECKED_PLAN_READY',arrived=False,plan_state=final.get('state'))
  return 'FAILED',dict(code='NO_CHECKED_PLAN'if code==0 else'PLANNING_PROCESS_FAILED',arrived=False)
 if not final:return 'FAILED',dict(code='NAVIGATOR_FINAL_STATUS_MISSING',arrived=False,exit_code=code)
 if code!=0:return 'FAILED',dict(code='NAVIGATOR_PROCESS_FAILED',arrived=False,exit_code=code)
 phase=final.get('phase');finished=final.get('finished')and not final.get('cleanup_pending')
 if kind=='test'and finished and phase in ('TEST_COMPLETE','ARRIVED')and final.get('mode')in ('LOAD_TEST_NO_MOTION','PREVIEW_NO_ROS'):
  return 'TEST_COMPLETE',dict(code='NO_MOTION_TEST_COMPLETE',arrived=False,navigator_phase=phase)
 e=final.get('goal_error')or{}
 valid_error=all(type(e.get(k))in (float,int)and math.isfinite(e[k])and 0<=e[k]<=limit for k,limit in [('distance_m',.05),('heading_error_deg',1.)])
 if kind=='navigate'and finished and phase=='ARRIVED'and final.get('mode')=='REAL_MANUAL_START'and valid_error:
  return 'SUCCEEDED',dict(code='ARRIVED',arrived=True,goal_error=e)
 return 'FAILED',dict(code=final.get('error_code')or final.get('reason')or final.get('error')or phase or'NAVIGATION_INCOMPLETE',arrived=False,navigator_phase=phase)

def status_summary(status):
 names=['active','session','mode','phase','compute_host','updated_wall_time','motion_commands_sent','nonzero_packets','finished','cleanup_pending','reason','error','error_code','goal_error','tracking_mode','limited_twist','requested_twist','settled_s','settle_samples','control_note','final_xyt','final_pose_observation_time','final_pose_evidence','navigation_stop_evidence','fine_pulse_count','fine_no_progress_pulses','translation_cap_mps','route_recovery','operator_session','joint_feedback_health','collision_report','motor_trace_file','blocking_checks','expired_deadlines','obstacle_wait','sensor_recovery','execution_profile','background_worker','command_gate_stats']
 result={k:copy.deepcopy(status[k])for k in names if k in status}
 for key in ['operator_estop_declared','hardware_stop_signal_verified']:
  if key in status:result[key]=status[key]
 if status.get('guard'):
  result['guard']={k:status['guard'][k]for k in ['state','cloud_age_s','localization_age_s','pose_age_s','forced_release_active','path_checked','model_sha256','map_sha256']if k in status['guard']}
 return result

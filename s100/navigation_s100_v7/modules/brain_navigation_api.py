"""Brain-facing projection of the existing S100 task journal and lifecycle."""
import copy,json,math
from pathlib import Path
from brain_navigation_contract import VERSION,stamp,world_binding,config,external_pose
from navigation_task_contract import ApiError,TERMINAL
STATES={'ACCEPTED':'accepted','STARTING':'planning','PLANNING':'planning','WAITING':'waiting_safety_ready','RUNNING':'navigating','CANCELING':'cancelling','SUCCEEDED':'succeeded','FAILED':'failed','CANCELED':'cancelled','INTERRUPTED':'failed'}
class BrainCompatibility:
 def __init__(self,manager,readiness):self.manager=manager;self.readiness=readiness
 def view(self,job,inject=True):
  body=job['brain_command'];r=job.get('result')or{};raw=job.get('navigator_status')or{};state=STATES.get(job['state'],'failed')
  view=dict(copy.deepcopy(body),state=state,accepted_at=stamp(job['created_at']),started_at=stamp(job.get('started_at')),completed_at=stamp(job.get('completed_at')),internal_task_id=job['id'],status_url='/v1/commands/'+body['command_id'],progress=dict(phase=raw.get('phase'),reason=raw.get('reason'),message=raw.get('reason')or raw.get('phase')or job['state']),result=None,map_binding=job['target'].get('coordinate_binding'))
  try:
   from global_plan_preview import read_job_preview
   view['planning_preview']=read_job_preview(self.manager.root,job)
  except (OSError,ValueError,KeyError,TypeError)as exc:
   view['planning_preview']=None;view['planning_preview_error']=str(exc)
  if job['state']in TERMINAL:
   evidence=r.get('stop_evidence')or{}
   from navigation_stop_evidence import valid_certificate
   stopped=(evidence.get('confirmed')is True and evidence.get('source')=='ISOLATED_CONTRACT_EXECUTOR'and evidence.get('physical_execution')is False)if self.manager.simulation else valid_certificate(evidence)
   pose=external_pose(job['target'],r.get('final_xyt'));error=r.get('goal_error')or{}
   finite_error=all(type(error.get(k))in(int,float)and math.isfinite(error[k])and 0<=error[k]<=bound for k,bound in [('distance_m',.05),('heading_error_deg',1.)])
   actual_ok=False
   if pose is not None:
    goal=body['goal_xyt'];distance=math.hypot(pose[0]-goal[0],pose[1]-goal[1]);heading=abs(math.atan2(math.sin(pose[2]-goal[2]),math.cos(pose[2]-goal[2])))
    actual_ok=distance<=.05 and heading<=math.radians(1.)
   success=job['state']=='SUCCEEDED'and r.get('arrived')is True and stopped and pose is not None and finite_error and actual_ok
   if job['state']=='SUCCEEDED'and not success:state='failed';view['state']=state
   view['result']=dict(success=success,reached=r.get('arrived')is True,navigation_stopped=stopped,final_xyt=pose,message=r.get('message')or r.get('code')or state,stop_evidence=evidence,goal_error=error)
   if (raw.get('final_pose_evidence')or{}).get('reason')=='MEASURED_ARRIVAL_BEFORE_PARKING':
    view['result'].update(arrival_pose_basis='BEFORE_PARKING',post_parking_pose_evaluated=False,arrival_observation_time=raw['final_pose_evidence'].get('observation_time'))
   if state=='failed':view['error']=dict(code='STOP_OR_FINAL_POSE_UNCONFIRMED'if job['state']=='SUCCEEDED'and not success else r.get('code','NAVIGATION_INCOMPLETE'),message=r.get('message')or r.get('code')or state,retryable=False,details=dict(original_result=r))
  if self.manager.simulation:
   view.update(simulated=True,physical_execution=False,test_evidence=dict(execution_count=r.get('execution_count',self.manager.runner.execution_count(job['id'])),motion_output_count=r.get('motion_output_count',self.manager.runner.motion_output_count)))
   if inject:view=self.manager.runner.decorate_response(job,view)
  return view
 def status(self):
  blockers=[]
  if not self.manager.motion_enabled:blockers.append('MOTION_DISABLED')
  try:
   data=self.readiness()
   if data.get('arm_fresh')is not True:blockers.append('ARMS_NOT_FRESH')
   if data.get('head_fresh')is not True:blockers.append('HEAD_NOT_FRESH')
   age=data.get('receiver_status_age_s')
   if type(age)not in(int,float)or not -.1<=age<=.6:blockers.append('RECEIVER_STATUS_EXPIRED')
   if not data.get('clearance_clear')or not data.get('shapemask_clear'):blockers.append('CAMERA_CLEARANCE_NOT_READY')
  except Exception as exc:data=dict(error=str(exc));blockers.append('READINESS_UNAVAILABLE')
  try:binding,_,cfg=world_binding(self.manager.root,self.manager.scene)
  except (ApiError,OSError,ValueError)as exc:binding=None;blockers.append(getattr(exc,'code','MAP_BINDING_UNAVAILABLE'))
  with self.manager.condition:
   active=self.manager.jobs.get(self.manager.active)if self.manager.active else None
   command=(active or{}).get('brain_command')or{}
   live=(active or{}).get('navigator_status')or{}
   if not live.get('guard',{}).get('path_checked'):blockers.append('LIVE_ADMISSION_NOT_CONFIRMED')
   result=dict(contract_version=VERSION,interface_online=True,motion_ready=not blockers,motion_blockers=blockers,active_command_id=command.get('command_id'),active_command_state=STATES.get((active or{}).get('state'))if command else None,motion_enabled=self.manager.motion_enabled,readiness=data,map_binding=binding)
  if self.manager.simulation:result['simulation']=dict(enabled=True,physical_execution=False,contract_test=True)
  return result
 def route(self,method,path,body=None):
  if method=='GET'and path=='/health':return 200,dict(contract_version=VERSION,service='tianji-navigation',status='ok',interface_online=True)
  if method=='GET'and path=='/v1/status':return 200,self.status()
  if method=='GET'and path in ['/v1/world','/total_scene_graph_latest.json']:
   binding,refs,cfg=world_binding(self.manager.root,self.manager.scene)
   if path=='/v1/world':return 200,dict(contract_version=VERSION,frame_id='map',relation_graph_url='/total_scene_graph_latest.json',updated_at=cfg['updated_at'],map_sha256=binding['map_sha256'],map_binding=binding,map_axes=cfg.get('map_axes'),goal_region=cfg.get('goal_region'))
   objects=[];legs=[]
   for i,row in enumerate(refs,1):
    name=self.manager.scene+'_waypoint_'+str(row['id']);objects.append(dict(id=name,type='navigation_waypoint'))
    external=external_pose(dict(coordinate_binding=binding),row['pose'])
    legs.append(dict(target_id=name,route_phase='',leg_index=i,goal_xyt=external,motion_mode='forward_path',require_final_orientation=True))
   return 200,dict(frame_id='map',objects=objects,agent_navigation_contract=dict(contract_version=VERSION,frame_id='map',supported_motion_modes=cfg['supported_motion_modes'],unsupported_motion_modes=cfg.get('unsupported_motion_modes',[]),goal_admission=dict(strategy='independent_map_goal',reviewed_coordinates_enforced=False,route_metadata_required=False),legs=legs,map_binding=binding))
  if method=='POST'and path=='/v1/navigation/goals':
   job,new=self.manager.submit_command(body);view=self.view(job,inject=False);view['accepted']=True
   return (202 if new else 200),view
  if method=='GET'and path.startswith('/v1/commands/'):
   from urllib.parse import unquote
   return 200,self.view(self.manager.command(unquote(path[len('/v1/commands/'):])) )
  if method=='POST'and path=='/v1/navigation/cancel':
   if not isinstance(body,dict)or set(body)-{'task_id','command_id','reason'}or not isinstance(body.get('task_id'),str)or not isinstance(body.get('command_id'),str):raise ApiError(422,'INVALID_REQUEST')
   with self.manager.condition:
    job=self.manager.command(body['command_id'])
    if job['brain_command']['task_id']!=body['task_id']:raise ApiError(422,'INVALID_REQUEST','Task/command identity mismatch')
    self.manager.cancel(job['id'])
   return 202,dict(accepted=True,task_id=body['task_id'],command_id=body['command_id'])
  raise ApiError(404,'ENDPOINT_NOT_FOUND')
 def error(self,exc,path,body=None):
  mapping={'MOTION_DISABLED':'SAFETY_NOT_READY','IDEMPOTENCY_KEY_CONFLICT':'COMMAND_ID_CONFLICT','TARGET_CHANGED_BEFORE_START':'SAFETY_NOT_READY'}
  code=mapping.get(exc.code,exc.code);details=dict(original_code=exc.code)
  if isinstance(body,dict):
   details.update({k:body[k]for k in ['task_id','command_id']if k in body})
   old=self.manager.commands.get(body.get('command_id'))
   details.update(action_started=False if old is None else None,action_state_uncertain=old is not None)
  with self.manager.condition:
   active=self.manager.jobs.get(self.manager.active)if self.manager.active else None
   if active and active.get('brain_command'):details['active_command_id']=active['brain_command']['command_id']
  return exc.status,dict(success=False,error=dict(code=code,message=str(exc),retryable=False,details=details))

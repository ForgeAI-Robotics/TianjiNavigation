"""Nonphysical executor injected into the SAME task broker for contract testing.
Never imports a robot SDK/ROS or launches any subprocess. Faults live here only.
"""
import copy,time,threading,collections
class ContractTestRunner:
 simulation=True
 def __init__(self):self.calls=0;self.counts=collections.Counter();self.lock=threading.Lock();self.motion_output_count=0
 def execution_count(self,ident):
  with self.lock:return self.counts[ident]
 def run(self,job,cancel,emit):
  with self.lock:self.calls+=1;self.counts[job['id']]+=1
  body=job['brain_command'];target=body['target_id'];emit('navigator',dict(session=job['id'],phase='FOLLOWING',reason='ISOLATED_CONTRACT_EXECUTOR',updated_wall_time=time.time()))
  if target in ('contract_cancel','contract_pending'):cancel.wait(300)
  else:cancel.wait(.25)
  stopped=dict(confirmed=True,source='ISOLATED_CONTRACT_EXECUTOR',physical_execution=False,motion_output_count=0)
  result=dict(code='CANCELED_BY_CLIENT'if cancel.is_set()else'CONTRACT_FIXTURE_SUCCESS',arrived=not cancel.is_set(),stop_evidence=stopped,final_xyt=job['target']['pose'],goal_error=dict(distance_m=0.,heading_error_deg=0.),execution_count=self.execution_count(job['id']),motion_output_count=self.motion_output_count,physical_execution=False)
  if cancel.is_set():return 'CANCELED',result
  if target=='contract_failure':result.update(code='NAVIGATION_EXECUTION_FAILED',arrived=False);return 'FAILED',result
  return 'SUCCEEDED',result
 def decorate_response(self,job,view):
  target=job['brain_command']['target_id']
  if view['result'] is not None:
   if target=='contract_stop_unknown':view['result']['navigation_stopped']=False
   if target=='contract_bad_time':view['completed_at']='not-a-timestamp'
  if target=='contract_foreign':view['task_id']='foreign-task'
  return view

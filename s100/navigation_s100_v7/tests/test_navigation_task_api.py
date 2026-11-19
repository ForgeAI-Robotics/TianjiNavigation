import copy,json,math,os,sys,tempfile,threading,time,unittest,uuid
from pathlib import Path
from unittest.mock import patch
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'modules'));sys.path.insert(0,str(R/'sdk'))
from navigation_task_contract import ApiError,normalize,terminal_result,TERMINAL
from navigation_task_runtime import TaskManager,NavigatorRunner
from navigation_task_api import make_server,token_file
from tianji_navigation import NavigationClient,NavigationApiError

def setup_root(root):
 (root/'config/scenes').mkdir(parents=True);rows={str(n):dict(id=n,pose=[n*.1,0.,0.],revision=1,map_voxel_sha256='a'*64)for n in (1,2,3)}
 (root/'config/scenes/office_targets.json').write_text(json.dumps(dict(targets=rows)))
 (root/'config/targets.json').write_text(json.dumps(dict(targets={k:v for k,v in rows.items()if k!='3'})))

class FakeRunner:
 def __init__(self,root,hold=False):self.root=root;self.hold=hold;self.calls=0;self.started=threading.Event()
 def run(self,job,cancel,emit):
  self.calls+=1;emit('navigator',dict(session=job['id'],phase='WAITING_FOR_OBSTACLE_CLEAR',reason='OBSTACLE',updated_wall_time=50.,mode='REAL_MANUAL_START'));self.started.set()
  if self.hold:
   cancel.wait(3.)
   if cancel.is_set():return 'CANCELED',dict(arrived=False,code='CANCELED_BY_CLIENT')
  time.sleep(.02)
  if job['kind']=='plan':
   folder=self.root/'results'/job['id']/'planning';folder.mkdir(parents=True);plan=dict(target=job['target'],selected=dict(geometry_recheck_passed=True),state='PATH_PREVIEW_READY');file=folder/'plan.json';file.write_text(json.dumps(plan));return 'PLAN_READY',dict(arrived=False,artifacts=dict(plan_file=str(file)))
  return 'SUCCEEDED',dict(arrived=True,code='ARRIVED')

class TaskContractTests(unittest.TestCase):
 def test_goal_only_request_has_bounded_defaults(self):
  q=normalize('navigate',{'goal':2});self.assertEqual(q['scene'],'office');self.assertEqual(q['translation_cap_mps'],.1)
 def test_agent_cannot_inject_pose_velocity_or_force_clear(self):
  for field in ['pose','cmd_vel','force_clear','ignore_collisions','open_grippers','use_saved_arms','enable_motion']:
   with self.assertRaises(ApiError):normalize('navigate',dict(goal=1,**{field:True}))
 def test_invalid_numbers_and_booleans_are_not_goals_or_speeds(self):
  for goal in [True,1.,0,4,'1']:
   with self.assertRaises(ApiError):normalize('navigate',dict(goal=goal))
  for v in [True,float('nan'),float('inf'),.101,.004]:
   with self.assertRaises(ApiError):normalize('navigate',dict(goal=1,translation_cap_mps=v))
 def test_success_requires_arrival_finalization_and_actual_tolerance(self):
  final=dict(phase='ARRIVED',finished=True,cleanup_pending=False,mode='REAL_MANUAL_START',goal_error=dict(distance_m=.04,heading_error_deg=.8))
  self.assertEqual(terminal_result('navigate',0,final)[0],'SUCCEEDED')
  for field,value in [('finished',False),('cleanup_pending',True),('mode','PREVIEW_NO_ROS'),('goal_error',dict(distance_m=.06,heading_error_deg=.8))]:
   self.assertEqual(terminal_result('navigate',0,dict(final,**{field:value}))[0],'FAILED')
  self.assertEqual(terminal_result('navigate',1,final)[0],'FAILED')
 def test_plan_and_test_completion_never_claim_navigation_success(self):
  state,result=terminal_result('plan',0,dict(selected=dict(geometry_recheck_passed=True),state='AT_TARGET'));self.assertEqual(state,'PLAN_READY');self.assertFalse(result['arrived'])
  self.assertEqual(terminal_result('plan',0,dict(selected=None))[0],'FAILED')
  state,result=terminal_result('test',0,dict(phase='TEST_COMPLETE',finished=True,mode='LOAD_TEST_NO_MOTION'));self.assertEqual(state,'TEST_COMPLETE');self.assertFalse(result['arrived'])
  self.assertEqual(terminal_result('test',0,dict(phase='TEST_COMPLETE',finished=True,mode='REAL_MANUAL_START'))[0],'FAILED')
 def test_waiting_or_missing_final_status_is_not_success(self):
  for p in [None,dict(phase='WAITING_FOR_OBSTACLE_CLEAR',finished=True)]:self.assertEqual(terminal_result('navigate',0,p)[0],'FAILED')
 def test_default_runner_never_implicitly_enables_motion(self):
  runner=NavigatorRunner(Path('/unused'));job=dict(id='a'*32,kind='navigate',request=normalize('navigate',dict(goal=1)))
  with self.assertRaises(ApiError):runner.command(job)
  job['kind']='test';job['request']=normalize('test',dict(goal=1));args,env=runner.command(job);self.assertNotIn('--enable-motion',args);self.assertIn('--load-test-no-motion',args);self.assertIn('--operator-estop-ready',args);self.assertFalse(runner.motion_enabled);self.assertEqual(env['TIANJI_NAV_SCENE'],'office')
 def test_token_is_private_and_not_logged(self):
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp)/'token';value=token_file(p);self.assertGreaterEqual(len(value),24);self.assertEqual(p.stat().st_mode&0o777,0o600);self.assertEqual(token_file(p),value)

class TaskManagerTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name);setup_root(self.root);self.runner=FakeRunner(self.root,True);self.manager=TaskManager(self.root,True,True,runner=self.runner,busy=lambda:False);self.addCleanup(self.manager.close)
 def finish(self,ident):
  end=time.monotonic()+3.
  while time.monotonic()<end:
   j=self.manager.get(ident)
   if j['state']in TERMINAL:return j
   time.sleep(.01)
  self.fail('task did not finish')
 def test_idempotency_and_one_active_task_even_for_multiple_clients(self):
  job,new=self.manager.submit('navigate',dict(goal=1),'same');again,new2=self.manager.submit('navigate',dict(goal=1),'same');self.assertTrue(new);self.assertFalse(new2);self.assertEqual(job['id'],again['id'])
  with self.assertRaises(ApiError)as e:self.manager.submit('navigate',dict(goal=2),'different')
  self.assertEqual(e.exception.code,'NAVIGATION_BUSY')
  with self.assertRaises(ApiError)as e:self.manager.submit('navigate',dict(goal=2),'same')
  self.assertEqual(e.exception.code,'IDEMPOTENCY_KEY_CONFLICT');self.assertTrue(self.runner.started.wait(1));self.assertEqual(self.runner.calls,1)
  self.manager.cancel(job['id']);self.assertEqual(self.finish(job['id'])['state'],'CANCELED')
 def test_client_disconnection_does_not_cancel_task(self):
  self.runner.hold=False;job,_=self.manager.submit('navigate',dict(goal=1));self.assertEqual(self.finish(job['id'])['state'],'SUCCEEDED')
 def test_cancel_is_idempotent_and_waiting_task_is_not_failed(self):
  j,_=self.manager.submit('navigate',dict(goal=2));self.runner.started.wait(1);self.assertEqual(self.manager.get(j['id'])['state'],'WAITING');self.manager.cancel(j['id']);self.manager.cancel(j['id']);done=self.finish(j['id']);self.assertEqual(done['state'],'CANCELED');self.assertFalse(done['result']['arrived'])
 def test_event_cursor_and_artifacts_and_persisted_terminal_state(self):
  self.runner.hold=False;j,_=self.manager.submit('plan',dict(goal=3));done=self.finish(j['id']);events=self.manager.events(j['id']);self.assertTrue(events['events']);self.assertFalse(self.manager.events(j['id'],events['next_sequence'])['events']);self.assertEqual(self.manager.plan(j['id'])['target']['id'],3)
  saved=json.loads((self.root/'runtime/task_api/tasks'/j['id']/'task.json').read_text());self.assertEqual(saved['state'],'PLAN_READY');self.assertEqual(done['navigator_status']['updated_wall_time'],50.)
 def test_scene_goal_is_validated_against_scene_configuration(self):
  with self.assertRaises(ApiError)as e:self.manager.submit('plan',dict(goal=3,scene='outside'))
  self.assertEqual(e.exception.code,'GOAL_NOT_CONFIGURED_IN_SCENE')
 def test_terminal_task_releases_slot_before_next_task_is_submitted(self):
  self.runner.hold=False
  for goal in [1,2,3,1]:
   job,_=self.manager.submit('plan',dict(goal=goal));self.assertEqual(self.finish(job['id'])['state'],'PLAN_READY');self.assertIsNone(self.manager.active)
 def test_restart_marks_incomplete_work_interrupted_without_restarting_it(self):
  folder=self.root/'runtime/task_api/tasks'/('f'*32);folder.mkdir();record=dict(id='f'*32,state='RUNNING',kind='navigate',revision=1,created_at=1.,request=dict(goal=1),fingerprint='old',idempotency_key='old-key');(folder/'task.json').write_text(json.dumps(record))
  other=TaskManager(self.root,runner=self.runner,busy=lambda:False)
  try:self.assertEqual(other.get('f'*32)['state'],'INTERRUPTED');self.assertEqual(self.runner.calls,0)
  finally:other.close()

class HttpApiTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);setup_root(self.root);self.runner=FakeRunner(self.root);self.manager=TaskManager(self.root,runner=self.runner,busy=lambda:False);self.token='t'*32;self.server=make_server(self.manager,self.token,port=0,readiness=lambda:dict(arm_fresh=True));self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start();self.url='http://127.0.0.1:'+str(self.server.server_port);self.client=NavigationClient(self.url,self.token)
 def tearDown(self):self.server.shutdown();self.server.server_close();self.thread.join(2);self.manager.close();self.tmp.cleanup()
 def test_auth_and_motion_disabled_and_no_velocity_endpoint(self):
  with self.assertRaises(NavigationApiError)as e:NavigationClient(self.url,'bad').capabilities()
  self.assertEqual(e.exception.status,401)
  with self.assertRaises(NavigationApiError)as e:self.client.navigate(1)
  self.assertEqual(e.exception.body['error']['code'],'MOTION_DISABLED');self.assertEqual(self.runner.calls,0)
  with self.assertRaises(NavigationApiError)as e:self.client.request('POST','/v1/cmd_vel',dict(vx=.1))
  self.assertEqual(e.exception.status,404)
 def test_goal_only_plan_round_trip_sdk_and_waypoints(self):
  job=self.client.plan(3,idempotency_key='sdk-plan');done=self.client.wait(job['id'],timeout=2);self.assertEqual(done['state'],'PLAN_READY');self.assertFalse(done['result']['arrived']);self.assertEqual(self.client.plan(3,idempotency_key='sdk-plan')['id'],job['id']);self.assertEqual(self.client.plan_result(job['id'])['target']['id'],3);self.assertEqual(len(self.client.waypoints()['waypoints']),3)
 def test_cancel_long_poll_and_request_journal_does_not_contain_token(self):
  self.runner.hold=True;j=self.client.plan(2);self.client.cancel(j['id']);done=self.client.wait(j['id'],timeout=2);self.assertEqual(done['state'],'CANCELED');text=(self.root/'runtime/task_api/requests.jsonl').read_text();self.assertNotIn(self.token,text)
 def test_unknown_fields_and_missing_goal_fail_before_runner(self):
  for body in [{},{'goal':True},{'goal':1,'force_clear':True}]:
   with self.assertRaises(NavigationApiError)as e:self.client.request('POST','/v1/navigation/tasks',body)
   self.assertEqual(e.exception.status,422)
  self.assertEqual(self.runner.calls,0)

class RunnerProcessTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
  script='''#!/usr/bin/python3
import sys,json,time
args=sys.argv;session=args[args.index('--session')+1];count=0
def emit(phase,finished=False):
 print('STATUS '+json.dumps(dict(session=session,phase=phase,finished=finished,cleanup_pending=False,mode='LOAD_TEST_NO_MOTION',updated_wall_time=time.time(),motion_commands_sent=0)),flush=True)
for line in sys.stdin:
 data=json.loads(line)
 if data.get('kind')=='stop':emit('STOPPED_BY_OPERATOR',True);break
 count+=1;emit('PREVIEW_READY')
 if count>=3:emit('TEST_COMPLETE',True);break
'''
  p=self.root/'execute.sh';p.write_text(script);p.chmod(0o700)
 def job(self):return dict(id=uuid.uuid4().hex,kind='test',request=normalize('test',dict(goal=1)),target={})
 def test_real_subprocess_transport_owns_heartbeat_and_consumes_final_status(self):
  events=[];state,result=NavigatorRunner(self.root).run(self.job(),threading.Event(),lambda k,d:events.append((k,d)))
  self.assertEqual(state,'TEST_COMPLETE');self.assertFalse(result['arrived']);self.assertEqual(result['exit_code'],0);self.assertTrue(any(k=='navigator'and d.get('finished')for k,d in events))
 def test_task_timeout_requests_stop_and_is_not_reported_as_success(self):
  job=self.job();job['request']['timeout_s']=.05;state,result=NavigatorRunner(self.root).run(job,threading.Event(),lambda *_:None)
  self.assertEqual(state,'FAILED');self.assertEqual(result['code'],'TASK_TIMEOUT')
 def test_failed_preflight_stderr_is_kept_without_any_status_record(self):
  p=self.root/'preview_plan.sh';p.write_text('#!/usr/bin/python3\nimport sys\nprint("FRESH_ARM_FEEDBACK_REQUIRED",file=sys.stderr)\nsys.exit(1)\n');p.chmod(0o700)
  job=self.job();job['kind']='plan';job['request']=normalize('plan',dict(goal=1));state,result=NavigatorRunner(self.root).run(job,threading.Event(),lambda *_:None)
  self.assertEqual(state,'FAILED');self.assertIn('FRESH_ARM_FEEDBACK_REQUIRED',result['stderr_tail'])
 def test_failure_to_record_spawn_stops_owned_child(self):
  def fail(*_):raise RuntimeError('journal failed')
  with self.assertRaisesRegex(RuntimeError,'journal failed'):NavigatorRunner(self.root).run(self.job(),threading.Event(),fail)
 def test_failed_preflight_is_saved_even_before_planning_result_folder_exists(self):
  setup_root(self.root);p=self.root/'preview_plan.sh';p.write_text('#!/usr/bin/python3\nimport sys\nprint("FRESH_ARM_FEEDBACK_REQUIRED",file=sys.stderr)\nsys.exit(1)\n');p.chmod(0o700)
  manager=TaskManager(self.root,busy=lambda:False)
  try:
   j,_=manager.submit('plan',dict(goal=1));end=time.monotonic()+3
   while time.monotonic()<end and manager.get(j['id'])['state']not in TERMINAL:time.sleep(.01)
   done=manager.get(j['id']);self.assertEqual(done['state'],'FAILED');text=(self.root/'runtime/task_api/tasks'/j['id']/'task.json').read_text();self.assertIn('FRESH_ARM_FEEDBACK_REQUIRED',text)
  finally:manager.close()

if __name__=='__main__':unittest.main()

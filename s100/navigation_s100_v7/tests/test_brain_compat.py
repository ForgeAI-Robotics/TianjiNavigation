import copy,json,hashlib,math,os,sys,tempfile,threading,time,unittest
from pathlib import Path
from unittest.mock import patch
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'modules'))
from navigation_task_contract import ApiError
from navigation_task_runtime import TaskManager,NavigatorRunner
from brain_navigation_contract import normalize,coordinate_target,external_pose,VERSION
from brain_navigation_api import BrainCompatibility
from brain_contract_fixture import ContractTestRunner
from navigation_stop_evidence import assess

def body(cid='cmd-1',target='contract_success'):
 return dict(contract_version=VERSION,task_id='business-1',command_id=cid,target_id=target,route_phase='',leg_index=1,frame_id='map',goal_xyt=[1.2,-.8,1.57],motion_mode='forward_path',require_final_orientation=True)
def setup(root):
 (root/'config/scenes').mkdir(parents=True);folder=root/'map';folder.mkdir();(folder/'voxels.npz').write_bytes(b'test-map');sha=hashlib.sha256(b'test-map').hexdigest();(folder/'map.json').write_text(json.dumps(dict(voxel_file_sha256=sha)))
 refs={str(n):dict(id=n,name=str(n),pose=[n*.1,0.,0.],revision=1,map_folder=str(folder),map_voxel_sha256=sha,frame_id='session_base_start')for n in [1,2,3]}
 (root/'config/scenes/office_targets.json').write_text(json.dumps(dict(targets=refs)))
 cfg=dict(enabled=True,contract_version=VERSION,scene='office',internal_frame_id='session_base_start',map_sha256=sha,T_internal_map_xyt=[0.,0.,0.],map_definition='Explicit fixed test-map alias',supported_motion_modes=['forward_path'],translation_cap_mps=.1,timeout_s=0.,updated_at='2026-11-10T00:00:00+00:00')
 (root/'config/brain_compat.json').write_text(json.dumps(cfg));return cfg
class ContractTests(unittest.TestCase):
 def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.cfg=setup(self.root)
 def tearDown(self):self.tmp.cleanup()
 def test_isolated_navigator_rejects_real_execution_before_io(self):
  import types,onboard_navigation
  (self.root/'ISOLATED_CONTRACT_TEST_ROOT').write_text('test')
  with patch.object(onboard_navigation,'ROOT',self.root):
   with self.assertRaisesRegex(ValueError,'ISOLATED_ROOT_CANNOT_ENABLE_PHYSICAL_MOTION'):onboard_navigation.run(types.SimpleNamespace(enable_motion=True))
 def test_exact_fields_and_identity(self):
  a=body();self.assertEqual(normalize(a),a)
  for key in a:
   b=copy.deepcopy(a);b.pop(key)
   with self.assertRaises(ApiError):normalize(b)
  for key,value,code in [('goal_xyt',[1,True,0],'INVALID_GOAL_XYT'),('goal_xyt',[1,float('nan'),0],'INVALID_GOAL_XYT'),('frame_id','odom','INVALID_FRAME_ID'),('motion_mode','anything','INVALID_MOTION_MODE'),('task_id','../bad','INVALID_REQUEST'),('require_final_orientation',False,'INVALID_REQUEST')]:
   b=copy.deepcopy(a);b[key]=value
   with self.assertRaises(ApiError)as e:normalize(b)
   self.assertEqual(e.exception.code,code)
 def test_direct_coordinate_not_nearest_waypoint(self):
  t=coordinate_target(self.root,'office',body());self.assertEqual(t['id'],0);self.assertEqual(t['pose'],[1.2,-.8,1.57]);self.assertNotEqual(t['pose'],[.1,0,0])
 def test_nonidentity_transform_roundtrip(self):
  self.cfg['T_internal_map_xyt']=[2.,3.,.7];(self.root/'config/brain_compat.json').write_text(json.dumps(self.cfg));t=coordinate_target(self.root,'office',body());p=external_pose(t,t['pose'])
  for a,b in zip(p,body()['goal_xyt']):self.assertAlmostEqual(a,b)
 def test_coordinate_child_binding_is_task_local_and_sealed(self):
  import targets
  t=coordinate_target(self.root,'office',body());file=self.root/'bound.json';blob=json.dumps(t,sort_keys=True,separators=(',',':'));file.write_text(blob);before=(self.root/'config/scenes/office_targets.json').read_bytes()
  with patch.object(targets,'ROOT',self.root),patch.object(targets,'FILE',self.root/'config/scenes/office_targets.json'),patch.dict(os.environ,dict(TIANJI_NAV_SCENE='office',TIANJI_NAV_COORDINATE_TARGET_FILE=str(file),TIANJI_NAV_COORDINATE_TARGET_SHA256=hashlib.sha256(blob.encode()).hexdigest())):
   self.assertEqual(targets.get(0)['pose'],body()['goal_xyt']);self.assertEqual(targets.get(1)['pose'],[.1,0,0]);self.assertTrue(targets.unchanged(t))
   file.write_text('{}');self.assertFalse(targets.unchanged(t))
  self.assertEqual(before,(self.root/'config/scenes/office_targets.json').read_bytes())
 def test_stale_map_rejected(self):
  (self.root/'map/voxels.npz').write_bytes(b'changed')
  with self.assertRaises(ApiError):coordinate_target(self.root,'office',body())
 def test_unsupported_mode_explicitly_rejected(self):
  b=body();b['motion_mode']='lateral_path_aligned'
  with self.assertRaises(ApiError)as e:coordinate_target(self.root,'office',b)
  self.assertEqual(e.exception.code,'INVALID_MOTION_MODE')
 def test_readonly_submission_has_no_runner_calls(self):
  runner=ContractTestRunner();runner.simulation=False;m=TaskManager(self.root,runner=runner)
  try:
   with self.assertRaises(ApiError)as e:m.submit_command(body())
   self.assertEqual(e.exception.code,'MOTION_DISABLED');self.assertEqual(runner.calls,0)
  finally:m.close()
 def test_runner_coordinate_binding_does_not_rewrite_goals(self):
  before=(self.root/'config/scenes/office_targets.json').read_bytes();t=coordinate_target(self.root,'office',body());runner=NavigatorRunner(self.root,True,True);job=dict(id='a'*32,kind='navigate',request=dict(scene='office',goal=0,translation_cap_mps=.1),target=t,brain_command=body())
  args,env=runner.command(job);self.assertIn('0',args);self.assertIn('TIANJI_NAV_COORDINATE_TARGET_SHA256',env);self.assertEqual(before,(self.root/'config/scenes/office_targets.json').read_bytes())
 def test_success_requires_physical_stop_and_actual_pose(self):
  runner=ContractTestRunner();runner.simulation=False;m=TaskManager(self.root,runner=runner);api=BrainCompatibility(m,lambda:{})
  try:
   job=dict(id='a'*32,brain_command=body(),target=coordinate_target(self.root,'office',body()),state='SUCCEEDED',created_at=1.,started_at=2.,completed_at=3.,result=dict(arrived=True,goal_error=dict(distance_m=0.,heading_error_deg=0.),final_xyt=[1.2,-.8,1.57]))
   self.assertEqual(api.view(job)['state'],'failed');job['result']['stop_evidence']=assess([dict(kind='feedback',received_wall_time=10.1+i*.06,source_stamp=10.1+i*.06,wheel=[[j,0.,0]for j in range(4)])for i in range(5)],10.,10.4);self.assertEqual(api.view(job)['state'],'succeeded')
   job['navigator_status']=dict(final_pose_evidence=None);self.assertEqual(api.view(job)['state'],'succeeded')
   job['navigator_status']=dict(final_pose_evidence=dict(reason='MEASURED_ARRIVAL_BEFORE_PARKING',observation_time=2.9))
   projected=api.view(job)['result'];self.assertEqual(projected['arrival_pose_basis'],'BEFORE_PARKING');self.assertFalse(projected['post_parking_pose_evaluated']);self.assertEqual(projected['arrival_observation_time'],2.9)
   job['result']['final_xyt']=[5.,5.,0.];self.assertEqual(api.view(job)['state'],'failed')
  finally:m.close()
class LifecycleTests(unittest.TestCase):
 def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);setup(self.root);self.runner=ContractTestRunner();self.m=TaskManager(self.root,runner=self.runner)
 def tearDown(self):self.m.close();self.tmp.cleanup()
 def finish(self,j):
  end=time.monotonic()+3
  while time.monotonic()<end:
   r=self.m.get(j['id'])
   if r['state']in ['SUCCEEDED','CANCELED','FAILED']:return r
   time.sleep(.01)
  self.fail('did not finish')
 def test_durable_dedup_conflict_and_query(self):
  j,new=self.m.submit_command(body());self.assertTrue(new);saved=json.loads((self.root/'runtime/task_api/tasks'/j['id']/'task.json').read_text());self.assertEqual(saved['brain_command'],body())
  same,new=self.m.submit_command(body());self.assertFalse(new);self.assertEqual(same['id'],j['id']);changed=body();changed['goal_xyt'][0]=1.3
  with self.assertRaises(ApiError)as e:self.m.submit_command(changed)
  self.assertEqual(e.exception.code,'COMMAND_ID_CONFLICT');self.finish(j);self.assertEqual(self.runner.calls,1)
 def test_journal_failure_never_executes_or_loses_in_memory_identity(self):
  with patch.object(self.m.store,'save',side_effect=ApiError(503,'TASK_JOURNAL_ERROR')):
   with self.assertRaises(ApiError):self.m.submit_command(body())
  old=self.m.command('cmd-1');self.assertEqual(old['state'],'FAILED');same,new=self.m.submit_command(body());self.assertFalse(new);self.assertEqual(old['id'],same['id']);self.assertEqual(self.runner.calls,0)
 def test_restart_keeps_command_query_and_no_replay(self):
  j,_=self.m.submit_command(body());self.finish(j);self.m.close();self.runner=ContractTestRunner();self.m=TaskManager(self.root,runner=self.runner);self.assertEqual(self.m.command('cmd-1')['id'],j['id']);self.m.submit_command(body());self.assertEqual(self.runner.calls,0)
 def test_busy_cancel_late_cancel_is_scoped(self):
  j,_=self.m.submit_command(body(target='contract_cancel'))
  with self.assertRaises(ApiError):self.m.submit_command(body('cmd-2'))
  self.m.cancel(j['id']);self.finish(j);second,_=self.m.submit_command(body('cmd-2'));self.m.cancel(j['id']);self.assertEqual(self.finish(second)['state'],'SUCCEEDED')
 def test_get_missing_does_not_create(self):
  with self.assertRaises(ApiError):self.m.command('absent')
  self.assertEqual(len(self.m.jobs),0);self.assertEqual(self.runner.calls,0)
 def test_legacy_motion_remains_disabled_in_fixture(self):
  with self.assertRaises(ApiError):self.m.submit('navigate',dict(goal=1))
class StopTests(unittest.TestCase):
 def rows(self,speed=0.):return [dict(kind='feedback',received_wall_time=10.1+i*.06,source_stamp=10.1+i*.06,wheel=[[j,speed,0]for j in range(4)])for i in range(5)]
 def test_final_pose_must_be_post_stop_and_same_map(self):
  from navigation_stop_evidence import final_pose
  pkt=dict(running=True,map_pose_valid=True,updated_wall_time=10.4,odometry_age_s=0.,lidar_fix_age_s=0.,map_voxel_sha256='a',source_boot_id='b',odometry_epoch=0,calibration_sha256='c',observation_time=10.35,T_map_base=[[1.,0.,0.,1.2],[0.,1.,0.,-.8],[0.,0.,1.,0.],[0.,0.,0.,1.]])
  record=dict(map_voxel_sha256='a',localization=dict(source_boot_id='b',odometry_epoch=0,calibration_sha256='c'))
  r=final_pose(pkt,record,[1.2,-.8,0.],10.,10.4);self.assertTrue(r['valid']);self.assertEqual(r['goal_error']['distance_m'],0.)
  pkt['observation_time']=9.9;self.assertFalse(final_pose(pkt,record,[1.2,-.8,0.],10.,10.4)['valid'])
  pkt['observation_time']=10.35;pkt['map_voxel_sha256']='foreign';self.assertFalse(final_pose(pkt,record,[1.2,-.8,0.],10.,10.4)['valid'])
 def test_fresh_feedback_after_stop(self):self.assertTrue(assess(self.rows(),10.,10.4)['confirmed'])
 def test_exit_or_zero_target_not_stop_proof(self):self.assertFalse(assess([],10.,10.4)['confirmed'])
 def test_stale_nonzero_and_pre_stop_do_not_pass(self):
  for rows,cutoff,now in [(self.rows(.001),10.,10.4),(self.rows(),9.,20.),(self.rows(),10.5,10.6)]:self.assertFalse(assess(rows,cutoff,now)['confirmed'])
 def test_repeated_old_source_stamp_not_fresh_samples(self):
  rows=self.rows()
  for r in rows:r['source_stamp']=10.1
  self.assertFalse(assess(rows,10.,10.4)['confirmed'])
if __name__=='__main__':unittest.main()

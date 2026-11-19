import unittest,tempfile,sys,json,copy,itertools,time,hashlib
from pathlib import Path
from unittest.mock import patch
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'modules'))
import paths
from manual_floor import candidate_mask
from prepare import reference_grid
from cable_precheck import deferred_mask,probe_prefix,check_prefix
from route_micro_probe import RouteMicroProbe
from motion_consistency import DualFrameObserver
import motion_cables

class ManualFloorTests(unittest.TestCase):
 def test_manual_floor_supplies_unknown_support_but_never_clears_obstacles(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);(p/'map.json').write_text('{}');np.savez(p/'voxels.npz',points=np.array([[0,0,0.]]))
   r=dict(points=[[.8,0,.2],[.9,0,1.]],grid_search_lower_m=[-1,-1],grid_search_upper_m=[2,1],preview_start_map_pose=[0,0,0],map_file=str(p/'map.json'),ground_plane=[0,0,0],ground_band_m=.02,local_snapshot_only=True)
   before,origin,res=reference_grid(r);r['manual_floor']=dict(enabled=True,bounds_map_m=[-.9,1.5,-.9,.9]);after,_,_=reference_grid(r)
   def cell(x,y):return int((y-origin[1])/res),int((x-origin[0])/res)
   self.assertEqual(before[cell(1.2,0)],-1);self.assertGreaterEqual(after[cell(1.2,0)],0);self.assertEqual(after[cell(.8,0)],100)
   self.assertEqual(r['points'],[[.8,0,.2],[.9,0,1.]])
 def test_rotated_map_boundary_does_not_fill_outside_polygon(self):
  m=candidate_mask(np.array([0.,1.,-1.]),np.zeros(3),[1,0,np.pi/2],dict(enabled=True,bounds_map_m=[.5,1.5,-.5,.5]))
  self.assertEqual(m.tolist(),[True,False,False])

class ProbeTests(unittest.TestCase):
 def setUp(self):
  self.sha='c'*64;lo=[-.03,-.03,.1];hi=[.03,.03,.92]
  self.shape=dict(link='assembly_lower_mount',name='pole',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))),mode='measured_or_fixed')
  self.cfg=dict(enabled=True,model_sha256=self.sha)
 def test_isolated_wire_deferred_only_in_candidate_layer(self):
  p=np.array([[.12,0,.72],[1.,1.,.72]]);original=p.copy()
  with patch('cable_zones.settings',return_value=self.cfg):mask,report=deferred_mask(p,[self.shape],self.sha)
  self.assertEqual(mask.tolist(),[True,False]);np.testing.assert_array_equal(p,original);self.assertFalse(report['execution_permission'])
 def test_table_surface_is_not_deferred_even_inside_wire_band(self):
  x,y=np.meshgrid(np.arange(-.16,.321,.02),np.arange(-.24,.241,.02));p=np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)]
  with patch('cable_zones.settings',return_value=self.cfg):mask,report=deferred_mask(p,[self.shape],self.sha)
  self.assertFalse(mask.any())
 def test_probe_is_a_full_five_cm_and_has_local_braking_reserve(self):
  route=[[0,0,0],[.2,0,0]];prefix=probe_prefix(route);self.assertAlmostEqual(prefix[-1,0],.06)
  p=RouteMicroProbe(route,.06,target_m=.052);odom=np.zeros(3);p.arm(odom,0)
  for i in range(300):
   r=p.step(odom,i*.1)
   if r['state']=='MICRO_STOP':break
   p.note_command(r['twist_preview'],i*.1);odom[:2]+=.1*np.asarray(r['twist_preview'][:2])
  self.assertEqual(r['reason'],'PREFIX_REACHED');self.assertGreaterEqual(odom[0],.05);self.assertLess(odom[0],.06)
 def test_native_guard_rejects_real_obstacle_on_probe_even_if_global_candidate_ignores_it(self):
  from test_navigation import NativeTests
  world=NativeTests().packet([1,0,0],[[.25,0,.3]])['world']
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'world.json';p.write_text(json.dumps(world))
   with self.assertRaisesRegex(ValueError,'LOCAL_PROBE_COLLISION'):check_prefix(p,[[0,0,0],[.2,0,0]])
 def test_body_fixed_return_can_be_published_after_five_cm_world_fixed_cannot(self):
  seed=np.array([[.12,0,.72],[.12,.1,.72]]);o=DualFrameObserver(2,tolerance_m=.01,minimum_frames=8,minimum_translation_m=.05,minimum_second_axis_m=0.)
  for i in range(23):
   T=np.eye(4);T[0,3]=min(i*.005,.052);raw=np.vstack([seed[0],seed[1]-T[:3,3]])
   o.observe(raw,seed,T,100+i,'front')
  e=o.summary();e.update(points_body_m=seed.tolist(),reference_joints={},model_sha256=self.sha,raw_unfiltered_input=True,source_identity=['boot',0,'calibration'])
  with tempfile.TemporaryDirectory() as d,patch.object(motion_cables,'FILE',Path(d)/'confirmed.json'):
   cfg=motion_cables.publish(e,self.sha)
   self.assertEqual(cfg['confirmed_count'],1);self.assertEqual(cfg['points_body_m'],[seed[0].tolist()])
  e['tracks'][0]['xy_span_m']=[.04,0]
  with tempfile.TemporaryDirectory() as d,patch.object(motion_cables,'FILE',Path(d)/'confirmed.json'):self.assertEqual(motion_cables.publish(e,self.sha)['confirmed_count'],0)
 def test_probe_can_retreat_from_existing_near_wire_but_never_approach_it(self):
  from execution_safety import sweep_guard
  p=np.array([[.079,0,.72]]);frames={s:(dict(pose=[0,0,0],source_age_s=0.),p,100.) for s in ['front','back']};T={s:np.eye(4) for s in frames}
  with patch('cable_zones.settings',return_value=self.cfg),patch('execution_safety.envelope_settings',return_value=dict(enabled=False)),patch('execution_safety.cable_settings',return_value=dict(enabled=False)):
   self.assertFalse(sweep_guard(frames,T,[0,0,0],[self.shape],self.sha,None,[-.005,0,0],100.)[0])
   self.assertTrue(sweep_guard(frames,T,[0,0,0],[self.shape],self.sha,None,[-.005,0,0],100.,probe_nonapproach=True)[0])
   r=sweep_guard(frames,T,[0,0,0],[self.shape],self.sha,None,[.005,0,0],100.,probe_nonapproach=True)
   self.assertFalse(r[0]);self.assertEqual(r[1],'CABLE_PROBE_APPROACHES_UNCONFIRMED_RETURN')
 def test_learned_mask_retains_tables_and_invalidates_when_posture_changes(self):
  cfg=dict(enabled=True,model_sha256=self.sha,boot_id=motion_cables.boot_id(),created_at=time.time()-.1,expires_at=time.time()+60,source_identity=['boot',0,hashlib.sha256((R/'config/lidar_calibration.json').read_bytes()).hexdigest()],reference_joints={},radius_m=.015,source='RAW_DUAL_FRAME_5CM_MOTION_CONFIRMED',points_body_m=[[.12,0,.72]])
  with tempfile.TemporaryDirectory() as d,patch.object(motion_cables,'FILE',Path(d)/'confirmed.json') as file,patch.object(motion_cables,'reference_geometry',return_value=[self.shape]),patch('cable_zones.settings',return_value=self.cfg):
   file.write_text(json.dumps(cfg));self.assertTrue(motion_cables.mask(np.array([[.12,0,.72]]),[self.shape],self.sha)[0].all())
   x,y=np.meshgrid(np.arange(-.16,.321,.02),np.arange(-.24,.241,.02));plane=np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)]
   self.assertFalse(motion_cables.mask(plane,[self.shape],self.sha)[0].any())
   moved=copy.deepcopy(self.shape);moved['matrix'][0][3]=.01
   self.assertFalse(motion_cables.mask(np.array([[.12,0,.72]]),[moved],self.sha)[0].any())
 def test_delayed_rear_scan_uses_fresh_static_pose_without_relabelling_scan_time(self):
  from observe_self_motion import measured_static_posture
  q={f'Joint{i}_{s}':0. for i in range(1,8) for s in 'LR'};packet=dict(model_sha256=self.sha,joint_positions_rad=q,observation_time=99.95);header=dict(observation_time=99.5)
  with patch('onboard_state.measured_joints',return_value=packet):
   self.assertEqual(measured_static_posture(header,self.sha,100.),packet);self.assertEqual(header['observation_time'],99.5)
   self.assertIsNone(measured_static_posture(header,self.sha,101.))
  with patch('onboard_state.measured_joints',return_value=dict(packet,observation_time=99.5)):self.assertIsNone(measured_static_posture(header,self.sha,100.))

if __name__=='__main__':unittest.main()

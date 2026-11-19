import unittest,tempfile,json,sys,subprocess,copy,math,os
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'modules'))
import targets
from follower import PreviewFollower
from global_localization import correlation_candidates

class TargetTests(unittest.TestCase):
 def test_edit_is_revisioned_and_does_not_modify_other_goal_or_original_waypoint(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);(p/'config').mkdir()
   with patch.object(targets,'ROOT',p),patch.object(targets,'FILE',p/'config/targets.json'):
    before=targets.initialize();original=targets.get(1);two=targets.get(2);targets.update(1,1.2,-2.3,45)
    self.assertFalse(targets.unchanged(original));self.assertEqual(two,targets.get(2));self.assertEqual(targets.get(1)['revision'],2)
    data=(p/'config/targets.json').read_bytes()
    with self.assertRaises(ValueError):targets.update(1,float('nan'),0,0)
    self.assertEqual(data,(p/'config/targets.json').read_bytes())

class NativeTests(unittest.TestCase):
 def packet(self,goal,points=(),long_body=False):
  q={f'Joint{i}_{side}':0. for side in ['L','R'] for i in range(1,8)}
  q.update(head_pitch_joint=0.,left_robotiq_85_left_knuckle_joint=0.,right_robotiq_85_left_knuckle_joint=0.)
  world=dict(schema_version=1,snapshot_id='synthetic-test',planning_mode='immutable_snapshot_replay',source='recorded',frame_id='nav3d_map',observation_time=1.,pose_time=1.,model_sha256='0'*64,alignment_verified=False,alignment_assumption='vendor_base_to_model_identity',geometry_owner='verified_local_urdf',joint_positions_rad=q,bounded_missing_joints=[],resolution_m=.01,clearance_assumed=True,execution_permission=False,shapes=[dict(link='test_body',matrix=np.eye(4).tolist(),bounds=[[-.4 if long_body else -.2,-.2,.1],[.4 if long_body else .2,.2,.5]])],occupied_points=list(points))
  cfg=json.loads((ROOT/'config/motion_policy.json').read_text());cfg.update(search_budget_s_per_stage=3.,maximum_nodes=70000)
  return dict(world=world,goal=goal,motion_policy=cfg,grid=dict(origin=[-3.,-3.],width=200,height=160,resolution=.05,data=[0]*(200*160)))
 def run_plan(self,packet):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);(p/'in.json').write_text(json.dumps(packet));subprocess.run([os.environ.get('TIANJI_HYBRID_BINARY',str(ROOT/'native/hybrid')),str(p/'in.json'),str(p/'out.json')],check=True,stdout=subprocess.DEVNULL,timeout=12);return json.loads((p/'out.json').read_text())
 def test_long_open_route_is_differential_and_has_no_lateral(self):
  p=self.run_plan(self.packet([4.,0.,0.]));r=p['selected'];self.assertIsNotNone(r);self.assertEqual(r['profile'],'differential_only');self.assertEqual(r['lateral_distance_m'],0.);self.assertTrue(r['geometry_recheck_passed']);self.assertFalse(p['execution_permission'])
 def test_simple_short_route_can_translate(self):
  p=self.run_plan(self.packet([0.,1.,0.]));r=p['selected'];self.assertEqual(r['profile'],'short_local_translation');self.assertAlmostEqual(r['lateral_distance_m'],1.);self.assertAlmostEqual(r['path'][-1][1],1.)
 def test_short_diagonal_route_is_straight_fixed_heading(self):
  r=self.run_plan(self.packet([.9,.9,0.]))['selected'];self.assertEqual(r['profile'],'short_diagonal_translation');self.assertAlmostEqual(r['length_m'],math.hypot(.9,.9));self.assertTrue(all(abs(p[0]-p[1])<1e-8 for p in r['path']));self.assertTrue(r['geometry_recheck_passed']);self.assertTrue(all(abs(p[2])<1e-8 for p in r['path']));self.assertAlmostEqual(r['lateral_distance_m'],.9)
 def test_short_right_angle_checks_both_legs_for_collision(self):
  packet=self.packet([.9,.9,0.],[[0.,.6,.3],[.6,0.,.3],[.45,.45,.3]])
  result=self.run_plan(packet);self.assertNotEqual((result.get('selected') or {}).get('profile'),'short_rectilinear_translation')
 def test_diagonal_blocked_falls_back_to_checked_rectilinear(self):
  r=self.run_plan(self.packet([.9,.9,0.],[[.45,.45,.3]]))['selected'];self.assertIsNotNone(r);self.assertEqual(r['profile'],'short_rectilinear_translation');self.assertTrue(r['geometry_recheck_passed'])
 def test_diagonal_does_not_cross_unknown_space(self):
  packet=self.packet([.9,.9,0.]);grid=np.array(packet['grid']['data']).reshape(160,200);grid[67:71,67:71]=-1;packet['grid']['data']=grid.ravel().tolist()
  r=self.run_plan(packet)['selected'];self.assertNotEqual((r or {}).get('profile'),'short_diagonal_translation')
 def test_long_route_curves_around_obstacle_without_unnecessary_sideways(self):
  wall=[[2.,float(y),.3] for y in np.arange(-.55,.56,.04)]
  p=self.run_plan(self.packet([4.,0.,0.],wall));r=p['selected'];self.assertIsNotNone(r);self.assertEqual(r['profile'],'differential_only');self.assertEqual(r['lateral_distance_m'],0.);self.assertGreater(r['length_m'],4.)
 def test_tight_local_scene_uses_bounded_side_step_then_differential_travel(self):
  wall=[[x,float(y),.3] for x in [-.48,.48] for y in np.arange(-.10,.101,.02)]
  p=self.run_plan(self.packet([3.,0.,0.],wall,True));r=p['selected'];self.assertIsNotNone(r);self.assertEqual(r['profile'],'differential_with_bounded_local_lateral');self.assertGreater(r['lateral_distance_m'],0);self.assertLessEqual(r['lateral_distance_m'],.75+1e-6)
  self.assertTrue(any(m['mode']=='differential_curve' for m in r['primitives']))
 def test_real_occupied_start_is_never_cleared_by_planner(self):
  p=self.run_plan(self.packet([2.,0.,0.],[[0.,0.,.3]]));self.assertIsNone(p['selected']);self.assertEqual(p['attempts'][0]['reason'],'START_COLLISION')
 def test_unknown_strip_is_not_treated_as_free(self):
  packet=self.packet([2.,0.,0.]);grid=np.array(packet['grid']['data']).reshape(160,200);grid[:,78:84]=-1;packet['grid']['data']=grid.ravel().tolist();p=self.run_plan(packet);self.assertIsNone(p['selected'])
 def test_fine_start_escapes_short_observed_corner_without_freeing_unknown(self):
  packet=self.packet([2.,.2,0.]);packet['motion_policy'].update(short_rectilinear_enabled=False,long_route_lateral_total_max_m=0.)
  grid=np.full((160,200),-1);packet['grid']['origin']=[-3.025,-3.025]
  path=[(x,0.) for x in np.arange(0,.101,.05)]+[(.1,y) for y in np.arange(.05,.201,.05)]+[(x,.2) for x in np.arange(.15,2.001,.05)]
  for x,y in path:
   ix,iy=np.floor((np.array([x,y])+3.025)/.05).astype(int);grid[iy,ix]=0
  packet['grid']['data']=grid.ravel().tolist();coarse=self.run_plan(packet);self.assertIsNone(coarse['selected'])
  packet['motion_policy']['near_start_fine_escape_enabled']=True;fine=self.run_plan(packet);self.assertIsNotNone(fine['selected']);self.assertTrue(fine['selected']['geometry_recheck_passed']);self.assertTrue(any(0<abs(m['axial_m'])<.20 for m in fine['selected']['primitives']))

class LocalizationTests(unittest.TestCase):
 def test_global_search_recovers_translation_and_yaw_without_previous_pose(self):
  rng=np.random.default_rng(3);a=np.c_[rng.uniform(-3,3,900),np.full(900,2.5),rng.uniform(.2,2.,900)];b=np.c_[np.full(800,-2.),rng.uniform(-2,2.5,800),rng.uniform(.2,1.5,800)];table=np.c_[rng.uniform(.3,1.8,600),rng.uniform(-1.3,.2,600),np.full(600,.75)];target=np.vstack([a,b,table]);yaw=math.radians(65);R=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]]);translation=np.array([1.4,-.8,0]);source=(target-translation)@R
  seeds,report=correlation_candidates(source,target,max_candidates=8)
  self.assertTrue(any(np.linalg.norm(np.array(s['pose'][:2])-translation[:2])<.3 and abs(math.atan2(math.sin(s['pose'][2]-yaw),math.cos(s['pose'][2]-yaw)))<math.radians(7) for s in seeds))

class FollowerTests(unittest.TestCase):
 def plan(self,lateral=False):
  path=[[0,float(v),0] if lateral else [float(v),0,0] for v in np.linspace(0,1,51)];cfg=json.loads((ROOT/'config/motion_policy.json').read_text());plan=dict(selected=dict(path=path,geometry_recheck_passed=True),target=dict(id=1,revision=1,pose=path[-1]),start_map_pose=[0,0,0]);return plan,cfg
 def test_feedback_follower_arrives_in_short_translation_simulation(self):
  plan,cfg=self.plan(True);f=PreviewFollower(plan,cfg);p=np.zeros(3);record=[]
  for i in range(1000):
   t=i*.05;r=f.step(p,t,t,1,True,True);v=np.array(r['twist_preview']);record.append(r)
   if r['state']=='ARRIVED':break
   f.admitted(v,p,t);p[:2]+=.05*np.array([math.cos(p[2])*v[0]-math.sin(p[2])*v[1],math.sin(p[2])*v[0]+math.cos(p[2])*v[1]]);p[2]+=.05*v[2]
  self.assertEqual(r['state'],'ARRIVED');self.assertLess(np.linalg.norm(p[:2]-[0,1]),.0751);self.assertLessEqual(max(abs(s['twist_preview'][1]) for s in record),cfg['max_vy_mps'])
 def test_diagonal_follower_uses_simultaneous_vx_vy_in_rotated_frame(self):
  for dx,dy in [(1.,1.),(-1.,1.),(1.,-.3),(-.3,-1.)]:
   plan,cfg=self.plan();start=np.array([2.,-1.,.7]);c,s=math.cos(start[2]),math.sin(start[2]);R=np.array([[c,-s],[s,c]])
   path=[[dx*t,dy*t,0.] for t in np.linspace(0,1,101)];plan.update(start_map_pose=start.tolist(),target=dict(id=1,revision=1,pose=[*(start[:2]+R@np.array([dx,dy])),start[2]]));plan['selected'].update(path=path,profile='short_diagonal_translation')
   f=PreviewFollower(plan,cfg);p=start.copy();diagonal=0
   for i in range(2400):
    t=i*.05;r=f.step(p,t,t,1,True,True);v=np.array(r['twist_preview'])
    if r['state']=='ARRIVED':break
    if not f.fine_active:
     self.assertEqual(r['mode'],'bounded_local_translation');self.assertAlmostEqual(v[2],0.);self.assertAlmostEqual(v[0]*dy,v[1]*dx,places=7);diagonal+=1
    f.admitted(v,p,t);p[:2]+=.05*(R@v[:2]);p[2]+=.05*v[2]
   self.assertGreater(diagonal,20);self.assertEqual(r['state'],'ARRIVED');self.assertLess(np.linalg.norm(p[:2]-plan['target']['pose'][:2]),.0151)
 def test_long_forward_follower_has_zero_vy_and_stops_on_stale_or_target_change(self):
  plan,cfg=self.plan(False);f=PreviewFollower(plan,cfg);r=f.step([0,0,0],1,1,1,True,True);self.assertEqual(r['twist_preview'][1],0.)
  for args in [([0,0,0],1,2,1,True,True),([0,0,0],1,1,2,True,True),([0,0,0],1,1,1,True,False)]:self.assertEqual(f.step(*args)['twist_preview'],[0.,0.,0.])

if __name__=='__main__':unittest.main()

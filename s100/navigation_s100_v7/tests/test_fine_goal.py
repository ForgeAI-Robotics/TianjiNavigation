import sys,unittest,json,math,copy
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'modules'))
from follower import PreviewFollower
from navigation_control import limited_twist
from execution_safety import sweep_guard

class LegacyFineGoalTests(unittest.TestCase):
 def follower(self):
  cfg=json.loads((ROOT/'config/motion_policy.json').read_text());cfg['fine_goal']['enabled']=True;cfg['fine_goal'].pop('strategy',None);cfg['fine_goal']['translation_speed_mps']=.01
  return PreviewFollower(dict(selected=dict(path=[[0,0,0],[1,0,0]],geometry_recheck_passed=True),target=dict(id=1,revision=1,pose=[1,0,0]),start_map_pose=[0,0,0]),cfg)
 def test_seven_cm_is_no_longer_declared_arrived(self):
  f=self.follower();r=f.step([.93,0,0],10,10,1,True,True)
  self.assertEqual(r['mode'],'fine_translation');self.assertEqual(r['twist_preview'],[.01,0.,0.]);self.assertAlmostEqual(r['goal_error']['position_tolerance_m'],.015)
 def test_heading_then_translation_avoids_simultaneous_wheel_modes(self):
  f=self.follower();r=f.step([.94,.025,.06],10,10,1,True,True)
  self.assertEqual(r['mode'],'fine_rotation');self.assertEqual(r['twist_preview'][:2],[0.,0.]);self.assertLessEqual(abs(r['twist_preview'][2]),.025)
  r=f.step([.94,.025,.005],10.1,10.1,1,True,True)
  self.assertEqual(r['mode'],'fine_translation');self.assertEqual(r['twist_preview'][2],0.);self.assertLess(r['twist_preview'][1],0)
  self.assertLessEqual(np.linalg.norm(r['twist_preview'][:2]),.01000001)
 def test_overshoot_is_corrected_with_short_reverse_translation(self):
  r=self.follower().step([1.05,0,0],10,10,1,True,True);self.assertLess(r['twist_preview'][0],0);self.assertEqual(r['twist_preview'][2],0)
 def test_arrival_requires_stable_interval_and_new_pose_samples(self):
  f=self.follower()
  for i in range(10):
   t=10+i*.1;r=f.step([.99,0,.005],t,t,1,True,True);self.assertNotEqual(r['state'],'ARRIVED');self.assertEqual(r['twist_preview'],[0.,0.,0.])
  r=f.step([.99,0,.005],11.1,11.1,1,True,True);self.assertEqual(r['state'],'ARRIVED');self.assertGreaterEqual(r['settled_s'],1.)
 def test_stale_pose_and_guard_loss_reset_settling(self):
  f=self.follower()
  for t in [10,10.3,10.6]:f.step([1,0,0],t,t,1,True,True)
  self.assertEqual(f.step([1,0,0],10.6,10.7,1,True,False)['state'],'OBSTACLE_OR_UNKNOWN_STOP')
  self.assertNotEqual(f.step([1,0,0],10.8,10.8,1,True,True)['state'],'ARRIVED')
  self.assertEqual(f.step([1,0,0],10.8,11.4,1,True,True)['state'],'LOCALIZATION_STALE')
  self.assertIsNone(f.settle_since)
 def test_leaving_tolerance_restarts_stable_confirmation(self):
  f=self.follower()
  for t in [10,10.3,10.6]:f.step([1,0,0],t,t,1,True,True)
  f.step([.98,0,0],10.7,10.7,1,True,True);self.assertIsNone(f.settle_since)
  self.assertNotEqual(f.step([1,0,0],11,11,1,True,True)['state'],'ARRIVED')
 def test_software_closed_loop_corrects_xy_and_heading_without_moving_hardware(self):
  f=self.follower();pose=np.array([.93,-.025,.05]);modes=set()
  for k in range(1000):
   t=k*.05;r=f.step(pose,t,t,1,True,True);modes.add(r.get('mode'))
   if r['state']=='ARRIVED':break
   x,y,w=limited_twist(r['twist_preview'],.8,1.4);a=pose[2];pose+=np.array([math.cos(a)*x-math.sin(a)*y,math.sin(a)*x+math.cos(a)*y,w])*.05
  self.assertEqual(r['state'],'ARRIVED');self.assertLessEqual(np.linalg.norm(pose[:2]-[1,0]),.015);self.assertLessEqual(abs(pose[2]),math.radians(1));self.assertTrue({'fine_rotation','fine_translation','goal_settling'}.issubset(modes))
 def test_fine_translation_does_not_waive_obstacle_guard(self):
  r=self.follower().step([.93,0,0],100,100,1,True,True);shape=dict(link='body',matrix=np.eye(4).tolist(),bounds=[[-.2,-.2,.1],[.2,.2,.5]]);p=np.array([[.22,0,.3]]);frames={};prepared={}
  for sensor in ['front','back']:
   h=dict(sensor=sensor,sequence=1,source_age_s=.1,pose=[0,0,0]);frames[sensor]=(h,p,100.);prepared[sensor]=dict(frame=frames[sensor],points_body=p,model_sha256='model',report={})
  self.assertFalse(sweep_guard(frames,{},[0,0,0],[shape],'model',None,r['twist_preview'],100.,prepared=prepared,revalidation_reserve_m=.005)[0])
 def test_same_observation_cannot_fake_one_second_stability(self):
  f=self.follower()
  for i in range(6):self.assertNotEqual(f.step([1,0,0],10,10+i*.09,1,True,True)['state'],'ARRIVED')
  self.assertEqual(f.settle_samples,1)
 def test_small_position_noise_does_not_restart_wheels_near_acceptance_edge(self):
  f=self.follower();f.step([.99,0,0],10,10,1,True,True)
  r=f.step([.986,0,0],10.2,10.2,1,True,True);self.assertEqual(r['mode'],'goal_settling');self.assertEqual(r['twist_preview'],[0.,0.,0.])
  r=f.step([.984,0,0],10.3,10.3,1,True,True);self.assertEqual(r['mode'],'fine_translation');self.assertGreater(r['twist_preview'][0],0)
 def test_close_goal_does_not_shortcut_a_long_planned_detour(self):
  f=self.follower();f.path=np.array([[0,0,0],[0,1,0],[1,1,0],[1,0,0]])
  f.step([.90,0,0],10,10,1,True,True);self.assertFalse(f.fine_active)
if __name__=='__main__':unittest.main()

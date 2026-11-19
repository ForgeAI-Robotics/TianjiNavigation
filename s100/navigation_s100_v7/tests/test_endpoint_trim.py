import sys,json,unittest,math,copy
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'modules'))
from follower import PreviewFollower
from endpoint_trim import EndpointTrim
from navigation_control import limited_twist

class PulseTrimTests(unittest.TestCase):
 def cfg(self):return json.loads((ROOT/'config/motion_policy.json').read_text())
 def follower(self,goal=(1,0,0),path=None):
  return PreviewFollower(dict(selected=dict(path=path or [[0,0,0],[1,0,0]],geometry_recheck_passed=True),target=dict(id=1,revision=1,pose=list(goal)),start_map_pose=[0,0,0]),self.cfg())
 def test_position_is_corrected_first_and_pulse_direction_does_not_follow_noise(self):
  f=self.follower();odom=np.array([0.,0.,0.]);commands=[]
  for i in range(6):
   t=i*.2;r=f.step([.94,.005*(-1)**i,.05],t,t,1,True,True,odometry_pose=odom,control_time=t)
   if any(r['twist_preview']):commands.append(r['twist_preview']);f.admitted(r['twist_preview'],odom,t)
  self.assertTrue(commands);self.assertTrue(all(v==[self.cfg()['fine_goal']['translation_speed_mps'],0.,0.]for v in commands))
 def test_map_pose_jump_cannot_fabricate_wheel_odometry_progress(self):
  c=self.cfg()['fine_goal'];trim=EndpointTrim([1,0,0],c)
  for i in range(3):r=trim.step([.94,0,0],[0,0,0],i*.2,i*.2)
  trim.admitted([0,0,0],.4)
  r=trim.step([1,0,0],[0,0,0],.6,.6);self.assertEqual(r['mode'],'fine_translation');self.assertEqual(trim.pulses,0)
  r=trim.step([1,0,0],[.0081,0,0],.8,.8);self.assertEqual(r['twist_preview'],[0.,0.,0.]);self.assertEqual(trim.pulses,1)
 def test_unadmitted_proposal_does_not_start_timeout_or_count_as_stall(self):
  trim=EndpointTrim([1,0,0],self.cfg()['fine_goal'])
  for t in [0,.2,.4,10,20]:trim.step([.94,0,0],[0,0,0],t,t)
  self.assertEqual(trim.pulses,0);self.assertIsNone(trim.action['start']);trim.pause();self.assertEqual(trim.no_progress,0)
 def test_no_measured_progress_pauses_instead_of_adjusting_forever(self):
  f=self.follower();mode=None
  for i in range(100):
   t=i*.2;r=f.step([.94,0,0],t,t,1,True,True,odometry_pose=[0,0,0],control_time=t);f.admitted(r['twist_preview'],[0,0,0],t)
   mode=r['mode']
   if mode=='fine_paused':break
  self.assertEqual(mode,'fine_paused');self.assertEqual(r['twist_preview'],[0.,0.,0.]);self.assertEqual(r['fine_no_progress_pulses'],3)
  for i in range(5):self.assertEqual(f.step([.94,0,0],t+i+1,t+i+1,1,True,True,odometry_pose=[0,0,0])['twist_preview'],[0.,0.,0.])
 def test_single_pose_outlier_does_not_trigger_reverse_corrections(self):
  trim=EndpointTrim([1,0,0],self.cfg()['fine_goal']);out=[]
  for i in range(14):
   t=i*.2;p=[1.06,0,0] if i==4 else [1.,0,0];r=trim.step(p,[0,0,0],t,t);out.append(r)
  self.assertTrue(all(r['twist_preview']==[0.,0.,0.]for r in out));self.assertEqual(out[-1]['state'],'ARRIVED')
  self.assertTrue(all(r['state']!='ARRIVED'for r in out[4:9]))
 def test_software_closed_loop_with_admission_and_realistic_five_hz_observation(self):
  f=self.follower();pose=np.array([.93,.02,.05]);odom=pose.copy();modes=[];max_xy=max_w=0.
  for i in range(700):
   t=i*.2;r=f.step(pose,t,t,1,True,True,odometry_pose=odom,control_time=t)
   if not modes or modes[-1]!=r['mode']:modes.append(r['mode'])
   if r['state']=='ARRIVED':break
   v=limited_twist(r['twist_preview'],.8,1.4);f.admitted(v,odom,t);x,y,w=v;a=pose[2];d=np.array([math.cos(a)*x-math.sin(a)*y,math.sin(a)*x+math.cos(a)*y,w])*.2;pose+=d;odom+=d;max_xy=max(max_xy,math.hypot(x,y));max_w=max(max_w,abs(w))
  self.assertEqual(r['state'],'ARRIVED');self.assertLessEqual(np.linalg.norm(pose[:2]-[1,0]),self.cfg()['fine_goal']['position_tolerance_m']);self.assertLessEqual(abs(pose[2]),math.radians(1));self.assertLessEqual(max_xy,self.cfg()['fine_goal']['translation_speed_mps']+1e-8);self.assertLessEqual(max_w,.02500001)
  first_translation=modes.index('fine_translation');first_rotation=modes.index('fine_rotation');self.assertLess(first_translation,first_rotation);self.assertIn('fine_observe',modes)
 def test_five_cm_one_degree_gate_without_inner_margin(self):
  cfg=self.cfg()['fine_goal'];self.assertEqual(cfg['position_settle_margin_m'],0.)
  trim=EndpointTrim([1,0,0],cfg)
  for i in range(15):r=trim.step([.956,0,math.radians(.3)],[0,0,0],i*.2,i*.2)
  self.assertEqual(r['state'],'ARRIVED');self.assertEqual(r['twist_preview'],[0.,0.,0.])
 def test_guard_pause_does_not_discard_admitted_curve_reference(self):
  theta=np.linspace(0,1,150);path=np.c_[.6*np.sin(theta),.6*(1-np.cos(theta)),theta].tolist();f=self.follower(goal=path[-1],path=path);r=f.step([0,0,0],1,1,1,True,True);f.admitted(r['twist_preview'],[0,0,0],1);curve=f.curvature
  f.step([.001,.01,0],1.2,1.2,1,True,True);f.pause('GUARD_LOST');self.assertEqual(f.curvature,curve);self.assertEqual(f.previous.tolist(),[0.,0.,0.])
 def test_straight_and_lateral_noise_do_not_create_small_yaw_corrections(self):
  for path,poses in [([[0,0,0],[1,0,0]],[[.3,.008*(-1)**i,.01*(-1)**i]for i in range(10)]),([[0,0,0],[0,1,0]],[[.008*(-1)**i,.3,.01*(-1)**i]for i in range(10)])]:
   f=self.follower(goal=path[-1],path=path)
   for i,p in enumerate(poses):
    r=f.step(p,i*.2,i*.2,1,True,True);self.assertEqual(r['twist_preview'][2],0.);f.admitted(r['twist_preview'],p,i*.2)
 def test_pulse_budget_does_not_prevent_final_stationary_arrival_confirmation(self):
  trim=EndpointTrim([1,0,0],self.cfg()['fine_goal']);trim.pulses=trim.cfg['maximum_pulses']
  for i in range(12):r=trim.step([1,0,0],[0,0,0],i*.2,i*.2)
  self.assertEqual(r['state'],'ARRIVED');self.assertFalse(trim.paused)
 def test_budget_hold_is_reported_separately_from_no_motion(self):
  trim=EndpointTrim([1,0,0],self.cfg()['fine_goal']);trim.pulses=trim.cfg['maximum_pulses']
  for i in range(3):r=trim.step([.94,0,0],[0,0,0],i*.2,i*.2)
  self.assertEqual(r['mode'],'fine_paused');self.assertEqual(r['control_note'],'FINE_PULSE_BUDGET_HOLD');self.assertEqual(r['fine_no_progress_pulses'],0)
if __name__=='__main__':unittest.main()

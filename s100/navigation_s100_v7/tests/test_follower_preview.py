import sys,unittest,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from follower import PreviewFollower
from navigation_control import limited_twist

class TrackingTests(unittest.TestCase):
 def follower(self,path,lookahead=.15,goal=None):
  plan=dict(selected=dict(path=np.asarray(path).tolist(),geometry_recheck_passed=True),target=dict(id=1,revision=1,pose=list(path[-1] if goal is None else goal)),start_map_pose=[0,0,0])
  cfg=dict(max_vx_mps=.2,max_reverse_mps=.1,max_vy_mps=.04,max_wz_radps=.2,tracking_lookahead_m=lookahead)
  return PreviewFollower(plan,cfg)
 def arc(self):
  theta=np.linspace(0,1.2,150);return np.c_[.6*np.sin(theta),.6*(1-np.cos(theta)),theta]
 def test_perfect_arc_keeps_planned_curvature_after_all_limits(self):
  f=self.follower(self.arc());r=f.step([0,0,0],1,1,1,True,True);v=r['twist_preview'];self.assertAlmostEqual(v[2]/v[0],1/.6,places=8)
  v=limited_twist(v,.6,1.4);self.assertAlmostEqual(v[2]/v[0],1/.6,places=8)
 def test_longer_preview_reduces_lateral_localization_noise_amplification(self):
  path=np.c_[np.linspace(0,1,201),np.zeros(201),np.zeros(201)];noise=np.linspace(-.004,.004,21);values=[]
  for lookahead in [.055,.15]:
   omega=[]
   for dy in noise:
    f=self.follower(path,lookahead);r=f.step([0,dy,0],1,1,1,True,True);omega.append(limited_twist(r['twist_preview'],.6,1.4)[2])
   values.append(float(np.std(omega)))
  self.assertLess(values[1],.6*values[0])
 def test_curvature_changes_are_bounded_before_safety_limiting(self):
  path=np.c_[np.linspace(0,1,201),np.zeros(201),np.zeros(201)];f=self.follower(path);previous=None
  for i,dy in enumerate([0,.004,-.004,.004,-.004]):
   now=1.+i*.05;r=f.step([0,dy,0],now,now,1,True,True);v=limited_twist(r['twist_preview'],.9,1.4);curvature=v[2]/v[0]
   if previous is not None:self.assertLessEqual(abs(curvature-previous),.8*.05+1e-10)
   previous=curvature
 def test_explicit_stop_resets_curvature_history_and_outputs_zero(self):
  f=self.follower(self.arc());f.step([0,0,0],1,1,1,True,True);self.assertIsNotNone(f.curvature)
  result=f.step([0,0,0],1,1,1,True,False);self.assertEqual(result['twist_preview'],[0.,0.,0.]);self.assertIsNone(f.curvature)
 def test_preview_does_not_cross_into_lateral_or_reverse_segment(self):
  path=[[float(x),0,0] for x in np.linspace(0,.1,21)]+[[.1,float(y),0] for y in np.linspace(.01,.4,40)]
  f=self.follower(path);r=f.step([.06,0,0],1,1,1,True,True);self.assertEqual(r['mode'],'differential');self.assertEqual(r['twist_preview'][1],0);self.assertAlmostEqual(r['twist_preview'][2],0.)
  reverse=[[float(x),0,0] for x in np.linspace(0,.1,21)]+[[float(x),0,0] for x in np.linspace(.095,-.5,120)]
  f=self.follower(reverse);r=f.step([.06,0,0],1,1,1,True,True);self.assertGreater(r['twist_preview'][0],0)
 def test_position_reached_only_corrects_heading_without_forward_hunting(self):
  f=self.follower([[0,0,0],[1,0,0]],goal=[1,0,.5]);r=f.step([.98,0,0],1,1,1,True,True)
  self.assertEqual(r['mode'],'final_rotation');self.assertEqual(r['twist_preview'][:2],[0.,0.]);self.assertGreater(r['twist_preview'][2],0)
 def test_curve_feedback_converges_in_motionless_software_simulation(self):
  path=self.arc();f=self.follower(path);pose=np.zeros(3)
  for k in range(2400):
   now=k*.05;r=f.step(pose,now,now,1,True,True)
   if r['state']=='ARRIVED':break
   x,y,w=limited_twist(r['twist_preview'],.6,1.4);a=pose[2];pose+=np.array([math.cos(a)*x-math.sin(a)*y,math.sin(a)*x+math.cos(a)*y,w])*.05
  self.assertEqual(r['state'],'ARRIVED');self.assertLess(np.linalg.norm(pose[:2]-path[-1,:2]),.0751)

if __name__=='__main__':unittest.main()

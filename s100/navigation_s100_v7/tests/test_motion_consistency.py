import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from motion_consistency import DualFrameObserver

class MotionConsistencyTests(unittest.TestCase):
 def sequence(self,kind,small=False):
  observer=DualFrameObserver(1,tolerance_m=.01,minimum_frames=5);seed=np.array([[.2,.2,1.]])
  poses=[(i*.04,0) for i in range(10)]+[(.36,i*.04) for i in range(1,11)]
  for i,(x,y) in enumerate(poses):
   if small:x*=.025;y*=.025
   T=np.eye(4);T[:2,3]=[x,y]
   if kind=='body':p=seed.copy()
   elif kind=='world':p=seed-T[:3,3]
   else:p=np.vstack([seed,seed-T[:3,3]])
   observer.observe(p,seed,T,100.+i,'front')
  return observer.summary()
 def test_robot_fixed_return_matches_body_frame(self):self.assertEqual(self.sequence('body')['features'][0]['status'],'SELF_CORRELATED')
 def test_static_environment_matches_world_frame(self):self.assertEqual(self.sequence('world')['features'][0]['status'],'ENVIRONMENT_FIXED')
 def test_both_hypotheses_supported_remains_unresolved(self):self.assertEqual(self.sequence('both')['features'][0]['status'],'UNRESOLVED')
 def test_tiny_motion_is_not_accepted_as_self_proof(self):
  r=self.sequence('body',True);self.assertEqual(r['features'][0]['status'],'INSUFFICIENT_MOTION');self.assertFalse(r['automatic_point_deletion']);self.assertEqual(r['motion_commands_sent'],0)
 def test_micro_translation_then_stop_uses_initial_anchor_for_baseline(self):
  observer=DualFrameObserver(1,tolerance_m=.01,minimum_frames=5,minimum_translation_m=.025,minimum_second_axis_m=0.)
  seed=np.array([[.2,.2,1.]])
  for i in range(12):
   T=np.eye(4);T[0,3]=0. if i==0 else .035
   observer.observe(seed,seed,T,100.+i,'front')
  self.assertEqual(observer.summary()['features'][0]['status'],'SELF_CORRELATED')

if __name__=='__main__':unittest.main()

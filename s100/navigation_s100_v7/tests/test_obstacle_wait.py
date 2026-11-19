import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from obstacle_wait import ObstacleWait
from guard_recovery import GuardRecovery
from execution_safety import sweep_guard
from navigation_admission import admission

COLLISION='LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY'
def frames(sequence,points=None,back=None):
 result={}
 for sensor,seq in [('front',sequence),('back',sequence if back is None else back)]:
  result[sensor]=(dict(sensor=sensor,sequence=seq,source_age_s=.1,pose=[0,0,0]),np.empty((0,3)) if points is None else points,100.)
 return result

class ObstacleWaitTests(unittest.TestCase):
 def test_long_obstacle_wait_keeps_task_then_requires_fresh_stable_clearance(self):
  w=ObstacleWait();w.blocked(COLLISION,0.,frames(1))
  w.blocked(COLLISION,120.,frames(2));self.assertTrue(w.active);self.assertEqual(w.episodes,1)
  self.assertFalse(w.checked_clear(frames(2),121.))
  self.assertFalse(w.checked_clear(frames(3),121.1))
  self.assertFalse(w.checked_clear(frames(4),121.5))
  self.assertTrue(w.checked_clear(frames(5),122.));self.assertFalse(w.active)
 def test_reusing_one_pair_or_advancing_one_sensor_cannot_resume(self):
  w=ObstacleWait();w.blocked(COLLISION,0,frames(1))
  self.assertFalse(w.checked_clear(frames(2),1))
  for seq in range(3,10):self.assertFalse(w.checked_clear(frames(seq,back=2),seq))
  self.assertEqual(w.clear_pairs,1)
 def test_clear_confirmation_finishes_only_on_a_new_pair(self):
  w=ObstacleWait();w.blocked(COLLISION,0)
  for seq,now in [(1,.1),(2,.2),(3,.3)]:self.assertFalse(w.checked_clear(frames(seq),now))
  self.assertFalse(w.checked_clear(frames(3),2.))
  self.assertTrue(w.checked_clear(frames(4),2.1))
 def test_new_obstacle_or_missing_feedback_restarts_clear_confirmation(self):
  for interrupt in ['obstacle','data']:
   w=ObstacleWait();w.blocked(COLLISION,0)
   self.assertFalse(w.checked_clear(frames(1),1))
   self.assertFalse(w.checked_clear(frames(2),1.5))
   if interrupt=='obstacle':w.blocked(COLLISION,1.6,frames(2))
   else:w.interrupt_clear()
   self.assertFalse(w.checked_clear(frames(3),2))
   self.assertFalse(w.checked_clear(frames(4),2.5))
   self.assertTrue(w.checked_clear(frames(5),2.9))
 def test_non_obstacle_failures_keep_original_stop_rules(self):
  for reason in ['LOCALIZATION_FRAME_OR_EPOCH_CHANGED','URDF_VERSION_MISMATCH','POSTURE_CHANGED_REPLAN_REQUIRED','OPERATOR_STOP','MAP_GUARD_TIMEOUT']:
   with self.assertRaises(ValueError):ObstacleWait().blocked(reason,0)
  recovery=GuardRecovery();self.assertTrue(recovery.waiting('LIDAR_STALE',None,0));self.assertFalse(recovery.within_budget(3.1))
 def test_collision_then_clear_replay_never_admits_blocking_scene(self):
  w=ObstacleWait();shape=dict(link='body',matrix=np.eye(4).tolist(),bounds=[[-.2,-.2,.1],[.2,.2,.5]])
  def check(seq,p,now):
   f=frames(seq,np.asarray(p).reshape(-1,3));prepared={s:dict(frame=v,points_body=v[1],model_sha256='model',report={})for s,v in f.items()}
   ok,reason,_=sweep_guard(f,{},[0,0,0],[shape],'model',None,[.01,0,0],now,prepared=prepared,revalidation_reserve_m=.005)
   return f,ok,reason
  f,ok,reason=check(1,[[.22,0,.3]],100.);self.assertFalse(ok);self.assertEqual(reason,COLLISION);w.blocked(reason,100.,f)
  for seq,now in [(2,100.1),(3,100.5),(4,100.95)]:
   f,ok,reason=check(seq,[],now);self.assertTrue(ok);released=w.checked_clear(f,now)
   self.assertEqual(released,seq==4)
  self.assertFalse(admission(dict(state='BLOCKED'),0))
if __name__=='__main__':unittest.main()

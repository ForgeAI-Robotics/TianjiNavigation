import unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from route_micro_probe import RouteMicroProbe

class RouteMicroTests(unittest.TestCase):
 def probe(self):return RouteMicroProbe([[0,0,0],[.01,0,.1],[.02,.002,.2],[.06,.01,.3]],.05)
 def test_checked_route_prefix_has_braking_reserve_and_no_turn_command(self):
  p=self.probe();self.assertLessEqual(np.linalg.norm(p.path[-1]),.035+1e-9);r=p.step([0,0,0],0);self.assertLessEqual(np.linalg.norm(r['twist_preview'][:2]),.005+1e-9);self.assertEqual(r['twist_preview'][2],0)
 def test_odometry_or_command_budget_latches_stop(self):
  p=self.probe();p.arm([0,0,0],0)
  for i in range(1,7):
   r=p.step([i*.006,0,0],i*.2)
   if r['state']=='MICRO_STOP':break
   p.note_command(r['twist_preview'],i*.2)
  self.assertEqual(r['state'],'MICRO_STOP');self.assertEqual(r['reason'],'DISTANCE_BUDGET_REACHED');self.assertEqual(p.step([0,0,0],10)['state'],'MICRO_STOP')
  q=self.probe();q.arm([0,0,0],0);q.note_command([.005,0,0],0);self.assertEqual(q.step([0,0,0],8)['reason'],'DISTANCE_BUDGET_REACHED')
 def test_unexpected_jump_heading_and_large_budget_are_rejected(self):
  p=self.probe();p.arm([0,0,0],0);self.assertEqual(p.step([.03,0,0],.1)['reason'],'ODOMETRY_JUMP')
  p=self.probe();p.arm([0,0,0],0);self.assertEqual(p.step([0,0,.03],.1)['reason'],'HEADING_CHANGED')
  with self.assertRaises(ValueError):RouteMicroProbe([[0,0,0],[1,0,0]],.06)
 def test_following_curved_prefix_stops_before_five_cm(self):
  p=self.probe();odom=np.zeros(3);p.arm(odom,0);t=0.
  for _ in range(300):
   r=p.step(odom,t)
   if r['state']=='MICRO_STOP':break
   command=np.array(r['twist_preview']);p.note_command(command,t);odom[:2]+=command[:2]*.1;t+=.1
  self.assertEqual(r['state'],'MICRO_STOP');self.assertLess(np.linalg.norm(odom[:2]),.05);self.assertAlmostEqual(odom[2],0)

if __name__=='__main__':unittest.main()

import unittest,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'modules'))
from follower import PreviewFollower

class GoalAcceptanceToleranceTests(unittest.TestCase):
 def follower(self):
  cfg=json.loads((ROOT/'config/motion_policy.json').read_text())
  return PreviewFollower(dict(selected=dict(path=[[0,0,0],[1,0,0]],geometry_recheck_passed=True),target=dict(id=1,revision=1,pose=[1,0,0]),start_map_pose=[0,0,0]),cfg)
 def run_pose(self,pose):
  f=self.follower();rows=[]
  for i in range(15):
   t=i*.2;rows.append(f.step(pose,t,t,1,True,True,odometry_pose=[0,0,0],control_time=t))
  return rows
 def test_inside_five_cm_one_degree_settles_without_extra_motion(self):
  rows=self.run_pose([.951,0,math.radians(.9)])
  self.assertEqual(rows[-1]['state'],'ARRIVED')
  self.assertTrue(all(r['twist_preview']==[0.,0.,0.]for r in rows))
 def test_exceeding_either_tolerance_cannot_arrive(self):
  for pose in [[.949,0,0],[1,0,math.radians(1.1)]]:
   self.assertTrue(all(r['state']!='ARRIVED'for r in self.run_pose(pose)))
 def test_fresh_outlier_restarts_arrival_confirmation(self):
  f=self.follower()
  for i in range(7):
   t=i*.2;pose=[.949,0,0] if i==5 else [.951,0,0]
   r=f.step(pose,t,t,1,True,True,odometry_pose=[0,0,0],control_time=t)
   self.assertNotEqual(r['state'],'ARRIVED')

 def test_turn_first_profile_aligns_before_translation(self):
  cfg=json.loads((ROOT/'config/motion_policy.json').read_text());a=math.radians(2.24)
  path=[[0,0,a*i/20]for i in range(21)]+[[i/20,.2*i/20,a]for i in range(1,21)]
  plan=dict(selected=dict(profile='short_turn_then_translation',path=path,geometry_recheck_passed=True),target=dict(id=1,revision=1,pose=[1,.2,a]),start_map_pose=[0,0,0])
  f=PreviewFollower(plan,cfg);r=f.step([0,0,0],0,0,1,True,True)
  self.assertEqual(r['mode'],'rotation');self.assertEqual(r['twist_preview'][:2],[0.,0.]);self.assertGreater(r['twist_preview'][2],0)
  r=f.step([0,0,math.radians(.9)],.1,.1,1,True,True)
  self.assertEqual(r['mode'],'rotation');self.assertGreater(r['twist_preview'][2],0)
  r=f.step([0,0,a],.2,.2,1,True,True)
  self.assertEqual(r['mode'],'bounded_local_translation');self.assertGreater(r['twist_preview'][0],0)

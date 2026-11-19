"""Recorded corner replay and kinematic checks; no transport or sensor IO."""
import copy
import importlib.util
import json
from pathlib import Path
import random
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'engine_clone/modules'))
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module.PreviewFollower


class CornerApproachTests(unittest.TestCase):
    def setUp(self):
        self.fixture=json.loads((ROOT/'tests/fixtures/corner_stall_20261116.json').read_text())
        self.Before=load('original_corner_follower',ROOT.parent/'navigation_s100_v7/modules/follower.py')
        self.After=load('candidate_corner_follower',ROOT/'planner_overrides/follower.py')
    def follower(self,cls):
        f=cls(copy.deepcopy(self.fixture['plan']),copy.deepcopy(self.fixture['config']));f.index=self.fixture['recorded']['path_index'];return f
    def step(self,f,pose,time=100.):return f.step(pose,time,time,f.goal['revision'],True,True,odometry_pose=pose,control_time=time)
    def test_recorded_old_stall_is_reproduced_and_correction_now_converges(self):
        pose=self.fixture['recorded']['pose'];old=self.follower(self.Before);new=self.follower(self.After)
        before=self.step(old,pose);after=self.step(new,pose)
        np.testing.assert_allclose(before['twist_preview'],self.fixture['recorded']['logged_twist'],atol=1e-12)
        self.assertEqual(after['mode'],'corner_approach');self.assertEqual(new.index,old.index)
        self.assertGreater(np.linalg.norm(after['twist_preview'][:2]),.01)
        self.assertLessEqual(np.linalg.norm(after['twist_preview'][:2]),.015+1e-12)
        self.assertEqual(after['twist_preview'][2],before['twist_preview'][2])
        target=new.path[new.index+1,:2];delta=target-np.asarray(pose[:2]);c,s=np.cos(pose[2]),np.sin(pose[2]);velocity=np.array([[c,-s],[s,c]])@after['twist_preview'][:2]
        self.assertGreater(float(delta@velocity),0.)
    def test_does_not_expand_the_corner_acceptance_radius(self):
        f=self.follower(self.After);record=self.fixture['recorded'];corner=np.asarray(record['corner']);pose=corner.copy();pose[0]+=.016
        self.step(f,pose.tolist());self.assertEqual(f.index,record['path_index'])
        f=self.follower(self.After);pose=corner.copy();pose[0]+=.014
        self.step(f,pose.tolist());self.assertGreater(f.index,record['path_index'])
    def test_bounded_integrator_reaches_original_switch_condition(self):
        f=self.follower(self.After);initial=f.index;pose=np.array(self.fixture['recorded']['pose']);switched=None
        for k in range(100):
            out=self.step(f,pose.tolist(),100+k*.05)
            if f.index>initial:switched=k*.05;break
            self.assertLessEqual(np.linalg.norm(out['twist_preview'][:2]),.015+1e-12)
            c,s=np.cos(pose[2]),np.sin(pose[2]);pose[:2]+=np.array([[c,-s],[s,c]])@np.array(out['twist_preview'][:2])*.05
            pose[2]+=out['twist_preview'][2]*.05
        self.assertIsNotNone(switched);self.assertLess(switched,2.)
    def test_axial_to_lateral_corner_corrects_position_without_spurious_turn(self):
        cfg=copy.deepcopy(self.fixture['config']);cfg['fine_goal']['enabled']=False
        plan=dict(start_map_pose=[0.,0.,0.],target=dict(pose=[.1,-.3,0.],revision=1),
            selected=dict(profile='short_rectilinear_translation',geometry_recheck_passed=True,
                path=[[0.,0.,0.],[.1,0.,0.],[.1,-.1,0.],[.1,-.3,0.]]))
        f=self.After(plan,cfg);out=self.step(f,[.1,.02005,0.])
        self.assertEqual(out['mode'],'corner_approach');self.assertEqual(f.index,0)
        self.assertAlmostEqual(out['twist_preview'][0],0.)
        self.assertLess(out['twist_preview'][1],-.01)
        self.assertAlmostEqual(out['twist_preview'][2],0.)
        self.assertLessEqual(np.linalg.norm(out['twist_preview'][:2]),.015+1e-12)

    def test_ordinary_motion_and_guard_failures_keep_original_behavior(self):
        rng=random.Random(18);compared=0
        for index in range(3,len(self.fixture['plan']['selected']['path'])-10,9):
            before=self.follower(self.Before);after=self.follower(self.After);before.index=after.index=index
            pose=before.path[index].copy();pose[:2]+=np.array([rng.uniform(-.003,.003),rng.uniform(-.003,.003)])
            original=self.step(before,pose.tolist());candidate=self.step(after,pose.tolist())
            if candidate['mode']!='corner_approach':self.assertEqual(candidate,original);compared+=1
        self.assertGreater(compared,10)
        for valid,clear,age,revision in [(False,True,0.,10),(True,False,0.,10),(True,True,1.,10),(True,True,0.,-1)]:
            before=self.follower(self.Before);after=self.follower(self.After);pose=self.fixture['recorded']['pose']
            args=(pose,100-age,100,revision,valid,clear)
            self.assertEqual(before.step(*args),after.step(*args))


if __name__=='__main__':unittest.main()

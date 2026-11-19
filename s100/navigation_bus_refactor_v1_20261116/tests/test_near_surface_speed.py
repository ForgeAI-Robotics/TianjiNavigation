"""Pure speed-policy tests; no hardware or sensor connections."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'engine_clone/modules'))
from execution_runtime.motion import NearSurfaceSpeed

class NearSurfaceTests(unittest.TestCase):
    def geometry(self,x):
        shapes=[dict(world_corners=np.array([[-.3,-.3,0],[.3,.3,1.]]))]
        prepared={s:dict(points_body=np.array([[x,0,.7],[2,2,0]])) for s in ('front','back')}
        return prepared,shapes
    def test_forward_and_reverse_halved_other_axes_preserved(self):
        p,s=self.geometry(.69)
        for vx in (.07,-.07,0.):
            self.assertEqual(NearSurfaceSpeed().apply([vx,.03,.02],p,s,.1,10),[vx*.5,.03,.02])
    def test_far_unchanged_and_floor_does_not_trigger(self):
        p,s=self.geometry(.71)
        for v in p.values():v['points_body']=np.vstack([v['points_body'],[0,0,.1]])
        self.assertEqual(NearSurfaceSpeed().apply([.07,.03,0],p,s,.1,10),[.07,.03,0])
    def test_unknown_slow_and_release_hysteresis(self):
        p,s=self.geometry(.71);f=NearSurfaceSpeed()
        self.assertEqual(f.apply([.07,0,0],{},s,.1,10)[0],.035)
        self.assertEqual(f.apply([.07,0,0],p,s,.1,11.9)[0],.035)
        self.assertEqual(f.apply([.07,0,0],p,s,.1,12.1)[0],.07)
        self.assertEqual(f.apply([.07,0,0],p,s,.8,13)[0],.035)
    def test_stop_unchanged_and_instances_independent(self):
        p,s=self.geometry(.71);a=NearSurfaceSpeed();a.apply([0,0,0],{},s,.1,10)
        self.assertEqual(a.apply([0,0,0],p,s,.1,11),[0,0,0])
        self.assertEqual(NearSurfaceSpeed().apply([.07,0,0],p,s,.1,11)[0],.07)

if __name__=='__main__':unittest.main()

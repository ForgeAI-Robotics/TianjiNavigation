import unittest,sys,math,tempfile,json
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from navigation_admission import admission
from navigation_control import limited_twist,TRANSLATION_CAP_MPS,UNOBSERVED_TRAVEL_BUDGET_M
from clearance_override import effective
class AdmissionTests(unittest.TestCase):
 def packet(self):return dict(state='CLEAR',path_checked=True,operator_estop_declared=True,map_sha256='a'*64,model_sha256='b'*64,localization_age_s=.01,pose_age_s=.01,clearance_age_s=.01,guard_age_s=.01,cloud_age_s=.1,operator_age_s=.01)
 def test_every_deadline_and_manual_estop_are_required(self):
  d=self.packet();self.assertTrue(admission(d,0));self.assertFalse(admission(d,.6))
  for key in ['localization_age_s','pose_age_s','guard_age_s','cloud_age_s','operator_age_s']:
   p=dict(d);p[key]=10.;self.assertFalse(admission(p,0))
  self.assertFalse(admission(dict(d,operator_estop_declared=False),0));self.assertFalse(admission(dict(d,state='BLOCKED'),0))
 def test_forced_clearance_requires_current_matching_s100_config(self):
  d=dict(self.packet(),forced_release_active=True,assumed_clearance=True,override_version=1);cfg=dict(enabled=True,force_camera_clearance=True,force_shapemask_release=True,map_sha256='a'*64,model_sha256='b'*64,updated_wall_time=1)
  with patch('clearance_override.settings',return_value=cfg):self.assertTrue(admission(d,0))
  with patch('clearance_override.settings',return_value=dict(cfg,enabled=False)):self.assertFalse(admission(d,0))
  with patch('clearance_override.settings',return_value=dict(cfg,updated_wall_time=2)):self.assertFalse(admission(d,0))
 def test_velocity_caps_and_unobserved_swept_distance(self):
  for t in [[10,10,10],[-10,-10,-10],[.12,0,.2]]:
   v=limited_twist(t,1.1,2.);self.assertLessEqual(math.hypot(v[0],v[1]),TRANSLATION_CAP_MPS+1e-6);self.assertLessEqual(abs(v[2]),.20001);self.assertLessEqual((math.hypot(v[0],v[1])+2*abs(v[2]))*1.35,UNOBSERVED_TRAVEL_BUDGET_M+1e-6)
 def test_limiting_preserves_path_curvature_and_translation_direction(self):
  for twist in [[.2,0,.2],[.1,0,.2],[-.2,.04,-.15],[.01,.08,.1]]:
   v=limited_twist(twist,.8,1.4);scales=[a/b for a,b in zip(v,twist) if b]
   self.assertGreater(scales[0],0);self.assertTrue(all(abs(s-scales[0])<1e-12 for s in scales))
 def test_invalid_motion_is_rejected(self):
  for twist in [[float('nan'),0,0],[0,0,float('inf')]]:
   with self.assertRaises(ValueError):limited_twist(twist,.1,1.)
if __name__=='__main__':unittest.main()

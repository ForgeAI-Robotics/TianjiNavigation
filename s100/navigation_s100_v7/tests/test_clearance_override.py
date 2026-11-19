import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from clearance_override import effective
class OverrideTests(unittest.TestCase):
 def setUp(self):self.cfg=dict(enabled=True,force_camera_clearance=True,force_shapemask_release=True,map_sha256='map',model_sha256='model')
 def test_override_preserves_raw_and_only_forces_requested_release(self):
  r=effective(False,False,'map','model',self.cfg);self.assertTrue(r['clearance_clear']);self.assertTrue(r['shapemask_clear']);self.assertFalse(r['raw_clearance_clear']);self.assertEqual(r['release_state'],'FORCED_CLEAR_TEST');self.assertFalse(r['bypasses_lidar_obstacles']);self.assertFalse(r['bypasses_localization'])
 def test_scope_changes_do_not_force_release(self):
  for m,u in [('other','model'),('map','other')]:self.assertFalse(effective(False,False,m,u,self.cfg)['clearance_clear'])
 def test_off_returns_sensor_state(self):
  self.cfg['enabled']=False;r=effective(False,False,'map','model',self.cfg);self.assertEqual(r['release_state'],'NOT_CLEAR');self.assertFalse(r['forced_release_active'])
if __name__=='__main__':unittest.main()

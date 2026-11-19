import json,os,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import targets
class SceneTargetsTests(unittest.TestCase):
 def setUp(self):self.file=targets.FILE;self.env=dict(os.environ)
 def tearDown(self):targets.FILE=self.file;os.environ.clear();os.environ.update(self.env)
 def test_scene_switch_preserves_original_and_rejects_cross_map_snapshot(self):
  with tempfile.TemporaryDirectory() as d,patch.object(targets,'ROOT',Path(d)):
   root=Path(d);(root/'config/scenes').mkdir(parents=True)
   for scene,path,digest in [('outside',root/'config/targets.json','a'*64),('office',root/'config/scenes/office_targets.json','b'*64)]:
    path.write_text(json.dumps(dict(targets={str(i):dict(id=i,pose=[0,0,0],revision=1,map_voxel_sha256=digest)for i in [1,2]})))
   before=(root/'config/targets.json').read_bytes();targets.select_scene('outside');outside=targets.get(1)
   targets.select_scene('office');self.assertFalse(targets.unchanged(outside));self.assertEqual(targets.get(2)['map_voxel_sha256'],'b'*64)
   targets.select_scene('outside');self.assertTrue(targets.unchanged(outside));self.assertEqual((root/'config/targets.json').read_bytes(),before)
 def test_unknown_scene_rejected_before_configuration_change(self):
  with self.assertRaisesRegex(ValueError,'UNKNOWN_NAVIGATION_SCENE'):targets.select_scene('../outside')
  self.assertEqual(targets.FILE,self.file)
 def test_missing_office_goals_never_initialized_from_outside_waypoints(self):
  with tempfile.TemporaryDirectory() as d,patch.object(targets,'ROOT',Path(d)):
   targets.FILE=Path(d)/'config/scenes/office_targets.json';os.environ['TIANJI_NAV_SCENE']='office'
   with self.assertRaisesRegex(ValueError,'SCENE_TARGETS_NOT_CONFIGURED'):targets.initialize()

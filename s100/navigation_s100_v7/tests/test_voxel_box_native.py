import os,subprocess,unittest
from pathlib import Path
class NativeVoxelBoxTests(unittest.TestCase):
 def test_full_volume_boundary_rotation_cache_sweep_and_contact_witnesses(self):
  root=Path(__file__).resolve().parents[1];binary=Path(os.environ.get('TIANJI_VOXEL_TEST_BINARY',str(root/'native/voxel_box_test')))
  result=subprocess.run([str(binary)],capture_output=True,text=True,timeout=12)
  self.assertEqual(result.returncode,0,result.stdout+result.stderr);self.assertIn('VOXEL_BOX_PASS witnesses=3000',result.stdout)

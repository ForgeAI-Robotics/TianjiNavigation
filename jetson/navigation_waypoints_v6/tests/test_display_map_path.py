import unittest,tempfile,json,hashlib,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from display_map_path import local_map
class DisplayPathTests(unittest.TestCase):
 def test_remote_path_resolves_verified_local_copy(self):
  with tempfile.TemporaryDirectory()as d:
   root=Path(d);folder=root/'office';folder.mkdir();data=b'test';(folder/'voxels.npz').write_bytes(data);digest=hashlib.sha256(data).hexdigest();(folder/'map.json').write_text(json.dumps(dict(voxel_file_sha256=digest)));row=dict(map_folder='/home/sunrise/tianji_table_approach_dev/navigation_s100_v7/maps/office',map_voxel_sha256=digest)
   self.assertEqual(local_map(row,root),folder);row['map_voxel_sha256']='0'*64
   with self.assertRaises(ValueError):local_map(row,root)
 def test_missing_copy_reports_local_path(self):
  with tempfile.TemporaryDirectory()as d:
   with self.assertRaisesRegex(FileNotFoundError,'Jetson'):local_map(dict(map_folder='/home/sunrise/tianji_table_approach_dev/navigation_s100_v7/maps/missing',map_voxel_sha256='0'*64),d)

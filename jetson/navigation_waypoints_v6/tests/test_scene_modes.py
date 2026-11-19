import unittest,sys,tempfile
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'modules'))
from prepare import prepare
class SceneModeTests(unittest.TestCase):
 def test_local_snapshot_does_not_pass_global_localization_contract(self):
  with self.assertRaisesRegex(ValueError,'GLOBAL_LOCALIZATION_REQUIRED'):prepare({'status':'LOCAL_SNAPSHOT_ONLY'},{},Path('/unused'),assume_clearance=True)
 def test_local_mode_must_be_explicit_and_honestly_labelled(self):
  with self.assertRaisesRegex(ValueError,'LOCAL_SNAPSHOT_REQUIRED'):prepare({'status':'GLOBAL_LOCALIZED'},{},Path('/unused'),assume_clearance=True,local_snapshot=True)
if __name__=='__main__':unittest.main()

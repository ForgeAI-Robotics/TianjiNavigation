import unittest,sys,tempfile,json,os
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import controller_extension as c
class ExtensionTests(unittest.TestCase):
 def test_disabled_configuration_never_activates_controller(self):
  with tempfile.TemporaryDirectory()as d,patch.object(c,'CONFIG',Path(d)/'cfg.json'),patch.object(c.subprocess,'run')as run:
   self.assertFalse(c.ensure()['active']);run.assert_not_called()
 def test_binary_hash_mismatch_prevents_process_switch(self):
  with tempfile.TemporaryDirectory()as d,patch.object(c.subprocess,'run')as run:
   root=Path(d);binary=root/'controller';binary.write_bytes(b'wrong build');cfg=root/'cfg.json';cfg.write_text(json.dumps(dict(enabled=True,binary=str(binary),manager='unused',binary_sha256='0'*64)))
   with patch.object(c,'CONFIG',cfg):
    with self.assertRaisesRegex(ValueError,'BINARY_MISMATCH'):c.ensure()
   run.assert_not_called()
 def test_process_health_is_bound_to_pid_start_executable_and_inode(self):
  p=Path('/proc')/str(os.getpid());ref=dict(active=True,pid=os.getpid(),start_ticks=(p/'stat').read_text().split()[21],inode=(p/'exe').stat().st_ino,executable=str((p/'exe').resolve()))
  self.assertTrue(c.healthy(ref))
  for key,value in [('start_ticks','bad'),('inode',-1),('executable','/unrelated')]:self.assertFalse(c.healthy(dict(ref,**{key:value})))
if __name__=='__main__':unittest.main()

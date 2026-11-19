import sys,time,threading,tempfile,json,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from async_diagnostics import AsyncDiagnostics
import live_pose
from execution_safety import position_check

class AsyncDiagnosticTests(unittest.TestCase):
 def test_disk_stall_does_not_block_new_measurements_or_ram_pose(self):
  entered=threading.Event();release=threading.Event();written=[]
  def slow(item):entered.set();release.wait(2);written.append(item)
  writer=AsyncDiagnostics(slow,capacity=2)
  try:
   writer.submit(0);self.assertTrue(entered.wait(1))
   with tempfile.TemporaryDirectory() as d,patch.object(live_pose,'FAST_POSE',Path(d)/'pose.json'):
    began=time.monotonic()
    for i in range(1,31):
     packet={'observation_time':10+i*.01,'updated_wall_time':20+i*.01,'status':'FRESH','sequence':i};live_pose.publish(packet);writer.submit(i)
    self.assertLess(time.monotonic()-began,.5);self.assertEqual(live_pose.read(),packet);self.assertGreater(writer.stats()['dropped'],0)
   release.set();self.assertTrue(writer.close());self.assertEqual(written[-1],30)
  finally:release.set();writer.close()
 def test_io_error_is_reported_and_writer_can_persist_later_status(self):
  calls=[]
  def write(item):
   if item==1:raise OSError('synthetic disk error')
   calls.append(item)
  writer=AsyncDiagnostics(write);writer.submit(1);writer.submit(2);self.assertTrue(writer.close());self.assertEqual(calls,[2]);self.assertIn('synthetic disk error',writer.stats()['error'])
 def test_stale_ram_never_falls_back_to_green_disk_or_refreshes_timestamp(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);(root/'runtime').mkdir();(root/'runtime/current_pose.json').write_text(json.dumps(dict(status='GREEN',observation_time=100.)))
   with patch.object(live_pose,'FAST_POSE',root/'fast.json'),patch.object(live_pose,'ROOT',root):
    old=dict(status='STOPPED',observation_time=10.,updated_wall_time=11.,map_pose_valid=False)
    live_pose.publish(old);self.assertEqual(live_pose.read(),old)
    (root/'fast.json').unlink();self.assertEqual(live_pose.read()['observation_time'],100.)
 def test_invalid_ram_json_is_a_fault_not_disk_fallback(self):
  with tempfile.TemporaryDirectory() as d,patch.object(live_pose,'FAST_POSE',Path(d)/'pose.json'):
   live_pose.FAST_POSE.write_text('{')
   with self.assertRaises(ValueError):live_pose.read()

if __name__=='__main__':unittest.main()

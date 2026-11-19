import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from pose_memory import PoseMemory
class PoseAgeTests(unittest.TestCase):
 def packet(self,age):return dict(boot_id='boot',epoch=0,calibration_sha256='cal',observation_time=100.,pose=[0,0,0],source_age_s=age)
 def test_recent_receipt_does_not_freshen_old_measured_odometry(self):
  m=PoseMemory();self.assertTrue(m.ingest(self.packet(.6),10.));q=m.snapshot(10.01)
  self.assertAlmostEqual(q['odometry_age_s'],.61);self.assertFalse(q['odometry_fresh']);self.assertFalse(q['map_pose_valid'])
 def test_transport_and_memory_delay_both_remain_in_the_age(self):
  m=PoseMemory();m.ingest(self.packet(.12),10.);q=m.snapshot(10.08)
  self.assertAlmostEqual(q['odometry_age_s'],.2);self.assertTrue(q['odometry_fresh']);self.assertEqual(q['observation_time'],100.)
if __name__=='__main__':unittest.main()

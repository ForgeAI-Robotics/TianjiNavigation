import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from joint_scan_history import JointScanHistory

class HistoryTests(unittest.TestCase):
 def packet(self,t):return dict(received_at=t,posture_observation_time=t-.02,last_arm_receive_wall_time=t,source_status=dict(head_valid=True,grippers_valid=['left','right']),arm_fresh=True,model_hash_match=True,joint_positions_rad={'arm':t},whole_model_sha256='a'*64)
 def test_uses_recorded_pose_near_scan_time_instead_of_current_pose(self):
  h=JointScanHistory()
  for t in [10.,10.1,10.2,10.3,10.4,10.5]:h.sample(self.packet(t),t+.01)
  row=h.nearest(10.08);self.assertAlmostEqual(row['observation_time'],10.08);self.assertEqual(row['joint_positions_rad']['arm'],10.1)
 def test_missing_feedback_and_old_scans_never_get_freshened(self):
  h=JointScanHistory();p=self.packet(10.);p['source_status']['head_valid']=False;h.sample(p,10.01);self.assertFalse(h.nearest(10.))
  h.sample(self.packet(10.),10.8);self.assertFalse(h.nearest(10.))
  h.sample(self.packet(10.),10.01);self.assertFalse(h.nearest(11.));self.assertAlmostEqual(h.nearest(10.)['observation_time'],9.98)

if __name__=='__main__':unittest.main()

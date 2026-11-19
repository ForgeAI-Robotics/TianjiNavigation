import sys,json,time,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'modules'))
bridge=Path.home()/'tianji_state_bridge'
if not bridge.exists():bridge=Path.home()/'tianji_robot_state_bridge'
sys.path.insert(0,str(bridge))
import onboard_state
from receiver_joint_snapshot import snapshot,ARM_NAMES,HEAD,GRIPPERS

class FastPoseTests(unittest.TestCase):
 def data(self,now=100.):
  q={k:0. for k in [*ARM_NAMES,HEAD,*GRIPPERS.values()]};stamps={k:now-.04 for k in q}
  source=dict(received_at=now-.1,head_valid=True,grippers_valid=['left','right'])
  return snapshot(q,stamps,now-.01,True,source,'c'*64,now,True)
 def test_snapshot_preserves_measurement_time_instead_of_regenerating_old_angles(self):
  d=self.data();self.assertAlmostEqual(d['posture_observation_time'],99.96);self.assertEqual(d['received_at'],100.);self.assertTrue(d['arm_fresh']);self.assertEqual(d['motion_commands_sent'],0)
 def test_stale_head_and_grippers_are_not_restored_by_new_snapshot_time(self):
  q={k:0. for k in [*ARM_NAMES,HEAD,*GRIPPERS.values()]};stamps={k:99.95 for k in q};stamps[HEAD]=99.;stamps[GRIPPERS['left']]=99.
  d=snapshot(q,stamps,99.99,True,dict(received_at=99.9,head_valid=True,grippers_valid=['left','right']),'c'*64,100.,True)
  self.assertFalse(d['source_status']['head_valid']);self.assertEqual(d['source_status']['grippers_valid'],['right'])
 def test_navigation_prefers_fast_packet_and_returns_actual_pose_time(self):
  now=time.time();d=self.data(now)
  with tempfile.TemporaryDirectory() as directory,patch.object(onboard_state,'FAST_STATUS',Path(directory)/'pose.json') as file,patch.object(onboard_state,'model',return_value=SimpleNamespace(sha256='c'*64)),patch.object(onboard_state,'state') as slow:
   file.write_text(json.dumps(d));p=onboard_state.measured_joints();self.assertEqual(p['observation_time'],d['posture_observation_time']);slow.assert_not_called()
   d['received_at']=now-1.;file.write_text(json.dumps(d))
   with self.assertRaisesRegex(ValueError,'FRESH_S100'):onboard_state.measured_joints()
   slow.assert_not_called()

if __name__=='__main__':unittest.main()

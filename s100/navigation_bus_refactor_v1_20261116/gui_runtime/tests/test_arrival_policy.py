import ast,math,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,'/home/sunrise/tianji_table_approach_dev/navigation_s100_v7/modules');sys.path.insert(0,str(ROOT/'modules'))
from navigation_stop_evidence import arrival_pose_before_parking,arrival_verdict,assess
class ArrivalPolicyTests(unittest.TestCase):
 def evidence(self):
  return assess([dict(kind='feedback',received_wall_time=10.1+i*.07,source_stamp=10.1+i*.07,wheel=[[n,0.,0.]for n in range(4)])for i in range(5)],10.,10.4)
 def pose(self,x=.0441062,yaw=.30355,stamp=9.9):return arrival_pose_before_parking([x,0.,math.radians(yaw)],[0.,0.,0.],stamp,10.)
 def test_accepted_arrival_remains_accepted_after_parking(self):
  accepted=self.pose();self.assertEqual(arrival_verdict(accepted,self.evidence()),'ARRIVED')
  # Parking may later shift to 5.39cm; it is not an input to the arrival verdict.
  later_pose=[.053893,0.,math.radians(.5564)]
  self.assertGreater(later_pose[0],.05);self.assertEqual(accepted['final_xyt'][0],.0441062)
  self.assertFalse(accepted['post_parking_pose_evaluated'])
 def test_position_outside_gate_rejected(self):self.assertEqual(arrival_verdict(self.pose(.05001),self.evidence()),'STOP_FINAL_POSE_OUTSIDE_TOLERANCE')
 def test_heading_outside_gate_rejected(self):self.assertEqual(arrival_verdict(self.pose(yaw=1.001),self.evidence()),'STOP_FINAL_POSE_OUTSIDE_TOLERANCE')
 def test_exact_position_boundary_accepted(self):self.assertEqual(arrival_verdict(self.pose(.05,yaw=0),self.evidence()),'ARRIVED')
 def test_heading_inside_gate_accepted(self):self.assertEqual(arrival_verdict(self.pose(yaw=.999),self.evidence()),'ARRIVED')
 def test_stale_pose_rejected(self):self.assertEqual(arrival_verdict(self.pose(stamp=9.4),self.evidence()),'STOP_FINAL_POSE_UNCONFIRMED')
 def test_invalid_pose_rejected(self):self.assertFalse(self.pose(float('nan'))['valid']);self.assertFalse(arrival_pose_before_parking(None,None,None,None)['valid'])
 def test_stop_proof_still_required(self):self.assertEqual(arrival_verdict(self.pose(),{}),'STOP_FINAL_STOP_UNCONFIRMED')
 def test_failed_control_shutdown_not_success(self):self.assertEqual(arrival_verdict(self.pose(),self.evidence(),False),'STOP_CONTROL_SHUTDOWN')
 def test_cleanup_does_not_read_post_parking_pose(self):
  source=(ROOT/'modules/onboard_navigation.py').read_text();cleanup=source[source.index('        arrival_pose = None'):source.index('        resources.stop_observers()')]
  calls=[n.func.id for n in ast.walk(ast.parse('def cleanup():\n'+cleanup))if isinstance(n,ast.Call)and isinstance(n.func,ast.Name)]
  self.assertNotIn('read_live_pose',calls);self.assertNotIn('final_pose',calls);self.assertIn('arrival_pose_before_parking',calls);self.assertIn('arrival_verdict',calls)
if __name__=='__main__':unittest.main()

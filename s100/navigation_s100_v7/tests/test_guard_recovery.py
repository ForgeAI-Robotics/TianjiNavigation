import sys,unittest,threading
from unittest.mock import patch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from guard_recovery import GuardRecovery,expired_deadlines
from navigation_admission import REQUIRED,admission
from navigation_control import ControlTransport

class RecoveryTests(unittest.TestCase):
 def test_stale_data_pauses_without_waiving_admission(self):
  guard=dict.fromkeys(REQUIRED,0.);guard['localization_age_s']=.7
  self.assertEqual(expired_deadlines(guard),['localization_age_s'])
  r=GuardRecovery();self.assertTrue(r.waiting('ADMISSION_DEADLINE_EXPIRED',guard,10))
  self.assertFalse(admission(guard,0))
  self.assertTrue(r.within_budget(12.9));self.assertFalse(r.within_budget(13.01))
 def test_lidar_loss_only_gets_bounded_zero_output_wait(self):
  r=GuardRecovery();self.assertTrue(r.waiting('LIDAR_STALE',None,10));self.assertFalse(r.within_budget(13.01))
  guard=dict.fromkeys(REQUIRED,0.);guard['cloud_age_s']=1.21
  self.assertFalse(admission(guard,0))
 def test_incomplete_feedback_wait_is_bounded(self):
  for reason in ['FULL_HEAD_ARM_GRIPPER_FEEDBACK_REQUIRED','FRESH_S100_RECEIVED_JOINT_FEEDBACK_REQUIRED']:
   r=GuardRecovery();self.assertTrue(r.waiting(reason,None,10));self.assertFalse(r.within_budget(13.01))
 def test_geometry_operator_and_identity_failures_never_retry(self):
  for reason in ['LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY','LOCALIZATION_FRAME_OR_EPOCH_CHANGED','URDF_VERSION_MISMATCH','POSTURE_CHANGED_REPLAN_REQUIRED','CLEARANCE_OR_SHAPEMASK_NOT_CLEAR']:
   self.assertFalse(GuardRecovery().waiting(reason,None,10))
  g=dict.fromkeys(REQUIRED,0.);g['operator_age_s']=.6
  self.assertFalse(GuardRecovery().waiting('ADMISSION_DEADLINE_EXPIRED',g,10))
  self.assertFalse(GuardRecovery().waiting('ADMISSION_DEADLINE_EXPIRED',dict.fromkeys(REQUIRED,0.),10))
 def test_repeated_loss_has_total_budget(self):
  r=GuardRecovery()
  for t in [10,13]:
   self.assertTrue(r.waiting('LOCALIZATION_STALE',None,t));r.resumed(t+2.5)
  self.assertTrue(r.waiting('LOCALIZATION_STALE',None,16));self.assertFalse(r.waiting('LOCALIZATION_STALE',None,17.1))
 def test_stable_running_resets_recent_budget_not_source_deadlines(self):
  r=GuardRecovery()
  for t in [10,20,30,40]:
   self.assertTrue(r.waiting('LOCALIZATION_STALE',None,t));self.assertTrue(r.within_budget(t+2.5));r.resumed(t+2.5)
  self.assertLessEqual(r.spent,2.5)
  g=dict.fromkeys(REQUIRED,0.);g['cloud_age_s']=1.21;self.assertFalse(admission(g,0))
 def test_wait_does_not_reset_when_some_checks_recover(self):
  r=GuardRecovery();self.assertTrue(r.waiting('JOINT_FEEDBACK_STALE',None,10))
  self.assertTrue(r.waiting('LOCALIZATION_STALE',None,12));self.assertFalse(r.within_budget(13.1))
 def test_transient_feedback_error_sends_zero_and_sender_recovers(self):
  import collections,types
  transport=ControlTransport.__new__(ControlTransport)
  transport.lock=threading.Lock();transport.error=None;transport.real=False;transport.zero_epoch=0;transport.sequence=0;transport.sent=0;transport.nonzero=0;transport.gate_counts=collections.Counter()
  transport.status={'challenge':'current','received':1.};transport.decision=([.01,0,0],dict.fromkeys(REQUIRED,0.),1.,{'reference':True});transport.process=types.SimpleNamespace(stdin=object())
  from unittest.mock import Mock
  transport.stop=Mock();transport.stop.wait.side_effect=[False,False,True];transport.stop.is_set.return_value=False
  transport.posture_refresh=Mock(side_effect=[ValueError('FRESH_S100_RECEIVED_JOINT_FEEDBACK_REQUIRED'),dict(age_s=.01,observation_time=1.)])
  with patch('navigation_control.time.monotonic',return_value=1.),patch('navigation_control.admission',return_value=True),patch('navigation_control.write_packet') as write:
   transport.send_loop()
  self.assertIsNone(transport.error);self.assertEqual(write.call_args_list[0].args[1]['command'],[0.,0.,0.]);self.assertEqual(write.call_args_list[1].args[1]['command'],[.01,0,0]);self.assertEqual(transport.gate_counts['posture_refresh_rejected'],1)
 def test_cold_control_bridge_not_ready_until_fresh_challenge(self):
  transport=ControlTransport.__new__(ControlTransport)
  transport.lock=threading.Lock();transport.error=None;transport.status={};transport.real=False
  self.assertFalse(transport.ready())
  transport.status={'challenge':'measured-bridge-response','received':10.}
  with patch('navigation_control.time.monotonic',return_value=10.1):self.assertTrue(transport.ready())
  with patch('navigation_control.time.monotonic',return_value=10.3):self.assertFalse(transport.ready())
  transport.error='bridge exited'
  with patch('navigation_control.time.monotonic',return_value=10.1):self.assertFalse(transport.ready())
 def test_real_control_waits_for_ros_bridge_subscription(self):
  transport=ControlTransport.__new__(ControlTransport)
  transport.lock=threading.Lock();transport.error=None;transport.real=True
  transport.status={'challenge':'response','received':10.}
  with patch('navigation_control.time.monotonic',return_value=10.1):
   self.assertFalse(transport.ready())
   transport.status['bridge_ready']=False;self.assertFalse(transport.ready())
   transport.status['bridge_ready']=True;self.assertTrue(transport.ready())

if __name__=='__main__':unittest.main()

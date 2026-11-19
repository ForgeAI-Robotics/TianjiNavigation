import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from sensor_recovery import SensorRecovery
from guard_recovery import GuardRecovery,waitable_data_fault
from navigation_admission import REQUIRED,admission

class SensorRecoveryTests(unittest.TestCase):
 def frames(self,a,b):return {'front':({'sequence':a},None,0),'back':({'sequence':b},None,0)}
 def test_long_fault_preserves_wait_but_stale_input_never_admits(self):
  transient=GuardRecovery();hold=SensorRecovery();self.assertTrue(transient.waiting('LIDAR_STALE',None,10))
  self.assertFalse(transient.waiting('LIDAR_STALE',None,14));hold.blocked('LIDAR_STALE',14,self.frames(1,1))
  hold.blocked('LIDAR_STALE',100,self.frames(2,2));self.assertTrue(hold.active)
  stale=dict.fromkeys(REQUIRED,0.);stale['cloud_age_s']=1.21;self.assertFalse(admission(stale,0))
 def test_requires_both_radars_to_advance_and_continuous_fresh_guards(self):
  h=SensorRecovery();h.blocked('LOCALIZATION_STALE',0,self.frames(1,1))
  self.assertFalse(h.checked_clear(self.frames(1,1),5))
  self.assertFalse(h.checked_clear(self.frames(2,1),5))
  self.assertFalse(h.checked_clear(self.frames(2,2),5));self.assertFalse(h.checked_clear(self.frames(3,3),5.4))
  self.assertTrue(h.checked_clear(self.frames(4,4),5.9));self.assertFalse(h.active)
 def test_new_fault_or_failed_geometry_interrupts_release_evidence(self):
  h=SensorRecovery();h.blocked('JOINT_FEEDBACK_STALE',0)
  h.checked_clear(self.frames(1,1),1);h.checked_clear(self.frames(2,2),1.5)
  h.interrupt_clear();self.assertFalse(h.checked_clear(self.frames(3,3),2))
  h.blocked('LIDAR_STALE',2.1,self.frames(3,3));self.assertFalse(h.checked_clear(self.frames(4,4),3));self.assertFalse(h.checked_clear(self.frames(5,5),3.4));self.assertTrue(h.checked_clear(self.frames(6,6),3.9))
 def test_identity_operator_geometry_and_unknown_faults_are_terminal(self):
  h=SensorRecovery()
  for reason in ['LIDAR_FRAME_CHANGED','LOCALIZATION_FRAME_OR_EPOCH_CHANGED','URDF_VERSION_MISMATCH','POSTURE_CHANGED_REPLAN_REQUIRED','OPERATOR_LINK_LOST','unexpected failure','LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY']:
   self.assertFalse(waitable_data_fault(reason,None))
   with self.assertRaises(ValueError):h.blocked(reason,0)
  guard=dict.fromkeys(REQUIRED,0.);guard['operator_age_s']=.6
  self.assertFalse(waitable_data_fault('ADMISSION_DEADLINE_EXPIRED',guard))
  with self.assertRaises(ValueError):h.blocked('ADMISSION_DEADLINE_EXPIRED',0,guard=guard)
 def test_only_actually_expired_data_deadlines_qualify(self):
  guard=dict.fromkeys(REQUIRED,0.)
  self.assertFalse(waitable_data_fault('ADMISSION_DEADLINE_EXPIRED',guard))
  guard['pose_age_s']=.4;self.assertTrue(waitable_data_fault('ADMISSION_DEADLINE_EXPIRED',guard))

if __name__=='__main__':unittest.main()

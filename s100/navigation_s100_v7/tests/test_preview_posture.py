import unittest,copy
from preview_posture import ARMS,GRIPPERS,posture,camera_check,validate_options
class PreviewSwitchTests(unittest.TestCase):
 def setUp(self):
  self.raw=dict(joint_positions_rad={k:.123 for k in ARMS},observation_time=10.,model_sha256='a'*64,live=False);self.opened=dict(zip(GRIPPERS,[.18,.20]))
 def test_open_switch_only_replaces_grippers_preserving_fresh_arms_and_head_unknown(self):
  original=copy.deepcopy(self.raw);r=posture(self.raw,dict(ignore_head_camera=True,open_grippers=True),self.opened,10.1)
  self.assertEqual(self.raw,original);self.assertNotIn('head_pitch_joint',r['joint_positions_rad']);self.assertEqual(r['assumed_joints'],self.opened)
  for k in ARMS:self.assertEqual(r['joint_positions_rad'][k],self.raw['joint_positions_rad'][k])
 def test_measured_mode_requires_grippers_and_does_not_force_open(self):
  options=dict(ignore_head_camera=True,open_grippers=False)
  with self.assertRaisesRegex(ValueError,'MEASURED_GRIPPER_FEEDBACK_REQUIRED'):posture(self.raw,options,self.opened,10.1)
  self.raw['joint_positions_rad'].update(dict(zip(GRIPPERS,[.4,.5])));r=posture(self.raw,options,self.opened,10.1)
  self.assertEqual(r['assumed_joints'],{});self.assertEqual(r['joint_positions_rad'][GRIPPERS[0]],.4)
 def test_camera_off_requires_raw_measured_clear_even_if_other_override_active(self):
  for open_grippers in [True,False]:
   options=dict(ignore_head_camera=False,open_grippers=open_grippers)
   with self.assertRaisesRegex(ValueError,'HEAD_CAMERA_SHAPEMASK_NOT_MEASURED_CLEAR'):camera_check(options,dict(clearance_clear=True,shapemask_clear=True,forced_release_active=True,raw_clearance_clear=False,raw_shapemask_clear=False))
   camera_check(options,dict(raw_clearance_clear=True,raw_shapemask_clear=True))
  camera_check(dict(ignore_head_camera=True,open_grippers=False),{})
 def test_both_switches_still_reject_stale_or_missing_arms(self):
  options=dict(ignore_head_camera=True,open_grippers=True)
  with self.assertRaisesRegex(ValueError,'FRESH_ARM'):posture(self.raw,options,self.opened,10.5)
  del self.raw['joint_positions_rad'][ARMS[0]]
  with self.assertRaisesRegex(ValueError,'FRESH_14'):posture(self.raw,options,self.opened,10.1)
 def test_switch_types_and_unknown_fields_rejected(self):
  with self.assertRaises(ValueError):validate_options(dict(ignore_head_camera=1,open_grippers=True))
  with self.assertRaises(ValueError):validate_options(dict(ignore_head_camera=True,open_grippers=True,enable_motion=True))

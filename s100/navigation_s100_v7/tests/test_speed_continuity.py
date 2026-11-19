import sys,unittest,threading,types,collections
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'modules'))
import paths
from navigation_control import ControlTransport,limited_twist,UNOBSERVED_TRAVEL_BUDGET_M
from posture_lease import motion_weights,corner_shift,joint_shift_bound
from execution_safety import REQUIRED_JOINTS

class SpeedContinuityTests(unittest.TestCase):
 def pump(self):
  t=ControlTransport.__new__(ControlTransport);t.real=False;t.error=None;t.lock=threading.Lock();t.zero_epoch=0;t.gate_counts=collections.Counter();t.sent=t.nonzero=t.sequence=0;t.process=types.SimpleNamespace(stdin=None)
  guard=dict(state='CLEAR',path_checked=True,operator_estop_declared=True,map_sha256='a'*64,model_sha256='b'*64,localization_age_s=.01,pose_age_s=.01,clearance_age_s=.01,guard_age_s=0.,cloud_age_s=.2,operator_age_s=.01)
  t.status=dict(challenge='old',received=.7);t.decision=([.01,0,0],guard,.95,{});t.stop=Mock();t.stop.is_set.return_value=False;return t,guard
 def test_latest_control_status_is_used_after_posture_validation(self):
  t,g=self.pump();t.stop.wait.side_effect=[False,True]
  def refresh(_):t.status=dict(challenge='new',received=1.);return dict(age_s=.01,observation_time=100.)
  t.posture_refresh=refresh
  with patch('navigation_control.time.monotonic',return_value=1.),patch('navigation_control.write_packet')as write:t.send_loop()
  self.assertIsNone(t.error);packet=write.call_args.args[1];self.assertEqual(packet['challenge'],'new');self.assertEqual(packet['command'],[.01,0,0])
 def test_superseded_old_decision_does_not_send_zero_over_new_admitted_decision(self):
  t,g=self.pump();t.status['received']=1.;t.stop.wait.side_effect=[False,False,True];calls=[0]
  def refresh(_):
   calls[0]+=1
   if calls[0]==1:t.update([.02,0,0],g,posture_reference={})
   return dict(age_s=.01,observation_time=100.)
  t.posture_refresh=refresh
  with patch('navigation_control.time.monotonic',return_value=1.),patch('navigation_control.write_packet')as write:t.send_loop()
  self.assertEqual(write.call_count,1);self.assertEqual(write.call_args.args[1]['command'],[.02,0,0]);self.assertEqual(t.gate_counts['superseded_decision_rechecks'],1)
 def test_missing_fresh_control_status_still_forces_zero(self):
  t,g=self.pump();t.stop.wait.side_effect=[False,True];t.posture_refresh=lambda _:dict(age_s=.01,observation_time=100.)
  with patch('navigation_control.time.monotonic',return_value=1.),patch('navigation_control.write_packet')as write:t.send_loop()
  self.assertEqual(write.call_args.args[1]['command'],[0.,0.,0.]);self.assertEqual(t.gate_counts['control_status_stale'],1)
 def test_new_cruise_is_modestly_faster_within_explicit_five_cm_budget(self):
  v=limited_twist([.2,0,0],.8,1.)
  self.assertGreater(v[0],.04/1.45*1.2);self.assertLessEqual(v[0],.05);self.assertLessEqual(v[0]*1.45,UNOBSERVED_TRAVEL_BUDGET_M+1e-12)
 def test_pure_translation_reaches_five_cm_in_both_axes_and_reverse(self):
  for command in [[.2,0,0],[-.2,0,0],[0,.2,0],[0,-.2,0],[.1,.1,0]]:
   v=limited_twist(command,.8,1.4);self.assertAlmostEqual(float(np.linalg.norm(v[:2])),.05);self.assertEqual(v[2],0.)
 def test_rotation_profile_matches_pre_change_recorded_outputs(self):
  cases=[([0,0,.2],1.4,[0,0,.024630541871921187]),([0,0,.025],1.4,[0,0,.024630541871921187]),([.2,0,.2],1.4,[.014367816091954025,0,.014367816091954025]),([-.1,.04,-.03],1.2,[-.02399580214670235,.00959832085868094,-.007198740644010704]),([.02,.04,.01],1.,[.012603034319417714,.025206068638835427,.006301517159708857])]
  for command,radius,previous in cases:np.testing.assert_allclose(limited_twist(command,.8,radius),previous,atol=1e-14,rtol=0)
 def test_tree_kinematic_bound_contains_actual_urdf_fk_displacements(self):
  from snapshot_builder import local_model
  from chassis_proxy import measured_shapes
  model=local_model();weights=motion_weights(model.geometry);self.assertTrue(set(REQUIRED_JOINTS)<=set(weights));self.assertLess(weights['Joint1_L'],4.);rng=np.random.default_rng(319)
  for _ in range(12):
   old={k:float(rng.uniform(-.2,.2))for k in REQUIRED_JOINTS};new={k:v+float(rng.uniform(-.001,.001))for k,v in old.items()};before=measured_shapes(model,old);after=measured_shapes(model,new)
   bound=joint_shift_bound(before,old,new,weights);self.assertLessEqual(corner_shift(before,after),bound+1e-10)
if __name__=='__main__':unittest.main()

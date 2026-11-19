import sys,unittest,itertools,threading,types,time,collections
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import paths
from execution_safety import REQUIRED_JOINTS
from posture_lease import revalidate,motion_weights
from navigation_control import ControlTransport

class PostureLeaseTests(unittest.TestCase):
 def setUp(self):
  self.q=dict.fromkeys(REQUIRED_JOINTS,0.);self.sha='a'*64
  self.shape=dict(link='arm',bounds=[[-.03,-.03,.1],[.03,.03,.9]],matrix=np.eye(4).tolist())
  self.ref=dict(joint_positions_rad=self.q,shapes=[self.shape],reserve_m=.003)
  self.saved=dict(joint_positions_rad=self.q)
 def current(self,**changes):return dict(joint_positions_rad=dict(self.q,**changes),model_sha256=self.sha,observation_time=100.)
 def shapes(self,q):
  T=np.eye(4);T[0,3]=q['Joint1_L'];return [dict(self.shape,matrix=T.tolist())]
 def test_new_measured_feedback_may_renew_only_its_actual_age(self):
  result=revalidate(self.ref,self.current(),self.saved,self.sha,self.shapes,now=100.02)
  self.assertAlmostEqual(result['age_s'],.02);self.assertEqual(result['observation_time'],100.)
 def test_geometric_shift_must_fit_remaining_sweep_reserve(self):
  result=revalidate(self.ref,self.current(Joint1_L=.002),self.saved,self.sha,self.shapes,now=100.02)
  self.assertAlmostEqual(result['joint_geometry_shift_m'],.002)
  self.assertIsNone(revalidate(self.ref,self.current(Joint1_L=.004),self.saved,self.sha,self.shapes,now=100.02))
 def test_stale_missing_or_wrong_model_feedback_never_renews(self):
  self.assertIsNone(revalidate(self.ref,self.current(),self.saved,self.sha,self.shapes,now=100.5))
  packet=self.current();packet['model_sha256']='b'*64
  self.assertIsNone(revalidate(self.ref,packet,self.saved,self.sha,self.shapes,now=100.02))
 def test_conservative_kinematic_certificate_avoids_full_geometry_work(self):
  geometry=dict(joints=[dict(name=k,type='revolute',axis=[0,0,1],xyz=[.01,0,0]) for k in REQUIRED_JOINTS],shapes=[dict(xyz=[0,0,0],bounds=[[-.1,-.1,-.1],[.1,.1,.1]])])
  weights=motion_weights(geometry);self.assertEqual(set(weights),set(REQUIRED_JOINTS))
  shape=dict(self.shape,mode='measured_or_fixed');reference=dict(self.ref,shapes=[shape],motion_weights_m=weights)
  with patch.object(self,'shapes',side_effect=AssertionError('Unexpected full FK')):
   result=revalidate(reference,self.current(Joint1_L=.001),self.saved,self.sha,self.shapes,now=100.02)
  self.assertIsNotNone(result);self.assertGreater(result['joint_geometry_shift_m'],0)
 def test_unsupported_kinematics_never_get_a_fast_certificate(self):
  geometry=dict(joints=[dict(name='slide',type='prismatic',axis=[1,0,0],xyz=[.1,0,0])],shapes=[dict(xyz=[0,0,0],bounds=[[0,0,0],[.1,.1,.1]])])
  self.assertIsNone(motion_weights(geometry))
  packet=self.current();packet['joint_positions_rad'].pop('head_pitch_joint')
  self.assertIsNone(revalidate(self.ref,packet,self.saved,self.sha,self.shapes,now=100.02))
 def test_explicit_zero_wins_even_while_posture_revalidation_is_running(self):
  transport=ControlTransport.__new__(ControlTransport);transport.real=False;transport.stop=threading.Event();transport.lock=threading.Lock();transport.zero_epoch=0;transport.gate_counts=collections.Counter();transport.sent=0;transport.nonzero=0;transport.sequence=0;transport.error=None;transport.process=types.SimpleNamespace(stdin=None)
  now=time.monotonic();guard=dict(state='CLEAR',path_checked=True,operator_estop_declared=True,map_sha256='a'*64,model_sha256='a'*64,localization_age_s=0.,pose_age_s=0.,clearance_age_s=0.,guard_age_s=0.,cloud_age_s=0.,operator_age_s=0.)
  transport.status=dict(challenge='challenge',received=now);transport.decision=([.01,0,0],guard,now,{})
  def refresh(_):transport.zero();return dict(age_s=0.,observation_time=100.)
  transport.posture_refresh=refresh;packets=[]
  def capture(_,packet):packets.append(packet);transport.stop.set()
  with patch('navigation_control.write_packet',side_effect=capture):transport.send_loop()
  self.assertEqual(packets[0]['command'],[0.,0.,0.]);self.assertEqual(packets[0]['navigation']['state'],'BLOCKED')

if __name__=='__main__':unittest.main()

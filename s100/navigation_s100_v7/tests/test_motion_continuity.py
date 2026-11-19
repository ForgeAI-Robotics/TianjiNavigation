import sys,unittest,tempfile,json,types,threading,collections
from pathlib import Path
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import paths
from navigation_control import ControlTransport
from motor_trace import MotorTrace

class ContinuityTests(unittest.TestCase):
 def output_at_age(self,age):
  t=ControlTransport.__new__(ControlTransport);t.real=False;t.error=None;t.lock=threading.Lock();t.zero_epoch=0;t.gate_counts=collections.Counter();t.sent=t.nonzero=t.sequence=0;t.posture_refresh=None;t.status=dict(challenge='current',received=1.);t.process=types.SimpleNamespace(stdin=None)
  guard=dict(state='CLEAR',path_checked=True,operator_estop_declared=True,map_sha256='a'*64,model_sha256='b'*64,localization_age_s=.01,pose_age_s=.01,clearance_age_s=.01,guard_age_s=0.,cloud_age_s=.3,operator_age_s=.01)
  t.decision=([.01,0.,0.],guard,1.-age,None);t.stop=Mock();t.stop.wait.side_effect=[False,True];t.stop.is_set.return_value=False
  with patch('navigation_control.time.monotonic',return_value=1.),patch('navigation_control.write_packet')as write:t.send_loop()
  self.assertIsNone(t.error);return write.call_args.args[1]
 def test_valid_full_certificate_not_cut_off_at_200ms(self):self.assertEqual(self.output_at_age(.23)['command'],[.01,0.,0.])
 def test_original_350ms_guard_deadline_still_stops(self):self.assertEqual(self.output_at_age(.351)['command'],[0.,0.,0.])
 def test_readonly_lock_probes_do_not_block_each_other(self):
  import fcntl
  from common import lock_held,exclusive
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp)/'mapping.lock';p.touch()
   with p.open('r')as probe:
    fcntl.flock(probe,fcntl.LOCK_SH|fcntl.LOCK_NB);self.assertFalse(lock_held(p))
   owner=exclusive(p)
   try:self.assertTrue(lock_held(p))
   finally:owner.close()
 def test_final_bridge_preserves_current_certified_curvature_and_local_caps(self):
  from mapping_guard import bounded_admitted_twist
  for requested,allowed in [([.02,0,.03],[.03,0,.02]),([.03,.01,.04],[.02,.02,.01]),([-.02,0,-.03],[-.03,0,-.02])]:
   result=bounded_admitted_twist(requested,allowed);ratios=[a/b for a,b in zip(result,allowed)if b]
   self.assertTrue(all(abs(x-ratios[0])<1e-12 for x in ratios));self.assertTrue(all(abs(x)<=abs(y)+1e-12 for x,y in zip(result,requested)))
 def test_final_bridge_zero_and_direction_disagreement_never_move(self):
  from mapping_guard import bounded_admitted_twist
  for requested,allowed in [([0,0,0],[.02,0,.02]),([.02,0,.02],[.02,0,-.02]),([.02,0,0],[0,.02,0])]:self.assertEqual(bounded_admitted_twist(requested,allowed),(0.,0.,0.))
 def test_passive_motor_trace_keeps_target_and_feedback_timestamps(self):
  callbacks={}
  node=Mock();node.create_subscription.side_effect=lambda kind,topic,callback,qos:callbacks.setdefault(topic,callback)
  mods={'driver':types.ModuleType('driver'),'driver.msg':types.SimpleNamespace(MotorsData=object,SetMotorCmd=object),'rclpy.qos':types.SimpleNamespace(QoSProfile=lambda **kw:kw,ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='best_effort'))}
  with tempfile.TemporaryDirectory()as tmp,patch.dict(sys.modules,mods):
   path=Path(tmp)/'motor_trace.jsonl';trace=MotorTrace(node,path)
   item=types.SimpleNamespace(id=5,speed=0.,position=.5);msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=10,nanosec=250000000)),wheel_data=[item],steer_data=[item])
   callbacks['/driver/motor/SetMotorCmd'](msg);callbacks['/driver/motor/Data'](msg);trace.close()
   rows=[json.loads(x)for x in path.read_text().splitlines()];self.assertEqual([x['kind']for x in rows],['target','feedback']);self.assertEqual(rows[0]['source_stamp'],10.25)
   summary=json.loads(path.with_suffix('.json').read_text());self.assertEqual(summary['queue_dropped'],0);self.assertEqual(summary['ros_publishers_created'],0);self.assertIsNone(summary['error']);node.create_publisher.assert_not_called()
if __name__=='__main__':unittest.main()

import unittest,tempfile,json,threading,types,sys,io
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import onboard_navigation as nav
class ExecutionFailureTests(unittest.TestCase):
 def test_blocked_route_preserves_reason_and_never_creates_control_or_sensor(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);(root/'runtime').mkdir();folder=root/'results/blocked';(folder/'planning').mkdir(parents=True);(folder/'scene').mkdir()
   (root/'runtime/latest_plan.json').write_text(json.dumps({'folder':str(folder)}));(folder/'planning/plan.json').write_text(json.dumps(dict(selected=None,attempts=[dict(expanded_nodes=1,reason='SEARCH_EXHAUSTED')])));(folder/'scene/frozen_input.json').write_text('{}')
   ready=threading.Event();done=threading.Event()
   class Operator:
    def __iter__(self):
     yield json.dumps(dict(kind='heartbeat',session='unit'))+'\n'
     ready.set();done.wait(2)
   args=types.SimpleNamespace(goal=1,session='unit',enable_motion=False,preview=True,operator_estop_ready=True,duration=.1)
   with patch.object(nav,'ROOT',root),patch.object(sys,'stdin',Operator()),patch('navigation.run',side_effect=lambda *_,**kwargs:ready.wait(1)),patch.object(nav.signal,'signal'),patch.object(nav,'ControlTransport') as control,patch.object(nav,'SensorStream') as sensors,patch.object(nav,'MapGuard') as mapguard,patch('sys.stdout',new_callable=io.StringIO) as output:
    try:nav.run(args)
    finally:done.set()
    control.assert_not_called();sensors.assert_not_called();mapguard.assert_not_called();last=json.loads(output.getvalue().strip().splitlines()[-1][7:]);self.assertTrue(last['finished']);self.assertFalse(last['active']);self.assertIn('SEARCH_EXHAUSTED',last['reason'])
   result=json.loads((root/'runtime/navigation_execution_status.json').read_text());self.assertEqual(result['phase'],'PLAN_BLOCKED');self.assertEqual(result['error_code'],'NO_CHECKED_ROUTE');self.assertIn('SEARCH_EXHAUSTED',result['reason']);self.assertFalse(result['active']);self.assertEqual(result['motion_commands_sent'],0)
if __name__=='__main__':unittest.main()

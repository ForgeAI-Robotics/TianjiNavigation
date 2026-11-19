import json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import brain_cancel_client as client
class Tests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);(self.root/'config').mkdir();token=self.root/'config/brain_cancel_token';token.write_text('fixture-cancel-token-xxxxxxxxxxxx');token.chmod(0o600);self.patch=patch.object(client,'ROOT',self.root);self.patch.start()
 def tearDown(self):self.patch.stop();self.temp.cleanup()
 def test_same_task_id_and_unconfirmed_stop_are_preserved(self):
  state=dict(project='tianji',task_id='original-id');result=dict(project='tianji',accepted=True,status=dict(state,stop_confirmation='UNCONFIRMED'))
  with patch.object(client,'request',side_effect=[state,result])as request:
   got=client.cancel('original-id');self.assertEqual(got['status']['stop_confirmation'],'UNCONFIRMED');args=request.call_args_list[1].args;self.assertEqual(args[0],'/api/stop');self.assertEqual(args[2],dict(project='tianji',task_id='original-id'))
 def test_changed_task_is_not_cancelled(self):
  with patch.object(client,'request',return_value=dict(project='tianji',task_id='new-id'))as request:
   with self.assertRaises(ValueError):client.cancel('old-id')
   self.assertEqual(request.call_count,1)
 def test_no_task_does_not_post_stop(self):
  with patch.object(client,'request',return_value=dict(project='tianji',task_id=None))as request:self.assertFalse(client.cancel()['accepted']);self.assertEqual(request.call_count,1)
 def test_no_navigation_submission_route(self):
  with self.assertRaises(ValueError):client.request('/api/navigation','fixture')
if __name__=='__main__':unittest.main()

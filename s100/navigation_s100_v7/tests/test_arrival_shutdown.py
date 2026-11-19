import sys,unittest,subprocess,tempfile,time,types,io,json,threading
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'modules'))
from client import LocalRemote
from follower import PreviewFollower
import onboard_navigation as nav

class ArrivalShutdownTests(unittest.TestCase):
 def test_full_sensor_pipe_does_not_prevent_shutdown(self):
  # A relay blocked in write must see EOF/BrokenPipe when its consumer exits.
  process=subprocess.Popen([sys.executable,'-c',"import os,signal;signal.signal(signal.SIGTERM,lambda *_:None)\ntry:\n while True:os.write(1,b'x'*65536)\nexcept BrokenPipeError:pass"],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
  remote=LocalRemote.__new__(LocalRemote);remote.process=process;remote.closed=False;remote.log=io.BytesIO();time.sleep(.05)
  try:
   started=time.monotonic();remote.close();self.assertLess(time.monotonic()-started,2.);self.assertIsNotNone(process.poll());remote.close()
  finally:
   if process.poll()is None:process.kill();process.wait()
 def test_unresponsive_sensor_gets_final_kill_fallback(self):
  remote=LocalRemote.__new__(LocalRemote);remote.closed=False;remote.log=io.BytesIO();remote.process=Mock();remote.process.wait.side_effect=[subprocess.TimeoutExpired('relay',4),subprocess.TimeoutExpired('relay',3),0]
  remote.close();remote.process.terminate.assert_called_once();remote.process.kill.assert_called_once();self.assertTrue(remote.log.closed)
 def test_recorded_final_pose_satisfies_original_tolerances(self):
  goal=[.9026185274,.9271614552,-.009753230143412346];pose=[.8294652087156305,.9257852169099537,.045641392407581]
  plan=dict(selected=dict(path=[[0,0,0],goal],geometry_recheck_passed=True),target=dict(pose=goal,revision=2),start_map_pose=[0,0,0]);f=PreviewFollower(plan,dict(tracking_lookahead_m=.15));r=f.step(pose,100.,100.,2,True,True)
  self.assertEqual(r['state'],'ARRIVED');self.assertAlmostEqual(r['goal_error']['distance_m']*100,7.316626317);self.assertLess(r['goal_error']['heading_error_deg'],5.);self.assertEqual(r['twist_preview'],[0.,0.,0.])
if __name__=='__main__':unittest.main()

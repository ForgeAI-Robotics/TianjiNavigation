import sys,unittest,threading,time,types,itertools,tempfile,json
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import paths
from prepared_frames import prepare,PreparedFrames
from execution_safety import REQUIRED_JOINTS,sweep_guard

class PreparedTests(unittest.TestCase):
 def setUp(self):
  self.model=types.SimpleNamespace(sha256='c'*64);lo=[-.03,-.03,.1];hi=[.03,.03,.9]
  self.shape=dict(link='pole',mode='measured_or_fixed',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))))
  self.h=dict(kind='frame',sensor='front',sequence=1,pose=[0.,0.,0.],observation_time=10.,source_age_s=0.,joints=dict(source='recorded_measured_feedback',model_sha256=self.model.sha256,observation_time=10.,joint_positions_rad=dict.fromkeys(REQUIRED_JOINTS,0.)))
  self.p=np.array([[.07,0,.5],[.5,0,.5]]);self.T={s:np.eye(4) for s in ['front','back']}
 def identity(self,p,*a,**kw):return dict(filtered=np.zeros(len(p),bool),protected=np.zeros(len(p),bool),cable_report={})
 def test_scan_filtering_uses_acquisition_body_frame_and_preserves_age(self):
  self.h['pose']=[10.,20.,.4]
  with patch('prepared_frames.measured_shapes',return_value=[self.shape]),patch('prepared_frames.evaluate',side_effect=self.identity):data=prepare(self.model,self.T,(self.h,self.p,100.))
  self.assertTrue(np.allclose(data['points_body'],self.p));self.assertEqual(data['frame'][2],100.);self.assertEqual(data['frame'][0]['observation_time'],10.)
 def test_missing_or_stale_measured_scan_posture_is_rejected(self):
  self.h['joints']['observation_time']=9.
  with self.assertRaisesRegex(ValueError,'STALE'):prepare(self.model,self.T,(self.h,self.p,100.))
  self.h['joints']['observation_time']=10.;self.h['joints']['joint_positions_rad'].pop('head_pitch_joint')
  with self.assertRaisesRegex(ValueError,'FULL_SCAN'):prepare(self.model,self.T,(self.h,self.p,100.))
 def test_guard_dewarps_prepared_external_points_without_repeating_filter(self):
  prepared={}
  with patch('prepared_frames.measured_shapes',return_value=[self.shape]),patch('prepared_frames.evaluate',side_effect=self.identity):
   for i,sensor in enumerate(['front','back']):
    h=dict(self.h,sensor=sensor,sequence=i);prepared[sensor]=prepare(self.model,self.T,(h,self.p,100.))
  frames={s:d['frame'] for s,d in prepared.items()};debug={}
  with patch('execution_safety.evaluate_self_filter') as filter_again:
   result=sweep_guard(frames,self.T,[.02,0,0],[self.shape],self.model.sha256,None,[0.,0.,0.],100.,prepared=prepared,diagnostics=debug)
  filter_again.assert_not_called();self.assertFalse(result[0]);self.assertAlmostEqual(debug['points_body'][0,0],.05)
  stale=sweep_guard(frames,self.T,[.02,0,0],[self.shape],self.model.sha256,None,[0.,0.,0.],101.3,prepared=prepared)
  self.assertFalse(stale[0]);self.assertEqual(stale[1],'LIDAR_STALE')
 def test_pending_process_does_not_block_or_refresh_navigation_snapshot(self):
  class Child:
   pid=123
   stopped=False
   def poll(self):return 0 if self.stopped else None
   def terminate(self):self.stopped=True
   def wait(self,timeout=None):return 0
  with tempfile.TemporaryDirectory() as folder,patch('prepared_frames.subprocess.Popen',side_effect=lambda *a,**kw:Child()):
   worker=PreparedFrames(self.model,folder)
   try:
    self.assertEqual(worker.snapshot(),{});self.assertIsNone(worker.error)
    metadata=dict(header=self.h,arrival_monotonic=100.,report={},model_sha256=self.model.sha256)
    np.savez(Path(folder)/'prepared_front.npz',metadata=np.array(json.dumps(metadata)),points_body=self.p,points_sensor=self.p)
    first=worker.snapshot();second=worker.snapshot()
    self.assertEqual(first['front']['frame'][2],100.);self.assertEqual(second['front']['frame'][2],100.)
    self.assertEqual(second['front']['frame'][0]['observation_time'],10.)
   finally:worker.close()

if __name__=='__main__':unittest.main()

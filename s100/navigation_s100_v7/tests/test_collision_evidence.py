import sys,unittest,tempfile,json,itertools
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import paths
from execution_safety import sweep_guard
from collision_evidence import save

class EvidenceTests(unittest.TestCase):
 def test_evidence_preserves_guard_decision_and_original_sensor_coordinates(self):
  lo=[-.03,-.03,.1];hi=[.03,.03,.9]
  shape=dict(link='pole',mode='measured_or_fixed',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))))
  p=np.array([[.07,0,.5],[.5,0,.5]]);frames={s:(dict(sensor=s,pose=[0.,0.,0.],sequence=i,observation_time=123.,source_age_s=0.),p,100.) for i,s in enumerate(['front','back'])}
  def identity(points,*a,**kw):return dict(filtered=np.zeros(len(points),bool),protected=np.zeros(len(points),bool),cable_report={})
  D={};T={s:np.eye(4) for s in frames}
  with patch('execution_safety.evaluate_self_filter',side_effect=identity):
   original=sweep_guard(frames,T,[0.,0.,0.],[shape],'a'*64,None,[.005,0.,0.],100.)
   traced=sweep_guard(frames,T,[0.,0.,0.],[shape],'a'*64,None,[.005,0.,0.],100.,diagnostics=D)
  self.assertEqual(original,traced);self.assertFalse(traced[0]);self.assertTrue(np.allclose(D['points_body'],[[.07,0,.5],[.07,0,.5]]))
  with tempfile.TemporaryDirectory() as folder:
   file=save(folder,D,frames,dict(joint_positions_rad={}), 'a'*64,'unit')
   metadata=json.loads(Path(file).read_text());self.assertEqual(metadata['session'],'unit');self.assertEqual(metadata['examples'][0]['nearest_link'],'pole');self.assertAlmostEqual(metadata['examples'][0]['gap_cm'],4.)
   self.assertFalse(metadata['examples'][0]['inside_cleanup_height']);self.assertTrue(metadata['examples'][0]['inside_cleanup_band']);self.assertEqual(metadata['frames']['front']['observation_time'],123.)
   with np.load(metadata['raw_file'],allow_pickle=False) as raw:self.assertTrue(np.array_equal(raw['front_raw_sensor'],p))

if __name__=='__main__':unittest.main()

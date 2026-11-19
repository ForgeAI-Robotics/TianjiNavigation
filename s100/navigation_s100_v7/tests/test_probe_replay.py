import unittest,tempfile,json,sys,itertools
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import paths
from replay_probe_evidence import replay

class ProbeReplayTests(unittest.TestCase):
 def test_judgement_is_forbidden_before_measured_five_cm(self):
  with self.assertRaisesRegex(ValueError,'MEASURED_5CM_REQUIRED'):replay('/unused',{},'c'*64,['boot',0,'calibration'],.049)
 def test_full_raw_replay_identifies_body_wire_and_fixed_environment(self):
  lo=[-.03,-.03,.1];hi=[.03,.03,.92];shape=dict(link='assembly_lower_mount',name='pole',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))),mode='measured_or_fixed');sha='c'*64
  with tempfile.TemporaryDirectory() as directory:
   folder=Path(directory);rows=[]
   for i in range(25):
    x=min(i*.005,.052)
    for sensor in ['front','back']:
     file=folder/f'{i}_{sensor}.npz';p=np.array([[.079,0,.72],[.15-x,.15,.72]]);np.savez(file,points_sensor=p)
     rows.append(dict(header=dict(sensor=sensor,boot_id='boot',epoch=0,calibration_sha256='calibration',pose=[x,0,0],observation_time=100.+i),file=str(file)))
   (folder/'acquisition.json').write_text(json.dumps(dict(rows=rows)))
   with patch('replay_probe_evidence.local_model',return_value=SimpleNamespace(sha256=sha)),patch('replay_probe_evidence.measured_shapes',return_value=[shape]),patch('replay_probe_evidence.transforms',return_value={s:np.eye(4) for s in ['front','back']}),patch('cable_zones.settings',return_value=dict(enabled=True,model_sha256=sha)):
    report=replay(folder,{},sha,['boot',0,'calibration'],.052)
   states={f['status'] for f in report['features']};self.assertIn('SELF_CORRELATED',states);self.assertIn('ENVIRONMENT_FIXED',states);self.assertTrue(report['posthoc_raw_replay']);self.assertFalse(report['automatic_point_deletion'])

if __name__=='__main__':unittest.main()

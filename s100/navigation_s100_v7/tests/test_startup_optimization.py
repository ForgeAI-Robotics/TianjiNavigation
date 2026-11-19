import json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from execution_safety import position_check
from gicp_batch import run
class StartupTests(unittest.TestCase):
 def test_initialization_does_not_admit_and_stale_stays_stale(self):
  p=dict(running=True,status='NEED_LIDAR_FIX',map_pose_valid=False,T_map_base=None,updated_wall_time=10.,odometry_age_s=.1)
  self.assertEqual(position_check(p,{'localization':{}},10.1)[1],'LOCALIZATION_INITIALIZING')
  self.assertFalse(position_check(p,{'localization':{}},10.1)[0])
  self.assertEqual(position_check(p,{'localization':{}},11)[1],'LOCALIZATION_STALE')
  p.update(status='NO_ODOMETRY',odometry_age_s=None)
  self.assertEqual(position_check(p,{'localization':{}},10.1)[1],'LOCALIZATION_INITIALIZING')
 def test_bootstrap_requires_both_current_sensors_and_all_quality_gates(self):
  from localization_tracker import bootstrap_quality
  groups={s:np.zeros((300,3)) for s in ['front','back']};good=dict(overlap=.9,rmse=.04,observable=True)
  with patch('localization_tracker.residuals',return_value=good):
   self.assertIsNotNone(bootstrap_quality(groups,np.eye(4),None))
   self.assertIsNone(bootstrap_quality({'front':groups['front']},np.eye(4),None))
  for bad in [dict(good,overlap=.74),dict(good,rmse=.066),dict(good,observable=False)]:
   with patch('localization_tracker.residuals',side_effect=[good,bad]):self.assertIsNone(bootstrap_quality(groups,np.eye(4),None))
 def test_parallel_candidates_preserve_all_seed_indices(self):
  with tempfile.TemporaryDirectory() as tmp:
   r=Path(tmp);seeds=r/'seeds';seeds.write_text(''.join(f'{i} 0 0 0\n'for i in range(12)))
   def fake(args,**kw):
    lines=Path(args[3]).read_text().splitlines();rows=[]
    for i,line in enumerate(lines):
     T=np.eye(4);T[0,3]=float(line.split()[0]);rows.append([i,1,0,.01,*T.ravel()])
    np.savetxt(args[4],rows)
   with patch('gicp_batch.subprocess.run',side_effect=fake) as proc:
    rows=run('binary','source','target',seeds,r/'fits',.45)
   self.assertEqual(proc.call_count,2);np.testing.assert_array_equal(rows[:,0],np.arange(12));np.testing.assert_array_equal(rows[:,7],np.arange(12))
 def test_corrupt_worker_indices_are_rejected(self):
  with tempfile.TemporaryDirectory() as tmp:
   r=Path(tmp);s=r/'seeds';s.write_text('0 0 0 0\n')
   def fake(args,**kw):np.savetxt(args[4],[[5,1,0,.01,*np.eye(4).ravel()]])
   with patch('gicp_batch.subprocess.run',side_effect=fake),self.assertRaisesRegex(ValueError,'SEED_INDEX'):run('binary','source','target',s,r/'fits',.45)
if __name__=='__main__':unittest.main()

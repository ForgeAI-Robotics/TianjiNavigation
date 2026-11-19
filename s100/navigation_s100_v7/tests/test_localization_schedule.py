import sys,unittest,threading,time,tempfile,subprocess
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'modules'))
from localization_schedule import RegistrationSlot,registration_timeout,prediction_evidence_current
from localization_tracker import bootstrap_quality,register_groups
from pose_memory import PoseMemory

class LocalizationScheduleTests(unittest.TestCase):
 def header(self,stamp=100.):return dict(boot_id='boot',epoch=0,calibration_sha256='cal',observation_time=stamp,odom_observation_time=stamp,pose=[0,0,0],source_age_s=.2)
 def test_slow_full_registration_does_not_block_new_scan_validations(self):
  entered=threading.Event();release=threading.Event()
  def slow(job):entered.set();release.wait(2);return job
  slot=RegistrationSlot(slow)
  try:
   self.assertTrue(slot.submit(dict(sequence=1)));self.assertTrue(entered.wait(1));m=PoseMemory();h=self.header();m.ingest(h,10.);self.assertTrue(m.fix(np.eye(4),h,10.,{}));groups={s:np.zeros((300,3))for s in ['front','back']};good=dict(overlap=.9,rmse=.04,observable=True)
   with patch('localization_tracker.residuals',return_value=good):
    for dt in [.3,.6,.9,1.2,1.5,1.8,2.1,2.4]:
     self.assertIsNone(slot.poll());self.assertFalse(slot.submit(dict(sequence=2)))
     new=self.header(100.+dt);m.ingest(new,10.+dt);metrics=bootstrap_quality(groups,np.eye(4),None);self.assertIsNotNone(metrics);self.assertTrue(m.fix(np.eye(4),new,10.+dt,metrics));self.assertLess(m.snapshot(10.+dt)['lidar_fix_age_s'],.01)
   release.set()
  finally:release.set();slot.close()
 def test_one_job_and_one_unconsumed_result_no_queue(self):
  slot=RegistrationSlot(lambda job:job*2)
  try:
   self.assertTrue(slot.submit(3))
   for _ in range(100):
    if slot.future.done():break
    time.sleep(.001)
   self.assertTrue(slot.busy);self.assertFalse(slot.submit(4));result=slot.poll();self.assertEqual(result['job'],3);self.assertEqual(result['result'],6);self.assertFalse(slot.busy)
  finally:slot.close()
  self.assertFalse(slot.submit(5))
 def test_prediction_checks_cannot_extend_original_four_second_authority(self):
  self.assertTrue(prediction_evidence_current(10.,13.999));self.assertFalse(prediction_evidence_current(10.,14.));self.assertFalse(prediction_evidence_current(None,10));self.assertFalse(prediction_evidence_current(10.,9.))
 def test_timeout_accounts_for_transport_queue_and_odom_pair_age(self):
  h=self.header();h['odom_observation_time']=99.97
  self.assertAlmostEqual(registration_timeout(h,10.,10.1),2.62)
  for header,now in [(h,13.),(dict(h,source_age_s=float('nan')),10.),(dict(h,source_age_s=-1.),10.)]:
   with self.assertRaises(ValueError):registration_timeout(header,10.,now)
 def test_late_result_and_changed_epoch_still_rejected(self):
  m=PoseMemory();old=self.header();m.ingest(old,10.);new=self.header(103.1);m.ingest(new,13.1);self.assertFalse(m.fix(np.eye(4),old,13.1,{}))
  changed=dict(new,epoch=1);m.ingest(changed,13.2);self.assertFalse(m.fix(np.eye(4),old,13.2,{}));self.assertIsNone(m.anchor)
 def test_background_exception_is_reported_not_applied(self):
  def fail(job):raise ValueError('BAD_REGISTRATION')
  slot=RegistrationSlot(fail)
  try:
   slot.submit(1)
   for _ in range(100):
    result=slot.poll()
    if result is not None:break
    time.sleep(.001)
   self.assertEqual(result['error'],'BAD_REGISTRATION');self.assertIsNone(result['result'])
  finally:slot.close()
 def test_registration_keeps_heldout_quality_gates(self):
  h=self.header();groups={s:np.zeros((300,3))for s in ['front','back']};good=dict(overlap=.9,rmse=.04,observable=True)
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp);job=dict(groups=groups,guess=np.eye(4),header=h,received_at=10.,generation=0)
   def native(*args,**kwargs):np.savetxt(p/'fit.txt',[[0,1,.01,.2,*np.eye(4).ravel()]])
   with patch('localization_tracker.time.monotonic',return_value=10.1),patch('localization_tracker.subprocess.run',side_effect=native)as run,patch('localization_tracker.residuals',side_effect=[good,dict(good,overlap=.69)]):
    result=register_groups(job,p,None);self.assertFalse(result['good']);self.assertLess(run.call_args.kwargs['timeout'],3.)
if __name__=='__main__':unittest.main()

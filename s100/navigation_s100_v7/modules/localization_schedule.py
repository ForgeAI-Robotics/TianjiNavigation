"""Single background registration slot; no backlog, ROS or motion interfaces."""
import math,time
from concurrent.futures import ThreadPoolExecutor

REGISTRATION_PERIOD_S=3.
PREDICTION_VALIDATION_WINDOW_S=4.
FAST_VALIDATION_PERIOD_S=.25

def prediction_evidence_current(last_registration,now):
 return last_registration is not None and 0<=now-last_registration<PREDICTION_VALIDATION_WINDOW_S

def registration_timeout(header,received_at,now):
 """Finish within the original 3s historical-odometry admission window."""
 age=now-received_at+float(header.get('source_age_s',0.))
 offset=max(0.,float(header['observation_time'])-float(header['odom_observation_time']))
 if not math.isfinite(age+offset) or age<0:raise ValueError('INVALID_REGISTRATION_SOURCE_AGE')
 remaining=3.-age-offset-.05
 if remaining<=.1:raise ValueError('REGISTRATION_SOURCE_TOO_OLD')
 return min(3.,remaining)

class RegistrationSlot:
 def __init__(self,compute):
  self.compute=compute;self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='map_registration');self.future=None;self.job=None;self.closed=False
 @property
 def busy(self):return self.future is not None
 def submit(self,job):
  # Never enqueue work behind a slow registration. The caller retains latest
  # raw frames and will select a genuinely new pair once this slot is free.
  if self.closed or self.busy:return False
  self.job=job;self.future=self.executor.submit(self.compute,job);return True
 def poll(self):
  if self.future is None or not self.future.done():return None
  job=self.job
  try:result=self.future.result();error=None
  except Exception as exc:result=None;error=str(exc)
  self.future=None;self.job=None
  return dict(job=job,result=result,error=error)
 def close(self):
  self.closed=True;self.executor.shutdown(wait=True,cancel_futures=True);self.future=None;self.job=None

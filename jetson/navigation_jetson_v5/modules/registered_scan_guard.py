"""Recoverable registered-frame age guard; clock/pose faults remain caller-fatal."""
import math
class RegisteredScanGuard:
 def __init__(self,timeout_s=2.):
  self.timeout=float(timeout_s);self.started=None;self.last_good=None;self.accepted=0;self.dropped=0;self.consecutive=0;self.recoveries=0;self.paused=False;self.last_delta=None;self.last_reason=None;self.max_queue_age=0.
 def check(self,latest_source,stamp,now,queue_age=0.):
  if self.started is None:self.started=now
  self.max_queue_age=max(self.max_queue_age,float(queue_age));self.last_delta=None if latest_source is None else float(latest_source-stamp)
  good=self.last_delta is not None and math.isfinite(self.last_delta) and -.25<=self.last_delta<=.5
  if not good:
   self.dropped+=1;self.consecutive+=1;self.last_reason='REGISTERED_SCAN_NOT_CURRENT_SOURCE';self.tick(now);return False
  return True
 def commit(self,now):
  if self.paused:self.recoveries+=1
  self.accepted+=1;self.last_good=now;self.consecutive=0;self.paused=False;self.last_reason=None
 def tick(self,now):
  reference=self.last_good if self.last_good is not None else self.started
  if reference is not None and now-reference>=self.timeout:self.paused=True
 def summary(self):
  return dict(paused=self.paused,accepted=self.accepted,dropped=self.dropped,consecutive_drops=self.consecutive,recoveries=self.recoveries,last_source_minus_registered_s=self.last_delta,last_reason=self.last_reason,max_queue_age_s=self.max_queue_age,pause_timeout_s=self.timeout)

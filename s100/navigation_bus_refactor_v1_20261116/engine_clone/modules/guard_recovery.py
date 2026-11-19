"""Bounded zero-output waits for transient data loss; never waive admission."""
import math
from navigation_admission import REQUIRED,LIMITS

def expired_deadlines(guard):
 if guard is None:return []
 return [k for k,limit in zip(REQUIRED,LIMITS)
         if not isinstance(guard.get(k),(float,int)) or not math.isfinite(guard[k]) or not 0<=guard[k]<=limit]

WAITABLE_DATA_REASONS=frozenset(('LOCALIZATION_STALE','LOCALIZATION_UNAVAILABLE','JOINT_FEEDBACK_STALE','FULL_HEAD_ARM_GRIPPER_FEEDBACK_REQUIRED','FRESH_S100_RECEIVED_JOINT_FEEDBACK_REQUIRED','LIDAR_STALE','WAIT_RAW_OBSERVER'))

def waitable_data_fault(reason,guard):
 if reason in WAITABLE_DATA_REASONS:return True
 if reason=='ADMISSION_DEADLINE_EXPIRED':
  expired=expired_deadlines(guard)
  return bool(expired) and set(expired)<=set(REQUIRED)-{'operator_age_s'}
 return False

class GuardRecovery:
 def __init__(self,maximum_wait_s=3.,total_wait_s=6.,healthy_reset_s=3.):
  self.maximum=maximum_wait_s;self.budget=total_wait_s;self.started=None;self.spent=0.;self.healthy_since=None;self.healthy_reset=healthy_reset_s
 def waiting(self,reason,guard,now):
  # Configuration, identity, geometry and operator failures are never retried.
  allowed=waitable_data_fault(reason,guard)
  if not allowed:return False
  if self.started is None:
   if self.healthy_since is not None and now-self.healthy_since>=self.healthy_reset:self.spent=0.
   self.healthy_since=None;self.started=now
  return self.within_budget(now)
 def within_budget(self,now):
  if self.started is None:return True
  age=now-self.started
  return 0<=age<=self.maximum and self.spent+age<=self.budget
 def resumed(self,now):
  if self.started is not None:self.spent+=max(0.,now-self.started)
  self.started=None
  if self.healthy_since is None:self.healthy_since=now

"""Measured operator lease with explicit, latched stop causes."""
class OperatorSession:
 def __init__(self,session,stop,began):
  self.session=session;self.stop=stop;self.began=began;self.last=None;self.reason=None;self.failed_at=None;self.heartbeats=0;self.maximum_gap_s=0.
 def fail(self,reason,now):
  if self.reason is None:self.reason=reason;self.failed_at=now
  self.stop.set()
 def receive(self,message,now):
  if message.get('session')!=self.session:return
  if message.get('kind')=='stop':self.fail('OPERATOR_STOP_COMMAND',now)
  elif message.get('kind')=='heartbeat':
   if self.last is not None:self.maximum_gap_s=max(self.maximum_gap_s,now-self.last)
   self.last=now;self.heartbeats+=1
 def age(self,now):return 999. if self.last is None else max(0.,now-self.last)
 def check(self,now):
  if self.last is None:
   if now-self.began>2.:self.fail('OPERATOR_INITIAL_HEARTBEAT_TIMEOUT',now)
  elif now-self.last>.5:self.fail('OPERATOR_HEARTBEAT_TIMEOUT',now)
  return self.reason
 def require(self,now):
  self.check(now)
  if self.stop.is_set():raise ValueError(self.reason or 'EXECUTION_STOP_REQUESTED')
 def summary(self,now):
  return dict(stop_reason=self.reason,stop_monotonic=self.failed_at,heartbeat_age_s=self.age(now),heartbeats=self.heartbeats,maximum_received_heartbeat_gap_s=self.maximum_gap_s,heartbeat_timeout_s=.5)

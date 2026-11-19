"""Zero-output obstacle hold; only freshly checked scenes may release it.

This is not data-fault recovery. Sensor/operator/model failures keep their
existing stop policies. No obstacle is classified as human or ignored here.
"""
import math

BLOCKING_REASONS=frozenset(('LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY','CLEARANCE_OR_SHAPEMASK_NOT_CLEAR'))

class ObstacleWait:
 def __init__(self,clear_duration_s=.8,minimum_pairs=3):
  if not math.isfinite(clear_duration_s) or clear_duration_s<=0 or minimum_pairs<2:raise ValueError('INVALID_OBSTACLE_RELEASE_POLICY')
  self.blocking_reasons=BLOCKING_REASONS;self.clear_duration=clear_duration_s;self.minimum_pairs=minimum_pairs;self.active=False;self.started=None;self.reason=None;self.episodes=0;self.interrupt_clear()
 def interrupt_clear(self):
  self.clear_since=None;self.clear_pairs=0;self.last_pair=None
 def blocked(self,reason,now,frames=None):
  if reason not in self.blocking_reasons:raise ValueError('NOT_A_WAITABLE_OBSTACLE')
  if not self.active:self.started=now;self.episodes+=1
  self.active=True;self.reason=reason;self.interrupt_clear()
  if frames is not None and set(frames)=={'front','back'}:self.last_pair=tuple(frames[s][0]['sequence'] for s in ('front','back'))
 def checked_clear(self,frames,now):
  """Caller must already pass full geometry AND every output admission check."""
  if not self.active:return True
  if set(frames)!= {'front','back'}:self.interrupt_clear();return False
  pair=tuple(frames[s][0]['sequence'] for s in ('front','back'))
  # Both sensors must advance; repeated inspection of one saved pair is
  # never evidence of a continuously clear environment.
  advanced=self.last_pair is None or all(a!=b for a,b in zip(pair,self.last_pair))
  if advanced:
   self.last_pair=pair;self.clear_pairs+=1
   if self.clear_since is None:self.clear_since=now
  if advanced and self.clear_since is not None and now-self.clear_since>=self.clear_duration and self.clear_pairs>=self.minimum_pairs:
   self.active=False;return True
  return False
 def summary(self,now):
  return dict(active=self.active,reason=self.reason,episodes=self.episodes,wait_s=max(0.,now-self.started) if self.started is not None else 0.,clear_s=max(0.,now-self.clear_since) if self.clear_since is not None else 0.,clear_pairs=self.clear_pairs,required_clear_s=self.clear_duration,required_pairs=self.minimum_pairs)

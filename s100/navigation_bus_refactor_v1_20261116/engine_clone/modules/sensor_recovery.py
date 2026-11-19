"""Retain a stopped navigation task after bounded transient retries expire.

Only classified data-age/feedback faults qualify. This changes task lifetime,
never sensor deadlines, identity checks or permission to output motion.
"""
from obstacle_wait import ObstacleWait
from guard_recovery import WAITABLE_DATA_REASONS,waitable_data_fault

class SensorRecovery(ObstacleWait):
 def __init__(self,*,clear_duration_s=.8,minimum_pairs=3):
  super().__init__(clear_duration_s=clear_duration_s,minimum_pairs=minimum_pairs)
  self.blocking_reasons=WAITABLE_DATA_REASONS|{'ADMISSION_DEADLINE_EXPIRED'}
 def blocked(self,reason,now,frames=None,guard=None):
  if not waitable_data_fault(reason,guard):raise ValueError('NOT_A_RECOVERABLE_DATA_FAULT')
  super().blocked(reason,now,frames)

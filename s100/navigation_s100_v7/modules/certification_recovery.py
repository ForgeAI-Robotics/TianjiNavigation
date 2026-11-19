"""Classify geometric revalidation waits without hiding real source faults."""
import math
from execution_safety import posture_check,position_check

POSE_RECHECK='POSE_MOVED_DURING_COLLISION_CHECK'

def classify(reason,saved,joints,model_hash,position,record,frames,now_wall,now_mono):
 if reason!=POSE_RECHECK:return False,reason
 ok,why,_=posture_check(saved,joints,model_hash,now_wall)
 if not ok:return False,why
 ok,why,_=position_check(position,record,now_wall)
 if not ok:return False,why
 if set(frames)!= {'front','back'}:return False,'BOTH_LIDARS_REQUIRED'
 loc=record['localization'];expected=(loc['source_boot_id'],loc['odometry_epoch'],loc['calibration_sha256'])
 for sensor,(h,_,arrival) in frames.items():
  if h.get('sensor')!=sensor or (h.get('boot_id'),h.get('epoch'),h.get('calibration_sha256'))!=expected:return False,'LIDAR_FRAME_CHANGED'
  age=now_mono-arrival+float(h.get('source_age_s',0.))
  if not math.isfinite(age) or not 0<=age<=1.2:return False,'LIDAR_STALE'
 return True,POSE_RECHECK

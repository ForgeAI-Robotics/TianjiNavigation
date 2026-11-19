"""Independent S100 control-pump checks on every output, including repeats."""
import math
REQUIRED=('localization_age_s','pose_age_s','clearance_age_s','guard_age_s','cloud_age_s','operator_age_s')
LIMITS=(.5,.35,.75,.35,1.2,.5)
def admission(data,elapsed):
 try:
  if data['state']!='CLEAR' or not data['path_checked'] or not data['operator_estop_declared']:return False
  if len(data['map_sha256'])!=64 or len(data['model_sha256'])!=64:return False
  if data.get('forced_release_active'):
   from clearance_override import settings,effective
   cfg=settings();r=effective(False,False,data['map_sha256'],data['model_sha256'],cfg)
   if not r['forced_release_active'] or not r['clearance_clear'] or not r['shapemask_clear'] or cfg.get('updated_wall_time')!=data.get('override_version'):return False
  elif data.get('assumed_clearance') or data.get('calibration_test_mode'):return False
  if 'swept_pose_lease_s' in data:
   lease=data['swept_pose_lease_s']
   if not math.isfinite(lease) or not 0<lease<=.35 or data['guard_age_s']+elapsed>lease:return False
  return all(math.isfinite(data[k]) and 0<=data[k]+elapsed<=limit for k,limit in zip(REQUIRED,LIMITS))
 except (KeyError,TypeError,ValueError):return False

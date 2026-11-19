"""S100 rechecks Jetson navigation lease deadlines on every output."""
import math
REQUIRED=('localization_age_s','pose_age_s','clearance_age_s','guard_age_s','cloud_age_s','operator_age_s')
LIMITS=(.5,.35,.75,.35,1.2,.5)
def admission(data,elapsed):
 try:
  if data['state']!='CLEAR' or data.get('assumed_clearance') or data.get('calibration_test_mode'):return False
  if len(data['map_sha256'])!=64 or len(data['model_sha256'])!=64 or not data['path_checked']:return False
  return all(math.isfinite(data[k]) and 0<=data[k]+elapsed<=limit for k,limit in zip(REQUIRED,LIMITS))
 except (KeyError,TypeError,ValueError):return False

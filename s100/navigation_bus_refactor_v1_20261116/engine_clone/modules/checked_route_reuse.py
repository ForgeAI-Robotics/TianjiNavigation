"""Small confirmed localization changes: recheck the fixed MAP route, no motion."""
import math,json,subprocess
import numpy as np
from paths import ROOT
from saved_map import base_transform
from execution_safety import position_check,posture_check

def candidate(recovery,packet,record,plan,follower,current,now):
 metrics=recovery.anchor_metrics or {}
 if recovery.reason!='LOCALIZATION_ANCHOR_CHANGED' or metrics.get('body_correction_m',999.)>.03 or metrics.get('heading_correction_deg',999.)>1.:return None
 if not position_check(packet,record,now)[0] or not posture_check(record['saved_pose'],current,record['model_sha256'],now)[0]:return None
 if not plan.get('selected',{}).get('geometry_recheck_passed'):return None
 T=np.asarray(packet['T_map_base']);pose=np.array([T[0,3],T[1,3],math.atan2(T[1,0],T[0,0])]);path=np.asarray(follower.path[follower.index:],float)
 if not len(path) or not np.isfinite(path).all() or np.min(np.linalg.norm(path[:,:2]-pose[:2],axis=1))>.06:return None
 inv=np.linalg.inv(base_transform(plan['start_map_pose']));result=[]
 for p in [pose,*path]:
  local=inv@base_transform(p);result.append([float(local[0,3]),float(local[1,3]),math.atan2(local[1,0],local[0,0])])
 return result

def checked(world_file,poses):
 try:
  run=subprocess.run([str(ROOT/'native/world_guard'),str(world_file)],input=json.dumps(dict(id=1,poses=poses))+'\n',capture_output=True,text=True,check=True,timeout=2.)
  result=json.loads(run.stdout);return result.get('id')==1 and result.get('clear')is True
 except (OSError,ValueError,subprocess.SubprocessError):return False

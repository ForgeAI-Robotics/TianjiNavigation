"""Joint/clearance data already received on S100; Jetson supplies measurements only."""
from pathlib import Path
import time,json
from functools import lru_cache
from snapshot_builder import local_model
from gripper_posture_policy import fixed as fixed_grippers,apply as apply_grippers
STATUS=Path('/home/sunrise/tianji_state_bridge/runtime/receiver_status.json')
FAST_STATUS=Path('/dev/shm/tianji_s100_joint_feedback.json')
_last_health={};_detail_time=0.;_detail={}
def feedback_health():
 global _detail_time,_detail
 packet=dict(_last_health);now=time.time()
 if packet.get('reasons') and now-_detail_time>=.5:
  _detail_time=now
  try:
   src=state().get('source_status',{})
   _detail={k:src.get(k) for k in ['publisher_timing_revision','received_at','arm_valid','arm_reason','arm_source','arm_feedback_age_s','arm_frame_serials','head_valid','head_age_s','grippers_valid','control_lease_active','sdk_owned']}
  except (OSError,ValueError):_detail={}
 packet['source_detail']=dict(_detail) if packet.get('reasons') else {}
 return packet
@lru_cache(maxsize=1)
def model():return local_model()
def state():return json.loads(STATUS.read_text())
def measured_joints():
 global _last_health,_detail_time,_detail
 m=model()
 fixed,fixed_evidence=fixed_grippers(m.sha256)
 # A stale existing fast stream is a fault, never a reason to fall back to
 # a slower diagnostics record which may still look green.
 d=json.loads(FAST_STATUS.read_text()) if FAST_STATUS.exists() else state();now=time.time()
 reasons=[];source=d.get('source_status',{})
 if not 0<=now-d['received_at']<=.5:reasons.append('RECEIVER_SNAPSHOT_EXPIRED')
 if not d.get('arm_fresh'):reasons.append('ARMS_NOT_FRESH')
 if not d.get('model_hash_match') or d.get('whole_model_sha256')!=m.sha256:reasons.append('MODEL_MISMATCH')
 if not source.get('head_valid'):reasons.append('HEAD_NOT_FRESH')
 for side in ('left','right'):
  if side not in source.get('grippers_valid',[]) and side+'_robotiq_85_left_knuckle_joint' not in fixed:reasons.append(side.upper()+'_GRIPPER_NOT_FRESH')
 _last_health=dict(checked_wall_time=now,reasons=reasons,receiver_age_s=now-d['received_at'],posture_age_s=now-d.get('posture_observation_time',0.),fixed_gripper_policy=fixed_evidence)
 if any(k in reasons for k in ['RECEIVER_SNAPSHOT_EXPIRED','ARMS_NOT_FRESH','MODEL_MISMATCH']):raise ValueError('FRESH_S100_RECEIVED_JOINT_FEEDBACK_REQUIRED')
 q=dict(d['joint_positions_rad']);source=d.get('source_status',{})
 # A historical head angle is not fresh feedback. Preserve the unknown bound.
 if not source.get('head_valid'):q.pop('head_pitch_joint',None)
 for side in ['left','right']:
  if side not in source.get('grippers_valid',[]):q.pop(side+'_robotiq_85_left_knuckle_joint',None)
 return apply_grippers(dict(joint_positions_rad=q,model_sha256=m.sha256,observation_time=d.get('posture_observation_time',d['last_arm_receive_wall_time']),live=False,source=d.get('source','S100_LOCAL_RECEIVER_MEASURED_FEEDBACK')),fixed,fixed_evidence)
def readiness():
 from navigation_readiness import received_readiness
 from targets import get
 return received_readiness(state(),get(1)['map_voxel_sha256'])

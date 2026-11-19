"""Measured posture history for scan-time filtering; never interpolates angles."""
import collections,math
from gripper_posture_policy import fixed as fixed_grippers,apply as apply_grippers,NAMES

class JointScanHistory:
 def __init__(self):self.rows=collections.deque(maxlen=100);self.last_received=None;self.rejections=collections.Counter();self.last_rejection=None;self.accepted=0
 def reject(self,reason):
  self.last_rejection=reason;self.rejections[reason]+=1
 def diagnostics(self):
  return dict(accepted=self.accepted,last_rejection=self.last_rejection,rejections=dict(self.rejections),history_samples=len(self.rows),latest_observation_time=self.rows[-1]['observation_time'] if self.rows else None)
 def sample(self,data,now):
  try:
   received=float(data['received_at']);stamp=float(data.get('posture_observation_time',data.get('last_arm_receive_wall_time',0.)));source=data['source_status']
   fixed,evidence=fixed_grippers(data.get('whole_model_sha256'));reasons=[]
   if not data['arm_fresh']:reasons.append('ARMS_NOT_FRESH')
   if not data['model_hash_match']:reasons.append('MODEL_MISMATCH')
   if not source.get('head_valid'):reasons.append('HEAD_NOT_FRESH')
   for side in ('left','right'):
    if side not in source.get('grippers_valid',[])and NAMES[side]not in fixed:reasons.append(side.upper()+'_GRIPPER_NOT_FRESH')
   if reasons:self.reject('+'.join(reasons));return
   if received==self.last_received:return
   if not math.isfinite(stamp) or not 0<=now-received<=.5:self.reject('RECEIVER_SNAPSHOT_EXPIRED_OR_INVALID');return
   if not -.10<=now-stamp<=.35:self.reject('POSTURE_TIMESTAMP_EXPIRED_OR_FUTURE');return
   self.rows.append(apply_grippers(dict(joint_positions_rad=dict(data['joint_positions_rad']),observation_time=stamp,model_sha256=data['whole_model_sha256'],live=False,source='recorded_measured_feedback'),fixed,evidence));self.last_received=received;self.accepted+=1;self.last_rejection=None
  except (KeyError,ValueError,TypeError):self.reject('MALFORMED_JOINT_SNAPSHOT');return
 def nearest(self,stamp):
  if not self.rows:self.reject('NO_VALID_JOINT_HISTORY');return {}
  row=min(reversed(self.rows),key=lambda r:abs(r['observation_time']-stamp))
  if abs(row['observation_time']-stamp)>=.3:self.reject('SCAN_POSTURE_SKEW_EXCEEDED');return {}
  return dict(row)

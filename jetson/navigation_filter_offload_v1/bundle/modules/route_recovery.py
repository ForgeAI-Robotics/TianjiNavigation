"""Stop/settle/replan state. Never authorizes output or modifies source ages."""
import math
import numpy as np
from saved_map import base_transform
from pose_memory import yaw,angle_delta

def anchor(localization):
 return np.asarray(localization['T_map_base'])@np.linalg.inv(base_transform(localization['wheel_pose']))

class RouteRecovery:
 def __init__(self,localization):
  self.reference=anchor(localization);self.reason=None;self.next_attempt=0.;self.attempts=0;self.replans=0;self.route_reuses=0;self.last_error=None;self.still_since=None;self.still_pose=None;self.last_stamp=None;self.samples=0;self.anchor_check=None;self.anchor_metrics=None
 @property
 def pending(self):return self.reason is not None
 def request(self,reason):
  if not self.pending:self.reason=reason;self.still_since=None;self.still_pose=None;self.samples=0;self.last_stamp=None
 @property
 def checking_anchor(self):return self.anchor_check is not None and not self.pending
 def observe(self,packet):
  if self.pending:return
  if packet.get('status')=='LOCALIZATION_RECHECK_REQUIRED':
   # Stop output while registration is invalid, without committing a replan
   # before the independent localization confirmation has completed.
   self.anchor_check=dict(state='WAITING_FOR_VALID_LOCALIZATION',candidate=None,first_stamp=None,last_stamp=None,samples=0);return
  T=packet.get('T_map_odom')
  if not(packet.get('running')and packet.get('map_pose_valid')and T is not None):return
  T=np.asarray(T);odom=packet.get('odometry_pose')
  # Evaluate map correction at the current body origin. Translation of a
  # map->odom transform alone includes rotation about the odometry origin.
  O=base_transform(odom) if odom is not None else np.eye(4)
  current=T@O;expected=self.reference@O
  shift=float(np.linalg.norm(current[:2,3]-expected[:2,3]));turn=abs(angle_delta(yaw(current),yaw(expected)))
  self.anchor_metrics=dict(body_correction_m=shift,heading_correction_deg=math.degrees(turn))
  if shift>.04 or turn>math.radians(2.):
   self.request('LOCALIZATION_ANCHOR_CHANGED');return
  if shift<=.015 and turn<=math.radians(.75):self.anchor_check=None;return
  if self.anchor_check is None and shift<=.02 and turn<=math.radians(1.):return
  stamp=packet.get('last_lidar_observation_time')
  if stamp is None:
   self.anchor_check=dict(state='WAITING_FOR_NEW_LIDAR_EVIDENCE',candidate=None,first_stamp=None,last_stamp=None,samples=0);return
  check=self.anchor_check
  if check and check['last_stamp'] is not None and stamp<=check['last_stamp']:return
  candidate=None if check is None else check['candidate']
  consistent=candidate is not None and np.linalg.norm((T@O)[:2,3]-(candidate@O)[:2,3])<=.005 and abs(angle_delta(yaw(T),yaw(candidate)))<=math.radians(.25)
  if not consistent:
   self.anchor_check=dict(state='CONFIRMING_ANCHOR_CORRECTION',candidate=T.copy(),first_stamp=stamp,last_stamp=stamp,samples=1);return
  check['last_stamp']=stamp;check['samples']+=1
  if check['samples']>=3 and stamp-check['first_stamp']>=.6:self.request('LOCALIZATION_ANCHOR_CHANGED')
 def map_failure(self,reason,links):
  if reason in links:self.request('MAP_COLLISION:'+reason);return 'MAP_COLLISION'
  return reason # Transport/parser failures retain their fatal policy.
 def ready(self,packet,fresh,now):
  if not fresh or packet.get('status')=='LOCALIZATION_RECHECK_REQUIRED':
   self.still_since=None;self.samples=0;return False
  stamp=packet.get('observation_time');pose=np.asarray(packet['odometry_pose'])
  if stamp is None or self.last_stamp is not None and stamp<=self.last_stamp:return False
  self.last_stamp=stamp
  if self.still_since is None or np.linalg.norm(pose[:2]-self.still_pose[:2])>.002 or abs(angle_delta(pose[2],self.still_pose[2]))>math.radians(.25):
   self.still_since=now;self.still_pose=pose.copy();self.samples=1;return False
  self.samples+=1
  return self.pending and self.samples>=3 and now-self.still_since>=.6 and now>=self.next_attempt
 def failed(self,error,now):self.attempts+=1;self.last_error=str(error);self.next_attempt=now+5.
 def adopt(self,localization):
  self.reference=anchor(localization);self.reason=None;self.anchor_check=None;self.anchor_metrics=None;self.replans+=1;self.attempts+=1;self.last_error=None;self.still_since=None;self.samples=0;self.last_stamp=None
 def accept_checked_route(self,packet):
  self.reference=np.asarray(packet['T_map_odom']).copy();self.reason=None;self.anchor_check=None;self.anchor_metrics=None;self.route_reuses+=1;self.attempts+=1;self.last_error=None;self.still_since=None;self.samples=0;self.last_stamp=None
 def summary(self):
  check=None if self.anchor_check is None else {k:v for k,v in self.anchor_check.items()if k!='candidate'}
  return dict(anchor_confirmation=check,anchor_metrics=self.anchor_metrics,pending=self.pending,reason=self.reason,replans=self.replans,route_reuses=self.route_reuses,attempts=self.attempts,last_error=self.last_error,stationary_samples=self.samples,retry_interval_s=5.)

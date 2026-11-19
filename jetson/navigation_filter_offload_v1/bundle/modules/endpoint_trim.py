"""Small odometry-bounded corrections separated by fresh-pose observations.

Only proposes twists. All proposed pulses and zero holds require the caller's
unchanged fresh-pose/map/whole-body admission; no motor interface exists here.
"""
import math,collections
import numpy as np

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

class EndpointTrim:
 def __init__(self,goal,config):
  self.goal=np.asarray(goal,float);self.cfg=config;self.samples=collections.deque(maxlen=5);self.last_stamp=None
  self.action=None;self.observe_since=None;self.observe_stamp=None;self.settle_since=None;self.settle_samples=0;self.settle_stamp=None;self.position_reached=False;self.pulses=0;self.no_progress=0;self.paused=False;self.pause_reason=None
  for name,lo,hi in [('observe_s',.4,2.),('pulse_distance_m',.002,.01),('pulse_heading_deg',.2,1.),('pulse_timeout_s',1.,3.),('no_progress_limit',2,5),('maximum_pulses',10,60)]:
   v=config.get(name)
   if not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError('INVALID_TRIM_'+name)
 def pause(self):
  # A guard interruption is never counted as a mechanical failure, nor as
  # fresh evidence of arrival. Keep the target and accumulated pulse count.
  self.action=None;self.samples.clear();self.last_stamp=None;self.observe_since=None;self.observe_stamp=None;self.reset_settle()
 def reset_settle(self):self.settle_since=None;self.settle_samples=0;self.settle_stamp=None
 def admitted(self,odometry,now):
  if self.action is not None and self.action['start'] is None:
   self.action['start']=now;self.action['odometry']=np.array(odometry,float)
 def step(self,pose,odometry,stamp,now):
  f=self.cfg;pose=np.asarray(pose,float);odometry=np.asarray(odometry,float)
  if pose.shape!=(3,) or odometry.shape!=(3,) or not np.isfinite(pose).all() or not np.isfinite(odometry).all():raise ValueError('INVALID_TRIM_POSE')
  if self.last_stamp is not None and stamp<self.last_stamp:self.pause()
  if self.last_stamp is None or stamp>self.last_stamp:self.samples.append((stamp,pose.copy()));self.last_stamp=stamp
  values=np.array([x[1] for x in self.samples]);filtered=np.median(values,axis=0);filtered[2]=wrap(pose[2]+float(np.median([wrap(p[2]-pose[2]) for _,p in self.samples])))
  delta=self.goal[:2]-filtered[:2];c,s=math.cos(pose[2]),math.sin(pose[2]);body=np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
  distance=float(np.linalg.norm(self.goal[:2]-pose[:2]));heading=wrap(self.goal[2]-pose[2]);control_distance=float(np.linalg.norm(delta));control_heading=wrap(self.goal[2]-filtered[2]);tol=f['position_tolerance_m'];angle=math.radians(f['heading_tolerance_deg'])
  error=dict(distance_m=distance,heading_error_deg=abs(math.degrees(heading)),position_tolerance_m=tol,heading_tolerance_deg=f['heading_tolerance_deg'],filtered_distance_m=control_distance,filtered_heading_error_deg=abs(math.degrees(control_heading)))
  def result(mode,command=(0.,0.,0.),note=None,state='FOLLOW_PREVIEW'):
   return dict(state=state,mode=mode,twist_preview=list(command),goal_error=error,settled_s=max(0.,now-self.settle_since) if self.settle_since is not None else 0.,settle_samples=self.settle_samples,fine_pulse_count=self.pulses,fine_no_progress_pulses=self.no_progress,control_note=note,execution_permission=False)
  if self.paused:return result('fine_paused',note=self.pause_reason)
  if len(self.samples)<3:return result('fine_observe',note='FINE_COLLECTING_NEW_POSES')
  if control_distance<=tol:self.position_reached=True
  elif control_distance>tol:self.position_reached=False
  if self.action is not None:
   a=self.action
   if a['start'] is not None:
    initial=a['odometry'];change=odometry[:2]-initial[:2];cs,si=math.cos(initial[2]),math.sin(initial[2]);local=np.array([cs*change[0]+si*change[1],-si*change[0]+cs*change[1]])
    progress=a['sign']*(wrap(odometry[2]-initial[2]) if a['axis']==2 else local[a['axis']]);elapsed=now-a['start']
    # A map-matching jump cannot fabricate pulse travel: progress uses actual
    # wheel odometry, with a separate maximum-time bound.
    if progress>=a['target'] or elapsed>=f['pulse_timeout_s']:
     minimum=math.radians(.1) if a['axis']==2 else .001
     self.no_progress=self.no_progress+1 if progress<minimum else 0;self.pulses+=1
     self.action=None;self.observe_since=now;self.observe_stamp=stamp;self.reset_settle()
     if self.no_progress>=f['no_progress_limit']:self.paused=True;self.pause_reason='FINE_NO_PROGRESS_HOLD'
     return result('fine_paused' if self.paused else 'fine_observe',note=self.pause_reason if self.paused else 'FINE_PULSE_OBSERVING')
   return result('fine_rotation' if a['axis']==2 else 'fine_translation',a['command'])
  if self.position_reached and distance<=tol and abs(heading)<=angle and abs(control_heading)<=angle:
   if self.settle_since is None:self.settle_since=now
   if self.settle_stamp is None or stamp>self.settle_stamp:self.settle_samples+=1;self.settle_stamp=stamp
   if now-self.settle_since>=f['settle_s'] and self.settle_samples>=f['minimum_samples']:return result('fine_complete',state='ARRIVED')
   return result('goal_settling')
  self.reset_settle()
  if self.observe_since is not None:
   new=sum(t>self.observe_stamp for t,_ in self.samples)
   if now-self.observe_since<f['observe_s'] or new<3:return result('fine_observe',note='FINE_PULSE_OBSERVING')
  command=np.zeros(3)
  # Correct position first. A small yaw fluctuation must not interrupt every
  # translation and repeatedly demand 90-degree steering mode changes.
  if not self.position_reached:
   axis=int(np.argmax(np.abs(body)));sign=1. if body[axis]>0 else -1.
   command[axis]=sign*f['translation_speed_mps'];target=min(f['pulse_distance_m'],max(.002,abs(body[axis])-.4*tol))
  elif abs(control_heading)>angle:
   axis=2;sign=1. if control_heading>0 else -1.
   # The prior proportional command fell below useful wheel response. Use
   # the EXISTING fine speed ceiling and bound measured angular travel.
   command[2]=sign*f['rotation_speed_radps'];target=min(math.radians(f['pulse_heading_deg']),max(math.radians(.1),abs(control_heading)-.5*angle))
  else:return result('fine_observe',note='FINE_WAITING_POSE_STABILITY')
  if self.pulses>=f['maximum_pulses']:
   self.paused=True;self.pause_reason='FINE_PULSE_BUDGET_HOLD';return result('fine_paused',note=self.pause_reason)
  self.action=dict(axis=axis,sign=sign,target=target,command=command.tolist(),start=None,odometry=None)
  return result('fine_rotation' if axis==2 else 'fine_translation',command)

"""Bounded position-prefix follower. Returns suggestions; no command transport."""
import math
import numpy as np

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

class RouteMicroProbe:
 def __init__(self,path,maximum_m=.05,target_m=None):
  if not 0<maximum_m<=(.06 if target_m is not None else .05):raise ValueError('MICRO_MAXIMUM_MUST_NOT_EXCEED_5CM')
  if target_m is not None and not .05<=target_m<=maximum_m-.005:raise ValueError('CABLE_PROBE_REQUIRES_5CM_AND_BRAKING_RESERVE')
  self.cable_test=target_m is not None;self.maximum=maximum_m;self.target_travel=max(0.,maximum_m-.015) if target_m is None else target_m
  if self.target_travel<.005:raise ValueError('MICRO_BUDGET_TOO_SMALL')
  self.path=[np.zeros(2)];length=0.;p=np.asarray(path,float)
  if p.ndim!=2 or p.shape[1]<2 or not np.isfinite(p).all():raise ValueError('INVALID_MICRO_ROUTE')
  origin=p[0,:2].copy()
  for a,b in zip(p[:-1,:2],p[1:,:2]):
   d=float(np.linalg.norm(b-a))
   if d<1e-8:continue
   take=min(d,self.target_travel-length)
   for u in np.linspace(0.,take,max(2,int(math.ceil(take/.005))+1))[1:]:self.path.append(a-origin+(b-a)*(u/d))
   length+=take
   if length>=self.target_travel-1e-9:break
  if length<.005:raise ValueError('NO_TRANSLATIONAL_ROUTE_PREFIX')
  self.path=np.asarray(self.path);self.speed=.005;self.reference=None;self.previous=None;self.traversed=0.;self.commanded=0.;self.command_speed=0.;self.command_time=None;self.started=None;self.index=1;self.stopped=None
 def arm(self,odom,now):
  if self.reference is not None:return
  self.reference=np.asarray(odom,float).copy();self.previous=self.reference[:2].copy();self.started=now;self.command_time=now
 def note_command(self,twist,now):
  if self.reference is None:return
  self.commanded+=self.command_speed*max(0.,now-self.command_time);self.command_time=now;self.command_speed=math.hypot(*twist[:2])
 def stop(self,reason):
  self.stopped=reason;self.command_speed=0.;return dict(state='MICRO_STOP',reason=reason,twist_preview=[0.,0.,0.],traversed_m=self.traversed,commanded_m=self.commanded)
 def step(self,odom,now):
  if self.stopped:return self.stop(self.stopped)
  if self.reference is None:
   d=self.path[1];v=d/np.linalg.norm(d)*self.speed;return dict(state='MICRO_PREVIEW',twist_preview=[float(v[0]),float(v[1]),0.])
  pose=np.asarray(odom,float)
  if pose.shape!=(3,) or not np.isfinite(pose).all():return self.stop('INVALID_ODOMETRY')
  step=float(np.linalg.norm(pose[:2]-self.previous));self.previous=pose[:2].copy();self.traversed+=step
  self.commanded+=self.command_speed*max(0.,now-self.command_time);self.command_time=now
  if step>.02:return self.stop('ODOMETRY_JUMP')
  if abs(wrap(pose[2]-self.reference[2]))>math.radians(1):return self.stop('HEADING_CHANGED')
  budget=self.maximum-.002 if self.cable_test else self.target_travel
  if self.traversed>=budget or self.commanded>=budget:return self.stop('DISTANCE_BUDGET_REACHED')
  if now-self.started>=(18. if self.cable_test else 12.):return self.stop('MICRO_TIME_LIMIT')
  c,s=math.cos(self.reference[2]),math.sin(self.reference[2]);local=(pose[:2]-self.reference[:2])@np.array([[c,-s],[s,c]])
  if np.linalg.norm(local-self.path[-1])<=(.0005 if self.cable_test else .002):return self.stop('PREFIX_REACHED')
  while self.index+1<len(self.path) and np.linalg.norm(local-self.path[self.index])<=.003:self.index+=1
  delta=self.path[self.index]-local
  if np.linalg.norm(delta)>.025:return self.stop('PREFIX_DEVIATION')
  v=delta/max(np.linalg.norm(delta),1e-9)*min(self.speed,max(.002,2*np.linalg.norm(delta)))
  return dict(state='MICRO_FOLLOW',twist_preview=[float(v[0]),float(v[1]),0.],traversed_m=self.traversed,commanded_m=self.commanded)

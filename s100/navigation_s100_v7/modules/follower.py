"""Measured-pose feedback controller prototype; returns suggestions, no transport.
Only a geometry-checked local side-step may use vy. A caller must supply a
fresh localization and independent obstacle guard on EVERY step.
"""
import math
import numpy as np
from endpoint_trim import EndpointTrim

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

class PreviewFollower:
 def __init__(self,plan,config):
  if not plan.get('selected') or not plan['selected'].get('geometry_recheck_passed'):raise ValueError('CHECKED_PATH_REQUIRED')
  self.goal=plan['target'];self.config=config;self.start=np.array(plan['start_map_pose']);p=np.array(plan['selected']['path'],float)
  c,s=np.cos(self.start[2]),np.sin(self.start[2]);self.path=p.copy();self.path[:,:2]=p[:,:2]@np.array([[c,-s],[s,c]]).T+self.start[:2];self.path[:,2]+=self.start[2]
  self.fixed_translation=plan['selected'].get('profile') in ('short_axial_translation','short_diagonal_translation','short_local_translation','short_turn_then_translation')
  self.index=0;self.previous=np.zeros(3);self.last_time=None;self.curvature=None;self.direction=None;self.last_mode=None
  self.lookahead=float(config.get('tracking_lookahead_m',.15))
  if not .05<=self.lookahead<=.30:raise ValueError('INVALID_TRACKING_LOOKAHEAD')
  self.fine=config.get('fine_goal',{});self.fine_active=False;self.fine_aligning=False;self.fine_position_reached=False;self.reset_settle()
  self.stability=config.get('tracking_stability',{});self.admitted_curvature=None;self.admitted_direction=None;self.admitted_time=None
  trim_config=dict(self.fine,translation_speed_mps=min(self.fine.get('translation_speed_mps',.01),config.get('max_vx_mps',.01),config.get('max_reverse_mps',.01),config.get('max_vy_mps',.01)),rotation_speed_radps=min(self.fine.get('rotation_speed_radps',.025),config.get('max_wz_radps',.025)))
  self.trim=EndpointTrim(self.goal['pose'],trim_config) if self.fine.get('enabled') and self.fine.get('strategy')=='pulse_observe' else None
  if self.fine.get('enabled'):
   f=self.fine
   for key,lower,upper in [('entry_distance_m',.075,.15),('position_tolerance_m',.005,.05),('heading_tolerance_deg',.3,3.),('settle_s',.5,3.),('minimum_samples',3,30),('translation_speed_mps',.001,.015),('rotation_speed_radps',.005,.04),('position_gain',.1,1.),('heading_gain',.1,1.)]:
    value=f.get(key)
    if not isinstance(value,(int,float)) or not math.isfinite(value) or not lower<=value<=upper:raise ValueError('INVALID_FINE_GOAL_'+key)
   if f['minimum_samples']!=int(f['minimum_samples']):raise ValueError('INVALID_FINE_GOAL_SAMPLE_COUNT')
 def reset_settle(self):self.settle_since=None;self.settle_samples=0;self.settle_observation=None
 def segment_motion(self,index):
  if index<=0:return ('rotation',0)
  delta=self.path[index,:2]-self.path[index-1,:2];a=self.path[index-1,2]
  bx=math.cos(a)*delta[0]+math.sin(a)*delta[1];by=-math.sin(a)*delta[0]+math.cos(a)*delta[1]
  if abs(bx)+abs(by)<1e-6:return ('rotation',0)
  if self.fixed_translation:return ('translation',0)
  if abs(by)>abs(bx)*2:return ('lateral',1 if by>=0 else -1)
  return ('differential',1 if bx>=0 else -1)
 def stop(self,state):
  self.previous[:]=0;self.curvature=None;self.direction=None;self.last_time=None;self.admitted_curvature=None;self.admitted_direction=None;self.admitted_time=None;self.reset_settle()
  if self.trim:self.trim.pause()
  return dict(state=state,twist_preview=[0.,0.,0.],execution_permission=False)
 def pause(self,state):
  # Zero remains immediate. Restore curvature from the last ADMITTED twist,
  # so a one-frame feedback outage cannot reset the steering target filter.
  self.previous[:]=0;self.curvature=self.admitted_curvature;self.direction=self.admitted_direction;self.last_time=self.admitted_time;self.reset_settle()
  if self.trim:self.trim.pause()
  return dict(state=state,twist_preview=[0.,0.,0.],execution_permission=False)
 def admitted(self,twist,odometry,now):
  if not self.fine_active and self.last_mode=='differential' and abs(twist[0])>1e-8:
   self.admitted_curvature=twist[2]/twist[0];self.admitted_direction=1 if twist[0]>0 else -1;self.admitted_time=self.last_time
  else:self.admitted_curvature=None;self.admitted_direction=None;self.admitted_time=None
  if self.trim:self.trim.admitted(odometry,now)
 def fine_step(self,pose,distance,heading,observation_time,now):
  f=self.fine;c,s=math.cos(pose[2]),math.sin(pose[2]);delta=np.asarray(self.goal['pose'])[:2]-pose[:2];body=np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
  error=dict(distance_m=float(distance),heading_error_deg=abs(math.degrees(heading)),position_tolerance_m=f['position_tolerance_m'],heading_tolerance_deg=f['heading_tolerance_deg'])
  heading_tolerance=math.radians(f['heading_tolerance_deg']);command=np.zeros(3);mode='fine_translation'
  # Stop inside the acceptance boundary, then retain zero through small
  # localization fluctuations. Leave the hold only beyond the outer bound.
  if distance<=f['position_tolerance_m']:self.fine_position_reached=True
  elif distance>f['position_tolerance_m']:self.fine_position_reached=False
  if self.fine_position_reached and abs(heading)<=heading_tolerance:
   if self.settle_since is None or now<self.settle_since or (self.settle_observation is not None and observation_time<self.settle_observation):self.reset_settle();self.settle_since=now
   if self.settle_observation is None or observation_time>self.settle_observation:self.settle_samples+=1;self.settle_observation=observation_time
   stable=max(0.,now-self.settle_since)
   if stable>=f['settle_s'] and self.settle_samples>=f['minimum_samples']:
    return dict(self.stop('ARRIVED'),goal_error=error,settled_s=stable,mode='fine_complete')
   mode='goal_settling'
  else:
   self.reset_settle()
   if abs(heading)>heading_tolerance:self.fine_aligning=True
   if self.fine_aligning and abs(heading)>.5*heading_tolerance:
    mode='fine_rotation';cap=min(f['rotation_speed_radps'],self.config['max_wz_radps']);command[2]=np.clip(f['heading_gain']*heading,-cap,cap)
   else:
    self.fine_aligning=False
    if not self.fine_position_reached:
     command[:2]=body*f['position_gain'];length=float(np.linalg.norm(command[:2]));cap=min(f['translation_speed_mps'],self.config['max_vy_mps'],self.config['max_vx_mps'],self.config['max_reverse_mps'])
     if length>cap:command[:2]*=cap/length
  self.previous=command;self.last_time=now;self.curvature=None;self.direction=None
  return dict(state='FOLLOW_PREVIEW',mode=mode,twist_preview=command.tolist(),path_index=self.index,goal_error=error,settled_s=max(0.,now-self.settle_since) if self.settle_since is not None else 0.,settle_samples=self.settle_samples,execution_permission=False)
 def step(self,pose,observation_time,now,target_revision,localization_valid,obstacle_guard_clear,odometry_pose=None,control_time=None):
  if target_revision!=self.goal['revision']:return self.stop('TARGET_CHANGED_REPLAN_REQUIRED')
  if not localization_valid or not 0<=now-observation_time<=.5:return self.stop('LOCALIZATION_STALE')
  if not obstacle_guard_clear:return self.stop('OBSTACLE_OR_UNKNOWN_STOP')
  pose=np.asarray(pose,float)
  if pose.shape!=(3,) or not np.isfinite(pose).all():return self.stop('INVALID_POSE')
  goal=np.array(self.goal['pose']);distance=np.linalg.norm(goal[:2]-pose[:2]);heading=wrap(goal[2]-pose[2])
  if self.fine.get('enabled'):
   remaining=float(np.linalg.norm(self.path[self.index,:2]-pose[:2])+np.linalg.norm(np.diff(self.path[self.index:,:2],axis=0),axis=1).sum())
   if not self.fine_active and distance<=self.fine['entry_distance_m'] and (remaining<=.30 or distance<=.075):self.fine_active=True
   if self.fine_active and distance>self.fine['entry_distance_m']+.08:
    self.fine_active=False;self.fine_position_reached=False;self.reset_settle()
    if self.trim:self.trim.pause()
   if self.fine_active:
    if self.trim:
     result=self.trim.step(pose,pose if odometry_pose is None else odometry_pose,observation_time,now if control_time is None else control_time);result['path_index']=self.index;self.previous=np.array(result['twist_preview']);self.curvature=None;self.direction=None;return result
    return self.fine_step(pose,distance,heading,observation_time,now)
  if distance<=.075 and not self.fine.get('enabled'):
   if abs(heading)<=math.radians(5):return dict(self.stop('ARRIVED'),goal_error=dict(distance_m=float(distance),heading_error_deg=abs(math.degrees(heading)),position_tolerance_m=.075,heading_tolerance_deg=5.))
   command=[0.,0.,float(np.clip(2*heading,-self.config['max_wz_radps'],self.config['max_wz_radps']))];self.previous=np.array(command);self.last_time=now
   return dict(state='FOLLOW_PREVIEW',mode='final_rotation',twist_preview=command,path_index=self.index,execution_permission=False)
  while self.index+1<len(self.path) and np.linalg.norm(self.path[self.index+1,:2]-pose[:2])<.055:
   next_point=self.path[self.index+1];rotation=np.linalg.norm(next_point[:2]-self.path[self.index,:2])<1e-6
   if rotation:
    end=self.index+1
    while end+1<len(self.path) and np.linalg.norm(self.path[end+1,:2]-next_point[:2])<1e-6:end+=1
    if abs(wrap(self.path[end,2]-pose[2]))>math.radians(self.fine.get('heading_tolerance_deg',1.)):break
   k=self.index+1
   if k+1<len(self.path) and self.segment_motion(k)!=self.segment_motion(k+1) and np.linalg.norm(next_point[:2]-pose[:2])>.015:break
   self.index+=1
  j=min(self.index+1,len(self.path)-1)
  if self.segment_motion(j)[0]=='rotation':
   while j+1<len(self.path) and np.linalg.norm(self.path[j+1,:2]-self.path[j,:2])<1e-6:j+=1
  if self.segment_motion(j)[0] in ('differential','translation'):
   kind=self.segment_motion(j);length=float(np.linalg.norm(self.path[j,:2]-pose[:2]))
   while j+1<len(self.path) and length<self.lookahead and self.segment_motion(j+1)==kind:
    length+=float(np.linalg.norm(self.path[j+1,:2]-self.path[j,:2]));j+=1
  target=self.path[j];delta=target[:2]-pose[:2];c,s=math.cos(pose[2]),math.sin(pose[2]);body=np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
  if j>0:
   segment=self.path[j,:2]-self.path[j-1,:2];a=self.path[j-1,2];bx=math.cos(a)*segment[0]+math.sin(a)*segment[1];by=-math.sin(a)*segment[0]+math.cos(a)*segment[1]
  else:bx=by=0.
  stable=self.stability.get('enabled',False);deadband=float(self.stability.get('lateral_deadband_m',.02));angle_deadband=math.radians(min(self.stability.get('heading_deadband_deg',2.),self.fine.get('heading_tolerance_deg',1.)))
  # Only explicit straight path portions get the position deadband. Planned
  # curves retain their pure-pursuit geometry and curvature continuity.
  straight=stable and all(abs(wrap(a-self.path[j,2]))<math.radians(.2) for a in self.path[self.index:j+1,2])
  if abs(bx)+abs(by)<1e-6:
   command=np.array([0.,0.,np.clip(2*wrap(target[2]-pose[2]),-self.config['max_wz_radps'],self.config['max_wz_radps'])]);mode='rotation'
  elif self.fixed_translation:
   # Follow the checked fixed-heading line with simultaneous body vx/vy.
   # Deadband acts across the line, preserving its diagonal direction.
   along=np.array([c*segment[0]+s*segment[1],-s*segment[0]+c*segment[1]],float);along/=np.linalg.norm(along);normal=np.array([-along[1],along[0]])
   cross=float(body@normal);correction=math.copysign(max(0.,abs(cross)-deadband),cross) if stable else cross
   desired=1.5*(along*float(body@along)+normal*correction)
   caps=[self.config['max_vx_mps'] if desired[0]>=0 else self.config['max_reverse_mps'],self.config['max_vy_mps']]
   scale=min([1.]+[cap/abs(v) for cap,v in zip(caps,desired) if abs(v)>1e-9]);desired*=scale
   heading_error=wrap(target[2]-pose[2])
   if stable:heading_error=math.copysign(max(0.,abs(heading_error)-angle_deadband),heading_error)
   command=np.array([desired[0],desired[1],np.clip(2*heading_error,-self.config['max_wz_radps'],self.config['max_wz_radps'])]);mode='bounded_local_translation'
  elif abs(by)>abs(bx)*2:
   cross=body[0];heading_error=wrap(target[2]-pose[2])
   if stable:
    cross=math.copysign(max(0.,abs(cross)-deadband),cross);heading_error=math.copysign(max(0.,abs(heading_error)-angle_deadband),heading_error)
   command=np.array([np.clip(cross,-self.config['max_reverse_mps'],self.config['max_vx_mps']),np.clip(body[1],-self.config['max_vy_mps'],self.config['max_vy_mps']),np.clip(2*heading_error,-self.config['max_wz_radps'],self.config['max_wz_radps'])]);mode='bounded_local_translation'
  else:
   sign=1 if bx>=0 else -1;vx=sign*min(self.config['max_vx_mps'] if sign>0 else self.config['max_reverse_mps'],max(.015,1.5*np.linalg.norm(body)))
   lateral=math.copysign(max(0.,abs(body[1])-deadband),body[1]) if straight else body[1]
   curvature=2*lateral/max(.0025,float(body@body))
   if self.curvature is not None and self.direction==sign and self.last_time is not None:
    dt=min(.2,max(0.,now-self.last_time));change=.8*dt
    curvature=float(np.clip(curvature,self.curvature-change,self.curvature+change))
   self.curvature=curvature;self.direction=sign
   omega=vx*curvature
   if abs(omega)>self.config['max_wz_radps']:
    scale=self.config['max_wz_radps']/abs(omega);vx*=scale;omega*=scale
   command=np.array([vx,0.,omega]);mode='differential'
  if mode!='differential':self.curvature=None;self.direction=None
  self.previous=command;self.last_time=now;self.last_mode=mode
  return dict(state='FOLLOW_PREVIEW',mode=mode,twist_preview=command.tolist(),path_index=self.index,execution_permission=False)

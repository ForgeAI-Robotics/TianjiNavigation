"""Measured-pose feedback controller prototype; returns suggestions, no transport.
Only a geometry-checked local side-step may use vy. A caller must supply a
fresh localization and independent obstacle guard on EVERY step.
"""
import math
import numpy as np

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

class PreviewFollower:
 def __init__(self,plan,config):
  if not plan.get('selected') or not plan['selected'].get('geometry_recheck_passed'):raise ValueError('CHECKED_PATH_REQUIRED')
  self.goal=plan['target'];self.config=config;self.start=np.array(plan['start_map_pose']);p=np.array(plan['selected']['path'],float)
  c,s=np.cos(self.start[2]),np.sin(self.start[2]);self.path=p.copy();self.path[:,:2]=p[:,:2]@np.array([[c,-s],[s,c]]).T+self.start[:2];self.path[:,2]+=self.start[2]
  self.index=0;self.previous=np.zeros(3);self.last_time=None
 def stop(self,state):
  self.previous[:]=0;return dict(state=state,twist_preview=[0.,0.,0.],execution_permission=False)
 def step(self,pose,observation_time,now,target_revision,localization_valid,obstacle_guard_clear):
  if target_revision!=self.goal['revision']:return self.stop('TARGET_CHANGED_REPLAN_REQUIRED')
  if not localization_valid or not 0<=now-observation_time<=.5:return self.stop('LOCALIZATION_STALE')
  if not obstacle_guard_clear:return self.stop('OBSTACLE_OR_UNKNOWN_STOP')
  pose=np.asarray(pose,float)
  if pose.shape!=(3,) or not np.isfinite(pose).all():return self.stop('INVALID_POSE')
  goal=np.array(self.goal['pose']);distance=np.linalg.norm(goal[:2]-pose[:2]);heading=wrap(goal[2]-pose[2])
  if distance<=.075 and abs(heading)<=math.radians(5):return self.stop('ARRIVED')
  while self.index+1<len(self.path) and np.linalg.norm(self.path[self.index+1,:2]-pose[:2])<.055:
   next_point=self.path[self.index+1];rotation=np.linalg.norm(next_point[:2]-self.path[self.index,:2])<1e-6
   if rotation and abs(wrap(next_point[2]-pose[2]))>math.radians(3):break
   self.index+=1
  j=min(self.index+1,len(self.path)-1);target=self.path[j];delta=target[:2]-pose[:2];c,s=math.cos(pose[2]),math.sin(pose[2]);body=np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
  if j>0:
   segment=self.path[j,:2]-self.path[j-1,:2];a=self.path[j-1,2];bx=math.cos(a)*segment[0]+math.sin(a)*segment[1];by=-math.sin(a)*segment[0]+math.cos(a)*segment[1]
  else:bx=by=0.
  if abs(bx)+abs(by)<1e-6:
   command=np.array([0.,0.,np.clip(2*wrap(target[2]-pose[2]),-self.config['max_wz_radps'],self.config['max_wz_radps'])]);mode='rotation'
  elif abs(by)>abs(bx)*2:
   command=np.array([np.clip(body[0],-self.config['max_reverse_mps'],self.config['max_vx_mps']),np.clip(body[1],-self.config['max_vy_mps'],self.config['max_vy_mps']),np.clip(2*wrap(target[2]-pose[2]),-self.config['max_wz_radps'],self.config['max_wz_radps'])]);mode='bounded_local_translation'
  else:
   sign=1 if bx>=0 else -1;vx=sign*min(self.config['max_vx_mps'] if sign>0 else self.config['max_reverse_mps'],max(.015,1.5*np.linalg.norm(body)))
   omega=2*vx*body[1]/max(.0025,float(body@body))+.5*wrap(target[2]-pose[2]);command=np.array([vx,0.,np.clip(omega,-self.config['max_wz_radps'],self.config['max_wz_radps'])]);mode='differential'
  if distance<.08:command[2]=np.clip(2*heading,-self.config['max_wz_radps'],self.config['max_wz_radps'])
  self.previous=command;self.last_time=now
  return dict(state='FOLLOW_PREVIEW',mode=mode,twist_preview=command.tolist(),path_index=self.index,execution_permission=False)

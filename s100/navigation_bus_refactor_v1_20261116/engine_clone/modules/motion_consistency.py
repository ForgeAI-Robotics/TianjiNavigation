"""Read-only two-frame motion evidence. No commands and no deletion policy."""
import numpy as np
from scipy.spatial import cKDTree

class DualFrameObserver:
 def __init__(self,count,tolerance_m=.025,minimum_frames=12,minimum_translation_m=.10,minimum_second_axis_m=.02):
  self.count=count;self.tolerance=tolerance_m;self.minimum_frames=minimum_frames;self.minimum_translation=minimum_translation_m;self.minimum_second_axis=minimum_second_axis_m;self.anchors={};self.anchor_poses={};self.rows={};self.windows={}
 def observe(self,points_body,predicted_body,T_world_body,stamp,sensor):
  p=np.asarray(points_body,float);pred=np.asarray(predicted_body,float);T=np.asarray(T_world_body,float)
  if pred.shape!=(self.count,3) or T.shape!=(4,4) or not np.isfinite(pred).all() or not np.isfinite(T).all():raise ValueError('INVALID_MOTION_EVIDENCE_GEOMETRY')
  window=int(stamp)
  if window in self.windows.setdefault(sensor,set()):return
  self.windows[sensor].add(window);p=p[np.isfinite(p).all(1)]
  if not len(p):return
  tree=cKDTree(p);body_d,index=tree.query(pred)
  anchors=self.anchors.setdefault(sensor,np.full_like(pred,np.nan));missing=~np.isfinite(anchors).all(1);first=missing&(body_d<=self.tolerance)
  anchors[first]=p[index[first]]@T[:3,:3].T+T[:3,3]
  anchor_poses=self.anchor_poses.setdefault(sensor,np.full_like(pred,np.nan));anchor_poses[first]=T[:3,3]
  valid=np.isfinite(anchors).all(1);world_pred=np.zeros_like(pred);world_pred[valid]=(anchors[valid]-T[:3,3])@T[:3,:3]
  world_d=np.full(self.count,np.inf);world_d[valid]=tree.query(world_pred[valid])[0]
  separation=np.zeros(self.count);separation[valid]=np.linalg.norm(pred[valid]-world_pred[valid],axis=1)
  self.rows.setdefault(sensor,[]).append(dict(stamp=stamp,pose=T[:3,3].copy(),body_distance=body_d,world_distance=world_d,separation=separation))
 def summary(self):
  tracks=[]
  for sensor,rows in self.rows.items():
   poses=np.asarray([r['pose'] for r in rows]);body=np.asarray([r['body_distance'] for r in rows]);world=np.asarray([r['world_distance'] for r in rows]);separation=np.asarray([r['separation'] for r in rows])
   for i in range(self.count):
    useful=separation[:,i]>=3*self.tolerance;n=int(useful.sum());status='INSUFFICIENT_MOTION';body_score=world_score=0.;xy_span=[0.,0.];second_axis=0.
    if n:
     body_score=float((body[useful,i]<=self.tolerance).mean());world_score=float((world[useful,i]<=self.tolerance).mean());q=np.vstack([poses[useful,:2],self.anchor_poses[sensor][i,:2]]);xy_span=np.ptp(q,axis=0).tolist()
     if len(q)>1:second_axis=float(np.linalg.svd(q-q.mean(0),compute_uv=False)[-1]/np.sqrt(len(q)))
     if n>=self.minimum_frames and max(xy_span)>=self.minimum_translation:
      status='UNRESOLVED'
      if body_score>=.70 and world_score<=.25 and second_axis>=self.minimum_second_axis:status='SELF_CORRELATED'
      elif world_score>=.70 and body_score<=.25:status='ENVIRONMENT_FIXED'
    tracks.append(dict(feature=i,sensor=sensor,status=status,informative_frames=n,body_match_fraction=body_score,world_match_fraction=world_score,xy_span_m=xy_span,second_translation_axis_std_m=second_axis))
  aggregate=[]
  for i in range(self.count):
   states={r['status'] for r in tracks if r['feature']==i};qualified=states-{'INSUFFICIENT_MOTION'}
   status=next(iter(qualified)) if len(qualified)==1 else 'INSUFFICIENT_MOTION' if not qualified else 'UNRESOLVED'
   aggregate.append(dict(feature=i,status=status))
  return dict(features=aggregate,tracks=tracks,tolerance_m=self.tolerance,minimum_informative_frames=self.minimum_frames,minimum_hypothesis_separation_m=3*self.tolerance,automatic_point_deletion=False,motion_commands_sent=0,free_space_certified=False)

"""Temporary, frozen initial-wire exemption for one bounded inspection move.

This is a user-authorized test assumption, not a self-identification result.
No runtime learning or permanent deletion occurs before motion is measured.
"""
import math,copy
import numpy as np
from scipy.spatial import cKDTree
from saved_map import base_transform
from cable_precheck import deferred_mask

class ProbeCableDeferral:
 def __init__(self,radius_m=.015,maximum_points=250):
  self.radius=radius_m;self.maximum_points=maximum_points;self.reference=None;self.seeds=np.empty((0,3));self.observations=set();self.sensors=set();self.model_sha256=None;self.frozen=False;self.last_deferred=0;self.last_points=np.empty((0,3));self.region_shapes=None
 def observe(self,points,shapes,model_sha256,odom,sensor,sequence):
  if self.frozen:return
  key=(sensor,sequence)
  if key in self.observations:return
  T=base_transform(odom)
  if self.reference is None:self.reference=T;self.model_sha256=model_sha256;self.region_shapes=copy.deepcopy(shapes)
  if self.model_sha256!=model_sha256:raise ValueError('PROBE_TEMPLATE_MODEL_CHANGED')
  yaw=math.atan2((self.reference[:3,:3].T@T[:3,:3])[1,0],(self.reference[:3,:3].T@T[:3,:3])[0,0])
  if np.linalg.norm(T[:2,3]-self.reference[:2,3])>.002 or abs(yaw)>math.radians(.1):raise ValueError('BASE_MOVED_BEFORE_PROBE_TEMPLATE_FROZEN')
  p=np.asarray(points,float);candidate,report=deferred_mask(p,shapes,model_sha256)
  if report['status']=='CANDIDATE_BUDGET_EXCEEDED_KEEP_ALL':raise ValueError('PROBE_TEMPLATE_BUDGET_EXCEEDED')
  selected=p[candidate];selected=(selected@T[:3,:3].T+T[:3,3]-self.reference[:3,3])@self.reference[:3,:3]
  q=np.vstack([self.seeds,selected]);_,ids=np.unique(np.floor(q/.01).astype(np.int64),axis=0,return_index=True);q=q[np.sort(ids)]
  if len(q)>self.maximum_points:raise ValueError('PROBE_TEMPLATE_BUDGET_EXCEEDED')
  self.seeds=q;self.observations.add(key);self.sensors.add(sensor)
 def ready_to_freeze(self):return self.sensors=={'front','back'} and all(sum(key[0]==s for key in self.observations)>=3 for s in ['front','back'])
 def freeze(self):
  if not self.ready_to_freeze():raise ValueError('PROBE_TEMPLATE_REQUIRES_THREE_SCANS_PER_LIDAR')
  self.frozen=True
 def mask(self,points,shapes,model_sha256,odom):
  p=np.asarray(points,float);result=np.zeros(len(p),bool)
  if self.reference is None:self.last_deferred=0;self.last_points=np.empty((0,3));return result
  if self.model_sha256!=model_sha256:raise ValueError('PROBE_TEMPLATE_MODEL_CHANGED')
  T=base_transform(odom);R=self.reference[:3,:3].T@T[:3,:3]
  if np.linalg.norm(T[:2,3]-self.reference[:2,3])>.06 or abs(math.atan2(R[1,0],R[0,0]))>math.radians(1):raise ValueError('PROBE_DEFERRAL_OUTSIDE_BOUNDED_TEST')
  # Freeze the *geometric region*, not individual noisy return locations.
  # A later sample of the same cable must not become a new stop just because
  # it fell 2cm away from an earlier return. Surface support still vetoes it.
  result,report=deferred_mask(p,self.region_shapes,model_sha256,minimum_gap_m=0.)
  if report['status']=='SURFACE_PROTECTION_UNAVAILABLE_KEEP_ALL':raise ValueError('PROBE_SURFACE_PROTECTION_UNAVAILABLE')
  self.last_deferred=int(result.sum());self.last_points=p[result].copy();return result
 def summary(self):
  return dict(source='USER_AUTHORIZED_NEAR_BODY_WIRE_REGION_TEST',frozen=self.frozen,seed_count=len(self.seeds),independent_scans=len(self.observations),sensors=sorted(self.sensors),region_surface_band_m=.20,region_height_m=[.6,1.5],temporarily_deferred_points=self.last_deferred,permanent_filter=False,applies_to_full_navigation=False)

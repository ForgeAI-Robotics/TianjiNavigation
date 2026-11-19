"""Small, frozen link-local cable zones with higher-priority surface retention.

Observed external planes/edges always override deletion. Removing self returns
does not clear map occupancy or award free space behind a cable.
"""
import json,math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from surface_components import touching_components
from surface_footprint import plane_supported,edge_supported
ROOT=Path(__file__).resolve().parents[1]

CONFIG=ROOT/'config/cable_zones.json'

def settings():
 try:return json.loads(CONFIG.read_text())
 except FileNotFoundError:return dict(enabled=False)

def centers(shapes,cfg,model_sha256):
 if not cfg.get('enabled'):return np.empty((0,3)), 'DISABLED'
 if cfg.get('model_sha256')!=model_sha256:return np.empty((0,3)), 'MODEL_CHANGED_KEEP_ALL'
 lookup={(s['link'],str(s.get('name',''))):s for s in shapes}
 result=[]
 for zone in cfg.get('zones',[]):
  s=lookup.get((zone['link'],str(zone['collision_name'])))
  if s is None or s['mode']!='measured_or_fixed':return np.empty((0,3)), 'ANCHOR_UNAVAILABLE_KEEP_ALL'
  T=np.asarray(s['matrix']);original=np.asarray(zone['reference_matrix'])
  angle=math.acos(float(np.clip((np.trace(T[:3,:3]@original[:3,:3].T)-1)/2,-1,1)))
  if np.linalg.norm(T[:3,3]-original[:3,3])>cfg['max_anchor_translation_m'] or angle>math.radians(cfg['max_anchor_rotation_deg']):
   return np.empty((0,3)), 'POSTURE_CHANGED_KEEP_ALL'
  result.append(np.asarray(zone['points_link'])@T[:3,:3].T+T[:3,3])
 return np.concatenate(result) if result else np.empty((0,3)), 'FROZEN_USER_CABLE_ZONES'

def candidate_mask(points,shapes,cfg,model_sha256,extra=0.):
 p=np.asarray(points,float);c,status=centers(shapes,cfg,model_sha256);result=np.zeros(len(p),bool)
 finite=np.isfinite(p).all(1)
 if len(c) and finite.any():
  radius=float(cfg['radius_m'])+extra
  near=finite&(p>=c.min(0)-radius).all(1)&(p<=c.max(0)+radius).all(1)
  if near.any():result[near]=cKDTree(c).query(p[near],distance_upper_bound=radius+1e-12)[0]<=radius
 return result,c,status

def protected_surfaces(points,shapes,model_sha256,cfg=None,voxel_resolution_m=0.,roi_override=None,centers_override=None,*,interest=None):
 """Fit only external support; retain in-zone surface and edge continuations.

All geometry is body-local. Fit budget failure keeps the affected zone. A
scene with no independent support cannot distinguish coincident wire/edge.
"""
 cfg=settings() if cfg is None else cfg;p=np.asarray(points,float);protected=np.zeros(len(p),bool)
 half=math.sqrt(3)*voxel_resolution_m/2
 if roi_override is None:
  roi,c,status=candidate_mask(p,shapes,cfg,model_sha256,extra=half)
 else:
  roi=np.asarray(roi_override,bool);c=np.asarray(centers_override,float).reshape(-1,3);status='NEAR_SURFACE_INDEPENDENT_SUPPORT'
  if roi.shape!=(len(p),) or not np.isfinite(c).all():raise ValueError('INVALID_SURFACE_PROTECTION_ROI')
  if not cfg.get('enabled') or cfg.get('model_sha256')!=model_sha256:return protected,dict(status='MODEL_OR_CONFIG_INVALID',protected_points=0)
 report=dict(status=status,protected_points=0,planes=0,edges=0,budget_keep_points=0)
 if not len(c) or not roi.any():return protected,report
 if roi_override is None:exact,_,_=candidate_mask(p,shapes,cfg,model_sha256,extra=.025+half)
 else:
  exact=np.zeros(len(p),bool);finite=np.isfinite(p).all(1)
  exact[finite]=cKDTree(c).query(p[finite],distance_upper_bound=.025+half)[0]<=.025+half
 finite=np.isfinite(p).all(1);ids=np.flatnonzero(finite);tree=cKDTree(p[ids]);rng=np.random.default_rng(731)
 # Components are ordered by the first anchor, matching the original walk.
 if len(c)<=1000:
  groups=touching_components(c,.22,np.ones(len(c),bool),max_pairs=len(c)*(len(c)-1)//2)
 else:
  used=set()
  ctree=cKDTree(c);groups=[]
  for i in range(len(c)):
   if i in used:continue
   group={i};todo=[i];used.add(i)
   while todo:
    for j in ctree.query_ball_point(c[todo.pop()],.22):
     if j not in used:used.add(j);group.add(j);todo.append(j)
   groups.append(sorted(group))
 if interest is not None:
  interest=np.asarray(interest,bool)&roi
  if not interest.any():return protected,report
  radius=float(cfg.get('support_radius_m',.35))
  nearby=cKDTree(p[interest]).query(c,distance_upper_bound=np.nextafter(radius,np.inf))[0]<=radius
  # Earlier groups consume the original RNG sequence. Trailing groups with
  # no relevant targets cannot affect any requested classification.
  last=max((i for i,group in enumerate(groups) if nearby[group].any()),default=-1)
  groups=groups[:last+1]
 else:
  interest=roi
 from environment import self_candidates
 inside_body=np.zeros(len(p),bool);inside_body[ids]=self_candidates(p[ids],[s for s in shapes if s['mode']=='measured_or_fixed'],.10+half,_tree=tree)
 for group in groups:
  radius=float(cfg.get('support_radius_m',.35))
  anchors=c[group]
  padding=np.nextafter(radius,np.inf)
  context=ids[((p[ids]>=anchors.min(0)-padding)&(p[ids]<=anchors.max(0)+padding)).all(1)]
  distances=cKDTree(anchors).query(p[context],distance_upper_bound=padding)[0]
  patch=context[distances<=radius];targets=patch[roi[patch]]
  if not len(targets):continue
  if len(patch)>int(cfg.get('max_patch_points',5000)):
   # Dense repeated returns are not additional independent evidence. Reduce
   # context only; retain every original target for the final decision.
   _,representatives=np.unique(np.floor(p[patch]/.015).astype(np.int64),axis=0,return_index=True)
   patch=np.unique(np.r_[patch[representatives],targets])
   if len(patch)>int(cfg.get('max_patch_points',5000)):
    protected[targets]=True;report['budget_keep_points']+=len(targets);continue
  targets=targets[interest[targets]]
  external=patch[~exact[patch]]
  if len(external)<6:continue
  # Same point/OBB predicate, evaluated once for all support patches.
  support_ids=external[~inside_body[external]];support=p[support_ids]
  if len(support)<6:continue
  remaining=support.copy()
  # Up to three planes: tops, undersides and vertical desk/wall faces.
  for _ in range(3):
   if len(remaining)<12:break
   # Preserve the 64 draws, seed and first-best tie rule; batch only the
   # numerical scoring instead of 64 Python/NumPy dispatches per plane.
   triple=remaining[np.asarray([rng.choice(len(remaining),3,replace=False) for trial in range(64)])]
   normals=np.cross(triple[:,1]-triple[:,0],triple[:,2]-triple[:,0]);length=np.linalg.norm(normals,axis=1);valid=length>=1e-8;normals/=np.maximum(length[:,None],1e-30)
   offsets=np.einsum('ij,ij->i',triple[:,0],normals);hits=np.abs(normals@remaining.T-offsets[:,None])<=.012+min(half,.006);counts=hits.sum(1);counts[~valid]=-1;best=int(np.argmax(counts))
   if counts[best]<12:break
   hit=hits[best];surface=remaining[hit]
   values,vectors=np.linalg.eigh(np.cov(surface.T));normal=vectors[:,0];offset=surface.mean(0)@normal
   # Require two-dimensional support, not an isolated horizontal wire.
   if np.sqrt(max(values[1],0))>=.025 and np.sqrt(max(values[2],0))>=.06:
    keep=plane_supported(surface,p[targets],normal,tolerance=float(cfg.get('surface_tolerance_m',.025))+min(half,.01),extension=.025+min(half,.01),support_gap=.08+half)
    protected[targets[keep]]=True;report['planes']+=1
   remaining=remaining[~hit]
  # Sparse desk edges may be only a scan line, with no 2-D plane support.
  # Preserve connected horizontal continuations into the zone instead.
  if not len(targets):continue
  hids=patch
  components=touching_components(p[hids],.065+half,interest[hids])
  if components is None:protected[targets]=True;report['budget_keep_points']+=len(targets);continue
  for local_component in components:
   component=hids[local_component];outside=np.intersect1d(component,support_ids)
   if len(outside)<6:continue
   q=p[outside]
   if np.ptp(q[:,2])<=.035+2*half and np.linalg.norm(np.ptp(q[:,:2],axis=0))>=.15:
    selected=component[interest[component]]
    # Do not grow this permission vertically along a hanging cable.
    selected=selected[(p[selected,2]>=q[:,2].min()-.025-half)&(p[selected,2]<=q[:,2].max()+.025+half)]
    keep=edge_supported(q,p[selected],tolerance=.025+min(half,.01),extension=.025+min(half,.01),support_gap=.08+half)
    protected[selected[keep]]=True;report['edges']+=1
 report['protected_points']=int(protected.sum());return protected,report

def zone_ignore(points,shapes,model_sha256,cfg=None,protection=None,voxel_resolution_m=0.):
 cfg=settings() if cfg is None else cfg
 candidate,_,status=candidate_mask(points,shapes,cfg,model_sha256,extra=math.sqrt(3)*voxel_resolution_m/2)
 report=dict(status=status,candidate_points=int(candidate.sum()),ignored_points=0,free_space_credit=False)
 if candidate.sum()>int(cfg.get('max_ignored_points_per_scan',250)):
  return np.zeros(len(points),bool),dict(report,status='ZONE_BUDGET_EXCEEDED_KEEP_ALL')
 protected,surface_report=protected_surfaces(points,shapes,model_sha256,cfg,voxel_resolution_m=voxel_resolution_m) if protection is None else protection
 result=candidate&~protected
 report.update(ignored_points=int(result.sum()),surface_protection=surface_report)
 return result,report

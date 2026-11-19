"""Near-body sparse suppression with current-scan surface priority.

Cached planes only PRESERVE returns. They never supply points/free space or
renew acquisition timestamps. New/curved objects use current-scan continuity.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree,ConvexHull,QhullError
ROOT=Path(__file__).resolve().parents[1]
from pillar_cables import distances
from surface_components import touching_components
from saved_map import base_transform
from self_envelope import settings as envelope_settings

CONFIG=ROOT/'config/continuous_surface_filter.json'

def settings():
 try:return json.loads(CONFIG.read_text())
 except FileNotFoundError:return dict(enabled=False)

class SurfaceCache:
 def __init__(self):self.planes=[];self.fitted_at=None;self.identity=None;self.last_pose=None
 def reset(self):self.planes=[];self.fitted_at=None
 def fit(self,support,stamp,cfg):
  self.fitted_at=stamp;self.planes=[]
  if len(support)<12:return
  _,ids=np.unique(np.floor(support/.025).astype(np.int64),axis=0,return_index=True)
  remaining=support[ids]
  if len(remaining)>1600:remaining=remaining[np.linspace(0,len(remaining)-1,1600,dtype=int)]
  rng=np.random.default_rng(731)
  for _ in range(4):
   if len(remaining)<12:break
   triples=remaining[[rng.choice(len(remaining),3,replace=False) for _ in range(32)]]
   normals=np.cross(triples[:,1]-triples[:,0],triples[:,2]-triples[:,0]);length=np.linalg.norm(normals,axis=1)
   normals/=np.maximum(length[:,None],1e-30);offset=np.einsum('ij,ij->i',normals,triples[:,0])
   hits=np.abs(normals@remaining.T-offset[:,None])<=.012;counts=hits.sum(1);counts[length<1e-8]=0
   best=int(np.argmax(counts))
   if counts[best]<12:break
   surface=remaining[hits[best]];remaining=remaining[~hits[best]]
   origin=surface.mean(0);values,axes=np.linalg.eigh(np.cov(surface.T));spread=np.sqrt(np.maximum(values,0))
   if spread[1]<.025 or spread[2]<.06:continue
   uv=(surface-origin)@axes[:,1:]
   try:equations=ConvexHull(uv).equations
   except QhullError:continue
   self.planes.append(dict(origin=origin,normal=axes[:,0],basis=axes[:,1:],equations=equations))
 def protect(self,world,external,targets,stamp,cfg,identity,pose,context=None):
  if identity!=self.identity or self.last_pose is not None and (np.linalg.norm(np.asarray(pose)[:2]-self.last_pose[:2])>.5 or abs(float(pose[2])-self.last_pose[2])>.5):self.reset()
  self.identity=identity;self.last_pose=np.array(pose)
  refit=self.fitted_at is None or stamp-self.fitted_at>=cfg['plane_refresh_s'] or stamp<self.fitted_at
  if refit:self.fit(world[external],stamp,cfg)
  protected=np.zeros(len(world),bool);validated=0
  for plane in self.planes:
   delta=world-plane['origin'];uv=delta@plane['basis'];on=np.abs(delta@plane['normal'])<=.025
   footprint=(uv@plane['equations'][:,:2].T+plane['equations'][:,2]<=.025).all(1)
   support=np.flatnonzero(external&on&footprint)
   if len(support)<12:continue
   # An old fitted desk is usable only with broad support in THIS scan.
   span=np.ptp(uv[support],axis=0)
   if min(span)<.05 or max(span)<.12:continue
   ids=np.flatnonzero(targets&on&footprint)
   if not len(ids):continue
   gap=float(cfg.get('current_surface_support_gap_m',.15))
   near=cKDTree(world[support]).query(world[ids],distance_upper_bound=gap+1e-9)[0]<=gap+1e-9
   ids=ids[near]
   context_mask=external if context is None else context
   local_ids=np.flatnonzero(context_mask&on&footprint)
   if len(ids) and len(local_ids)>=12:
    # Coplanarity/one large convex hull alone cannot bridge a cable into a
    # desk. Require a broad observed patch around each retained return.
    distance,neighbors=cKDTree(world[local_ids]).query(world[ids],k=48,distance_upper_bound=.08)
    valid=np.isfinite(distance);count=valid.sum(1);neighbors=np.minimum(neighbors,len(local_ids)-1)
    patch=uv[local_ids[neighbors]];weight=valid[:,:,None]
    center=(patch*weight).sum(1)/np.maximum(count[:,None],1)
    delta=(patch-center[:,None,:])*weight
    covariance=np.einsum('nki,nkj->nij',delta,delta)/np.maximum(count[:,None,None]-1,1)
    spread=np.sqrt(np.maximum(np.linalg.eigvalsh(covariance),0))
    broad=(count>=12)&(spread[:,0]>=.018)&(spread[:,1]>=.024)
    protected[ids[broad]]=True
   validated+=1
  return protected,dict(plane_refit=refit,cached_planes=len(self.planes),currently_validated_planes=validated,plane_cache_source_age_s=max(0.,stamp-self.fitted_at),plane_refresh_s=cfg['plane_refresh_s'])

def evaluate(points,shapes,model_sha256,cfg,cache=None,header=None):
 if not .10<=float(cfg['surface_band_m'])<=.30 or not .5<=float(cfg['plane_refresh_s'])<=2. or not .08<=float(cfg.get('current_surface_support_gap_m',.15))<=.15:raise ValueError('INVALID_CONTINUOUS_SURFACE_POLICY')
 p=np.asarray(points,float);n=len(p);finite=np.isfinite(p).all(1);known=[s for s in shapes if s['mode']=='measured_or_fixed']
 protected=np.zeros(n,bool);candidate=np.zeros(n,bool);own=np.zeros(n,bool);chassis_interior=np.zeros(n,bool)
 report=dict(status='CONTINUOUS_SURFACE_PRIORITY',enabled=True,surface_band_m=cfg['surface_band_m'],free_space_credit=False,collision_geometry_modified=False)
 if not known or not finite.all():
  report['status']='INVALID_GEOMETRY_OR_POINTS_KEEP_ALL'
 else:
  arms=[s for s in known if s['link']!='YD_link']
  arm_gap=distances(p,arms,limit=cfg['surface_band_m'])
  # All-body gap is only compared at 2.5/10cm. Reuse the arm query and
  # query the chassis once, without traversing every arm OBB a second time.
  chassis=[s for s in known if s['link']=='YD_link']
  chassis_gap=distances(p,chassis,limit=.10)
  gap=np.minimum(arm_gap,chassis_gap);own=gap<=.025
  # The operator confirms cables around the full moving arm assembly.
  # Follow measured geometry, not a static robot-workspace box.
  cable_links=[s for s in known if s['link'] in ('Base_L','Base_R') or s['link'].startswith(('Link1_','Link2_','Link3_','Link4_','Link5_','Link6_','Link7_','left_robotiq_','right_robotiq_','left_adapter_','right_adapter_','cam_left_wrist','cam_right_wrist'))]
  cable_gap=distances(p,cable_links,limit=.30) if cable_links else np.full(n,np.inf)
  cable_envelope=(cable_gap>.025)&(cable_gap<=.30)&(p[:,2]>=.30)&(p[:,2]<=1.80)
  candidate=((arm_gap>.025)&(arm_gap<=cfg['surface_band_m'])&(p[:,2]>=.60)&(p[:,2]<=1.50))|cable_envelope
  report['arm_cable_envelope_candidates']=int(cable_envelope.sum())
  mount=[s for s in known if s['link']=='assembly_lower_mount']
  low_mount=np.zeros(n,bool);band=float(cfg.get('lower_mount_sparse_band_m',0.))
  if band:
   height=np.asarray(cfg.get('lower_mount_height_m',[.50,.60]),float)
   if not 0<band<=.10 or height.shape!=(2,) or not np.isfinite(height).all() or not .40<=height[0]<height[1]<=.60:raise ValueError('INVALID_LOWER_MOUNT_POLICY')
   low_mount=(distances(p,mount,limit=band)<=band)&~own&(p[:,2]>=height[0])&(p[:,2]<=height[1]) if mount else low_mount
   candidate|=low_mount
  report['lower_mount_candidates']=int(low_mount.sum())
  for s in known:
   if s['link']!='YD_link':continue
   T=np.asarray(s['matrix']);lo,hi=np.asarray(s['bounds']);q=(p-T[:3,3])@T[:3,:3]
   inside=(q[:,0]>=lo[0]+.05)&(q[:,0]<=hi[0]-.05)&(q[:,1]>=lo[1]+.05)&(q[:,1]<=hi[1]-.05)
   chassis_interior|=inside
   candidate|=inside&(((q[:,2]>hi[2])&(q[:,2]<=hi[2]+.08))|((p[:,2]>=.4)&(p[:,2]<=.6)&(arm_gap>.025)&(arm_gap<=cfg['surface_band_m'])))
  # Preserve the already configured low-body self envelope. The new 25cm
  # arm band must not silently drop the existing chassis-side 6cm mask.
  envelope=envelope_settings();legacy_extra=np.zeros(n,bool);lower_extra=np.zeros(n,bool)
  if envelope.get('enabled') and envelope.get('model_sha256')==model_sha256:
   margin=float(envelope['extra_surface_margin_m'])
   if not 0<=margin<=.10:raise ValueError('SELF_ENVELOPE_MARGIN_OUT_OF_RANGE')
   links=set(envelope.get('links',[]));supports=[s for s in known if s['link'] in links]
   legacy_extra=(distances(p,supports,limit=margin)<=margin)&~own
   candidate|=legacy_extra
   if 'YD_link' in links:lower_extra|=legacy_extra&(p[:,2]<.60)&(chassis_gap<=margin)&~chassis_interior
  report['legacy_envelope_candidates']=int(legacy_extra.sum())
  header={} if header is None else header;pose=header.get('pose',[0.,0.,0.]);T=base_transform(pose);world=p@T[:3,:3].T+T[:3,3]
  stamp=float(header.get('observation_time',0.));identity=(model_sha256,header.get('boot_id'),header.get('epoch'),header.get('calibration_sha256'))
  cache=SurfaceCache() if cache is None else cache
  plane_keep,plane_report=cache.protect(world,gap>.10,candidate|own,stamp,cfg,identity,pose,context=gap>.025);protected|=plane_keep;report.update(plane_report)
  # One current-scan connectivity pass replaces per-target RANSAC/PCA/hulls.
  ids=np.flatnonzero(~own);q=p[ids]
  if len(q):
   _,representatives,inverse=np.unique(np.floor(q/.015).astype(np.int64),axis=0,return_index=True,return_inverse=True)
   reduced=q[representatives];reduced_gap=gap[ids[representatives]]
   roi=np.zeros(len(reduced),bool);roi[inverse[candidate[ids]]]=True
   components=touching_components(reduced,.065,roi,max_pairs=200000) if roi.any() else []
   if components is None:
    protected|=candidate;report['status']='CONNECTIVITY_BUDGET_KEEP_ALL'
   else:
    eligible=np.zeros(len(reduced),bool);edge_keep=np.zeros(n,bool)
    lower_representatives=np.zeros(len(reduced),bool);lower_representatives[inverse[lower_extra[ids]]]=True
    for component in components:
     external=component[reduced_gap[component]>.10]
     # Only the chassis SIDE gets this independent-support exception.
     # Interior roof returns can be a broad patch of the robot itself;
     # external desk/overhangs must pass the normal current-surface rules.
     # In the restored low chassis band, retain a broad connected object
     # even if its whole visible face lies inside 10cm. Never erase a wall
     # or low box merely because independent support is occluded by the body.
     if lower_representatives[component].any() and len(component)>=12:
      values=np.linalg.eigvalsh(np.cov(reduced[component].T));spread=np.sqrt(np.maximum(values,0))
      if spread[1]>=.015 and spread[2]>=.024:
       rows=ids[lower_extra[ids]&np.isin(inverse,component)]
       protected[rows]=True
     if len(external)<12:continue
     support=reduced[external];span=np.ptp(support,axis=0)
     if np.linalg.norm(span)<.15:continue
     eligible[component]=True
     # Preserve independently observed horizontal table edges as well.
     if span[2]<=.035 and np.linalg.norm(span[:2])>=.15:
      rows=ids[np.isin(inverse,component)&candidate[ids]]
      if len(rows):
       near=cKDTree(support).query(p[rows],distance_upper_bound=.080000001)[0]<=.080000001
       edge_keep[rows[near&(p[rows,2]>=support[:,2].min()-.025)&(p[rows,2]<=support[:,2].max()+.025)]]=True
    protected|=edge_keep
    targets=ids[candidate[ids]&eligible[inverse]&~protected[ids]]
    if len(targets)>2000:protected[targets]=True;report['status']='TARGET_BUDGET_KEEP_ALL'
    elif len(targets):
     distance,neighbors=cKDTree(reduced).query(p[targets],k=48,distance_upper_bound=.08)
     valid=np.isfinite(distance);count=valid.sum(1);neighbors=np.minimum(neighbors,len(reduced)-1)
     patch=reduced[neighbors];weight=valid[:,:,None];center=(patch*weight).sum(1)/np.maximum(count[:,None],1)
     delta=(patch-center[:,None,:])*weight;cov=np.einsum('nki,nkj->nij',delta,delta)/np.maximum(count[:,None,None]-1,1)
     values,axes=np.linalg.eigh(cov);spread=np.sqrt(np.maximum(values,0))
     residual=np.abs(np.einsum('ni,ni->n',p[targets]-center,axes[:,:,0]))
     sheet=(count>=12)&(spread[:,1]>=.018)&(spread[:,2]>=.024)&(spread[:,0]<=.012)&(residual<=.020)
     solid=(count>=20)&(spread[:,0]>=.018)&(spread[:,1]>=.024)&(spread[:,2]>=.030)
     protected[targets[sheet|solid]]=True
  # Inside the lower mount's narrow band, robot reflections can themselves
  # form a broad patch. Require nearby support OUTSIDE the whole-body band,
  # rather than inheriting area from the robot or a distant connected desk.
  mount_ids=np.flatnonzero(low_mount&protected)
  outside=np.flatnonzero(gap>.10)
  if len(mount_ids):
   independent=np.zeros(len(mount_ids),bool)
   if len(outside)>=12:
    distances_out,_=cKDTree(p[outside]).query(p[mount_ids],k=12,distance_upper_bound=.15)
    independent=np.isfinite(distances_out).sum(1)>=12
   protected[mount_ids[~independent]]=plane_keep[mount_ids[~independent]]|edge_keep[mount_ids[~independent]]
  # Operator-confirmed fixed cable bundle around the central mount. A broad
  # patch made from nearby robot/cable returns is not independent evidence.
  # Preserve real external sheets/solids using current support outside the arm/assembly 10cm band,
  # and retain the existing independently validated plane/edge protection.
  cable_cfg=cfg.get('confirmed_mount_cables')
  if cable_cfg and cable_cfg.get('enabled'):
   bounds=np.asarray(cable_cfg['body_bounds_m'],float)
   if bounds.shape!=(2,3) or not np.isfinite(bounds).all() or (bounds[1]<=bounds[0]).any() or (bounds[1]-bounds[0]>[.30,.30,.60]).any():raise ValueError('INVALID_CONFIRMED_MOUNT_CABLE_REGION')
   mounts=[s for s in known if s['link'] in ('assembly_lower_mount','assembly_upper_socket')]
   region=((p>=bounds[0])&(p<=bounds[1])).all(1)&candidate
   if mounts:region&=distances(p,mounts,limit=.10)<=.10
   else:region[:]=False
   rows=np.flatnonzero(region&protected&~plane_keep&~edge_keep)
   independent=np.zeros(len(rows),bool)
   support=np.flatnonzero(arm_gap>.10)
   if len(rows) and len(support)>=12:
    distance,neighbors=cKDTree(p[support]).query(p[rows],k=96,distance_upper_bound=.15)
    valid=np.isfinite(distance);count=valid.sum(1);neighbors=np.minimum(neighbors,len(support)-1)
    patch=p[support[neighbors]];weight=valid[:,:,None]
    center=(patch*weight).sum(1)/np.maximum(count[:,None],1)
    delta=(patch-center[:,None,:])*weight
    covariance=np.einsum('nki,nkj->nij',delta,delta)/np.maximum(count[:,None,None]-1,1)
    values,axes=np.linalg.eigh(covariance);spread=np.sqrt(np.maximum(values,0))
    residual=np.abs(np.einsum('ni,ni->n',p[rows]-center,axes[:,:,0]))
    sheet=(count>=24)&(spread[:,1]>=.024)&(spread[:,2]>=.030)&(spread[:,0]<=.012)&(residual<=.012)
    solid=(count>=20)&(spread[:,0]>=.008)&(spread[:,1]>=.024)&(spread[:,2]>=.030)&(residual<=.025)
    # A densely observed broad curved surface need not fit a plane.
    broad_curved=(count>=64)&(spread[:,1]>=.020)&(spread[:,2]>=.040)
    independent=sheet|solid|broad_curved
   protected[rows[~independent]]=False
   report['confirmed_mount_cables']=dict(candidate_points=int(region.sum()),rejected_self_supported_points=int((~independent).sum()),independent_surface_points=int(independent.sum()))
  report.update(candidate_points=int(candidate.sum()),continuous_points=int((protected&candidate).sum()),filtered_points=int((candidate&~protected).sum()))
 own&=~protected;cables=candidate&~protected&~own;filtered=own|cables
 from isolated_returns import mask as isolated_mask
 # Independent support is evaluated above before any isolated-point removal.
 eligible=(arm_gap<=cfg['surface_band_m'])&(p[:,2]>.10)&~filtered if known and finite.all() else np.zeros(n,bool)
 isolated,isolated_report=isolated_mask(p,eligible,protected,model_sha256)
 cables|=isolated;filtered|=isolated;report['isolated_returns']=isolated_report
 return dict(filtered=filtered,own=own,cables=cables,extra=cables.copy(),protected=protected,classification_complete=True,
  envelope_report=dict(enabled=True,extra_points=int(cables.sum()),original_margin_m=.025,effective_extension_m=cfg['surface_band_m'],collision_geometry_modified=False,table_surface_priority=report),
  cable_report=dict(near_body_cleanup=report),source='S100_V7_NAVIGATION_SELF_FILTER',free_space_credit=False)

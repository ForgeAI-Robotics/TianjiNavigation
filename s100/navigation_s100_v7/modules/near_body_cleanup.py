"""User-selected near-body sparse-return filter; no free-space inference."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[1]
from pillar_cables import distances
from cable_zones import protected_surfaces,settings as surface_settings
from surface_components import touching_components
from surface_footprint import plane_supported

def settings():
 p=ROOT/'config/near_body_cleanup.json'
 return json.loads(p.read_text()) if p.exists() else dict(enabled=False)

def evaluate(points,shapes,model_sha256,cfg=None,*,interest=None):
 p=np.asarray(points,float);cfg=settings() if cfg is None else cfg
 empty=np.zeros(len(p),bool)
 report=dict(enabled=False,filtered_points=0,continuous_points=0,free_space_credit=False,collision_geometry_modified=False)
 if not cfg.get('enabled'):return empty,empty.copy(),report
 if cfg.get('model_sha256')!=model_sha256:return empty,empty.copy(),dict(report,status='MODEL_CHANGED_KEEP_ALL')
 known=[s for s in shapes if s['mode']=='measured_or_fixed' and s['link']!='YD_link']
 if not known:return empty,empty.copy(),dict(report,status='NO_MEASURED_GEOMETRY_KEEP_ALL')
 # Only the 2.5cm, 10cm and configured surface-band comparisons use gap.
 gap=distances(p,known,limit=max(.10,cfg['surface_band_m']));support_gap=gap.copy();lower,upper=cfg['height_m'];finite=np.isfinite(p).all(1)
 candidate=finite&(gap>.025)&(gap<=cfg['surface_band_m'])&(p[:,2]>=lower)&(p[:,2]<=upper)
 roof=np.zeros(len(p),bool);interior=np.zeros(len(p),bool);roof_band=float(cfg.get('chassis_roof_self_band_m',0.));low_window=cfg.get('chassis_interior_cleanup_height_m')
 if roof_band or low_window:
  if roof_band and not .06<=roof_band<=.08:raise ValueError('INVALID_CHASSIS_ROOF_SELF_BAND')
  if low_window and not .4<=low_window[0]<low_window[1]<=.6:raise ValueError('INVALID_CHASSIS_INTERIOR_HEIGHT')
  for s in shapes:
   if s['link']!='YD_link' or s['mode']!='measured_or_fixed':continue
   T=np.asarray(s['matrix']);lo,hi=np.asarray(s['bounds']);local=(p-T[:3,3])@T[:3,:3]
   footprint=finite&(local[:,0]>=lo[0]+.05)&(local[:,0]<=hi[0]-.05)&(local[:,1]>=lo[1]+.05)&(local[:,1]<=hi[1]-.05)
   if roof_band:roof|=footprint&(local[:,2]>hi[2])&(local[:,2]<=hi[2]+roof_band)
   if low_window:interior|=footprint&(p[:,2]>=low_window[0])&(p[:,2]<=low_window[1])&(gap>.025)&(gap<=cfg['surface_band_m'])
   support_gap=np.minimum(support_gap,distances(p,[s],limit=.10))
  candidate|=roof|interior
 report.update(enabled=True,candidate_points=int(candidate.sum()),roof_candidate_points=int(roof.sum()),chassis_interior_candidate_points=int(interior.sum()),status='NEAR_BODY_SPARSE_CLEANUP')
 targets_to_check=candidate if interest is None else candidate&interest
 if not targets_to_check.any():return empty,empty.copy(),report
 # Ambiguous dense scenes fail closed; they are never mass-deleted.
 if candidate.sum()>1000:return empty,candidate,dict(report,status='DENSE_REGION_KEEP_ALL')
 surface_cfg=surface_settings()
 if not surface_cfg.get('enabled') or surface_cfg.get('model_sha256')!=model_sha256:
  return empty,empty.copy(),dict(report,status='SURFACE_PROTECTION_UNAVAILABLE_KEEP_ALL')
 protected,surface_report=protected_surfaces(p,shapes,model_sha256,cfg=surface_cfg,roi_override=candidate,centers_override=p[candidate],interest=interest)
 # Broad connected surfaces/objects remain obstacles even without a planar
 # fit. Exclude actual body geometry from independent continuity support.
 ids=np.flatnonzero(finite&(gap>.025));q=p[ids];roi=targets_to_check[ids] if interest is None else (targets_to_check&~protected)[ids]
 components=touching_components(q,cfg.get('connection_radius_m',.065),roi) if roi.any() else []
 if components is None:protected|=candidate;components=[]
 if interest is not None and not roi.any():protected|=targets_to_check
 for component in components:
  # Bound each local support patch below; large connected room surfaces
  # must not blanket-protect every cable candidate in the entire scan.
  rows=ids[component];targets=rows[targets_to_check[rows]&~protected[rows]]
  if not len(targets):continue
  # A noisy body face/cable bundle cannot provide its own obstacle evidence.
  # Fit continuity from independent returns beyond the 10cm body band; keep
  # near-body targets only when connected to that external support.
  support=p[rows[support_gap[rows]>.10]]
  _,unique=np.unique(np.floor(support/.015).astype(np.int64),axis=0,return_index=True);support=support[unique]
  if len(support)<12:continue
  # Evaluate local independent support for every target. A global PCA of
  # table + legs + distant surfaces can form a tilted plane through cables.
  support_tree=cKDTree(support);context_tree=cKDTree(p[rows])
  for target in targets:
   if protected[target]:continue
   local=support[support_tree.query_ball_point(p[target],.18)]
   if len(local)>5000:protected[target]=True;continue
   if len(local)<6:continue
   values,axes=np.linalg.eigh(np.cov(local.T));spread=np.sqrt(np.maximum(values,0))
   if spread[1]>=.012 and spread[2]>=.035 and spread[0]<=.015:
    keep=plane_supported(local,p[[target]],axes[:,0],tolerance=.025)
    if keep[0]:protected[target]=True;continue
   # Retain observed curved surfaces as well: their small local patches
   # must contain a two-dimensional sheet through the actual target.
   context=p[rows[context_tree.query_ball_point(p[target],.055)]]
   if len(context)<8:continue
   _,unique=np.unique(np.floor(context/.008).astype(np.int64),axis=0,return_index=True);context=context[unique]
   if len(context)<8:continue
   values,axes=np.linalg.eigh(np.cov(context.T));spread=np.sqrt(np.maximum(values,0))
   if spread[1]>=.008 and spread[2]>=.018 and spread[0]<=.007 and abs((p[target]-context.mean(0))@axes[:,0])<=.012:
    protected[target]=True
 result=targets_to_check&~protected;report.update(filtered_points=int(result.sum()),continuous_points=int((protected&candidate).sum()),surface_protection=surface_report)
 return result,protected&candidate,report

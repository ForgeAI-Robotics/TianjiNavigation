"""S100 display adapter to the shared v7 cable zones. No control interfaces.

Keep the alignment gate on actual deletion; display candidates separately.
"""
import importlib.util,time
from pathlib import Path
import numpy as np

def load_shared():
 path=Path(__file__).resolve().parent.parent/'navigation_s100_v7/modules/cable_zones.py'
 spec=importlib.util.spec_from_file_location('tianji_shared_cable_zones',path)
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

class ViewerCableFilter:
 def __init__(self,shared=None):self.shared=shared if shared is not None else load_shared()
 def apply_navigation(self,result,shapes,model_sha256,alignment_verified,static=False):
  if static:return self.apply(result,shapes,model_sha256,alignment_verified,True)
  from viewer_navigation_filter import load
  started=time.perf_counter();nav=load()(result['points_model'],shapes,model_sha256)
  cfg=self.shared.settings();centers,status=self.shared.centers(shapes,cfg,model_sha256)
  labels=np.asarray(result['labels']).copy();valid=(labels!=0)&np.isfinite(result['points_model']).all(1)
  labels[valid&(labels!=5)]=1;labels[nav['filtered']&valid]=2
  removal=np.asarray(result['removal_mask'],bool)&nav['filtered']
  result.update(labels=labels,removal_mask=removal,keep_mask=~removal)
  elapsed=(time.perf_counter()-started)*1000
  result['cable_overlay']=dict(schema_version=1,status=status,ignored_mask=nav['cables'],protected_mask=nav['protected'],navigation_filtered_mask=nav['filtered'],navigation_filter_source=nav['source'],navigation_filtered_count=int(nav['filtered'].sum()),centers_body_m=centers.tolist(),radius_m=float(cfg.get('radius_m',0.)),configuration_version=cfg.get('version'),candidate_count=int(nav['cables'].sum()),protected_count=int(nav['protected'].sum()),surface_candidate_count=int(nav['own'].sum()),compute_ms=elapsed,actual_deletion_enabled=bool(alignment_verified),free_space_inferred=False,posture_bound=True,compute_host='S100')
  from lidar_shapemask import LABEL_NAMES
  result['summary'].update(cable_zones={k:v for k,v in result['cable_overlay'].items() if not k.endswith('_mask') and k!='centers_body_m'},removed_points=int(removal.sum()),labels={name:int((labels==i).sum()) for i,name in enumerate(LABEL_NAMES)})
  result['summary']['compute_ms']+=elapsed
  return result
 def apply(self,result,shapes,model_sha256,alignment_verified,static=False,navigation_view=False):
  if navigation_view:return self.apply_navigation(result,shapes,model_sha256,alignment_verified,static)
  start=time.perf_counter();p=np.asarray(result['points_model']);n=len(p)
  ignored=np.zeros(n,bool);protected=np.zeros(n,bool);centers=np.empty((0,3))
  cfg=self.shared.settings();near_self=np.asarray(result.get('surface_near_mask',np.zeros(n,bool)),bool).copy()
  if near_self.shape!=(n,):raise ValueError('INVALID_NEAR_SURFACE_MASK')
  extra_protection=dict(protected_points=0)
  status='SAVED_POSTURE_DISPLAY_ONLY' if static else 'DISABLED'
  if not static:
   centers,status=self.shared.centers(shapes,cfg,model_sha256)
   protected,surface_report=self.shared.protected_surfaces(p,shapes,model_sha256,cfg)
   ignored,zone_report=self.shared.zone_ignore(p,shapes,model_sha256,cfg,protection=(protected,surface_report))
   status=zone_report['status']
   if near_self.any() and cfg.get('enabled') and cfg.get('model_sha256')==model_sha256:
    _,ids=np.unique(np.floor(p[near_self]/.06).astype(np.int64),axis=0,return_index=True)
    support_centers=p[near_self][ids]
    if len(support_centers)>128:
     protected|=near_self;extra_protection=dict(status='PROXIMITY_BUDGET_KEEP_ALL',protected_points=int(near_self.sum()))
    else:
     extra,extra_protection=self.shared.protected_surfaces(p,shapes,model_sha256,cfg,roi_override=near_self,centers_override=support_centers)
     protected|=extra
   else:near_self[:]=False
  else:near_self[:]=False
  labels=np.asarray(result['labels']).copy();valid=(labels!=0)&np.isfinite(p).all(1)
  ignored&=valid;protected&=valid;ignored&=~protected;near_self&=valid&~protected
  labels[protected]=1  # Independent tabletop/edge overrides self candidates.
  labels[ignored]=2
  labels[near_self]=2
  removal=np.asarray(result['removal_mask'],bool).copy()
  removal[protected]=False
  if alignment_verified and not static:removal|=ignored
  removal[near_self]=False  # This recheck never deletes an original return.
  if static:removal[:]=False
  result.update(labels=labels,removal_mask=removal,keep_mask=~removal)
  elapsed=(time.perf_counter()-start)*1000
  from lidar_shapemask import LABEL_NAMES,SURFACE_RECHECK_TOLERANCE_M
  overlay=dict(schema_version=1,status=status,ignored_mask=ignored,protected_mask=protected,
   centers_body_m=centers.tolist(),radius_m=float(cfg.get('radius_m',0.)),configuration_version=cfg.get('version'),
   candidate_count=int(ignored.sum()),protected_count=int(protected.sum()),compute_ms=elapsed,
   surface_candidate_count=int(near_self.sum()),surface_recheck_tolerance_m=SURFACE_RECHECK_TOLERANCE_M,surface_recheck_deletes_points=False,
   surface_support=extra_protection,
   actual_deletion_enabled=bool(alignment_verified and not static),free_space_inferred=False,
   posture_bound=True,compute_host='S100')
  result['cable_overlay']=overlay
  summary=result['summary'];summary['cable_zones']={k:v for k,v in overlay.items() if not k.endswith('_mask') and k!='centers_body_m'}
  summary['removed_points']=int(removal.sum());summary['compute_ms']+=elapsed
  summary['labels']={name:int((labels==i).sum()) for i,name in enumerate(LABEL_NAMES)}
  return result

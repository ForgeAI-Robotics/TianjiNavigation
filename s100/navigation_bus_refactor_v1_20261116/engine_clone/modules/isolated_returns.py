"""Count complete spatial components, never just points inside a collision box."""
import json
from pathlib import Path
import numpy as np
from surface_components import touching_components

CONFIG=Path(__file__).resolve().parents[1]/'config/isolated_returns.json'

def settings():
 try:return json.loads(CONFIG.read_text())
 except FileNotFoundError:return dict(enabled=False)

def mask(points,candidate,protected,model_sha256,cfg=None):
 p=np.asarray(points,float);candidate=np.asarray(candidate,bool);protected=np.asarray(protected,bool)
 cfg=settings() if cfg is None else cfg;removed=np.zeros(len(p),bool)
 report=dict(status='DISABLED',filtered_points=0,filtered_components=0,free_space_credit=False)
 if not cfg.get('enabled'):return removed,report
 if cfg.get('model_sha256')!=model_sha256:return removed,dict(report,status='MODEL_CHANGED_KEEP_ALL')
 leaf=float(cfg['voxel_m']);radius=float(cfg['connection_m']);extent=float(cfg['maximum_extent_m']);minimum=cfg['minimum_voxels']
 if not (.02<=leaf<=.05 and leaf<=radius<=.10 and .03<=extent<=.10 and type(minimum)is int and 2<=minimum<=4):raise ValueError('INVALID_ISOLATED_RETURN_THRESHOLDS')
 if p.shape!=(len(p),3) or candidate.shape!=(len(p),) or protected.shape!=(len(p),) or not np.isfinite(p).all():raise ValueError('INVALID_ISOLATED_RETURN_INPUT')
 if not len(p) or not candidate.any():return removed,dict(report,status='NO_CANDIDATES')
 selected=candidate&~protected
 if not selected.any():return removed,dict(report,status='NO_CANDIDATES')
 # Include support outside the removal region. The halo exceeds both the
 # maximum removable extent and a connection step; cropping cannot turn a
 # long component into an eligible tiny one. Distant room walls need no graph.
 halo=extent+radius+leaf
 context=((p>=p[selected].min(0)-halo)&(p<=p[selected].max(0)+halo)).all(1)
 source=np.flatnonzero(context);q=p[source]
 _,ids,inverse=np.unique(np.floor(q/leaf).astype(np.int64),axis=0,return_index=True,return_inverse=True)
 targets=np.zeros(len(ids),bool);targets[inverse[selected[source]]]=True
 components=touching_components(q[ids],radius,targets,max_pairs=100000)
 if components is None:return removed,dict(report,status='CONNECTIVITY_BUDGET_KEEP_ALL')
 for component in components:
  if len(component)>=minimum:continue
  rows=source[np.isin(inverse,component)]
  if protected[rows].any() or np.linalg.norm(np.ptp(p[rows],axis=0))>extent:continue
  # The whole component must be inside the caller's near-body filter scope.
  if not candidate[rows].all():continue
  removed[rows]=True;report['filtered_components']+=1
 report.update(status='ISOLATED_COMPONENT_THRESHOLD',filtered_points=int(removed.sum()),minimum_voxels=minimum,voxel_m=leaf,connection_m=radius,maximum_extent_m=extent)
 return removed,report

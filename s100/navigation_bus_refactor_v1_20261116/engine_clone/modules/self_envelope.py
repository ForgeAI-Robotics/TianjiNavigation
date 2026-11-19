"""Separate self-return envelope; never mutates collision shapes. Preview only."""
import json,math
import numpy as np
from paths import ROOT
from environment import self_candidates
CONFIG=ROOT/'config/self_filter_envelope.json'

def settings():
 return json.loads(CONFIG.read_text()) if CONFIG.exists() else dict(enabled=False)

def mask(points,shapes,cfg,model_sha256,voxel_resolution_m=0.,include_protected_mask=False):
 # Keep the existing raw/voxel self exclusion when the extension is disabled.
 half=math.sqrt(3)*voxel_resolution_m/2
 original_margin=half if voxel_resolution_m else .025
 known=[s for s in shapes if s['mode']=='measured_or_fixed']
 original=self_candidates(points,known,original_margin)
 result=original.copy();extension=np.zeros(len(points),bool)
 valid=cfg.get('enabled') and cfg.get('model_sha256')==model_sha256
 if valid:
  margin=float(cfg['extra_surface_margin_m'])
  if not 0<=margin<=.10:raise ValueError('SELF_ENVELOPE_MARGIN_OUT_OF_RANGE')
  supports=[s for s in known if s['link'] in cfg['links']]
  expanded=self_candidates(points,supports,margin+half)
  extension=expanded&~original;result|=expanded
 from cable_zones import protected_surfaces
 protected,surface_report=protected_surfaces(points,shapes,model_sha256,voxel_resolution_m=voxel_resolution_m)
 result&=~protected;extension&=~protected
 report=dict(enabled=bool(valid),extra_points=int(extension.sum()),original_margin_m=original_margin,table_surface_priority=surface_report,
  configured_extension_m=cfg.get('extra_surface_margin_m',0.),effective_extension_m=(cfg.get('extra_surface_margin_m',0.)+half) if valid else 0.,
  links=cfg.get('links',[]),collision_geometry_modified=False,free_space_credit=False,execution_permission=False)
 if include_protected_mask:report['_protected_mask']=protected
 return result,extension,report


def set_enabled(enabled):
 from common import atomic_json
 cfg=settings();cfg['enabled']=bool(enabled);atomic_json(CONFIG,cfg)
 atomic_json(ROOT/'runtime/plan_state.json',dict(state='FILTER_CHANGED_REPLAN_REQUIRED',self_envelope_enabled=bool(enabled),execution_permission=False))
 return cfg

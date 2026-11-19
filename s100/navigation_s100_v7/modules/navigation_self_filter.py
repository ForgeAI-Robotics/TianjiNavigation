"""Single live self/cable mask shared by navigation and its display adapter.

Consumes already-aligned body points. Does not move the robot, change collision
shapes, alter a stored map, or certify the unobserved shadow behind a cable.
"""
import numpy as np
from self_envelope import mask,settings as envelope_settings
from pillar_cables import ignore_mask,settings as cable_settings

def evaluate(points,shapes,model_sha256,envelope_cfg=None,cable_cfg=None,*,conservative_fast=False,surface_cache=None,frame_header=None):
 p=np.asarray(points,float)
 from continuous_surface_filter import settings as surface_policy,evaluate as surface_evaluate
 policy=surface_policy()
 if policy.get('enabled') and policy.get('model_sha256')==model_sha256:
  return surface_evaluate(p,shapes,model_sha256,policy,cache=surface_cache,header=frame_header)
 own,extra,envelope_report=mask(p,shapes,envelope_settings() if envelope_cfg is None else envelope_cfg,model_sha256,include_protected_mask=True)
 protected=envelope_report.pop('_protected_mask');remaining=np.flatnonzero(~own)
 # Own-body points removed here were <=10cm from known geometry; surface
 # fitting had already excluded them from external support. Reuse the same
 # external-surface decision rather than fitting that support a second time.
 protection=(protected[remaining],envelope_report['table_surface_priority'])
 wire,cable_report=ignore_mask(p[remaining],shapes,cable_settings() if cable_cfg is None else cable_cfg,model_sha256,protection=protection)
 cables=np.zeros(len(p),bool);cables[remaining[wire]]=True;cables&=~protected
 filtered=(own|cables)&~protected
 from motion_cables import mask as motion_mask
 motion,motion_report=motion_mask(p,shapes,model_sha256);motion&=~protected
 cables|=motion;filtered|=motion;cable_report['motion_evidence']=motion_report
 result=dict(filtered=filtered,own=own,cables=cables,extra=extra,protected=protected,
             envelope_report=envelope_report,cable_report=cable_report,
             source='S100_V7_NAVIGATION_SELF_FILTER',free_space_credit=False)
 if conservative_fast:
  cable_report['near_body_cleanup']=dict(status='CONSERVATIVE_FAST_KEEP_SPARSE_RETURNS')
 else:
  refine_sparse(p,shapes,model_sha256,result)
 return result

def refine_sparse(points,shapes,model_sha256,result,*,interest=None):
 """Complete the original cleanup using the same base mask and scan context."""
 if result.get('classification_complete'):return
 from near_body_cleanup import evaluate as cleanup
 sparse,continuous,report=cleanup(points,shapes,model_sha256,interest=interest)
 filtered=result['filtered'];protected=result['protected']
 continuous&=~filtered;protected|=continuous;sparse&=~protected
 result['cables']|=sparse;filtered|=sparse
 result['cable_report']['near_body_cleanup']=report

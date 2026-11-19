"""Candidate-route deferral and independently checked 5cm probe geometry.

Deferral is a user-authorized test assumption for candidate planning and the
bounded first inspection move. Permanent self filtering still needs motion
evidence. External surface/obstacle protection remains in force.
"""
import copy,json,subprocess
import numpy as np
from scipy.spatial import cKDTree
from common import ROOT,atomic_json
from pillar_cables import distances

def deferred_mask(points,shapes,model_sha256,minimum_gap_m=.025):
 from cable_zones import settings,protected_surfaces
 p=np.asarray(points,float);known=[s for s in shapes if s['mode']=='measured_or_fixed' and s['link']!='YD_link']
 cfg=settings()
 if not cfg.get('enabled') or cfg.get('model_sha256')!=model_sha256:return np.zeros(len(p),bool),dict(status='SURFACE_PROTECTION_UNAVAILABLE_KEEP_ALL',count=0)
 # Raw mesh fragments sit inside the merged per-link collision envelope;
 # allow 20cm only for the unconfirmed candidate/initial test hypothesis.
 gap=distances(p,known)
 candidates=(gap>minimum_gap_m)&(gap<=.20)&(p[:,2]>.60)&(p[:,2]<1.50)
 if candidates.sum()>250:return np.zeros(len(p),bool),dict(status='CANDIDATE_BUDGET_EXCEEDED_KEEP_ALL',count=int(candidates.sum()))
 if not candidates.any():return candidates,dict(status='NO_DEFERRED_CANDIDATES',count=0)
 protected,report=protected_surfaces(p,shapes,model_sha256,cfg=cfg,roi_override=candidates,centers_override=p[candidates])
 candidates&=~protected
 return candidates,dict(status='UNCONFIRMED_CABLE_CANDIDATES',count=int(candidates.sum()),table_surface_priority=report,execution_permission=False)

def candidate_scene(recorded,world,shapes,output):
 p=np.asarray(recorded['points']);mask,report=deferred_mask(p,shapes,recorded['model_sha256'])
 np.savez_compressed(output/'deferred_cable_candidates.npz',points_body=p[mask])
 atomic_json(output/'deferred_cable_candidates.json',report)
 r=copy.deepcopy(recorded);r['points']=p[~mask].tolist();r['cable_precheck']=report
 w=copy.deepcopy(world);w['occupied_points']=p[~mask].tolist()
 atomic_json(output/'candidate_world.json',w)
 return r,w,report

def probe_prefix(path,distance=.05,reserve=.01):
 """Translate along route positions with heading held; dense check is separate."""
 p=np.asarray(path,float);result=[np.zeros(3)];length=0.
 for a,b in zip(p[:-1],p[1:]):
  d=float(np.linalg.norm(b[:2]-a[:2]))
  if d<1e-9:continue
  take=min(d,distance+reserve-length)
  for u in np.linspace(0,take,max(2,int(np.ceil(take/.002))+1))[1:]:result.append(np.r_[a[:2]+(b[:2]-a[:2])*u/d,0.])
  length+=take
  if length>=distance+reserve-1e-8:break
 if length<distance+reserve-1e-6:raise ValueError('PROBE_ROUTE_TOO_SHORT_FOR_5CM_AND_STOPPING_RESERVE')
 return np.asarray(result)

def check_prefix(world_file,path):
 request=json.dumps(dict(id=1,poses=probe_prefix(path).tolist()))+'\n'
 r=subprocess.run([str(ROOT/'native/world_guard'),str(world_file)],input=request,text=True,capture_output=True,check=True,timeout=8)
 answer=json.loads(r.stdout)
 if not answer.get('clear'):raise ValueError('LOCAL_PROBE_COLLISION: '+str(answer.get('link',answer.get('error'))))
 return answer

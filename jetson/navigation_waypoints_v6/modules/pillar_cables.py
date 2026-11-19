"""User-confirmed, temporary near-pillar cable ignore for preview LIVE scans only.
Not semantic recognition or free-space evidence. Original returns are retained.
"""
import json,time
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from paths import ROOT
from common import atomic_json
CONFIG=ROOT/'config/temporary_pillar_cables.json'

def settings():
 try:return json.loads(CONFIG.read_text())
 except FileNotFoundError:return dict(enabled=False)

def set_enabled(enabled):
 cfg=settings();cfg['enabled']=bool(enabled);cfg['updated_wall_time']=time.time();atomic_json(CONFIG,cfg)
 atomic_json(ROOT/'runtime/plan_state.json',dict(state='FILTER_CHANGED_REPLAN_REQUIRED',pillar_cable_ignore_enabled=bool(enabled),execution_permission=False))
 return cfg

def distances(points,shapes):
 result=np.full(len(points),np.inf)
 for shape in shapes:
  T=np.array(shape['matrix']);lo,hi=np.array(shape['bounds']);q=(points-T[:3,3])@T[:3,:3]
  gap=np.linalg.norm(np.maximum(np.maximum(lo-q,q-hi),0),axis=1);result=np.minimum(result,gap)
 return result

def ignore_mask(points,shapes,cfg,model_sha256):
 p=np.asarray(points,float);ignored=np.zeros(len(p),bool)
 report=dict(status='DISABLED',ignored_points=0,temporary=True,source='USER_CONFIRMED_PILLAR_WIRING',free_space_credit=False,execution_permission=False,scope='current live scan only; no static-map deletion')
 if not cfg.get('enabled'):return ignored,report
 if cfg.get('model_sha256')!=model_sha256:return ignored,dict(report,status='MODEL_CHANGED_KEEP_ALL')
 support=[s for s in shapes if s['link'] in cfg['links'] and s['mode']=='measured_or_fixed']
 if not support:return ignored,dict(report,status='NO_FIXED_SUPPORT_GEOMETRY_KEEP_ALL')
 pole=[s for s in support if s['link']=='assembly_lower_mount' and np.ptp(np.asarray(s['world_corners'])[:,2])>.4]
 if not pole:return ignored,dict(report,status='NO_VERTICAL_PILLAR_KEEP_ALL')
 center=np.mean(np.concatenate([np.array(s['world_corners']) for s in pole])[:,:2],axis=0)
 z0,z1=cfg['height_m'];radius=float(cfg['surface_band_m']);context=float(cfg['context_radius_m'])
 # Full context, not just the masked band: a nearby wall/table must retain its
 # wider connected support, even when a few of its points enter the band.
 valid=np.isfinite(p).all(1);near=valid&(np.linalg.norm(p[:,:2]-center,axis=1)<=context)&(p[:,2]>=z0-.10)&(p[:,2]<=z1+.10)
 ids=np.flatnonzero(near);candidate=np.zeros(len(p),bool)
 if not len(ids):return ignored,dict(report,status='NO_NEAR_PILLAR_RETURNS')
 gap=distances(p[ids],support);candidate[ids]=(gap<=radius)&(p[ids,2]>=z0)&(p[ids,2]<=z1)&(np.linalg.norm(p[ids,:2]-center,axis=1)<=float(cfg['maximum_radius_m']))
 if candidate.sum()>cfg['max_ignored_points_per_scan']:return ignored,dict(report,status='EXCLUSION_BUDGET_EXCEEDED_KEEP_ALL',candidate_points=int(candidate.sum()))
 tree=cKDTree(p[ids]);pairs=tree.query_pairs(float(cfg['connectivity_m']),output_type='ndarray')
 if len(pairs):
  a=np.r_[pairs[:,0],pairs[:,1]];b=np.r_[pairs[:,1],pairs[:,0]];graph=coo_matrix((np.ones(len(a)),(a,b)),shape=(len(ids),len(ids))).tocsr();count,labels=connected_components(graph,directed=False)
 else:count=len(ids);labels=np.arange(len(ids))
 rejected=0;components=[]
 for component in np.unique(labels[candidate[ids]]):
  group=ids[labels==component];selected=group[candidate[group]]
  if not len(selected):continue
  q=p[group];spread=np.zeros(3);vertical_alignment=0.
  if len(q)>=3:
   covariance=np.cov(q.T);eigen,vectors=np.linalg.eigh(covariance);spread=np.sqrt(np.maximum(eigen,0));vertical_alignment=float(abs(vectors[2,-1]))
  second=float(spread[-2]);extent_xy=float(np.linalg.norm(np.ptp(q[:,:2],axis=0)))
  outside_xy=int(np.count_nonzero(np.linalg.norm(q[:,:2]-center,axis=1)>float(cfg['maximum_radius_m'])))
  external_support=outside_xy>=cfg['outside_support_points'] and extent_xy>cfg['max_local_xy_extent_m']
  ratio=float(spread[-1]/max(second,1e-9));vertical_span=float(np.ptp(q[:,2]))
  bundle=(second<=cfg.get('vertical_bundle_second_std_m',.04) and ratio>=cfg.get('vertical_bundle_min_axis_ratio',2.0) and vertical_alignment>=.8
   and vertical_span>=.15 and extent_xy<=.20 and not external_support)
  broad=(second>cfg['max_second_axis_std_m'] and not bundle) or external_support
  if broad:rejected+=len(selected)
  else:ignored[selected]=True
  components.append(dict(points=len(group),candidate_points=len(selected),kept_as_obstacle=bool(broad),second_axis_std_m=second,extent_xy_m=extent_xy,outside_support_points=outside_xy,vertical_bundle=bool(bundle),vertical_alignment=vertical_alignment,axis_ratio=ratio,vertical_span_m=vertical_span))
 report.update(status='TEMPORARY_PILLAR_CABLE_IGNORE' if ignored.any() else 'NO_SMALL_CONFIRMED_ZONE_RETURNS',ignored_points=int(ignored.sum()),candidate_points=int(candidate.sum()),protected_broad_points=int(rejected),surface_band_m=radius,height_m=[z0,z1],pillar_center_xy_m=center.tolist(),maximum_radius_m=cfg['maximum_radius_m'],links=cfg['links'],components=components)
 return ignored,report

"""Posture/boot-bound wire mask admitted only by differential motion evidence."""
import json,time,math,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from functools import lru_cache
from common import ROOT,atomic_json
FILE=ROOT/'runtime/confirmed_motion_cables.json'

def boot_id():return Path('/proc/sys/kernel/random/boot_id').read_text().strip()

@lru_cache(maxsize=4)
def reference_geometry(model_sha256,joints_json):
 from snapshot_builder import local_model
 from chassis_proxy import measured_shapes
 m=local_model()
 if m.sha256!=model_sha256:raise ValueError('MOTION_EVIDENCE_MODEL_CHANGED')
 return measured_shapes(m,json.loads(joints_json))

def publish(evidence,model_sha256):
 if evidence.get('model_sha256')!=model_sha256 or not evidence.get('raw_unfiltered_input') or evidence.get('error'):raise ValueError('INVALID_RAW_MOTION_EVIDENCE')
 tracks=evidence.get('tracks',[]);points=np.asarray(evidence['points_body_m'],float);accepted=[]
 span_required=.03 if evidence.get('posthoc_raw_replay') and evidence.get('bounded_inspection_displacement_m',0)>=.05 else .05
 for row in evidence.get('features',[]):
  i=row['feature'];qualified=[t for t in tracks if t['feature']==i and t['status']!='INSUFFICIENT_MOTION']
  if row['status']=='SELF_CORRELATED' and qualified and all(t['status']=='SELF_CORRELATED' and t['informative_frames']>=8 and max(t['xy_span_m'])>=span_required and t['body_match_fraction']>=.8 and t['world_match_fraction']<=.15 for t in qualified):accepted.append(i)
 cfg=dict(enabled=True,model_sha256=model_sha256,boot_id=boot_id(),created_at=time.time(),expires_at=time.time()+600,points_body_m=points[accepted].reshape(-1,3).tolist(),reference_joints=evidence['reference_joints'],radius_m=.015,source='RAW_DUAL_FRAME_5CM_MOTION_CONFIRMED',source_identity=evidence['source_identity'],evidence_file=evidence.get('raw_capture_folder'),confirmed_count=len(accepted),free_space_credit=False)
 cfg.update(experiment_displacement_m=evidence.get('bounded_inspection_displacement_m'),minimum_feature_translation_m=span_required,posthoc_raw_replay=bool(evidence.get('posthoc_raw_replay')))
 atomic_json(FILE,cfg);return cfg

def mask(points,shapes,model_sha256):
 p=np.asarray(points,float);result=np.zeros(len(p),bool);report=dict(status='NO_CONFIRMED_MOTION_CABLES',ignored_points=0)
 try:cfg=json.loads(FILE.read_text())
 except FileNotFoundError:return result,report
 if not cfg.get('enabled') or cfg['model_sha256']!=model_sha256 or cfg['boot_id']!=boot_id() or not cfg['created_at']<=time.time()<=cfg['expires_at']:return result,dict(report,status='STALE_MOTION_EVIDENCE_KEEP_ALL')
 if cfg['source_identity'][2]!=hashlib.sha256((ROOT/'config/lidar_calibration.json').read_bytes()).hexdigest():return result,dict(report,status='CALIBRATION_CHANGED_KEEP_ALL')
 reference=reference_geometry(model_sha256,json.dumps(cfg['reference_joints'],sort_keys=True));old={(s['link'],str(s.get('name',''))):np.asarray(s['matrix']) for s in reference}
 for s in shapes:
  T=np.asarray(s['matrix']);O=old.get((s['link'],str(s.get('name',''))))
  if O is None or np.linalg.norm(T[:3,3]-O[:3,3])>.005 or np.trace(T[:3,:3]@O[:3,:3].T)<1+2*math.cos(math.radians(.5)):return result,dict(report,status='POSTURE_CHANGED_KEEP_ALL')
 seeds=np.asarray(cfg['points_body_m'],float).reshape(-1,3)
 if not len(seeds):return result,report
 finite=np.isfinite(p).all(1);result[finite]=cKDTree(seeds).query(p[finite])[0]<=cfg['radius_m']
 if result.any():
  from cable_zones import settings,protected_surfaces
  protected,_=protected_surfaces(p,shapes,model_sha256,cfg=settings(),roi_override=result,centers_override=seeds)
  result&=~protected
 return result,dict(status=cfg['source'],ignored_points=int(result.sum()),confirmed_features=len(seeds),free_space_credit=False)

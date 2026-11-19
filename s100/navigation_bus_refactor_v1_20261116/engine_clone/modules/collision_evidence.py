"""Persist blocked-point evidence after the caller has requested zero speed."""
import os,time
from pathlib import Path
import numpy as np
from common import atomic_json
from pillar_cables import distances

def json_native(value):
 if isinstance(value,np.ndarray):return json_native(value.tolist())
 if isinstance(value,np.generic):return json_native(value.item())
 if isinstance(value,dict):return {k:json_native(v) for k,v in value.items()}
 if isinstance(value,(list,tuple)):return [json_native(v) for v in value]
 return value

def save(folder,diagnostic,frames,joints,model_sha256,session):
 folder=Path(folder);points=np.asarray(diagnostic['points_body']);shapes=diagnostic['shapes'];examples=[]
 from continuous_surface_filter import settings as surface_settings
 cfg=surface_settings();band=float(cfg['surface_band_m']) if cfg.get('enabled') and cfg.get('model_sha256')==model_sha256 else .20
 sample=points[:200]
 if len(sample):
  gaps=np.column_stack([distances(sample,[shape])for shape in shapes])
  nearest=gaps.argmin(1);known_ids=[i for i,shape in enumerate(shapes)if shape['link']!='YD_link' and shape['mode']=='measured_or_fixed']
  near=gaps[:,known_ids].min(1) if known_ids else np.full(len(sample),np.inf)
  for i,p in enumerate(sample):
   index=int(nearest[i])
   examples.append(dict(point_body_m=p.tolist(),nearest_link=shapes[index]['link'],gap_cm=float(gaps[i,index]*100),inside_cleanup_height=bool(.6<p[2]<1.5),inside_cleanup_band=bool(.025<near[i]<=band)))
 arrays=dict(points_body=points,retained_points_body=diagnostic['retained_points_body'])
 for sensor,(_,raw,_) in frames.items():arrays[sensor+'_raw_sensor']=raw
 target=folder/'live_collision.npz';temporary=target.with_suffix('.tmp')
 with temporary.open('wb') as handle:np.savez(handle,**arrays)
 os.replace(temporary,target)
 metadata={k:v for k,v in diagnostic.items() if k not in ['points_body','retained_points_body']}
 metadata.update(active_surface_band_m=band,created_at=time.time(),session=session,model_sha256=model_sha256,measured_joints=joints,frames={s:row[0] for s,row in frames.items()},examples=examples,hit_count=len(points),raw_file=str(target),source='EXACT_BLOCKING_CONTROL_ITERATION',snapshot_motion_commands_sent=0)
 path=folder/'live_collision.json';atomic_json(path,json_native(metadata));return str(path)

"""User-painted 2D ground candidates. Never erases 3D occupancy."""
import argparse,json,hashlib,time
import numpy as np
from common import ROOT,atomic_json,exclusive,lock_held
from targets import scene_config
FILE=scene_config('manual_floor.json')

def settings(map_sha256=None):
 cfg=json.loads(FILE.read_text()) if FILE.exists() else dict(enabled=False,revision=0)
 if cfg.get('enabled'):
  bounds=np.asarray(cfg['bounds_map_m'],float)
  if bounds.shape!=(4,) or not np.isfinite(bounds).all() or not(bounds[0]<bounds[1] and bounds[2]<bounds[3]) or (bounds[1]-bounds[0])*(bounds[3]-bounds[2])>100:raise ValueError('INVALID_MANUAL_FLOOR_BOUNDS')
  if map_sha256 is not None and cfg['map_voxel_sha256']!=map_sha256:raise ValueError('MANUAL_FLOOR_MAP_MISMATCH')
 return cfg

def fingerprint(cfg):return hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()).hexdigest()

def candidate_mask(x,y,pose,cfg):
 if not cfg.get('enabled'):return np.zeros(np.broadcast(x,y).shape,bool)
 a,b,c,d=cfg['bounds_map_m'];s,t=np.sin(pose[2]),np.cos(pose[2]);mx=pose[0]+t*x-s*y;my=pose[1]+s*x+t*y
 return (mx>=a)&(mx<=b)&(my>=c)&(my<=d)

def display_points(cfg,plane,pose,step=.10):
 if not cfg.get('enabled'):return np.empty((0,3))
 a,b,c,d=cfg['bounds_map_m'];x,y=np.meshgrid(np.arange(a+step/2,b,step),np.arange(c+step/2,d,step));p=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)];p[:,2]=np.c_[p[:,:2],np.ones(len(p))]@plane
 from saved_map import base_transform
 T=base_transform(pose);return (p-T[:3,3])@T[:3,:3]

def update(enabled,bounds=None):
 # Serialize against every navigation start, including the probing stage.
 with exclusive(ROOT/'runtime/navigation_execution.lock'),exclusive(ROOT/'runtime/navigation.lock'):
  from targets import get
  cfg=settings();cfg.update(enabled=bool(enabled),revision=cfg.get('revision',0)+1,map_voxel_sha256=get(1)['map_voxel_sha256'],source='USER_MANUAL_GROUND_ASSUMPTION',updated_wall_time=time.time(),clears_occupied_points=False)
  if bounds is not None:cfg['bounds_map_m']=list(map(float,bounds))
  elif 'bounds_map_m' not in cfg:cfg['bounds_map_m']=[-1.55,3.,-3.25,1.65]
  a,b,c,d=cfg['bounds_map_m']
  if not all(np.isfinite([a,b,c,d])) or not(a<b and c<d) or (b-a)*(d-c)>100:raise ValueError('INVALID_MANUAL_FLOOR_BOUNDS')
  history=ROOT/'config/history';history.mkdir(exist_ok=True)
  if FILE.exists():atomic_json(history/f'manual_floor_{time.time_ns()}.json',settings())
  atomic_json(FILE,cfg);atomic_json(ROOT/'runtime/plan_state.json',dict(state='FLOOR_CHANGED_REPLAN_REQUIRED',execution_permission=False))
  return cfg

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['status','on','off']);p.add_argument('--bounds',nargs=4,type=float);a=p.parse_args();print(json.dumps(settings() if a.action=='status' else update(a.action=='on',a.bounds),ensure_ascii=False))

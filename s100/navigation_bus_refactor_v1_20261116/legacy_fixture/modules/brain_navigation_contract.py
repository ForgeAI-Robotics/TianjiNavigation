"""Coordinate contract and map binding. No ROS, controller, or task executor."""
import copy,hashlib,json,math,re,time
from datetime import datetime,timezone
from pathlib import Path
from navigation_task_contract import ApiError
VERSION='fq/reception-lan/v1'
FIELDS={'contract_version','task_id','command_id','target_id','route_phase','leg_index','frame_id','goal_xyt','motion_mode','require_final_orientation'}
IDENTITY=re.compile(r'[A-Za-z0-9_.:-]{1,160}\Z')
def stamp(value):return None if value is None else datetime.fromtimestamp(value,timezone.utc).isoformat()
def normalize(body):
 if not isinstance(body,dict)or set(body)!=FIELDS:raise ApiError(422,'INVALID_REQUEST','Exact navigation contract fields required')
 if body['contract_version']!=VERSION:raise ApiError(422,'UNSUPPORTED_CONTRACT_VERSION')
 for key in ['task_id','command_id']:
  if not isinstance(body[key],str)or not IDENTITY.fullmatch(body[key]):raise ApiError(422,'INVALID_REQUEST',key)
 if not isinstance(body['target_id'],str)or not body['target_id'].strip()or not isinstance(body['route_phase'],str):raise ApiError(422,'INVALID_REQUEST')
 if type(body['leg_index'])is not int or body['leg_index']<=0:raise ApiError(422,'INVALID_REQUEST','Positive leg_index required')
 if body['frame_id']!='map':raise ApiError(422,'INVALID_FRAME_ID')
 g=body['goal_xyt']
 if not isinstance(g,list)or len(g)!=3 or any(type(v)not in(int,float)or not math.isfinite(v)for v in g):raise ApiError(422,'INVALID_GOAL_XYT')
 if body['motion_mode']not in ('forward_path','lateral_path_aligned'):raise ApiError(422,'INVALID_MOTION_MODE')
 if body['require_final_orientation']is not True:raise ApiError(422,'INVALID_REQUEST','Final orientation is required')
 return copy.deepcopy(body)
def config(root):
 try:d=json.loads((Path(root)/'config/brain_compat.json').read_text())
 except (OSError,ValueError)as e:raise ApiError(503,'COMPATIBILITY_NOT_CONFIGURED',str(e))
 if not d.get('enabled'):raise ApiError(503,'COMPATIBILITY_DISABLED')
 if d.get('contract_version')!=VERSION:raise ApiError(503,'COMPATIBILITY_VERSION_MISMATCH')
 if d.get('supported_motion_modes')!=['forward_path']:raise ApiError(503,'INVALID_SUPPORTED_MODES','Only forward_path is implemented; lateral mode must not be advertised')
 speed=d.get('translation_cap_mps');timeout=d.get('timeout_s')
 if type(speed)not in(int,float)or not math.isfinite(speed)or not .005<=speed<=.10:raise ApiError(503,'INVALID_CONFIGURED_SPEED')
 if type(timeout)not in(int,float)or not math.isfinite(timeout)or not 0<=timeout<=3600:raise ApiError(503,'INVALID_CONFIGURED_TIMEOUT')
 return d

def world_binding(root,scene):
 cfg=config(root)
 if scene!=cfg['scene']:raise ApiError(409,'MAP_BINDING_UNAVAILABLE','API scene differs from configured external map')
 from navigation_task_contract import Waypoints
 refs=Waypoints(root).list(scene);ref=refs[0]
 if any(r['map_voxel_sha256']!=ref['map_voxel_sha256']or r.get('frame_id')!=ref.get('frame_id')or r.get('map_folder')!=ref.get('map_folder')for r in refs):raise ApiError(409,'MAP_BINDING_UNAVAILABLE','Mixed target-map identities')
 if cfg['map_sha256']!=ref['map_voxel_sha256']or cfg['internal_frame_id']!=ref.get('frame_id'):raise ApiError(409,'MAP_BINDING_UNAVAILABLE','Declared external map binding is stale')
 transform=cfg['T_internal_map_xyt']
 if len(transform)!=3 or any(type(x)not in(int,float)or not math.isfinite(x)for x in transform):raise ApiError(503,'INVALID_MAP_TRANSFORM')
 binding=dict(scene=scene,frame_id=ref['frame_id'],map_folder=ref['map_folder'],map_sha256=ref['map_voxel_sha256'],transform=transform,definition=cfg['map_definition'],config_sha256=hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()).hexdigest())
 return binding,refs,cfg

def coordinate_target(root,scene,body,simulation=False):
 binding,_,cfg=world_binding(root,scene)
 if body['motion_mode']not in cfg['supported_motion_modes']:raise ApiError(422,'INVALID_MOTION_MODE','Mode is not implemented by this navigation backend')
 if not simulation:
  folder=Path(binding['map_folder']);meta=json.loads((folder/'map.json').read_text());actual=hashlib.sha256((folder/'voxels.npz').read_bytes()).hexdigest()
  if meta.get('voxel_file_sha256')!=binding['map_sha256']or actual!=binding['map_sha256']:raise ApiError(409,'MAP_BINDING_UNAVAILABLE','Map assets changed')
 x,y,a=body['goal_xyt'];tx,ty,ta=binding['transform'];c,s=math.cos(ta),math.sin(ta)
 pose=[tx+c*x-s*y,ty+s*x+c*y,math.atan2(math.sin(ta+a),math.cos(ta+a))]
 return dict(id=0,name=body['target_id'],pose=pose,revision=1,map_folder=binding['map_folder'],map_voxel_sha256=binding['map_sha256'],frame_id=binding['frame_id'],scene_id=scene,source='API_COORDINATE_GOAL',coordinate_binding=binding,external_goal_xyt=body['goal_xyt'],motion_mode=body['motion_mode'],require_final_orientation=True)

def validate_bound_target(root,target):
 try:
  binding,_,_=world_binding(root,target['scene_id'])
  return target['source']=='API_COORDINATE_GOAL'and target['id']==0 and binding==target['coordinate_binding']
 except (ApiError,KeyError,OSError,ValueError):return False

def external_pose(target,pose):
 if not isinstance(pose,(list,tuple))or len(pose)!=3 or any(type(x)not in(int,float)or not math.isfinite(x)for x in pose):return None
 tx,ty,a=target['coordinate_binding']['transform'];dx,dy=pose[0]-tx,pose[1]-ty;c,s=math.cos(a),math.sin(a)
 return [c*dx+s*dy,-s*dx+c*dy,math.atan2(math.sin(pose[2]-a),math.cos(pose[2]-a))]

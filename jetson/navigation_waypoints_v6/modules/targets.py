"""Revisioned editable map goals; original numbered waypoints are immutable inputs."""
import copy,json,math,time,hashlib,os
from paths import ROOT,NAV
from common import atomic_json,exclusive
SCENES={'office':'办公室','outside':'外侧'}
def scene_id():
 value=os.environ.get('TIANJI_NAV_SCENE','outside')
 if value not in SCENES:raise ValueError('UNKNOWN_NAVIGATION_SCENE')
 return value
def scene_file(value):
 if value not in SCENES:raise ValueError('UNKNOWN_NAVIGATION_SCENE')
 return ROOT/'config/scenes/office_targets.json' if value=='office' else ROOT/'config/targets.json'
def scene_config(name):
 if '/' in name or not name.endswith('.json'):raise ValueError('INVALID_SCENE_CONFIG_NAME')
 return ROOT/'config/scenes'/('office_'+name) if scene_id()=='office' else ROOT/'config'/name
def select_scene(value):
 global FILE
 FILE=scene_file(value);os.environ['TIANJI_NAV_SCENE']=value
 return load()
FILE=scene_file(scene_id())

def initialize():
 if FILE.exists():return load()
 if scene_id()!='outside':raise ValueError('SCENE_TARGETS_NOT_CONFIGURED')
 points={}
 for n in (1,2):
  source=NAV/f'waypoints/point_{n:02d}.json';original=json.loads(source.read_text());p=original['global_map_pose']
  points[str(n)]=dict(id=n,name=original['name'],map_folder=p['map_folder'],map_voxel_sha256=p['map_voxel_sha256'],frame_id=p['frame_id'],pose=[p['position_m']['x'],p['position_m']['y'],math.radians(p['rpy_deg'][2])],source=str(source),revision=1,updated_wall_time=time.time())
 data=dict(schema_version=1,revision=1,targets=points,execution_enabled=False)
 atomic_json(FILE,data);return data

def validate(data):
 if set(data['targets']) not in ({'1','2'},{'1','2','3'}):raise ValueError('TARGET_IDS_1_2_OPTIONAL_3_REQUIRED')
 for key,row in data['targets'].items():
  if row['id']!=int(key) or len(row['pose'])!=3 or not all(type(x) in (int,float) and math.isfinite(x) for x in row['pose']):raise ValueError('INVALID_TARGET_POSE')
  if len(row['map_voxel_sha256'])!=64:raise ValueError('MAP_ID_REQUIRED')
 return data

def load():return validate(json.loads(FILE.read_text()))
def get(n):return copy.deepcopy(load()['targets'][str(int(n))])
def update(n,x,y,yaw_deg):
 with exclusive(ROOT/'runtime/targets.lock'):
  data=load();key=str(int(n))
  if key not in data['targets']:raise ValueError('TARGET_NOT_CONFIGURED_IN_SCENE')
  pose=[float(x),float(y),math.radians(float(yaw_deg))]
  if not all(math.isfinite(v) for v in pose):raise ValueError('NONFINITE_GOAL')
  old=copy.deepcopy(data);row=data['targets'][key];row.update(pose=pose,revision=row['revision']+1,updated_wall_time=time.time());data['revision']+=1
  validate(data);history=ROOT/'config/history';history.mkdir(exist_ok=True)
  atomic_json(history/f'targets_{time.time_ns()}.json',old);atomic_json(FILE,data)
  atomic_json(ROOT/'runtime/plan_state.json',dict(state='TARGET_CHANGED_REPLAN_REQUIRED',target_id=int(n),target_revision=row['revision'],execution_permission=False))
  return row

def unchanged(snapshot):
 current=get(snapshot['id'])
 return current['revision']==snapshot['revision'] and current['pose']==snapshot['pose'] and current['map_voxel_sha256']==snapshot['map_voxel_sha256']

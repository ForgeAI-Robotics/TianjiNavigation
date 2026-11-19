"""Lifetime/transport adapter around the unchanged read-only localization tracker.

No controller imports or motion publishers. Warm starts require fresh measured
pose identity plus NEW dual-lidar validation in global_localization.
"""
import json,os,signal,subprocess,sys,time,uuid
from pathlib import Path
import numpy as np
from common import ROOT,atomic_json,exclusive
from live_pose import read as read_pose
from sensor_bus import SnapshotStream
OWNER=ROOT/'runtime/resident_localization_owner.json'

def owner_alive(owner):
 try:
  args=Path('/proc',str(int(owner['pid'])),'cmdline').read_bytes().split(b'\0')
  script=str(ROOT/'modules/localization_tracker.py').encode();token=str(owner['token']).encode()
  i=args.index(b'--resident-token')
  return script in args and b'--resident'in args and args[i+1]==token
 except (OSError,ValueError,KeyError,TypeError,IndexError):return False

def current_owner():
 try:
  owner=json.loads(OWNER.read_text());folder=Path(owner['sensor_bus'])
  if folder.parent!=Path('/dev/shm') or folder.name!=f"tianji_navigation_resident_{owner['pid']}_{owner['token']}":return None
  return owner if owner_alive(owner)and folder.is_dir()else None
 except (OSError,ValueError,KeyError,TypeError):return None

class Handle:
 resident=True
 def __init__(self,owner):self.owner=dict(owner);self.pid=int(owner['pid'])
 def poll(self):
  if owner_alive(self.owner):return None
  owner=current_owner()
  if same_source(self.owner,owner):self.owner=dict(owner);self.pid=int(owner['pid']);return None
  return 1

def same_source(old,new):
 fields=('map_voxel_sha256','frame_id','source_boot_id','odometry_epoch','calibration_sha256')
 return bool(new is not None and all(k in old and k in new and old[k]==new[k]for k in fields))

class CaptureStream:
 def __init__(self,owner):
  self.owner=dict(owner);self.stream=SnapshotStream(owner['sensor_bus']);self.folder=Path(owner['sensor_bus']);self.seen_status=None;self.remote=self;self.stdout=self;self.channel=self
 def settimeout(self,value):self.stream.settimeout(value)
 def receive(self):
  try:return self._receive()
  except (EOFError,TimeoutError,FileNotFoundError):
   if owner_alive(self.owner):raise
   self._handover();return self._receive()
 def _handover(self):
  end=time.monotonic()+3.
  while time.monotonic()<end:
   owner=current_owner()
   if owner is not None:
    if not same_source(self.owner,owner):raise EOFError('RESIDENT_SENSOR_IDENTITY_CHANGED')
    self.stream.close();self.owner=dict(owner);self.folder=Path(owner['sensor_bus']);self.stream=SnapshotStream(self.folder);self.seen_status=None;return
   time.sleep(.05)
  raise EOFError('RESIDENT_LOCALIZATION_RESTART_TIMEOUT')
 def _receive(self):
  if not owner_alive(self.owner):raise EOFError('RESIDENT_LOCALIZATION_STOPPED')
  # Preserve the original relay diagnostic header. It never authorizes motion.
  file=self.folder/'relay_status.json'
  try:
   identity=file.stat().st_mtime_ns
   if identity!=self.seen_status:
    header=json.loads(file.read_text());self.seen_status=identity
    return header,None
  except FileNotFoundError:pass
  return self.stream.receive()
 def close(self):self.stream.close() # A task never stops the shared producer.

def capture_stream():
 owner=current_owner()
 if owner is None:return None
 try:packet=read_pose();elapsed=time.time()-packet['updated_wall_time']
 except (OSError,ValueError,KeyError,TypeError):return None
 if not packet.get('running')or packet.get('resident_session')!=owner['token']or not 0<=elapsed<=.4:return None
 return CaptureStream(owner)

def stop_owned(owner):
 if not owner_alive(owner):return
 os.kill(int(owner['pid']),signal.SIGINT)
 for sig,wait in [(None,8.),(signal.SIGTERM,3.),(signal.SIGKILL,1.)]:
  if sig is not None and owner_alive(owner):os.kill(int(owner['pid']),sig)
  end=time.monotonic()+wait
  while owner_alive(owner)and time.monotonic()<end:time.sleep(.05)
 if owner_alive(owner):raise RuntimeError('RESIDENT_LOCALIZATION_DID_NOT_STOP')

def ensure(reference,reinitialize=False):
 reference=Path(reference).resolve();result=json.loads(reference.read_text())
 if result.get('status')!='GLOBAL_LOCALIZED':raise ValueError('RESIDENT_REQUIRES_VALIDATED_LOCALIZATION')
 with exclusive(ROOT/'runtime/resident_localization_manager.lock'):
  owner=current_owner()
  if owner and owner['map_voxel_sha256']==result['map_voxel_sha256']and not reinitialize:return Handle(owner)
  if owner:stop_owned(owner)
  token=uuid.uuid4().hex;logpath=ROOT/'runtime/resident_localization.log';logpath.parent.mkdir(parents=True,exist_ok=True)
  from navigation_runtime_profile import LOCALIZATION_MATCH_PERIOD_S
  with logpath.open('ab')as log:
   process=subprocess.Popen([sys.executable,str(ROOT/'modules/localization_tracker.py'),'--reference',str(reference),'--resident','--resident-token',token,'--match-period-s',str(LOCALIZATION_MATCH_PERIOD_S)],stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True,close_fds=True)
  end=time.monotonic()+8.
  while time.monotonic()<end:
   owner=current_owner()
   if owner and owner['token']==token:
    try:packet=read_pose()
    except (OSError,ValueError):packet={}
    if packet.get('resident_session')==token and packet.get('running'):return Handle(owner)
   if process.poll()is not None:raise RuntimeError('RESIDENT_LOCALIZATION_START_FAILED:'+str(process.returncode))
   time.sleep(.05)
  # Stop only the process started by this call; never an unrelated owner.
  if process.poll()is None:process.terminate()
  raise RuntimeError('RESIDENT_LOCALIZATION_START_TIMEOUT')

def usable_pose(packet,owner,map_sha256,frame_id,header,now):
 """Same or stricter ages than position_check; never refresh a source stamp."""
 try:
  elapsed=now-float(packet['updated_wall_time']);age=elapsed+float(packet['odometry_age_s'])
  if not packet['running']or not packet['map_pose_valid']or packet['status']!='LIDAR_ODOMETRY':return False
  if not 0<=elapsed<=.4 or not 0<=age<=.5 or not 0<=float(packet['lidar_fix_age_s'])+elapsed<=3.:return False
  if packet.get('resident_session')!=owner['token']or packet.get('resident_pid')!=owner['pid']:return False
  if packet['map_voxel_sha256']!=map_sha256 or owner['map_voxel_sha256']!=map_sha256 or packet['frame_id']!=frame_id:return False
  for field,source in [('source_boot_id','boot_id'),('odometry_epoch','epoch'),('calibration_sha256','calibration_sha256')]:
   if packet[field]!=header[source]or owner[field]!=header[source]:return False
  T=np.asarray(packet['T_map_odom'],float)
  return bool(T.shape==(4,4)and np.isfinite(T).all()and np.allclose(T[3],[0,0,0,1])and np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-4)and np.isclose(np.linalg.det(T[:3,:3]),1.,atol=1e-4))
 except (KeyError,TypeError,ValueError):return False

def candidate(map_sha256,frame_id,header):
 owner=current_owner()
 if owner is None:return None
 try:packet=read_pose()
 except (OSError,ValueError):return None
 return (packet,owner)if usable_pose(packet,owner,map_sha256,frame_id,header,time.time())else None

def main():
 import argparse
 parser=argparse.ArgumentParser();group=parser.add_mutually_exclusive_group(required=True);group.add_argument('--status',action='store_true');group.add_argument('--stop',action='store_true');group.add_argument('--start',type=Path,metavar='VALIDATED_LOCALIZATION_JSON');args=parser.parse_args()
 if args.start:owner=ensure(args.start).owner
 else:owner=current_owner()
 if args.stop and owner:
  with exclusive(ROOT/'runtime/resident_localization_manager.lock'):stop_owned(owner)
  print('RESIDENT_LOCALIZATION_STOPPED');return
 print(json.dumps(dict(running=owner is not None,owner=owner,motion_commands_sent=0),ensure_ascii=False))
if __name__=='__main__':main()

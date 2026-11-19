"""Latest-only local raw sensor fan-out. No ROS publisher or control imports."""
import json,os,time,threading
from pathlib import Path
import numpy as np

class SensorBus:
 def __init__(self,folder):
  self.folder=Path(folder);self.folder.mkdir(mode=0o700,parents=True,exist_ok=False)
 def publish(self,header,points,arrival=None):
  kind=header.get('kind')
  if kind=='sensor_status':
   from common import atomic_json
   atomic_json(self.folder/'relay_status.json',dict(header));return
  if kind=='odometry':name='odometry'
  elif kind=='frame' and points is not None and header.get('sensor') in ['front','back']:name=header['sensor']
  else:return
  arrival=time.monotonic() if arrival is None else arrival
  metadata=dict(header=header,arrival_monotonic=arrival)
  path=self.folder/(name+'.npz');temporary=path.with_suffix('.tmp')
  p=np.empty((0,3),dtype=np.float32) if points is None else np.asarray(points,dtype=np.float32)
  if p.ndim!=2 or p.shape[1]!=3 or len(p)>100000 or not np.isfinite(p).all():raise ValueError('SENSOR_BUS_INVALID_POINTS')
  with temporary.open('wb') as f:np.savez(f,metadata=np.array(json.dumps(metadata,allow_nan=False)),points=p)
  os.replace(temporary,path)

class SnapshotStream:
 def __init__(self,folder,names=('odometry','front','back')):
  self.names=tuple(names)
  if not self.names or len(set(self.names))!=len(self.names) or any(n not in ('odometry','front','back') for n in self.names):raise ValueError('INVALID_SENSOR_BUS_SUBSCRIPTION')
  self.folder=Path(folder);self.remote=self;self.stdout=self;self.channel=self;self.timeout=3.;self.stopped=threading.Event();self.seen={};self.next=0
 def settimeout(self,value):self.timeout=float(value)
 def receive(self):
  began=time.monotonic();names=self.names
  while not self.stopped.is_set():
   # Round-robin so a busy consumer still sees the slower cloud packets.
   for offset in range(len(names)):
    index=(self.next+offset)%len(names);name=names[index];file=self.folder/(name+'.npz')
    try:
     identity=file.stat().st_mtime_ns
     if self.seen.get(name)==identity:continue
     with file.open('rb') as raw:
      identity=os.fstat(raw.fileno()).st_mtime_ns
      if self.seen.get(name)==identity:continue
      with np.load(raw,allow_pickle=False) as f:meta=json.loads(str(f['metadata']));p=np.array(f['points'],copy=True)
    except FileNotFoundError:continue
    h=dict(meta['header']);age=time.monotonic()-meta['arrival_monotonic']
    if not 0<=age<=1.2:continue
    if p.ndim!=2 or p.shape[1]!=3 or len(p)>100000 or not np.isfinite(p).all():raise ValueError('SENSOR_BUS_INVALID_POINTS')
    self.seen[name]=identity;self.next=(index+1)%len(names)
    # Add actual transport delay. Never refresh the original scan/pose stamp.
    h['bus_queue_s']=age
    h['source_age_s']=float(h.get('source_age_s',0.))+age
    return h,p if h['kind']=='frame' else None
   if time.monotonic()-began>=self.timeout:raise TimeoutError('LOCAL_SENSOR_BUS_TIMEOUT')
   self.stopped.wait(.005)
  raise EOFError('local sensor bus closed')
 def close(self):self.stopped.set()

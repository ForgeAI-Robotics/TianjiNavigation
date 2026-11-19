"""One read-only scan/collision worker. Local RAM input/output, no ROS imports."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import argparse,ctypes,json,signal,time
from pathlib import Path
import numpy as np
from common import atomic_json
from prepared_frames import prepare,configuration_stamp
from snapshot_builder import local_model
from extrinsics import transforms
from sensor_bus import SnapshotStream
from continuous_surface_filter import SurfaceCache

def main():
 p=argparse.ArgumentParser();p.add_argument('--sensor',choices=['front','back'],required=True);p.add_argument('--bus',type=Path,required=True);p.add_argument('--model-sha',required=True);p.add_argument('--conservative-fast',action='store_true');p.add_argument('--refine-near-hits',action='store_true');p.add_argument('--refine-radius-m',type=float,default=.065);a=p.parse_args()
 ctypes.CDLL(None).prctl(1,signal.SIGTERM)
 if os.getppid()==1:return
 stream=SnapshotStream(a.bus,names=(a.sensor,));stopped=False;status_file=a.bus/('filter_status_'+a.sensor+'.json');last_status=0.;last_error=None;count=0
 def halt(*_):
  nonlocal stopped
  stopped=True;stream.close()
 for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,halt)
 def status(state):atomic_json(status_file,dict(state=state,pid=os.getpid(),updated_monotonic=time.monotonic(),error=last_error,frames=count))
 surface_cache=SurfaceCache()
 status('WARMING');m=local_model();M=transforms();stamp=configuration_stamp()
 if m.sha256!=a.model_sha:raise ValueError('FILTER_WORKER_MODEL_MISMATCH')
 def publish(h,points,arrival,data):
  metadata=dict(header=h,arrival_monotonic=arrival,model_sha256=m.sha256,report=data['report'])
  target=a.bus/('prepared_'+a.sensor+'.npz');temporary=target.with_suffix('.tmp')
  with temporary.open('wb') as f:np.savez(f,metadata=np.array(json.dumps(metadata,allow_nan=False)),points_body=data['points_body'],points_sensor=points)
  os.replace(temporary,target)
 try:
  while not stopped:
   h,points=stream.receive();arrival=time.monotonic()
   if h.get('kind')=='frame' and h.get('sensor')==a.sensor:
    if configuration_stamp()!=stamp:raise RuntimeError('FILTER_CONFIGURATION_CHANGED_REPLAN_REQUIRED')
    try:
     # SnapshotStream already added its actual RAM-queue delay to source_age.
     # Retaining that aged header and this arrival preserves the original age.
     request=None
     if a.conservative_fast and a.refine_near_hits:
      try:request=json.loads((a.bus/'refinement_request.json').read_text())
      except (OSError,ValueError):pass
     data=prepare(m,M,(h,points,arrival),conservative_fast=a.conservative_fast,
                  refine_near_hits=a.refine_near_hits,refine_radius_m=a.refine_radius_m,refine_request=request,surface_cache=surface_cache);last_error=None
     if not stopped:
      publish(h,points,arrival,data);count+=1
    except ValueError as exc:last_error=str(exc)
   if time.monotonic()-last_status>.2:status('READY' if count else 'WAIT_SCAN_POSTURE');last_status=time.monotonic()
 except EOFError:
  if not stopped:raise
 finally:stream.close();status('STOPPED')

if __name__=='__main__':main()

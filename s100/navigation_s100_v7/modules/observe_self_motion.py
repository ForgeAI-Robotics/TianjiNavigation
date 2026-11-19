"""Passive raw-LiDAR/odometry experiment. This module never sends velocity."""
import argparse,json,time,hashlib,os,signal
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from paths import ROOT
from client import SensorStream
from snapshot_builder import local_model
from chassis_proxy import replace_shapes,profile,measured_shapes
from saved_map import base_transform
from extrinsics import transforms
from pillar_cables import distances
from cable_zones import settings,centers
from motion_consistency import DualFrameObserver
from common import atomic_json

def measured_static_posture(header,model_sha256,now):
 # Rear scans arrive later than the arm-status file's sampling interval.
 # Use fresh measured joint feedback at receive time, then require that the
 # unchanged-posture interval covers the original scan timestamp. Keep that
 # timestamp/odometry transform intact; this is not a fresh scan relabel.
 from onboard_state import measured_joints
 try:packet=measured_joints()
 except (OSError,ValueError,KeyError):return None
 if packet.get('model_sha256')!=model_sha256 or not 0<=now-packet.get('observation_time',0)<=.35:return None
 if not -.02<=now-header['observation_time']<=.8:return None
 q=packet.get('joint_positions_rad',{})
 if any(f'Joint{i}_{side}' not in q for i in range(1,8) for side in 'LR'):return None
 return packet

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--duration',type=float,default=30.);parser.add_argument('--micro',action='store_true');parser.add_argument('--features',type=Path);parser.add_argument('--sensor-bus',type=Path);parser.add_argument('--features-only',action='store_true');parser.add_argument('--wait-features',action='store_true');args=parser.parse_args()
 if args.features_only and args.features is None:parser.error('--features-only requires --features')
 if not 5<=args.duration<=60:parser.error('duration must be 5..60 seconds')
 out=ROOT/'results'/('passive_motion_observation_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
 model=local_model();cfg=settings();matrices=transforms();calibration_hash=hashlib.sha256((ROOT/'config/lidar_calibration.json').read_bytes()).hexdigest()
 if args.sensor_bus:
  from sensor_bus import SnapshotStream
  stream=SnapshotStream(args.sensor_bus)
 else:stream=SensorStream(localization=True,consumer='observer')
 started=time.monotonic();observer=None;reference_joints=None;static_since=None;identity=None;rows=[];last={};reason='DURATION_COMPLETE';stopping=[False];sensor_counts={'front':0,'back':0}
 def stopped(*_):stopping[0]=True
 for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,stopped)
 def progress(ready):
  atomic_json(ROOT/'runtime/motion_observer_status.json',dict(parent_pid=os.getppid(),pid=os.getpid(),updated_at=time.time(),ready=bool(ready),frames=len(rows),source_identity=identity,model_sha256=model.sha256,raw_unfiltered_input=True,motion_commands_sent=0,folder=str(out)))
  if observer is not None:
   evidence=observer.summary();evidence.update(parent_pid=os.getppid(),updated_at=time.time(),points_body_m=predictions.tolist(),reference_joints=reference_pose,source_identity=identity,model_sha256=model.sha256,raw_unfiltered_input=True,raw_capture_folder=str(out),error=reason if reason!='DURATION_COMPLETE' else None)
   atomic_json(out/'evidence.json',evidence)
 progress(False)
 try:
  while time.monotonic()-started<args.duration and not stopping[0]:
   h,p=stream.receive()
   if h.get('kind')!='frame' or p is None:continue
   if h.get('raw_point_frame')!='sensor' or h.get('calibration_sha256')!=calibration_hash:raise ValueError('RAW_INPUT_FRAME_OR_CALIBRATION_CHANGED')
   packet=measured_static_posture(h,model.sha256,time.time())
   if packet is None:continue
   q=packet['joint_positions_rad']
   current_identity=(h['boot_id'],h['epoch'],h['calibration_sha256'])
   if identity is None:identity=current_identity
   if identity!=current_identity:raise ValueError('ODOMETRY_RESET_RESTART_EXPERIMENT')
   if reference_joints is None:reference_joints={k:v for k,v in q.items() if k.startswith('Joint')};static_since=packet['observation_time']
   if any(abs(q[k]-v)>np.deg2rad(.5) for k,v in reference_joints.items()):raise ValueError('ARMS_MOVED_RESTART_EXPERIMENT')
   if h['observation_time']<static_since:continue
   if observer is None and args.wait_features and args.features and not args.features.exists():progress(False);continue
   if h['observation_time']-last.get(h['sensor'],0.)<.8:continue
   last[h['sensor']]=h['observation_time'];T=matrices[h['sensor']];body=p@T[:3,:3].T+T[:3,3];body=body[np.isfinite(body).all(1)]
   if observer is None:
    reference_pose=dict(q)
    shapes=measured_shapes(model,q);known,status=centers(shapes,cfg,model.sha256)
    if status!='FROZEN_USER_CABLE_ZONES' or args.features_only:known=np.empty((0,3))
    if args.features:
     with np.load(args.features) as f:requested=f['points_body']
     if args.features_only:known=np.asarray(requested,float).reshape(-1,3)
     elif len(requested) and len(body):
      d,ids=cKDTree(body).query(requested);known=np.vstack([known,body[ids[d<=.06]]])
    arms=[s for s in shapes if s['mode']=='measured_or_fixed' and s['link'].startswith(tuple(f'Link{i}_' for i in range(1,8)))]
    gap=distances(body,arms);candidates=body[(gap>.025)&(gap<=.15)&(body[:,2]>.75)&(body[:,2]<1.25)]
    if len(candidates) and not args.features_only:
     _,ids=np.unique(np.floor(candidates/.03).astype(int),axis=0,return_index=True);ids=ids[np.linspace(0,len(ids)-1,min(64,len(ids)),dtype=int)];known=np.vstack([known,candidates[ids]])
    if len(known)>256:known=known[:256]
    predictions=known.copy();observer=DualFrameObserver(len(predictions),tolerance_m=.01,minimum_frames=8,minimum_translation_m=.05,minimum_second_axis_m=0.) if args.micro else DualFrameObserver(len(predictions))
    atomic_json(out/'features.json',dict(points_body_m=predictions.tolist(),scope='fixed-arm posture, user-confirmed near-body cable region; unknown is not deletion proof'))
   file=out/f'raw_{len(rows):03d}_{h["sensor"]}.npz';np.savez_compressed(file,points_sensor=p)
   rows.append(dict(header=h,verified_static_joints=packet,static_posture_since=static_since,file=str(file)));observer.observe(body,predictions,base_transform(h['pose']),h['observation_time'],h['sensor'])
   sensor_counts[h['sensor']]+=1;progress(min(sensor_counts.values())>=1)
   print(json.dumps(dict(phase='OBSERVING_RAW_INPUT_ONLY',elapsed_s=time.monotonic()-started,frames=len(rows),motion_commands_sent=0)),flush=True)
 except Exception as exc:reason=type(exc).__name__+': '+str(exc)
 finally:stream.close();progress(False)
 report=observer.summary() if observer is not None else dict(features=[],tracks=[])
 if reason=='DURATION_COMPLETE' and min(sensor_counts.values())==0:reason='BOTH_RAW_LIDARS_REQUIRED'
 report.update(reason=reason,valid=reason=='DURATION_COMPLETE',raw_capture_folder=str(out),source_identity=identity,model_sha256=model.sha256,raw_unfiltered_input=True,frames=len(rows),automatic_point_deletion=False,motion_commands_sent=0,command_output_connected=False)
 if observer is not None:report.update(points_body_m=predictions.tolist(),reference_joints=reference_pose)
 atomic_json(out/'acquisition.json',dict(rows=rows,motion_commands_sent=0));atomic_json(out/'result.json',report);print(json.dumps(report,ensure_ascii=False),flush=True)

if __name__=='__main__':main()

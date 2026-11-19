"""On-S100 positioning, path-following and live collision integration; ZERO actuation."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import sys,json,time,math,threading,subprocess,signal,resource,argparse
from pathlib import Path
import numpy as np
from paths import ROOT
from common import atomic_json,exclusive
from client import SensorStream
from onboard_state import measured_joints,readiness,model
from follower import PreviewFollower
from execution_safety import posture_check,position_check,sweep_guard
from extrinsics import transforms
from chassis_proxy import replace_shapes,profile

def run(duration):
 lock=exclusive(ROOT/'runtime/onboard_dry_run.lock');folder=Path(json.loads((ROOT/'runtime/latest_plan.json').read_text())['folder']);record=json.loads((folder/'scene/frozen_input.json').read_text());plan=json.loads((folder/'planning/plan.json').read_text())
 if not plan.get('selected'):raise ValueError('CHECKED_PATH_REQUIRED')
 if record['localization'].get('offline_replay'):raise ValueError('FRESH_LOCALIZATION_REQUIRED')
 follower=PreviewFollower(plan,json.loads((ROOT/'config/motion_policy.json').read_text()));stop=threading.Event();frames={};mutex=threading.Lock();errors=[];stream=None;reader=None;tracker=None;samples=[];began=time.monotonic();cpu=time.process_time()
 log=(ROOT/'runtime/onboard_tracking.log').open('w');matrices=transforms();m=model()
 def receive():
  try:
   while not stop.is_set():
    h,p=stream.receive()
    if h.get('kind')=='frame' and p is not None:
     expected=record['localization']
     if h['boot_id']!=expected['source_boot_id'] or h['epoch']!=expected['odometry_epoch'] or h['calibration_sha256']!=expected['calibration_sha256']:raise ValueError('FRAME_IDENTITY_CHANGED')
     with mutex:frames[h['sensor']]=(h,p,time.monotonic())
  except Exception as exc:
   if not stop.is_set():errors.append(str(exc))
 def halt(*_):stop.set()
 for s in (signal.SIGINT,signal.SIGTERM):signal.signal(s,halt)
 try:
  tracker=subprocess.Popen([sys.executable,str(ROOT/'modules/localization_tracker.py'),'--reference',str(folder/'localization/localization.json'),'--duration',str(duration+3)],stdout=log,stderr=log)
  stream=SensorStream();reader=threading.Thread(target=receive,daemon=True);reader.start()
  while not stop.is_set() and time.monotonic()-began<duration:
   started=time.monotonic();now=time.time();pkt={}
   try:pkt=json.loads((ROOT/'runtime/current_pose.json').read_text())
   except (OSError,ValueError):pass
   now=time.time();pos,reason,age=position_check(pkt,record,now);decision=dict(state=reason,twist_preview=[0.,0.,0.],execution_permission=False);geometric=False;why='WAIT_LOCALIZATION';pose_ok=False;head_count=0
   if pos:
    T=np.array(pkt['T_map_base']);pose=[T[0,3],T[1,3],math.atan2(T[1,0],T[0,0])]
    suggested=follower.step(pose,now-age,now,plan['target']['revision'],True,True)
    try:
     q=measured_joints();head_count=len(q['joint_positions_rad']);pose_ok,pose_reason,_=posture_check(record['saved_pose'],q,m.sha256,time.time())
     shapes=replace_shapes(m.shapes(q['joint_positions_rad']),profile(m));a,b,c=record['ground_plane'];normal=T[:3,:3].T@np.array([-a,-b,1.]);offset=np.array([-a,-b,1.])@T[:3,3]-c;floor=[-normal[0]/normal[2],-normal[1]/normal[2],-offset/normal[2]]
     with mutex:copy=dict(frames)
     geometric,why,cloud_age=sweep_guard(copy,matrices,pkt['odometry_pose'],shapes,m.sha256,floor,suggested['twist_preview'],time.monotonic())
     decision=dict(suggested);decision['state']='DRY_RUN_ONLY';decision['candidate_twist']=decision.pop('twist_preview');decision['twist_preview']=[0.,0.,0.]
     decision['pose_reason']=pose_reason
    except Exception as exc:why=str(exc)
   release=readiness()
   guard_ready=bool(pos and pose_ok and geometric and release['clearance_clear'] and release['shapemask_clear'] and not errors)
   decision['admission_state']='ALL_GUARDS_READY_TEST' if guard_ready else 'OTHER_GUARDS_INHIBIT'
   decision['clearance_release_state']=release['release_state']
   row=dict(clearance_release_state=release['release_state'],effective_clearance_clear=release['clearance_clear'],effective_shapemask_clear=release['shapemask_clear'],raw_clearance_clear=release['raw_clearance_clear'],raw_shapemask_clear=release['raw_shapemask_clear'],elapsed_s=time.monotonic()-began,position_valid=pos,position_reason=reason,pose_valid=pose_ok,joint_count=head_count,geometric_clear=geometric,geometry_reason=why,decision=decision,compute_s=time.monotonic()-started,motion_commands_sent=0)
   samples.append(row);atomic_json(ROOT/'runtime/onboard_pipeline_status.json',dict(row,active=True,compute_host='S100',execution_permission=False));stop.wait(max(0.,.2-(time.monotonic()-started)))
 finally:
  stop.set()
  if stream:stream.close()
  if reader:reader.join(timeout=4)
  if tracker and tracker.poll() is None:
   tracker.send_signal(signal.SIGINT)
   try:tracker.wait(timeout=8)
   except subprocess.TimeoutExpired:tracker.terminate();tracker.wait(timeout=3)
  log.close()
 children=resource.getrusage(resource.RUSAGE_CHILDREN)
 report=dict(child_cpu_s=children.ru_utime+children.ru_stime,compute_host='S100',mode='READ_ONLY_PIPELINE_NO_ACTUATOR_IMPORTS',folder=str(folder),wall_s=time.monotonic()-began,python_cpu_s=time.process_time()-cpu,samples=len(samples),position_valid_samples=sum(r['position_valid'] for r in samples),geometry_clear_samples=sum(r['geometric_clear'] for r in samples),max_compute_s=max((r['compute_s'] for r in samples),default=0),readiness=readiness(),errors=errors,motion_commands_sent=0,execution_permission=False)
 atomic_json(ROOT/'runtime/onboard_pipeline_status.json',dict(report,active=False));atomic_json(folder/'onboard_pipeline_report.json',report);atomic_json(folder/'onboard_pipeline_samples.json',samples);print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=15);args=p.parse_args();run(args.duration)

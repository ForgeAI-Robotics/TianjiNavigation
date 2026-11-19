"""Background self filtering in each scan's measured body frame.

Raw timestamps/arrival times stay unchanged. Collision checks and all output
leases still run on the caller's fresh measurements. No ROS/control objects.
"""
import hashlib,json,time,sys,subprocess,os
from pathlib import Path
import numpy as np
from paths import ROOT
from chassis_proxy import measured_shapes
from navigation_self_filter import evaluate,refine_sparse
from execution_safety import REQUIRED_JOINTS,translation_sweep_shapes
from environment import self_candidates
from gripper_posture_policy import valid_scan_evidence

def configuration_stamp():
 digest=hashlib.sha256()
 for name in ['self_filter_envelope.json','temporary_pillar_cables.json','cable_zones.json','near_body_cleanup.json','continuous_surface_filter.json','isolated_returns.json','lidar_calibration.json','gripper_posture_policy.json']:
  p=ROOT/'config'/name;digest.update(name.encode());digest.update(p.read_bytes() if p.exists() else b'ABSENT')
 p=ROOT/'runtime/confirmed_motion_cables.json';digest.update(p.read_bytes() if p.exists() else b'ABSENT')
 return digest.hexdigest()

def prepare(model,matrices,frame,*,conservative_fast=False,refine_near_hits=False,refine_radius_m=.065,refine_request=None,surface_cache=None):
 if not np.isfinite(refine_radius_m) or not .05<=refine_radius_m<=.16:raise ValueError('INVALID_REFINEMENT_RADIUS')
 h,raw,arrival=frame;j=h.get('joints',{});q=j.get('joint_positions_rad',{})
 if not valid_scan_evidence(j,model.sha256):raise ValueError('SCAN_JOINT_EVIDENCE_REQUIRED')
 stamp=float(j.get('observation_time',0.));scan=float(h['observation_time'])
 if not np.isfinite([stamp,scan]).all() or abs(stamp-scan)>=.3:raise ValueError('SCAN_JOINT_EVIDENCE_STALE')
 if any(k not in q or not np.isfinite(q[k]) for k in REQUIRED_JOINTS):raise ValueError('FULL_SCAN_JOINT_EVIDENCE_REQUIRED')
 began=time.monotonic();cpu_began=time.process_time();shapes=measured_shapes(model,q);T=matrices[h['sensor']];p=np.asarray(raw)@T[:3,:3].T+T[:3,3]
 corners=np.concatenate([s['world_corners'] for s in shapes]);p=p[np.isfinite(p).all(1)&((p>=corners.min(0)-.60)&(p<=corners.max(0)+.60)).all(1)]
 geometry_done=time.monotonic()
 result=evaluate(p,shapes,model.sha256,conservative_fast=conservative_fast,surface_cache=surface_cache,frame_header=h);kept=p[~result['filtered']]
 base_done=time.monotonic()
 # The short micro preview can trigger just beyond the static 5cm band.
 # Use this radius only to decide whether to run the full filter.
 if conservative_fast and refine_near_hits and not result.get('classification_complete',False):
  interest=np.zeros(len(p),bool)
  selected,interest_mode=refinement_interest(kept,shapes,h,refine_radius_m,refine_request)
  interest[~result['filtered']]=selected
  if interest.any():
   refine_sparse(p,shapes,model.sha256,result,interest=interest)
   kept=p[~result['filtered']]
 refine_done=time.monotonic()
 scan_hits=self_candidates(kept,shapes,.05)
 report=dict(refinement_interest_mode='CONTINUOUS_SURFACE_PRIORITY' if result.get('classification_complete') else interest_mode if conservative_fast and refine_near_hits else 'FULL',filter_cpu_s=time.process_time()-cpu_began,filter_stages_s=dict(geometry=geometry_done-began,base_filter=base_done-geometry_done,sparse_refinement=refine_done-base_done,final_hit_check=time.monotonic()-refine_done),points=len(p),filtered=int(result['filtered'].sum()),protected=int(result['protected'].sum()),scan_pose_collision_points=int(scan_hits.sum()),cable_report=result['cable_report'],filter_compute_s=time.monotonic()-began,filter_frame='SCAN_BODY_BEFORE_ODOMETRY_ALIGNMENT',joint_observation_time=stamp)
 return dict(frame=frame,points_body=kept,report=report,model_sha256=model.sha256)

def refinement_interest(points,shapes,header,radius,request):
 """Select classification work only; every unselected return stays retained."""
 if request is not None:
  try:
   twist=np.asarray(request['twist'],float);pose=np.asarray(request['odometry_pose'],float)
   age=time.monotonic()-float(request['written_monotonic'])
   valid=twist.shape==(3,) and pose.shape==(3,) and np.isfinite(np.r_[twist,pose,age]).all()
   if valid and 0<=age<=1.2 and abs(twist[2])<1e-8 and np.linalg.norm(twist[:2])<=.10+1e-9:
    angle=pose[2]-float(header['pose'][2]);c,s=np.cos(angle),np.sin(angle)
    velocity=[c*twist[0]-s*twist[1],s*twist[0]+c*twist[1],0.]
    # Max admitted age + future sweep + measured advance certificate.
    swept=translation_sweep_shapes(shapes,velocity,horizon=2.05)
    return self_candidates(points,swept,.065),'CURRENT_TRANSLATION_SWEEP'
  except (KeyError,TypeError,ValueError):pass
 return self_candidates(points,shapes,radius),'ALL_DIRECTIONS_FALLBACK'

def latency_summary(prepared,now):
 """Report actual scan age even when admission failed; never renew evidence."""
 result={}
 for sensor,data in prepared.items():
  h,_,arrival=data['frame'];r=data['report'];compute=float(r.get('filter_compute_s',0.))
  result[sensor]=dict(sequence=h['sequence'],observation_time=h.get('observation_time'),
   source_age_at_filter_start_s=float(h.get('source_age_s',0.)),
   raw_bus_queue_s=float(h.get('bus_queue_s',0.)),filter_compute_s=compute,
   filter_cpu_s=r.get('filter_cpu_s'),filter_stages_s=r.get('filter_stages_s'),refinement_interest_mode=r.get('refinement_interest_mode'),
   compute_host=r.get('compute_host','S100'),filter_backend=r.get('filter_backend','cpu'),round_trip_s=r.get('round_trip_s'),remote_filter_compute_s=r.get('remote_filter_compute_s'),remote_filter_cpu_s=r.get('remote_filter_cpu_s'),request_bytes=r.get('request_bytes'),response_bytes=r.get('response_bytes'),
   after_filter_wait_s=max(0.,now-arrival-compute),
   source_age_now_s=float(h.get('source_age_s',0.))+now-arrival)
 return result

class PreparedFrames:
 def __init__(self,model,folder,*,conservative_fast=False,refine_near_hits=False,refine_radius_m=.065):
  self.folder=Path(folder);self.latest={};self.seen={};self.children={};self.logs=[];self.closed=False;self.final_health=None
  try:
   for sensor in ['front','back']:
    log=(self.folder/('filter_'+sensor+'.log')).open('ab');self.logs.append(log)
    self.children[sensor]=subprocess.Popen([sys.executable,str(ROOT/'modules/prepared_worker.py'),'--sensor',sensor,'--bus',str(self.folder),'--model-sha',model.sha256]+(['--conservative-fast'] if conservative_fast else [])+(['--refine-near-hits','--refine-radius-m',str(refine_radius_m)] if refine_near_hits else []),stdout=log,stderr=log)
  except Exception:
   self.close();raise
 @property
 def error(self):
  if self.closed:return None
  for sensor,child in self.children.items():
   if child.poll() is not None:return 'FILTER_WORKER_EXITED_'+sensor
  return None
 def request_refinement(self,twist,odometry_pose):
  from common import atomic_json
  atomic_json(self.folder/'refinement_request.json',dict(twist=list(twist),odometry_pose=list(odometry_pose),written_monotonic=time.monotonic()))
 def has_unread(self):
  for sensor in ('front','back'):
   try:
    stamp=(self.folder/('prepared_'+sensor+'.npz')).stat().st_mtime_ns
   except FileNotFoundError:
    continue
   if self.seen.get(sensor)!=stamp:return True
  return False
 def snapshot(self):
  for sensor in ['front','back']:
   file=self.folder/('prepared_'+sensor+'.npz')
   try:
    stamp=file.stat().st_mtime_ns
    if self.seen.get(sensor)==stamp:continue
    with file.open('rb') as raw:
     stamp=os.fstat(raw.fileno()).st_mtime_ns
     with np.load(raw,allow_pickle=False) as z:
      meta=json.loads(str(z['metadata']));body=np.array(z['points_body']);original=np.array(z['points_sensor'])
    if meta['header']['sensor']!=sensor:raise ValueError('PREPARED_SENSOR_MISMATCH')
    self.latest[sensor]=dict(frame=(meta['header'],original,meta['arrival_monotonic']),points_body=body,report=meta['report'],model_sha256=meta['model_sha256']);self.seen[sensor]=stamp
   except FileNotFoundError:continue
  return dict(self.latest)
 def health(self):
  if self.closed:return self.final_health or dict(error=None,workers={})
  result=dict(error=self.error,workers={})
  for sensor,child in self.children.items():
   try:result['workers'][sensor]=json.loads((self.folder/('filter_status_'+sensor+'.json')).read_text())
   except (FileNotFoundError,ValueError):result['workers'][sensor]=dict(state='WARMING',pid=child.pid)
  return result
 def close(self):
  if self.closed:return
  self.final_health=self.health();self.final_health['error']=None
  for value in self.final_health['workers'].values():value['state']='STOPPED'
  self.closed=True
  for child in self.children.values():
   if child.poll() is None:child.terminate()
  for child in self.children.values():
   try:child.wait(timeout=4)
   except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
  for log in self.logs:log.close()

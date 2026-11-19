"""S100 navigator. Hardware bridge is lazy-started only by explicit manual launch.
--preview validates the complete command chain without creating any ROS publisher.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import argparse,json,sys,time,math,threading,signal,subprocess,select,hashlib,resource,ctypes,collections
from pathlib import Path
import numpy as np
from paths import ROOT
from common import atomic_json,exclusive
from async_diagnostics import AsyncDiagnostics
from live_pose import read as read_live_pose
from controller_extension import ensure as ensure_controller_extension,healthy as controller_extension_healthy
from client import SensorStream
from onboard_state import measured_joints,readiness,model,state,feedback_health
from clearance_override import settings as override_settings
from navigation_control import ControlTransport,limited_twist,TRANSLATION_CAP_MPS
from follower import PreviewFollower
from execution_safety import posture_check,position_check,sweep_guard,trajectory_deviation,sweep_reference_drift
from extrinsics import transforms
from saved_map import base_transform
from chassis_proxy import replace_shapes,profile,measured_shapes
import targets
from sensor_bus import SensorBus
from guard_recovery import GuardRecovery,expired_deadlines,waitable_data_fault
from sensor_recovery import SensorRecovery
from posture_lease import revalidate as revalidate_posture,motion_weights,joint_shift_bound
from posture_lease import corner_shift
from motion_certificate import certify_motion,BASE_ADVANCE_S,FUTURE_HORIZON_S
from certification_recovery import classify as classify_pose_wait,POSE_RECHECK
from obstacle_wait import ObstacleWait,BLOCKING_REASONS
from execution_metrics import signature,summarize
from operator_session import OperatorSession
from planning_process import run as isolated_plan

from navigation_runtime_profile import RAW_SCAN_HZ,LOCALIZATION_MATCH_PERIOD_S,CONTROL_CHECK_PERIOD_S

class PlanningBlocked(Exception):pass

class MapGuard:
 def __init__(self,file):self.sequence=0;self.failed=False;self.child=subprocess.Popen([str(ROOT/'native/world_guard'),str(file)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,bufsize=1)
 def check(self,local_pose,twist):
  if self.failed:return False,'MAP_GUARD_FAILED'
  self.sequence+=1
  x,y,yaw=local_pose;vx,vy,w=twist;poses=[]
  for t in np.linspace(0,.5,6):
   a=w*t
   dx,dy=(vx*t,vy*t) if abs(w)<1e-8 else ((vx*math.sin(a)+vy*(math.cos(a)-1))/w,(vx*(1-math.cos(a))+vy*math.sin(a))/w)
   poses.append([x+math.cos(yaw)*dx-math.sin(yaw)*dy,y+math.sin(yaw)*dx+math.cos(yaw)*dy,yaw+a])
  self.child.stdin.write(json.dumps(dict(id=self.sequence,poses=poses))+'\n');self.child.stdin.flush()
  if not select.select([self.child.stdout],[],[],.4)[0]:self.failed=True;return False,'MAP_GUARD_TIMEOUT'
  r=json.loads(self.child.stdout.readline())
  if r.get('id')!=self.sequence:self.failed=True;return False,'MAP_GUARD_RESPONSE_MISMATCH'
  return (True,'OK') if r.get('clear') else (False,r.get('link') or r.get('error','MAP_COLLISION'))
 def close(self):
  self.child.stdin.close()
  try:self.child.wait(timeout=1)
  except subprocess.TimeoutExpired:self.child.terminate();self.child.wait(timeout=2)


def run(args):
 if getattr(args,'enable_motion',False) and (ROOT/'ISOLATED_CONTRACT_TEST_ROOT').exists():raise ValueError('ISOLATED_ROOT_CANNOT_ENABLE_PHYSICAL_MOTION')
 load_test=bool(getattr(args,'load_test_no_motion',False))
 if load_test and args.enable_motion:raise ValueError('LOAD_TEST_CANNOT_ENABLE_MOTION')
 right_micro=bool(getattr(args,'right_micro_preview',False))
 if right_micro and (args.enable_motion or args.max_travel_m or args.cable_precheck):raise ValueError('RIGHT_MICRO_PREVIEW_ONLY')
 micro_limit=.05 if right_micro else float(getattr(args,'max_travel_m',0.))
 cable_precheck=bool(getattr(args,'cable_precheck',False))
 requested_cap=getattr(args,'translation_cap_mps',None)
 if requested_cap is not None and (micro_limit or cable_precheck or not .005<=requested_cap<=TRANSLATION_CAP_MPS):
  raise ValueError('INVALID_TRANSLATION_SPEED_CAP')
 from near_body_cleanup import settings as cleanup_settings
 if (cable_precheck or micro_limit and not right_micro) and not cleanup_settings().get('micro_probe_enabled',False):raise ValueError('MICRO_PROBE_DISABLED_SNAPSHOT_ONLY')
 if micro_limit and not .02<=micro_limit<=.05:raise ValueError('MICRO_TRAVEL_LIMIT_MUST_BE_2_TO_5_CM')
 if cable_precheck:micro_limit=.06 # 5.2cm observation target plus 8mm stop reserve.
 resident_motor_trace=[None];frame_worker=None;micro=None;observer_child=None;observer_log=None;last_micro_cloud=None;sensor_bus=None;wait_reasons=collections.Counter();last_wait_reason=None;probe_deferral=None;recovery=GuardRecovery();route_recovery=None;active_plan_folder=None
 code_signature=signature(ROOT)
 execution_cpu_affinity=sorted(os.sched_getaffinity(0))
 obstacle_wait=ObstacleWait();sensor_recovery=SensorRecovery();controller_extension_state={}
 lock=exclusive(ROOT/'runtime/navigation_execution.lock');stop=threading.Event();frames={};mutex=threading.Lock();error=[];stream=None;reader=None;tracker=None;transport=None;map_guard=None;record=None;folder=None;sent_nonzero=0;began=time.monotonic();phase='PREPARING';final_details={};last_phase=None;last_print=0.;ready_count=0;was_admitted=False;history=[];relay_health={};last_collision_capture=0.
 lease=OperatorSession(args.session,stop,began)
 def halt(signum,*_):lease.fail('PROCESS_SIGNAL_'+signal.Signals(signum).name,time.monotonic())
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,halt)
 def operator_reader():
  try:
   for line in sys.stdin:
    lease.receive(json.loads(line),time.monotonic())
    if stop.is_set():return
   if not stop.is_set():lease.fail('OPERATOR_INPUT_CLOSED',time.monotonic())
  except (ValueError,OSError) as exc:lease.fail('OPERATOR_INPUT_ERROR:'+type(exc).__name__,time.monotonic())
 threading.Thread(target=operator_reader,daemon=True).start()
 def operator_watchdog():
  while not stop.wait(.05):lease.check(time.monotonic())
 threading.Thread(target=operator_watchdog,daemon=True).start()
 def write_status(item):
  r,audit_folder=item
  atomic_json(ROOT/'runtime/navigation_execution_status.json',r)
  if audit_folder:
   with (audit_folder/'navigation_execution_history.jsonl').open('a')as audit:audit.write(json.dumps(r,ensure_ascii=False,allow_nan=False)+'\n')
 def emit_status(r):
  try:print('STATUS '+json.dumps(r,ensure_ascii=False),flush=True)
  except (BrokenPipeError,OSError):lease.fail('OPERATOR_OUTPUT_CLOSED',time.monotonic())
 diagnostic_io=AsyncDiagnostics(write_status)
 console_io=AsyncDiagnostics(emit_status,capacity=16)
 def status(phase,**extra):
  nonlocal last_phase,last_print
  r=dict(active=not stop.is_set() and not phase.startswith('STOP') and phase not in ['ARRIVED','TEST_COMPLETE','PLAN_BLOCKED'],pid=os.getpid(),session=args.session,mode='LOAD_TEST_NO_MOTION' if load_test else 'REAL_MANUAL_START' if args.enable_motion else 'PREVIEW_NO_ROS',translation_cap_mps=requested_cap,phase=phase,compute_host='S100',updated_wall_time=time.time(),micro_maximum_m=micro_limit,operator_estop_declared=args.operator_estop_ready,hardware_stop_signal_verified=False,motion_commands_sent=transport.sent if transport and args.enable_motion else 0,nonzero_packets=transport.nonzero if transport else 0,command_gate_stats=transport.output_stats() if transport else {},background_worker=frame_worker.health() if frame_worker else {},motor_trace_file=str(folder/'motor_trace.jsonl') if folder else None,sensor_relay=dict(relay_health),**extra)
  if frame_worker:
   from prepared_frames import latency_summary
   r['perception_timing']=latency_summary(frame_worker.latest,time.monotonic())
  r['joint_feedback_health']=feedback_health()
  r['operator_session']=lease.summary(time.monotonic())
  r['code_signature']=code_signature
  r['compute_cpu_affinity']=execution_cpu_affinity
  r['execution_profile']=dict(execution_profile) if frame_worker else None
  if route_recovery:r['route_recovery']=dict(route_recovery.summary(),active_plan_folder=str(active_plan_folder))
  r['controller_extension']=dict(controller_extension_state)
  r['diagnostic_io']=diagnostic_io.stats();history.append(r)
  emit=bool(extra.get('finished') or phase!=last_phase or time.monotonic()-last_print>=1.)
  if extra.get('finished') and folder:atomic_json(folder/'navigation_performance_summary.json',summarize(history))
  diagnostic_io.submit((r,folder))
  if emit:console_io.submit(r);last_phase=phase;last_print=time.monotonic()

 def operator_age():return lease.age(time.monotonic())
 def refresh_output_posture(reference):
  return revalidate_posture(reference,measured_joints(),record['saved_pose'],m.sha256,lambda q:measured_shapes(m,q))
 log=(ROOT/'runtime/navigation_tracking.log').open('w')
 try:
  if args.enable_motion and not args.operator_estop_ready:raise ValueError('OPERATOR_ESTOP_DECLARATION_REQUIRED')
  if args.enable_motion:
   status('PREPARING_CONTROLLER_EXTENSION')
   controller_extension_state=ensure_controller_extension()
   if stop.is_set()or operator_age()>.5:raise ValueError('OPERATOR_LINK_LOST_BEFORE_PLANNING')
  status('LOCALIZING_AND_PLANNING')
  def start_perception(plan_folder,localization):
   nonlocal sensor_bus,tracker,frame_worker,stream,reader
   if stop.is_set() or operator_age()>.5:raise ValueError('OPERATOR_LINK_LOST_BEFORE_START')
   def child_death():ctypes.CDLL(None).prctl(1,signal.SIGINT)
   sensor_bus=SensorBus(Path('/dev/shm')/('tianji_navigation_sensors_'+str(os.getpid())+'_'+args.session))
   # Latest-only raw acquisition runs ahead of filtering. A 2 Hz source
   # added up to 0.5s wait to every slow filter cycle. Workers still have
   # one slot each, and localization is separately limited to its old 2 Hz.
   from resident_localization import capture_stream
   stream=capture_stream()
   if stream is None:stream=SensorStream(localization=True,trace_path=plan_folder/'motor_trace.jsonl',sensor_hz=RAW_SCAN_HZ)
   else:
    # Keep the original passive motor trace when raw ROS acquisition is shared.
    from client import motor_trace_environment
    trace_env=motor_trace_environment()
    resident_trace=subprocess.Popen([sys.executable,str(ROOT/'modules/motor_trace_worker.py'),'--output',str(plan_folder/'motor_trace.jsonl')],stdout=log,stderr=log,env=trace_env,preexec_fn=child_death)
    resident_motor_trace[0]=resident_trace
   def receive():
    try:
     while not stop.is_set():
      h,p=stream.receive();received=time.monotonic();sensor_bus.publish(h,p,received)
      if h.get('kind')=='sensor_status':relay_health.clear();relay_health.update(h)
      if h.get('kind')=='frame' and p is not None:
       if h['boot_id']!=localization['source_boot_id'] or h['epoch']!=localization['odometry_epoch'] or h['calibration_sha256']!=localization['calibration_sha256']:raise ValueError('LIDAR_FRAME_CHANGED')
       with mutex:frames[h['sensor']]=(h,p,received)
    except Exception as exc:
     if not stop.is_set():error.append(str(exc))
   reader=threading.Thread(target=receive,daemon=True);reader.start()
  # A saved route is never directly armed. Re-localize and replan at every start.
  from navigation import run as create_plan
  with exclusive(ROOT/'runtime/navigation.lock'):
   if cable_precheck:create_plan(args.goal,True,cable_precheck=True,on_localized=start_perception)
   else:create_plan(args.goal,True,on_localized=start_perception,right_micro=right_micro,reviewed_map_cleanup=not micro_limit)
  folder=Path(json.loads((ROOT/'runtime/latest_plan.json').read_text())['folder']);plan=json.loads((folder/'planning/plan.json').read_text());record=json.loads((folder/'scene/frozen_input.json').read_text())
  active_plan_folder=folder
  if stop.is_set() or operator_age()>.5:raise ValueError('OPERATOR_LINK_LOST_BEFORE_START')
  if not plan.get('selected'):
   # Ordinary navigation retains its goal at a blocked start too. The first
   # raw reader stays alive; retries must NOT create duplicate subscriptions.
   while not micro_limit and not cable_precheck and not stop.is_set():
    status('WAITING_FOR_ROUTE_CLEAR',reason='INITIAL_ROUTE_BLOCKED',limited_twist=[0.,0.,0.],planning_attempts=plan.get('attempts',[]))
    retry_at=time.monotonic()+5.
    while not stop.is_set()and time.monotonic()<retry_at:
     if operator_age()>.5:raise ValueError('OPERATOR_LINK_LOST_WHILE_ROUTE_BLOCKED')
     if not args.enable_motion and args.duration and time.monotonic()-began>=args.duration:
      phase='TEST_COMPLETE';final_details.update(reason='STATIC_BLOCKED_ROUTE_HOLD_VALIDATED',planning_attempts=plan.get('attempts',[]));return 0
     stop.wait(.1)
    if stop.is_set():phase='STOP_REQUESTED';return 0
    if not targets.unchanged(plan['target']):raise ValueError('TARGET_CHANGED_REPLAN_REQUIRED')
    status('REPLANNING_STOPPED',reason='INITIAL_ROUTE_RETRY',limited_twist=[0.,0.,0.])
    with exclusive(ROOT/'runtime/navigation.lock'):isolated_plan(args.goal,stop,lease.require,folder/'replanning')
    active_plan_folder=Path(json.loads((ROOT/'runtime/latest_plan.json').read_text())['folder']);plan=json.loads((active_plan_folder/'planning/plan.json').read_text());record=json.loads((active_plan_folder/'scene/frozen_input.json').read_text())
    if plan.get('selected'):break
   if not plan.get('selected'):
    if stop.is_set():phase='STOP_REQUESTED';return 0
    attempts=plan.get('attempts',[]);reasons=[r.get('reason','UNKNOWN') for r in attempts]
    detail='起步候选动作全部被碰撞/未知区域拒绝' if attempts and all(r.get('expanded_nodes',99)<=1 for r in attempts) else '没有通过三维检查的路线'
    raise PlanningBlocked(detail+'；'+','.join(reasons))
  if not targets.unchanged(plan['target']):raise ValueError('TARGET_CHANGED_REPLAN_REQUIRED')
  m=model();posture_weights=motion_weights(m.geometry);cfg=json.loads((ROOT/'config/motion_policy.json').read_text());follower=PreviewFollower(plan,cfg);matrices=transforms();radius=max(np.linalg.norm(np.asarray(s['world_corners'])[:,:2],axis=1).max() for s in measured_shapes(m,record['saved_pose']['joint_positions_rad']));guard_world=active_plan_folder/'scene/candidate_world.json' if cable_precheck else active_plan_folder/'scene/immutable_world.json';map_guard=MapGuard(guard_world);map_file=Path(record['map_file']).parent/'voxels.npz';last_hash=0.;inv_start=np.linalg.inv(base_transform(plan['start_map_pose']))
  if micro_limit:
   from route_micro_probe import RouteMicroProbe
   if cable_precheck:
    from cable_precheck import check_prefix
    # Initial sparse wire candidates are a temporary inspection assumption.
    # Every retained table/wall/other obstacle still participates in the sweep.
    check_prefix(guard_world,plan['selected']['path'])
    from probe_cable_deferral import ProbeCableDeferral
    probe_deferral=ProbeCableDeferral()
   micro=RouteMicroProbe(plan['selected']['path'],micro_limit,target_m=.052 if cable_precheck else None)
  def child_death():ctypes.CDLL(None).prctl(1,signal.SIGINT)
  # Raw acquisition overlaps planning; expensive matching/filtering starts
  # afterward so it cannot compete with the initial planner on this CPU.
  tracker_started_wall=time.time()
  from resident_localization import ensure as ensure_resident
  tracker=ensure_resident(active_plan_folder/'localization/localization.json')
  from prepared_frames import PreparedFrames
  from navigation_runtime_profile import filter_profile
  execution_profile=filter_profile(targets.scene_id(),args.goal,micro,cable_precheck,plan['selected']['profile'],requested_cap,TRANSLATION_CAP_MPS,BASE_ADVANCE_S)
  office_live_filter=execution_profile['live_filter'];office_slow_translation=execution_profile['directional_translation']
  translation_cap=execution_profile['translation_cap_mps'];refine_radius=execution_profile['refine_radius_m']
  frame_worker=PreparedFrames(m,sensor_bus.folder,conservative_fast=office_live_filter,refine_near_hits=office_live_filter,
                              refine_radius_m=refine_radius)
  if office_slow_translation:
   # Two independently advanced front/back pairs across >=300ms suffice to exit a data-only hold. The obstacle hold and
   # every per-command age, posture and collision check still apply.
   sensor_recovery=SensorRecovery(clear_duration_s=.30,minimum_pairs=2)
  def limit_live_twist(requested,cloud_age):
   vx,vy,w=limited_twist(requested,cloud_age,radius)
   speed=math.hypot(vx,vy)
   if abs(w)<1e-8 and speed>translation_cap:
    scale=translation_cap/speed
    return [vx*scale,vy*scale,w]
   return [vx,vy,w]
  def wait_cycle(started,wake_for_scan=False):
   deadline=started+CONTROL_CHECK_PERIOD_S
   if frame_worker is None or not (office_slow_translation and wake_for_scan):
    stop.wait(max(0.,deadline-time.monotonic()))
    return
   # Run a new full check when a prepared frame is available. Never extend
   # an existing lease or use file creation time as observation time.
   stop.wait(max(0.,min(deadline,started+.10)-time.monotonic()))
   while not stop.is_set():
    remaining=deadline-time.monotonic()
    if remaining<=0 or frame_worker.has_unread():return
    stop.wait(min(.025,remaining))
  warm=time.monotonic()
  if micro and not right_micro:
   observer_log=(folder/'micro_motion_observer.log').open('w')
   observer_args=[sys.executable,str(ROOT/'modules/observe_self_motion.py'),'--duration','50','--micro','--sensor-bus',str(sensor_bus.folder)]
   if cable_precheck:observer_args+=['--features',str(folder/'probe_initial_wire_features.npz'),'--features-only','--wait-features']
   observer_child=subprocess.Popen(observer_args,stdout=observer_log,stderr=observer_log,preexec_fn=child_death)
  if not args.enable_motion:transport=ControlTransport(False,posture_refresh=refresh_output_posture,load_test_no_motion=load_test)
  from route_recovery import RouteRecovery
  route_recovery=RouteRecovery(record['localization'])
  while not stop.is_set():
   started=time.monotonic();reason='WAIT_SENSORS';guard=None;twist=[0.,0.,0.];pose_ok=False;geometric=False;collision_debug={};posture_reference=None
   if args.duration and time.monotonic()-warm>=args.duration:
    if right_micro:
     phase='STOP_GUARDS_NOT_READY'
    else:
     phase='STOP_TEST_DURATION' if args.enable_motion else 'TEST_COMPLETE'
     final_details['reason']='ONE_SHOT_TEST_DURATION_REACHED' if args.enable_motion else 'PREVIEW_DURATION_REACHED'
    break
   if args.enable_motion and not controller_extension_healthy(controller_extension_state):phase='STOP_CONTROLLER_EXTENSION';final_details['reason']='CONTROLLER_EXTENSION_PROCESS_LOST';break
   if operator_age()>.5:phase='STOP_OPERATOR_LINK';break
   if error:phase='STOP_SENSOR_LINK';final_details=dict(reason='SENSOR_LINK_FAILED',sensor_errors=list(error));break
   if frame_worker and frame_worker.error:phase='STOP_FILTER_WORKER';final_details=dict(reason=frame_worker.error);break
   if observer_child and observer_child.poll() is not None:phase='STOP_RAW_OBSERVER';break
   if tracker.poll() is not None:phase='STOP_LOCALIZATION_PROCESS';break
   if transport and transport.error:phase='STOP_CONTROL_LINK';final_details=dict(reason='CONTROL_LINK_FAILED',control_error=transport.error);break
   if not targets.unchanged(plan['target']):phase='STOP_TARGET_CHANGED';break
   if time.monotonic()-last_hash>1:
    if hashlib.sha256(map_file.read_bytes()).hexdigest()!=record['map_voxel_sha256']:phase='STOP_MAP_CHANGED';break
    from manual_floor import settings as current_floor,fingerprint
    if fingerprint(current_floor(record['map_voxel_sha256']))!=fingerprint(record.get('manual_floor',dict(enabled=False,revision=0))):phase='STOP_MANUAL_FLOOR_CHANGED';break
    last_hash=time.monotonic()
   try:
    pkt=read_live_pose();now=time.time();pos,reason,pos_age=position_check(pkt,record,now)
    if not micro:
     route_recovery.observe(pkt)
     if route_recovery.checking_anchor:
      if reason=='LOCALIZATION_FRAME_OR_EPOCH_CHANGED':raise ValueError(reason)
      if transport:transport.zero()
      follower.pause('LOCALIZATION_ANCHOR_CONFIRMING');ready_count=0
      status('WAITING_FOR_LOCALIZATION_STABILITY',reason='LOCALIZATION_ANCHOR_CONFIRMING',limited_twist=[0.,0.,0.])
      wait_cycle(started,wake_for_scan=True);continue
     if route_recovery.pending:
      # Replanning has NO output path. The existing transport is zeroed
      # before any capture/search and all original guards must arm afterward.
      if transport:transport.zero()
      follower.pause('STOPPED_FOR_ROUTE_RECOVERY');ready_count=0
      if reason=='LOCALIZATION_FRAME_OR_EPOCH_CHANGED':raise ValueError(reason)
      if route_recovery.ready(pkt,pos,time.monotonic()):
       # Keep both filter workers hot while stopped. A small confirmed map
       # correction may reuse the original fixed-map route ONLY after a new
       # whole remaining-route geometry check; live admission still follows.
       from checked_route_reuse import candidate as reuse_candidate,checked as reuse_checked
       reusable=None
       try:
        reusable=reuse_candidate(route_recovery,pkt,record,plan,follower,measured_joints(),time.time())
        if reusable is not None and reuse_checked(active_plan_folder/'scene/immutable_world.json',reusable):
         lease.require(time.monotonic())
         # Re-read state after the bounded geometry check. Do not adopt an
         # anchor which changed again while checking the route.
         fresh=read_live_pose()
         same_anchor=np.allclose(fresh.get('T_map_odom'),pkt.get('T_map_odom'),atol=1e-7,rtol=0.)
         if same_anchor and reuse_candidate(route_recovery,fresh,record,plan,follower,measured_joints(),time.time()) is not None:
          route_recovery.accept_checked_route(fresh);sensor_recovery.blocked('LOCALIZATION_STALE',time.monotonic())
          status('ROUTE_REVALIDATED_ARMING_GUARDS',reason='UNCHANGED_MAP_ROUTE_REVALIDATED',limited_twist=[0.,0.,0.])
          wait_cycle(started,wake_for_scan=True);continue
       except (ValueError,OSError,TypeError):pass
       if frame_worker:
        frame_worker.close();frame_worker=None
       status('REPLANNING_STOPPED',reason=route_recovery.reason,limited_twist=[0.,0.,0.])
       try:
        previous_target=dict(plan['target'])
        with exclusive(ROOT/'runtime/navigation.lock'):
         renewed=isolated_plan(args.goal,stop,lease.require,folder/'replanning')
        new_folder=Path(json.loads((ROOT/'runtime/latest_plan.json').read_text())['folder'])
        new_record=json.loads((new_folder/'scene/frozen_input.json').read_text())
        lease.require(time.monotonic())
        if renewed['target']!=previous_target:raise ValueError('TARGET_CHANGED_REPLAN_REQUIRED')
        if new_record['model_sha256']!=m.sha256 or new_record['map_voxel_sha256']!=record['map_voxel_sha256']:raise ValueError('REPLAN_IDENTITY_CHANGED')
        for field in ['source_boot_id','odometry_epoch','calibration_sha256']:
         if new_record['localization'][field]!=record['localization'][field]:raise ValueError('REPLAN_SENSOR_IDENTITY_CHANGED')
        still_valid,posture_reason,_=posture_check(record['saved_pose'],measured_joints(),m.sha256,time.time())
        if not still_valid:raise ValueError(posture_reason)
        if not renewed.get('selected'):raise ValueError('NO_REVALIDATED_ROUTE')
        replacement_guard=MapGuard(new_folder/'scene/immutable_world.json')
        map_guard.close();map_guard=replacement_guard
        plan=renewed;record=new_record;active_plan_folder=new_folder;follower=PreviewFollower(plan,cfg)
        inv_start=np.linalg.inv(base_transform(plan['start_map_pose']))
        radius=max(np.linalg.norm(np.asarray(s['world_corners'])[:,:2],axis=1).max()for s in measured_shapes(m,record['saved_pose']['joint_positions_rad']))
        frame_worker=PreparedFrames(m,sensor_bus.folder,conservative_fast=office_live_filter,refine_near_hits=office_live_filter,refine_radius_m=refine_radius)
        route_recovery.adopt(record['localization'])
        sensor_recovery.blocked('LOCALIZATION_STALE',time.monotonic())
        status('REPLAN_READY_ARMING_GUARDS',reason='NEW_ROUTE_REQUIRES_FRESH_ADMISSION',limited_twist=[0.,0.,0.])
       except Exception as replan_error:
        route_recovery.failed(replan_error,time.monotonic())
        status('WAITING_FOR_ROUTE_CLEAR',reason=str(replan_error),limited_twist=[0.,0.,0.])
      else:status('WAITING_FOR_LOCALIZATION_RECHECK'if pkt.get('status')=='LOCALIZATION_RECHECK_REQUIRED'else'WAITING_FOR_ROUTE_CLEAR',reason=route_recovery.reason,limited_twist=[0.,0.,0.])
      wait_cycle(started,wake_for_scan=True);continue
    if pkt.get('updated_wall_time',0)<tracker_started_wall:pos=False;reason='LOCALIZATION_INITIALIZING'
    if pos:
     T=np.array(pkt['T_map_base']);pose=[T[0,3],T[1,3],math.atan2(T[1,0],T[0,0])]
     if trajectory_deviation(pose,follower.path)>.12:raise ValueError('PATH_DEVIATION_REPLAN_REQUIRED')
     q=measured_joints();pose_ok,reason,pose_age=posture_check(record['saved_pose'],q,m.sha256,time.time());release=readiness()
     if pose_ok and release['clearance_clear'] and release['shapemask_clear']:
      suggested=micro.step(pkt['odometry_pose'],time.monotonic()) if micro else follower.step(pose,float(pkt['observation_time']),time.time(),plan['target']['revision'],True,True,odometry_pose=pkt['odometry_pose'],control_time=time.monotonic())
      if suggested['state']=='MICRO_STOP':
       if transport:transport.zero()
       final_details=dict(reason=suggested['reason'],traversed_m=micro.traversed,commanded_m=micro.commanded,planned_prefix_m=micro.target_travel)
       phase='MICRO_STOPPED'
       if suggested['reason'] in ['DISTANCE_BUDGET_REACHED','PREFIX_REACHED']:
        displacement=float(np.linalg.norm(np.asarray(pkt['odometry_pose'])[:2]-micro.reference[:2])) if micro.reference is not None else 0.
        final_details['measured_displacement_m']=displacement
        if cable_precheck and (not args.enable_motion or displacement<.05):
         phase='PREVIEW_CABLE_PROBE_VALIDATED' if not args.enable_motion else 'STOP_CABLE_BASELINE_INSUFFICIENT';break
        status('MICRO_STOPPED_SAMPLING',**final_details);sample_until=time.monotonic()+12.
        while time.monotonic()<sample_until and not stop.wait(.2) and operator_age()<=.5:
         if observer_child.poll() is not None:break
        phase='MICRO_TEST_COMPLETE'
        if cable_precheck:
         observer_status=json.loads((ROOT/'runtime/motion_observer_status.json').read_text())
         if stop.is_set() or operator_age()>.5 or observer_status.get('parent_pid')!=os.getpid() or time.time()-observer_status.get('updated_at',0)>2.:raise ValueError('CABLE_EVIDENCE_OR_OPERATOR_STALE')
         evidence=json.loads((Path(observer_status['folder'])/'evidence.json').read_text())
         if evidence.get('parent_pid')!=os.getpid() or not 0<=time.time()-evidence.get('updated_at',0)<=2.:raise ValueError('CABLE_EVIDENCE_SESSION_MISMATCH')
         expected=(record['localization']['source_boot_id'],record['localization']['odometry_epoch'],record['localization']['calibration_sha256'])
         if tuple(evidence.get('source_identity',[]))!=expected:raise ValueError('CABLE_EVIDENCE_FRAME_CHANGED')
         observer_child.send_signal(signal.SIGINT);observer_child.wait(timeout=5)
         capture_result=json.loads((Path(observer_status['folder'])/'result.json').read_text())
         if not capture_result.get('valid'):raise ValueError('INVALID_RAW_CABLE_CAPTURE: '+str(capture_result.get('reason')))
         from replay_probe_evidence import replay
         evidence=replay(observer_status['folder'],evidence['reference_joints'],m.sha256,expected,displacement)
         if stop.is_set() or operator_age()>.5:raise ValueError('OPERATOR_STOPPED_DURING_CABLE_JUDGEMENT')
         from motion_cables import publish
         learned=publish(evidence,m.sha256);fixed=sum(f['status']=='ENVIRONMENT_FIXED' for f in evidence.get('features',[]));initial_count=len(evidence.get('features',[]))
         final_details.update(confirmed_cable_features=learned['confirmed_count'],fixed_environment_features=fixed,initial_candidate_count=initial_count,next_action='REPLAN_REMAINDER_WITH_CONFIRMED_SELF_FILTER')
         phase='CABLE_PRECHECK_FOUND_STATIC_OBSTACLE' if fixed else 'CABLE_PRECHECK_COMPLETE' if learned['confirmed_count'] or initial_count==0 else 'CABLE_EVIDENCE_UNRESOLVED'
         if phase!='CABLE_PRECHECK_COMPLETE':final_details['next_action']='HOLD_POSITION'
       break
      # Finish FK before selecting the newest prepared scans.
      shapes=measured_shapes(m,q['joint_positions_rad']);a,b,c=record['ground_plane'];n=np.array([-a,-b,1.]);normal=T[:3,:3].T@n;offset=n@T[:3,3]-c;floor=[-normal[0]/normal[2],-normal[1]/normal[2],-offset/normal[2]]
      prepared=frame_worker.snapshot();copy={sensor:data['frame'] for sensor,data in prepared.items()}
      cloud_age=max((time.monotonic()-v[2]+float(v[0].get('source_age_s',0)) for v in copy.values()),default=999.)
      twist=limit_live_twist(suggested['twist_preview'],cloud_age)
      if office_slow_translation:frame_worker.request_refinement(twist,pkt['odometry_pose'])
      cloud_checked_at=time.monotonic()
      if cable_precheck:geometric,reason,cloud_age=sweep_guard(copy,matrices,pkt['odometry_pose'],shapes,m.sha256,floor,twist,time.monotonic(),probe_deferral=probe_deferral,revalidation_reserve_m=.005,diagnostics=collision_debug,prepared=prepared)
      else:geometric,reason,cloud_age=sweep_guard(copy,matrices,pkt['odometry_pose'],shapes,m.sha256,floor,twist,time.monotonic(),revalidation_reserve_m=.005,diagnostics=collision_debug,prepared=prepared,motion_advance_s=BASE_ADVANCE_S if not micro else 0.,directional_age_compensation=office_slow_translation)
      if probe_deferral and not probe_deferral.frozen and probe_deferral.ready_to_freeze():
       probe_deferral.freeze()
       atomic_json(folder/'probe_initial_wire_deferral.json',dict(**probe_deferral.summary(),points_body_m=probe_deferral.seeds.tolist(),reference_odometry_matrix=probe_deferral.reference.tolist()))
       temporary=folder/'probe_initial_wire_features.tmp'
       with temporary.open('wb') as f:np.savez(f,points_body=probe_deferral.seeds)
       os.replace(temporary,folder/'probe_initial_wire_features.npz')
      if geometric and micro and not right_micro:
       observer_status=json.loads((ROOT/'runtime/motion_observer_status.json').read_text())
       if observer_status.get('parent_pid')!=os.getpid() or not observer_status.get('ready') or not 0<=time.time()-observer_status.get('updated_at',0.)<=2.:
        geometric=False;reason='WAIT_RAW_OBSERVER'
      if geometric:
       # Revalidate a newer measured pose against a suffix of the extended
       # checked trajectory. Only residual base/joint error consumes the 5mm
       # reserve. Original samples keep aging; the map is checked again.
       elapsed=time.monotonic()-started;raw=state().get('clearance_state',{});clear_age=0. if release['forced_release_active'] else max(0.,float(raw.get('source_age_s',999.)));clear_checked_at=time.monotonic()
       scene_refresh_rechecks=0
       for refresh_attempt in range(2):
        fresh_joints=measured_joints();fresh_ok,fresh_reason,fresh_age=posture_check(record['saved_pose'],fresh_joints,m.sha256,time.time())
        if not fresh_ok:raise ValueError(fresh_reason)
        newest=read_live_pose();valid,new_reason,new_age=position_check(newest,record,time.time());newest_checked_at=time.monotonic()
        if not micro:
         route_recovery.observe(newest)
         if route_recovery.pending or route_recovery.checking_anchor:raise ValueError('LOCALIZATION_RECHECK_REQUIRED')
        if not valid:raise ValueError(new_reason)
        joint_bound=joint_shift_bound(shapes,q['joint_positions_rad'],fresh_joints['joint_positions_rad'],posture_weights)
        certificate=certify_motion(shapes,pkt['odometry_pose'],newest['odometry_pose'],twist,joint_bound,max_advance_s=BASE_ADVANCE_S if not micro else 0.) if math.isfinite(joint_bound) else None
        bounded_revalidation=certificate is not None and certificate['proven']
        if bounded_revalidation:
         # Keep geometry paired with its ORIGINAL measured joints. Output
         # revalidation must account for all subsequent joint displacement.
         base_drift=certificate['base_residual_bound_m'];fresh_shapes=shapes;reference_q=q['joint_positions_rad'];drift=certificate['total_residual_bound_m'];reference_reserve=.005-base_drift
        else:
         fresh_shapes=measured_shapes(m,fresh_joints['joint_positions_rad']);reference_q=fresh_joints['joint_positions_rad']
         certificate=certify_motion(shapes,pkt['odometry_pose'],newest['odometry_pose'],twist,corner_shift(shapes,fresh_shapes),max_advance_s=BASE_ADVANCE_S if not micro else 0.)
         base_drift=certificate['base_residual_bound_m'];drift=certificate['total_residual_bound_m'];reference_reserve=max(0.,.005-drift)
        refresh_scene=False
        if drift<=.005 and refresh_attempt==0:
         old_scene_age=max((time.monotonic()-v[2]+float(v[0].get('source_age_s',0.)) for v in copy.values()),default=999.)
         if old_scene_age>.75:
          latest_prepared=frame_worker.snapshot();latest_copy={sensor:data['frame'] for sensor,data in latest_prepared.items()}
          latest_age=max((time.monotonic()-v[2]+float(v[0].get('source_age_s',0.)) for v in latest_copy.values()),default=999.)
          if set(latest_copy)=={'front','back'} and latest_age+.1<old_scene_age:
           prepared=latest_prepared;copy=latest_copy;refresh_scene=True;scene_refresh_rechecks+=1
           # Keep exact geometry paired with its reference q. The measured
           # joint displacement remains bounded by the existing 5mm reserve.
           if bounded_revalidation:fresh_shapes=shapes
        if drift<=.005 and not refresh_scene:break
        if refresh_attempt:raise ValueError('POSE_MOVED_DURING_COLLISION_CHECK')
        # Actual base movement is not a sensor fault. Start a NEW complete
        # swept-volume check at the newest measured base and joint geometry.
        # Never reuse/freshen the old check or enlarge its 5mm reserve.
        shapes=fresh_shapes
        if not bounded_revalidation:q=fresh_joints
        pkt=newest;T_live=np.asarray(pkt['T_map_base']);normal=T_live[:3,:3].T@n;offset=n@T_live[:3,3]-c
        floor=[-normal[0]/normal[2],-normal[1]/normal[2],-offset/normal[2]]
        prepared=frame_worker.snapshot();copy={sensor:data['frame'] for sensor,data in prepared.items()}
        current_cloud_age=max((time.monotonic()-v[2]+float(v[0].get('source_age_s',0.)) for v in copy.values()),default=999.)
        twist=limit_live_twist(suggested['twist_preview'],current_cloud_age);cloud_checked_at=time.monotonic()
        geometric,reason,cloud_age=sweep_guard(copy,matrices,pkt['odometry_pose'],shapes,m.sha256,floor,twist,time.monotonic(),probe_deferral=probe_deferral,revalidation_reserve_m=.005,diagnostics=collision_debug,prepared=prepared,motion_advance_s=BASE_ADVANCE_S if not micro else 0.,directional_age_compensation=office_slow_translation)
        if not geometric:raise ValueError(reason)
       new_T=np.asarray(newest['T_map_base']);new_pose=[new_T[0,3],new_T[1,3],math.atan2(new_T[1,0],new_T[0,0])]
       if trajectory_deviation(new_pose,follower.path)>.12:raise ValueError('PATH_DEVIATION_REPLAN_REQUIRED')
       relative=inv_start@base_transform(new_pose);local=[relative[0,3],relative[1,3],math.atan2(relative[1,0],relative[0,0])]
       clear,new_reason=map_guard.check(local,twist)
       if not clear:
        if not micro:new_reason=route_recovery.map_failure(new_reason,{s['link']for s in shapes})
        raise ValueError(new_reason)
       localization_age=new_age+time.monotonic()-newest_checked_at;position_revalidated=True;pkt=newest
       posture_reference=dict(joint_positions_rad=reference_q,shapes=fresh_shapes,reserve_m=reference_reserve,motion_weights_m=posture_weights)
       final_posture=refresh_output_posture(posture_reference)
       if final_posture is None:
        latest_joint_input=measured_joints();valid_joint_input,joint_input_reason,_=posture_check(record['saved_pose'],latest_joint_input,m.sha256,time.time())
        if not valid_joint_input:raise ValueError(joint_input_reason)
        raise ValueError('POSE_MOVED_DURING_COLLISION_CHECK')
       measured_pose_age=final_posture['age_s']
       localization_age=new_age+time.monotonic()-newest_checked_at
       if not release['forced_release_active']:clear_age+=time.monotonic()-clear_checked_at
       guard=dict(state='CLEAR',path_checked=True,operator_estop_declared=args.operator_estop_ready,map_sha256=record['map_voxel_sha256'],model_sha256=m.sha256,localization_age_s=localization_age,pose_age_s=measured_pose_age,posture_revalidated=True,posture_feedback_observation_time=final_posture['observation_time'],position_revalidated=position_revalidated,full_sweep_rechecks=refresh_attempt,measured_sweep_drift_m=drift,revalidation_reserve_m=.005,clearance_age_s=clear_age,guard_age_s=0.,cloud_age_s=cloud_age+time.monotonic()-cloud_checked_at,operator_age_s=operator_age(),forced_release_active=release['forced_release_active'],override_version=override_settings().get('updated_wall_time'),assumed_clearance=release['forced_release_active'],perception_mode='BACKGROUND_SCAN_FILTER_AND_COLLISION',background_filter_reports={sensor:{k:data['report'][k] for k in ['points','filtered','protected','filter_compute_s','filter_frame','scan_pose_collision_points']} for sensor,data in prepared.items()})
       guard.update(posture_revalidation_mode='KINEMATIC_BOUND' if bounded_revalidation else 'EXACT_FK',base_revalidation_drift_m=base_drift,scene_refresh_rechecks=scene_refresh_rechecks,motion_certificate=certificate,swept_pose_lease_s=max(0.,min(.35,FUTURE_HORIZON_S-localization_age)))
       if probe_deferral:guard.update(perception_mode='INITIAL_WIRES_DEFERRED_TEST',probe_limit_m=.06,probe_deferred_points=probe_deferral.last_deferred)
     elif pose_ok:reason='CLEARANCE_OR_SHAPEMASK_NOT_CLEAR'
   except Exception as exc:reason=str(exc)
   from navigation_admission import admission
   admitted=guard is not None and admission(guard,0.)
   if guard is not None and not admitted:reason='ADMISSION_DEADLINE_EXPIRED'
   pose_retry=False
   if reason==POSE_RECHECK and not micro:
    try:pose_retry,reason=classify_pose_wait(reason,record['saved_pose'],measured_joints(),m.sha256,read_live_pose(),record,copy,time.time(),time.monotonic())
    except Exception as exc:reason=str(exc)
   if micro and was_admitted and not recovery.within_budget(time.monotonic()):
    if transport:transport.zero()
    phase='STOP_GUARD_LOST';final_details=dict(reason='FRESH_DATA_RECOVERY_TIMEOUT',last_data_reason=last_wait_reason,blocking_checks=dict(wait_reasons));break
   required_ready=3 if micro or not was_admitted else 1
   if admitted:
    if sensor_recovery.active:
     # Keep the task stopped until the configured number of NEW dual-radar
     # pairs passes every original guard across the recovery interval.
     recovery.resumed(time.monotonic())
     if not sensor_recovery.checked_clear(copy,time.monotonic()):
      if transport:transport.zero()
      ready_count=0;follower.pause('CONFIRMING_SENSOR_RECOVERY')
      status('CONFIRMING_SENSOR_RECOVERY',reason='VERIFYING_CONTINUOUS_FRESH_DATA',limited_twist=[0.,0.,0.],guard=guard,sensor_recovery=sensor_recovery.summary(time.monotonic()))
      stop.wait(max(0.,CONTROL_CHECK_PERIOD_S-(time.monotonic()-started)));continue
    if obstacle_wait.active:
     # Every clear sample passed NEW whole-body, map, posture and source
     # checks above. Keep zero until both radars confirm stable clearance.
     recovery.resumed(time.monotonic())
     if not obstacle_wait.checked_clear(copy,time.monotonic()):
      if transport:transport.zero()
      ready_count=0;follower.pause('WAITING_FOR_OBSTACLE_CLEAR')
      status('CONFIRMING_OBSTACLE_CLEAR',reason='VERIFYING_STABLE_CLEARANCE',limited_twist=[0.,0.,0.],guard=guard,obstacle_wait=obstacle_wait.summary(time.monotonic()))
      stop.wait(max(0.,CONTROL_CHECK_PERIOD_S-(time.monotonic()-started)));continue
    if suggested['state']=='ARRIVED':
     if transport:transport.zero()
     # Goal completion must also hold at the newer pose used for the final
     # collision/map check, not only at the loop's earlier controller sample.
     confirmed=follower.step(new_pose,float(pkt['observation_time']),time.time(),plan['target']['revision'],True,True,odometry_pose=pkt['odometry_pose'],control_time=time.monotonic())
     if confirmed['state']!='ARRIVED':
      follower.pause('GOAL_MOVED_DURING_FINAL_CHECK');ready_count=0
      status('FINE_OBSERVING',reason='GOAL_MOVED_DURING_FINAL_CHECK',limited_twist=[0.,0.,0.],goal_error=confirmed.get('goal_error'))
      stop.wait(max(0.,CONTROL_CHECK_PERIOD_S-(time.monotonic()-started)));continue
     suggested=confirmed
     phase='ARRIVED';final_details.update(reason='GOAL_TOLERANCE_REACHED',goal_error=suggested.get('goal_error'),settled_s=suggested.get('settled_s',0.),final_xyt=[float(v)for v in new_pose],final_pose_observation_time=float(pkt['observation_time']))
     status(phase,cleanup_pending=True,limited_twist=[0.,0.,0.],**final_details)
     break
    if not micro:ready_count+=1
    else:
     cloud_key=tuple((s,copy[s][0]['sequence']) for s in sorted(copy))
     if cloud_key!=last_micro_cloud:ready_count+=1;last_micro_cloud=cloud_key
    if args.enable_motion and transport is None and ready_count>=required_ready:transport=ControlTransport(True,posture_refresh=refresh_output_posture)
    control_ready=transport is not None and transport.ready()
    if right_micro and ready_count>=required_ready and control_ready:
     phase='TEST_COMPLETE'
     final_details=dict(reason='RIGHT_MICRO_GUARDS_PASSED',candidate_twist=twist,checked_cloud_pairs=ready_count)
     break
    if micro and ready_count>=required_ready and control_ready and micro.reference is None:
     if probe_deferral and not probe_deferral.frozen:raise ValueError('PROBE_TEMPLATE_NOT_READY')
     micro.arm(pkt['odometry_pose'],time.monotonic())
    if control_ready and ready_count>=required_ready:
     transport.update(twist,guard,posture_reference=posture_reference)
     if not micro:follower.admitted(twist,pkt['odometry_pose'],time.monotonic())
     if micro and micro.reference is not None:micro.note_command(twist,time.monotonic())
     was_admitted=True;recovery.resumed(time.monotonic())
    elif transport:
     transport.zero()
     if micro:micro.note_command([0.,0.,0.],time.monotonic())
     else:follower.pause('CONTROL_NOT_READY')
    following_phase='FINE_ADJUSTMENT_PAUSED' if suggested.get('mode')=='fine_paused' else 'FINE_OBSERVING' if suggested.get('mode')=='fine_observe' else 'GOAL_SETTLING' if suggested.get('mode')=='goal_settling' else 'FINE_ADJUSTING' if str(suggested.get('mode','')).startswith('fine_') else 'FOLLOWING'
    status(('CABLE_PROBE_MOVING' if cable_precheck else following_phase) if args.enable_motion and control_ready and ready_count>=required_ready else 'PREVIEW_READY' if not args.enable_motion and control_ready and ready_count>=required_ready else 'WAIT_CONTROL_READY' if ready_count>=required_ready and not control_ready else 'ARMING_GUARDS',reason=reason,requested_twist=suggested['twist_preview'],limited_twist=twist if control_ready and ready_count>=required_ready else [0.,0.,0.],guard=guard,tracking_mode=suggested.get('mode'),goal_error=suggested.get('goal_error'),settled_s=suggested.get('settled_s',0.),settle_samples=suggested.get('settle_samples',0),control_note=suggested.get('control_note'),fine_pulse_count=suggested.get('fine_pulse_count',0),fine_no_progress_pulses=suggested.get('fine_no_progress_pulses',0),loop_compute_s=time.monotonic()-started,probe_wire_deferral=probe_deferral.summary() if probe_deferral else None)
   else:
    ready_count=0
    if transport:transport.zero()
    if micro:micro.note_command([0.,0.,0.],time.monotonic())
    else:follower.pause('GUARD_NOT_ADMITTED')
    if collision_debug and reason=='LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY' and time.monotonic()-last_collision_capture>=1.:
     try:
      from collision_evidence import save as save_collision
      final_details['collision_report']=save_collision(folder/'scene',collision_debug,copy,q,m.sha256,args.session)
      last_collision_capture=time.monotonic()
     except Exception as evidence_error:final_details['collision_evidence_error']=str(evidence_error)
    if sensor_recovery.active:sensor_recovery.interrupt_clear()
    wait_reasons[reason]+=1;last_wait_reason=reason
    if not micro and reason in ('MAP_COLLISION','LOCALIZATION_RECHECK_REQUIRED'):
     status('WAITING_FOR_ROUTE_CLEAR',reason=route_recovery.reason,limited_twist=[0.,0.,0.],blocking_checks=dict(wait_reasons))
     wait_cycle(started,wake_for_scan=True);continue
    if pose_retry:
     # Every source was independently rechecked fresh above. A conservative
     # geometry proof miss is a zero-speed retry, not a sensor-loss timeout.
     recovery.resumed(time.monotonic())
     status('WAITING_FOR_POSE_CERTIFICATE',reason=reason,limited_twist=[0.,0.,0.],blocking_checks=dict(wait_reasons),motion_certificate=certificate)
     stop.wait(max(0.,CONTROL_CHECK_PERIOD_S-(time.monotonic()-started)));continue
    if not micro and reason in BLOCKING_REASONS:
     # Ordinary live obstacles pause the existing task indefinitely while
     # the operator connection stays alive. They are never waived or filtered.
     obstacle_wait.blocked(reason,time.monotonic(),copy if reason=='LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY' else None);follower.pause('WAITING_FOR_OBSTACLE_CLEAR')
     recovery.resumed(time.monotonic())
     status('WAITING_FOR_OBSTACLE_CLEAR',reason=reason,limited_twist=[0.,0.,0.],obstacle_wait=obstacle_wait.summary(time.monotonic()),blocking_checks=dict(wait_reasons),**{k:v for k,v in final_details.items() if k in ['collision_report','collision_evidence_error']})
     stop.wait(max(0.,CONTROL_CHECK_PERIOD_S-(time.monotonic()-started)));continue
    if obstacle_wait.active:obstacle_wait.interrupt_clear()
    if was_admitted or obstacle_wait.active:
     diagnostics=dict(reason=reason,blocking_checks=dict(wait_reasons),guard=guard,expired_deadlines=expired_deadlines(guard),loop_compute_s=time.monotonic()-started,**{k:v for k,v in final_details.items() if k in ['collision_report','collision_evidence_error']})
     bounded_wait=recovery.waiting(reason,guard,time.monotonic())
     if not micro and waitable_data_fault(reason,guard) and (sensor_recovery.active or not bounded_wait):
      sensor_recovery.blocked(reason,time.monotonic(),copy if 'copy' in locals() else None,guard=guard)
      status('WAITING_FOR_SENSOR_RECOVERY',limited_twist=[0.,0.,0.],sensor_recovery=sensor_recovery.summary(time.monotonic()),**diagnostics)
      wait_cycle(started,wake_for_scan=True);continue
     if bounded_wait:
      status('PAUSED_FOR_FRESH_DATA',**diagnostics);wait_cycle(started,wake_for_scan=True);continue
     phase='STOP_GUARD_LOST';final_details=diagnostics;status(phase,**final_details);break
    status('WAITING_GUARDS',reason=reason,limited_twist=[0.,0.,0.],guard=guard,loop_compute_s=time.monotonic()-started)
    if time.monotonic()-warm>30.:phase='STOP_GUARDS_NOT_READY';break
   wait_cycle(started,wake_for_scan=reason=='LIDAR_STALE' or bool(guard and guard['cloud_age_s']>.85))
  if phase=='PREPARING':
   phase='STOPPED_BY_OPERATOR' if lease.reason=='OPERATOR_STOP_COMMAND' else 'STOP_SESSION_LOST' if lease.reason else 'STOP_REQUESTED'
   if lease.reason:final_details.update(reason=lease.reason)
 except PlanningBlocked as exc:phase='PLAN_BLOCKED';final_details=dict(reason=str(exc),error_code='NO_CHECKED_ROUTE');status(phase,**final_details)
 except Exception as exc:phase='STOP_ERROR';final_details=dict(error=str(exc));status(phase,**final_details)
 finally:
  if lease.reason and phase in ('STOP_REQUESTED','STOPPED_BY_OPERATOR','STOP_SESSION_LOST'):
   phase='STOPPED_BY_OPERATOR' if lease.reason=='OPERATOR_STOP_COMMAND' else 'STOP_SESSION_LOST'
   final_details['reason']=lease.reason
  if probe_deferral:final_details['probe_wire_deferral']=probe_deferral.summary()
  if phase=='STOP_GUARDS_NOT_READY':final_details.update(reason=last_wait_reason,error_code='GUARDS_NOT_READY',blocking_checks=dict(wait_reasons))
  stop.set()
  cleanup_errors=[]
  def cleanup(name,action):
   try:action();return True
   except Exception as exc:cleanup_errors.append(dict(resource=name,error=str(exc)));return False
  coordinate_task=bool(os.environ.get('TIANJI_NAV_COORDINATE_TARGET_FILE'))
  stop_requested_at=time.time()
  if transport and not cleanup('control_transport',transport.close):
   phase='STOP_CONTROL_SHUTDOWN';final_details['reason']='CONTROL_SHUTDOWN_FAILED'
  if coordinate_task and args.enable_motion and transport and folder:
   from navigation_stop_evidence import wait_trace,final_pose
   final_details['navigation_stop_evidence']=wait_trace(folder/'motor_trace.jsonl',stop_requested_at)
   try:measured_final=final_pose(read_live_pose(),record,plan['target']['pose'],stop_requested_at,time.time())
   except Exception as exc:measured_final=dict(valid=False,reason='FINAL_POSE_UNAVAILABLE',error=str(exc))
   final_details['final_pose_evidence']=measured_final
   if measured_final.get('valid'):
    final_details.update(final_xyt=measured_final['final_xyt'],goal_error=measured_final['goal_error'],final_pose_observation_time=measured_final['observation_time'])
    if phase=='ARRIVED'and (measured_final['goal_error']['distance_m']>.05 or measured_final['goal_error']['heading_error_deg']>1.):phase='STOP_FINAL_POSE_OUTSIDE_TOLERANCE'
   elif phase=='ARRIVED':phase='STOP_FINAL_POSE_UNCONFIRMED'
  if coordinate_task and 'navigation_stop_evidence'not in final_details:
   final_details['navigation_stop_evidence']=dict(confirmed=False,reason='NO_FRESH_PHYSICAL_STOP_FEEDBACK')
  if frame_worker:cleanup('filter_workers',frame_worker.close)
  if stream:cleanup('sensor_stream',stream.close)
  if reader:cleanup('sensor_reader',lambda:reader.join(timeout=4))
  def stop_child(child):
   if child.poll() is not None:return
   child.send_signal(signal.SIGINT)
   try:child.wait(timeout=8)
   except subprocess.TimeoutExpired:
    child.terminate()
    try:child.wait(timeout=3)
    except subprocess.TimeoutExpired:child.kill();child.wait(timeout=1)
  if tracker and not getattr(tracker,'resident',False):cleanup('localization_tracker',lambda:stop_child(tracker))
  if resident_motor_trace[0]:cleanup('motor_trace',lambda:stop_child(resident_motor_trace[0]))
  if map_guard:cleanup('map_guard',map_guard.close)
  if observer_child:cleanup('raw_observer',lambda:stop_child(observer_child))
  if observer_log:cleanup('observer_log',observer_log.close)
  if sensor_bus:
   import shutil
   cleanup('sensor_bus',lambda:shutil.rmtree(sensor_bus.folder,ignore_errors=True))
  if cleanup_errors:final_details['cleanup_errors']=cleanup_errors
  cleanup('tracking_log',log.close);status(phase,finished=True,cleanup_pending=False,wall_s=time.monotonic()-began,**final_details)
  console_io.close();drained=diagnostic_io.close()
  if not drained:print('DIAGNOSTIC_IO_WARNING final state='+phase+'; writer drain timed out; chassis output already stopped',flush=True)
  if folder:
   atomic_json(folder/'navigation_execution_history.json',history)
   atomic_json(folder/'navigation_performance_summary.json',summarize(history))
  lock.close()
 return 0
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--goal',type=int,required=True,choices=[0,1,2,3]);p.add_argument('--session',required=True);mode=p.add_mutually_exclusive_group(required=True);mode.add_argument('--enable-motion',action='store_true');mode.add_argument('--preview',action='store_true');p.add_argument('--operator-estop-ready',action='store_true');p.add_argument('--duration',type=float,default=0.);p.add_argument('--max-travel-m',type=float,default=0.);p.add_argument('--cable-precheck',action='store_true');p.add_argument('--right-micro-preview',action='store_true');p.add_argument('--translation-cap-mps',type=float);p.add_argument('--load-test-no-motion',action='store_true');a=p.parse_args();sys.exit(run(a))

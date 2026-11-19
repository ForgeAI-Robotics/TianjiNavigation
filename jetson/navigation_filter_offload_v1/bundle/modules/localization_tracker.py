"""Passive LiDAR map matching + measured odometry memory for Jetson teleop.
Only read-only sensor transport and file-only PCL GICP are used. No publishers.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import argparse,hashlib,json,math,resource,signal,subprocess,sys,threading,time,uuid
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from common import ROOT,atomic_json,exclusive
from client import SensorStream
from async_diagnostics import AsyncDiagnostics
from live_pose import publish as publish_live_pose
from extrinsics import transforms
from saved_map import base_transform
from pose_memory import PoseMemory,yaw,angle_delta
from localization_schedule import RegistrationSlot,registration_timeout,prediction_evidence_current,REGISTRATION_PERIOD_S,FAST_VALIDATION_PERIOD_S

EVAL=ROOT/'localization_backend'

def voxel(points,leaf=.10):
    keys=np.floor(points/leaf).astype(np.int32)
    _,ids=np.unique(keys,axis=0,return_index=True)
    return np.ascontiguousarray(points[ids],dtype='<f4')

def usable(points):
    radius=np.linalg.norm(points[:,:2],axis=1)
    return points[np.isfinite(points).all(1)&(radius>.8)&(radius<8)&(points[:,2]>-.08)&(points[:,2]<2.08)]

def residuals(points,T,tree):
    q=points@T[:3,:3].T+T[:3,3]
    structure=points[:,2]>.15;q=q[structure]
    if len(q)<100:return dict(points=len(q),overlap=0.,rmse=1.,observable=False)
    distance,ids=tree.query(q);good=distance<.15
    rmse=float(np.sqrt(np.mean(distance[good]**2))) if good.any() else 1.
    # Point-to-plane planar information detects single-wall / corridor ambiguity.
    selected=q[good][::max(1,int(good.sum())//600)]
    observable=False;condition=None
    if len(selected)>=100:
        _,neighbors=tree.query(selected,k=12);patch=tree.data[neighbors];center=patch.mean(1)
        covariance=np.einsum('nki,nkj->nij',patch-center[:,None,:],patch-center[:,None,:])
        values,vectors=np.linalg.eigh(covariance);normal=vectors[:,:,0]
        planar=(values[:,0]<.35*np.maximum(values[:,1],1e-8))&(np.abs(normal[:,2])<.85)
        arm=selected-T[:3,3]
        J=np.column_stack([normal[:,0],normal[:,1],arm[:,0]*normal[:,1]-arm[:,1]*normal[:,0]])[planar]
        if len(J)>=100:
            J=J/np.maximum(np.linalg.norm(J,axis=0),1e-9)
            eigen=np.linalg.eigvalsh(J.T@J);condition=float(eigen[-1]/max(eigen[0],1e-12))
            observable=bool(eigen[0]>.003 and condition<1000)
    return dict(points=len(q),overlap=float(good.mean()),rmse=rmse,observable=observable,planar_condition=condition)

def bootstrap_quality(groups,guess,tree):
    """A prior alone is never a fix; both NEW radars need strict evidence."""
    if set(groups)!={'front','back'} or any(len(p)<300 for p in groups.values()):return None
    metrics={s:residuals(p[1::2],guess,tree) for s,p in groups.items()}
    return metrics if all(m['overlap']>.75 and m['rmse']<.065 and m['observable'] for m in metrics.values()) else None

def register_groups(job,session,tree):
    """One file-only GICP job. Never reads or updates live pose memory."""
    began=time.monotonic();groups=job['groups'];guess=job['guess'];h=job['header']
    training=np.concatenate([p[::2] for p in groups.values()]);training.astype('<f4').tofile(session/'source.xyz')
    (session/'seed.txt').write_text(f'{guess[0,3]} {guess[1,3]} {guess[2,3]} {yaw(guess)}\n')
    budget=registration_timeout(h,job['received_at'],time.monotonic())
    subprocess.run([str(EVAL/'native/gicp'),str(session/'source.xyz'),str(session/'target.xyz'),str(session/'seed.txt'),str(session/'fit.txt'),'.35'],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,timeout=budget,check=True)
    row=np.loadtxt(session/'fit.txt');T=row[4:].reshape(4,4)
    metrics={s:residuals(p[1::2],T,tree) for s,p in groups.items()};metrics['compute_s']=float(row[3])
    correction=float(np.linalg.norm(T[:2,3]-guess[:2,3]));rotation=abs(angle_delta(yaw(T),yaw(guess)))
    good=bool(row[1]) and correction<.35 and rotation<math.radians(8) and abs(T[2,3])<.15 and np.arccos(np.clip(T[2,2],-1,1))<math.radians(8)
    good=good and all(m['overlap']>.70 and m['rmse']<.08 and m['observable'] for s,m in metrics.items() if s in groups)
    choice='GICP_MEASURED_CORRECTION';proposed=dict(correction_xy_m=correction,correction_yaw_deg=math.degrees(rotation),metrics=metrics)
    if good and (correction>.02 or rotation>math.radians(1.)):
        prior={s:residuals(p[1::2],guess,tree)for s,p in groups.items()}
        from registration_choice import retain_validated_prior
        if retain_validated_prior(prior,metrics,correction,rotation):
            # A NEW dual scan independently validates the original pose. No
            # source timestamp is rewritten and the unsafe jump is not applied.
            T=np.array(guess,copy=True);metrics=dict(prior,compute_s=float(row[3]));correction=0.;rotation=0.;choice='NEW_DUAL_SCAN_VALIDATED_PRIOR_NO_MEANINGFUL_GICP_GAIN'
    return dict(matrix=T,good=good,metrics=metrics,correction_xy_m=correction,correction_yaw_deg=math.degrees(rotation),total_compute_s=time.monotonic()-began,registration_choice=choice,proposed_correction=proposed)

def main():
    process_start=time.monotonic();cpu_start=time.process_time()
    parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=0);parser.add_argument('--reference',type=Path,help='v6 fresh global localization record');parser.add_argument('--sensor-bus',type=Path);parser.add_argument('--match-period-s',type=float,default=FAST_VALIDATION_PERIOD_S);parser.add_argument('--resident',action='store_true');parser.add_argument('--resident-token');args=parser.parse_args()
    if not .25<=args.match_period_s<=.5:raise ValueError('INVALID_LOCALIZATION_MATCH_PERIOD')
    lock=exclusive(ROOT/'runtime/localization.lock')
    result=json.loads((args.reference or EVAL/'reports/localization_result.json').read_text())
    if args.reference:
        if result.get('status')!='GLOBAL_LOCALIZED':raise RuntimeError('全局雷达定位参考未通过')
        if not all(v['overlap']>.75 and v['rmse']<.065 and v['observable'] for v in result['quality'].values()):raise RuntimeError('全局雷达参考质量不足')
        result['T_map_odom_advisory_only']=(np.array(result['T_map_base'])@np.linalg.inv(base_transform(result['wheel_pose']))).tolist()
    elif result['status']!='MATCHED_STATIC_VALIDATED':raise RuntimeError('没有经过静态验证的地图定位参考')
    folder=Path(result['map_folder']);digest=hashlib.sha256((folder/'voxels.npz').read_bytes()).hexdigest()
    if digest!=result['map_voxel_sha256']:raise RuntimeError('定位地图已变化，需要重新建立参考')
    with np.load(folder/'voxels.npz',allow_pickle=False) as data:target=np.array(data['points'])
    target=voxel(target[(target[:,2]>-.08)&(target[:,2]<2.08)],.08);tree=cKDTree(target)
    session=ROOT/'localization/sessions'/ (time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);session.mkdir(parents=True)
    target.astype('<f4').tofile(session/'target.xyz')
    calibration=hashlib.sha256((ROOT/'config/lidar_calibration.json').read_bytes()).hexdigest();matrices=transforms()
    runtime=ROOT/'runtime/current_pose.json';prior={}
    try:prior=json.loads(runtime.read_text())
    except (OSError,ValueError):pass
    memory=PoseMemory();stop=threading.Event();mutex=threading.Lock();pending={};last_match=[None];error=[None];last_registration=[None]
    registration=RegistrationSlot(lambda job:register_groups(job,session,tree));last_full_attempt=[None]
    scheduling=dict(fast_attempts=0,fast_accepted=0,fast_rejected=0,full_submitted=0,full_accepted=0,full_rejected=0,max_fast_compute_s=0.,max_fast_fix_gap_s=0.,last_fast_fix_monotonic=None,last_full_attempt_monotonic=None)
    source_boot=result.get('source_boot_id') if args.reference else json.loads((ROOT/'waypoints/point_02.json').read_text())['source_boot_id']
    initial_T=np.array(result['T_map_base']);initial_anchor=np.array(result['T_map_odom_advisory_only'])
    atomic_json(session/'session.json',dict(map_folder=str(folder),map_voxel_sha256=digest,created_wall_time=time.time(),
                mode='read_only_tracking',motion_commands_sent=0,waypoints_modified=False,map_modified=False,
                description='Existing map used as geometric localization reference; no navigation clearance inferred.'))
    def write_diagnostic(item):
        kind,packet=item
        if kind=='match':
            with (session/'matches.jsonl').open('a') as handle:handle.write(json.dumps(packet,allow_nan=False)+'\n')
        else:
            atomic_json(runtime,packet);atomic_json(session/'last_pose.json',packet)
            with (session/'trajectory.jsonl').open('a') as handle:
                row={k:packet[k] for k in ('updated_wall_time','observation_time','status','map_pose_valid','T_map_base','odometry_pose','session_distance_m','odometry_epoch','reset_count')}
                handle.write(json.dumps(row,allow_nan=False)+'\n')
    diagnostic_io=AsyncDiagnostics(write_diagnostic,capacity=32)
    def save(running=True):
        with mutex:
            packet=memory.snapshot(time.monotonic());packet.update(updated_wall_time=time.time(),running=running,
                map_folder=str(folder),map_voxel_sha256=digest,frame_id=result['frame_id'],session=str(session),
                last_match_attempt=last_match[0],error=error[0],motion_commands_sent=0,waypoints_modified=False,
                navigation_ready=False,absolute_accuracy_verified=False,localization_scheduling=dict(scheduling,registration_worker_busy=registration.busy,last_full_attempt=last_full_attempt[0]))
            if not running:packet.update(last_tracking_status=packet['status'],status='STOPPED',map_pose_valid=False,odometry_fresh=False)
        if args.resident:packet.update(resident_pid=os.getpid(),resident_session=args.resident_token or resident_owner['token'])
        packet['diagnostic_io']=diagnostic_io.stats()
        publish_live_pose(packet);diagnostic_io.submit(('pose',packet))
        return packet
    resident_bus=None;resident_owner=None
    if args.resident:
        if args.sensor_bus:raise ValueError('RESIDENT_OWNS_ITS_SENSOR_INPUT')
        from sensor_bus import SensorBus
        from resident_localization import OWNER
        token=args.resident_token or uuid.uuid4().hex
        resident_bus=SensorBus(Path('/dev/shm')/f'tianji_navigation_resident_{os.getpid()}_{token}')
        resident_owner=dict(pid=os.getpid(),token=token,sensor_bus=str(resident_bus.folder),reference=str(args.reference),map_folder=str(folder),map_voxel_sha256=digest,frame_id=result['frame_id'],source_boot_id=result['source_boot_id'],odometry_epoch=result['odometry_epoch'],calibration_sha256=result['calibration_sha256'],global_reference=result.get('global_reference',str(args.reference)),motion_commands_sent=0)
        atomic_json(OWNER,resident_owner)
    stream=None
    def receive():
        try:
            while not stop.is_set():
                h,p=stream.receive();received=time.monotonic()
                if h.get('calibration_sha256')!=calibration:raise ValueError('雷达外参不一致')
                if h.get('source_age_s',0)>1.:continue
                if resident_bus is not None:
                    resident_bus.publish(h,p,received)
                    h=dict(h,source_age_s=float(h.get('source_age_s',0.))+time.monotonic()-received)
                with mutex:
                    if h['kind']=='odometry':memory.ingest(h,time.monotonic())
                    elif h['kind']=='frame' and p is not None:
                        pending[h['sensor']]=(h,np.array(p),time.monotonic())
        except Exception as exc:
            if not stop.is_set():
                with mutex:error[0]=str(exc)
            # Keep writing stale status; never silently treat old data as current.
            stop.set()
    def log_match(attempt):
        diagnostic_io.submit(('match',attempt))
    def match():
        next_time=0.
        while not stop.wait(.05):
            # Polling is nonblocking. Only this thread may apply a registration;
            # the background worker never changes memory or source timestamps.
            completed=registration.poll()
            if completed is not None:
                job=completed['job'];fit=completed['result'];now=time.monotonic()
                with mutex:
                    accepted=False
                    if fit is not None:
                        accepted=fit['good'] and job['generation']==memory.reset_count and memory.fix(fit['matrix'],job['header'],now,fit['metrics'],registration=True)
                        attempt={k:fit[k] for k in ['metrics','correction_xy_m','correction_yaw_deg','total_compute_s','registration_choice','proposed_correction']}
                    else:attempt=dict(error=completed['error'])
                    attempt.update(accepted=bool(accepted),method='FULL_GICP_REGISTRATION',observation_time=job['header']['observation_time'],wall_time=time.time(),reset_count=memory.reset_count,correction_pending=None if memory.pending_correction is None else {k:v for k,v in memory.pending_correction.items()if k!='anchor'},anchor_revision=memory.anchor_revision)
                    scheduling['full_accepted' if accepted else 'full_rejected']+=1
                    if accepted:last_registration[0]=now;last_match[0]=dict(attempt);error[0]=None
                    last_full_attempt[0]=dict(attempt)
                log_match(attempt)
            if time.monotonic()<next_time:continue
            with mutex:
                pair=dict(pending)
                if set(pair)!= {'front','back'}:continue
                headers=[row[0] for row in pair.values()]
                h=max(headers,key=lambda row:row['observation_time']);generation=memory.reset_count
                if (max(row['observation_time'] for row in headers)-min(row['observation_time'] for row in headers)>1.
                    or any(time.monotonic()-row[2]>1.5 for row in pair.values())):continue
                pending.clear()
                if memory.identity is None:continue
                if any((x['boot_id'],x['epoch'],x['calibration_sha256'])!=memory.identity for x in headers):continue
                guess=memory.prediction(h['pose'])
                first_global=bool(args.reference and memory.anchor is None)
                recent_registration=memory.anchor is not None and prediction_evidence_current(last_registration[0],time.monotonic())
                if guess is None:
                    # A saved transform remains a seed, requiring actual NEW
                    # dual-scan evidence under the unchanged quality gates.
                    if args.reference and generation==0 and h['boot_id']==source_boot and h['epoch']==result['odometry_epoch'] and h['calibration_sha256']==result['calibration_sha256']:
                        guess=initial_anchor@base_transform(h['pose'])
                    elif prior.get('map_voxel_sha256')==digest and prior.get('T_map_base') is not None:
                        guess=np.array(prior['T_map_base'])
                        if (generation==0 and prior.get('source_boot_id')==h['boot_id'] and prior.get('calibration_sha256')==calibration and prior.get('T_map_odom') is not None):
                            guess=np.array(prior['T_map_odom'])@base_transform(h['pose'])
                    elif generation==0 and h['boot_id']==source_boot:guess=initial_anchor@base_transform(h['pose'])
                    else:guess=initial_T.copy()
            began=time.monotonic()
            try:
                reference=base_transform(h['pose']);groups={}
                for sensor,(header,points,_) in pair.items():
                    T=np.linalg.inv(reference)@base_transform(header['pose'])@matrices[sensor]
                    groups[sensor]=voxel(usable(points@T[:3,:3].T+T[:3,3]))
                if any(len(p)<300 for p in groups.values()):raise ValueError('有效结构点不足')
                bootstrap=bootstrap_quality(groups,guess,tree) if first_global or recent_registration else None
                with mutex:
                    scheduling['fast_attempts']+=1
                    # A slow quality calculation cannot extend the original 4s
                    # authority of the last full registration/global bootstrap.
                    authority=first_global or (memory.anchor is not None and prediction_evidence_current(last_registration[0],time.monotonic()))
                    accepted=bootstrap is not None and authority and generation==memory.reset_count and memory.fix(guess,h,time.monotonic(),bootstrap)
                    scheduling['fast_accepted' if accepted else 'fast_rejected']+=1
                    if accepted:
                        now=time.monotonic();previous=scheduling['last_fast_fix_monotonic']
                        if previous is not None:scheduling['max_fast_fix_gap_s']=max(scheduling['max_fast_fix_gap_s'],now-previous)
                        scheduling['last_fast_fix_monotonic']=now
                        last_match[0]=dict(accepted=True,method='LIVE_DUAL_LIDAR_VALIDATED_GLOBAL_SEED' if first_global else 'NEW_DUAL_SCAN_VALIDATED_ODOMETRY_PREDICTION',observation_time=h['observation_time'],metrics=bootstrap,correction_xy_m=0.,correction_yaw_deg=0.)
                        error[0]=None
                        if first_global:last_registration[0]=now
                    else:last_match[0]=dict(accepted=False,method='DUAL_SCAN_PREDICTION_VALIDATION',observation_time=h['observation_time'],error='DUAL_SCAN_QUALITY_NOT_ACCEPTED' if authority else 'FULL_REGISTRATION_VALIDATION_WINDOW_EXPIRED')
                    attempt=dict(last_match[0],wall_time=time.time(),reset_count=generation) if accepted else None
                    last_full=last_registration[0];last_attempt=scheduling['last_full_attempt_monotonic']
                if attempt is not None:log_match(attempt)
                # Start periodic registration BEFORE its 4s validation window
                # ends. At most one job exists; busy jobs never queue old scans.
                now=time.monotonic();due=last_full is None or now-last_full>=REGISTRATION_PERIOD_S
                if (not accepted or due) and not registration.busy and (last_attempt is None or now-last_attempt>=.5):
                    received=pair[h['sensor']][2]
                    registration_timeout(h,received,now)
                    job=dict(groups=groups,guess=np.array(guess,copy=True),header=dict(h),received_at=received,generation=generation)
                    if registration.submit(job):
                        with mutex:scheduling['full_submitted']+=1;scheduling['last_full_attempt_monotonic']=now
            except Exception as exc:
                with mutex:error[0]=str(exc);last_match[0]=dict(accepted=False,error=str(exc))
            with mutex:scheduling['max_fast_compute_s']=max(scheduling['max_fast_compute_s'],time.monotonic()-began)
            next_time=time.monotonic()+args.match_period_s
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:stop.set())
    worker=None;reader=None;start=time.monotonic()
    try:
        if args.sensor_bus:
            from sensor_bus import SnapshotStream
            stream=SnapshotStream(args.sensor_bus)
        else:
            from navigation_runtime_profile import RAW_SCAN_HZ
            stream=SensorStream(localization=True,consumer='tracker',sensor_hz=RAW_SCAN_HZ if args.resident else None)
        save();stream.remote.stdout.channel.settimeout(3.)
        reader=threading.Thread(target=receive,daemon=True);reader.start();worker=threading.Thread(target=match,daemon=True);worker.start()
        # Publish freshly received 20Hz wheel feedback at 10Hz; the previous
        # 5Hz file cadence consumed most of the admission freshness budget.
        while not stop.wait(.1):
            save()
            if args.duration and time.monotonic()-start>=args.duration:break
    finally:
        stop.set()
        if stream:stream.close()
        if reader:reader.join(timeout=4)
        if worker:worker.join(timeout=4)
        registration.close()
        save(False);diagnostic_io.close()
        if resident_bus is not None:
            import shutil
            shutil.rmtree(resident_bus.folder,ignore_errors=True)
            try:
                if json.loads(OWNER.read_text()).get('token')==resident_owner['token']:OWNER.unlink()
            except (OSError,ValueError):pass
        usage=resource.getrusage(resource.RUSAGE_CHILDREN)
        atomic_json(session/'performance.json',dict(wall_s=time.monotonic()-process_start,python_cpu_s=time.process_time()-cpu_start,
            matching_children_cpu_s=usage.ru_utime+usage.ru_stime,python_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            child_peak_rss_kib=usage.ru_maxrss,motion_commands_sent=0))
        print('位姿记录已保存：'+str(session),flush=True)

if __name__=='__main__':main()

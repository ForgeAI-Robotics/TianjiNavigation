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
from extrinsics import transforms
from saved_map import base_transform
from pose_memory import PoseMemory,yaw,angle_delta

EVAL=ROOT.parent/'lidar_localization_eval'

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

def main():
    process_start=time.monotonic();cpu_start=time.process_time()
    parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=0);args=parser.parse_args()
    lock=exclusive(ROOT/'runtime/localization.lock')
    try:
        result=json.loads((EVAL/'reports/localization_result.json').read_text())
        if result['status']!='MATCHED_STATIC_VALIDATED':raise RuntimeError('没有经过静态验证的地图定位参考')
        folder=Path(result['map_folder']);digest=hashlib.sha256((folder/'voxels.npz').read_bytes()).hexdigest()
        if digest!=result['map_voxel_sha256']:raise RuntimeError('定位地图已变化，需要重新建立参考')
    except (OSError,ValueError,KeyError,RuntimeError) as exc:
        # Manual teleop needs fresh measured odometry, not a valid old map.
        # Never relabel the historical map reference as current localization.
        from teleop_odometry import run
        return run(args.duration,str(exc))
    with np.load(folder/'voxels.npz',allow_pickle=False) as data:target=np.array(data['points'])
    target=voxel(target[(target[:,2]>-.08)&(target[:,2]<2.08)],.08);tree=cKDTree(target)
    session=ROOT/'localization/sessions'/ (time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);session.mkdir(parents=True)
    target.astype('<f4').tofile(session/'target.xyz')
    calibration=hashlib.sha256((ROOT/'config/lidar_calibration.json').read_bytes()).hexdigest();matrices=transforms()
    runtime=ROOT/'runtime/current_pose.json';prior={}
    try:prior=json.loads(runtime.read_text())
    except (OSError,ValueError):pass
    memory=PoseMemory();stop=threading.Event();mutex=threading.Lock();pending={};last_match=[None];error=[None]
    source_boot=json.loads((ROOT/'waypoints/point_02.json').read_text())['source_boot_id']
    initial_T=np.array(result['T_map_base']);initial_anchor=np.array(result['T_map_odom_advisory_only'])
    atomic_json(session/'session.json',dict(map_folder=str(folder),map_voxel_sha256=digest,created_wall_time=time.time(),
                mode='read_only_tracking',motion_commands_sent=0,waypoints_modified=False,map_modified=False,
                description='Existing map used as geometric localization reference; no navigation clearance inferred.'))
    def save(running=True):
        with mutex:
            packet=memory.snapshot(time.monotonic());packet.update(updated_wall_time=time.time(),running=running,
                map_folder=str(folder),map_voxel_sha256=digest,frame_id=result['frame_id'],session=str(session),
                last_match_attempt=last_match[0],error=error[0],motion_commands_sent=0,waypoints_modified=False,
                navigation_ready=False,absolute_accuracy_verified=False)
            if not running:packet.update(last_tracking_status=packet['status'],status='STOPPED',map_pose_valid=False,odometry_fresh=False)
        atomic_json(runtime,packet);atomic_json(session/'last_pose.json',packet)
        with (session/'trajectory.jsonl').open('a') as handle:
            row={k:packet[k] for k in ('updated_wall_time','observation_time','status','map_pose_valid','T_map_base','odometry_pose','session_distance_m','odometry_epoch','reset_count')}
            handle.write(json.dumps(row,allow_nan=False)+'\n')
        return packet
    stream=None
    def receive():
        try:
            while not stop.is_set():
                h,p=stream.receive()
                if h.get('calibration_sha256')!=calibration:raise ValueError('雷达外参不一致')
                if h.get('source_age_s',0)>1.:continue
                with mutex:
                    if h['kind']=='odometry':memory.ingest(h,time.monotonic())
                    elif h['kind']=='frame' and p is not None:
                        pending[h['sensor']]=(h,np.array(p),time.monotonic())
        except Exception as exc:
            if not stop.is_set():
                with mutex:error[0]=str(exc)
            # Keep writing stale status; never silently treat old data as current.
            stop.set()
    def match():
        next_time=0.
        while not stop.wait(.05):
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
                if guess is None:
                    # Persisted transforms are seeds only. A fresh scan must pass all gates first.
                    if prior.get('map_voxel_sha256')==digest and prior.get('T_map_base') is not None:
                        guess=np.array(prior['T_map_base'])
                        if (generation==0 and prior.get('source_boot_id')==h['boot_id'] and prior.get('calibration_sha256')==calibration and prior.get('T_map_odom') is not None):
                            guess=np.array(prior['T_map_odom'])@base_transform(h['pose'])
                    elif generation==0 and h['boot_id']==source_boot:
                        guess=initial_anchor@base_transform(h['pose'])
                    else:guess=initial_T.copy()
            try:
                reference=base_transform(h['pose']);groups={}
                for sensor,(header,points,_) in pair.items():
                    T=np.linalg.inv(reference)@base_transform(header['pose'])@matrices[sensor]
                    groups[sensor]=voxel(usable(points@T[:3,:3].T+T[:3,3]))
                if any(len(p)<300 for p in groups.values()):raise ValueError('有效结构点不足')
                # Different held-out points from both sensors validate the fit.
                training=np.concatenate([p[::2] for p in groups.values()]);training.astype('<f4').tofile(session/'source.xyz')
                (session/'seed.txt').write_text(f'{guess[0,3]} {guess[1,3]} {guess[2,3]} {yaw(guess)}\n')
                proc=subprocess.run([str(EVAL/'native/gicp'),str(session/'source.xyz'),str(session/'target.xyz'),str(session/'seed.txt'),str(session/'fit.txt'),'.35'],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,timeout=3.,check=True)
                row=np.loadtxt(session/'fit.txt');T=row[4:].reshape(4,4)
                metrics={s:residuals(p[1::2],T,tree) for s,p in groups.items()}
                metrics['compute_s']=float(row[3])
                correction=float(np.linalg.norm(T[:2,3]-guess[:2,3]));rotation=abs(angle_delta(yaw(T),yaw(guess)))
                good=bool(row[1]) and correction<.35 and rotation<math.radians(8) and abs(T[2,3])<.15 and np.arccos(np.clip(T[2,2],-1,1))<math.radians(8)
                good=good and all(m['overlap']>.70 and m['rmse']<.08 and m['observable'] for s,m in metrics.items() if s in groups)
                with mutex:
                    accepted=good and generation==memory.reset_count and memory.fix(T,h,time.monotonic(),metrics)
                    last_match[0]=dict(accepted=bool(accepted),observation_time=h['observation_time'],metrics=metrics,correction_xy_m=correction,correction_yaw_deg=math.degrees(rotation))
                    error[0]=None if accepted else '雷达匹配未通过质量/连续性检查'
            except Exception as exc:
                with mutex:error[0]=str(exc);last_match[0]=dict(accepted=False,error=str(exc))
            with mutex:attempt=dict(last_match[0],wall_time=time.time(),reset_count=memory.reset_count)
            with (session/'matches.jsonl').open('a') as handle:handle.write(json.dumps(attempt,allow_nan=False)+'\n')
            next_time=time.monotonic()+1.
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:stop.set())
    worker=None;reader=None;start=time.monotonic()
    try:
        save();stream=SensorStream(localization=True);stream.remote.stdout.channel.settimeout(3.)
        reader=threading.Thread(target=receive,daemon=True);reader.start();worker=threading.Thread(target=match,daemon=True);worker.start()
        while not stop.wait(.2):
            save()
            if args.duration and time.monotonic()-start>=args.duration:break
    finally:
        stop.set()
        if stream:stream.close()
        if reader:reader.join(timeout=4)
        if worker:worker.join(timeout=4)
        save(False)
        usage=resource.getrusage(resource.RUSAGE_CHILDREN)
        atomic_json(session/'performance.json',dict(wall_s=time.monotonic()-process_start,python_cpu_s=time.process_time()-cpu_start,
            matching_children_cpu_s=usage.ru_utime+usage.ru_stime,python_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            child_peak_rss_kib=usage.ru_maxrss,motion_commands_sent=0))
        print('位姿记录已保存：'+str(session),flush=True)

if __name__=='__main__':main()

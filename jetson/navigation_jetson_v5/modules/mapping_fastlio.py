"""Jetson FAST-LIO2 mapping CLI. S100 read-only source; no actuator publishers."""
import argparse,collections,ctypes,hashlib,json,math,os,select,signal,struct,subprocess,sys,threading,time
from datetime import datetime
from pathlib import Path
import numpy as np
from common import ROOT,CONFIG,atomic_json,exclusive
from mapping_core import VoxelRecorder
from extrinsics import transforms
from cloud import decode_cloud
from fastlio_support import pose_matrix,estimator_ready,interpolate_poses
from registered_scan_guard import RegisteredScanGuard
TOPICS=[('/driver/lidar/lidar_front/point_cloud/Data','sensor_msgs/msg/PointCloud2'),('/driver/lidar/lidar_front/imu/Data','sensor_msgs/msg/Imu'),('/driver/lidar/lidar_back/point_cloud/Data','sensor_msgs/msg/PointCloud2'),('/controller/odom','nav_msgs/msg/Odometry')]

def main():
    parser=argparse.ArgumentParser(description='FAST-LIO2 Jetson 三维建图；S100只读采集；不发送运动指令')
    parser.add_argument('--duration',type=float,default=0.,help='只读定时采集秒数；0为交互式')
    parser.add_argument('--validation-only',action='store_true',help='测试保存单独地图，并禁止与实机遥操联动放行')
    parser.add_argument('--replay',type=Path,help='离线回放原始PointCloud2/IMU rosbag；自动测试禁行')
    parser.add_argument('--rate',type=float,default=1.,help='离线回放速率')
    args=parser.parse_args()
    if args.duration<0 or not .1<=args.rate<=2.:parser.error('Invalid duration/rate')
    if os.environ.get('ROS_DOMAIN_ID')!='92' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('FAST-LIO2 requires isolated localhost ROS domain 92')
    import rclpy,rosbag2_py,yaml
    from sensor_msgs.msg import PointCloud2,Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    cfg=CONFIG['fast_lio2'];workspace=Path(cfg['workspace']);binary=workspace/'install/fast_lio/lib/fast_lio'
    for name in ('fastlio_mapping','s100_livox_adapter'):
        if not (binary/name).is_file():raise RuntimeError('Missing FAST-LIO2 binary: '+name)
    for cmdline in Path('/proc').glob('[0-9]*/cmdline'):
        try:argv=cmdline.read_bytes().split(bytes([0]))
        except OSError:continue
        if argv and argv[0] in [str(binary/name).encode() for name in ('fastlio_mapping','s100_livox_adapter')]:raise RuntimeError('已有FAST-LIO2评估/建图进程，请先退出后再启动')
    lock=exclusive(ROOT/'runtime/mapping.lock')
    folder=ROOT/'maps'/datetime.now().strftime('fastlio2_%Y%m%d_%H%M%S_%f');recorder=VoxelRecorder(folder,CONFIG['mapping']);recorder.validation_only=args.validation_only or args.replay is not None
    mutex=threading.RLock();stop=threading.Event();children=[];logs=[];stream=None;worker=None;writer=None;node=None;self_filter=None;self_pruned=0;ros_worker=None;health_worker=None
    health=dict(active=True,paused=False,input_ready=False,pid=os.getpid(),sensors={},invalid=None,folder=str(folder),backend='fast_lio2',ros_domain=92,readonly_test_inhibit=recorder.validation_only,motion_commands_sent=0,estimator_frames=0,raw_counts=[0]*4,source_dropped=[0]*4)
    expected_hash=hashlib.sha256((ROOT/'config/lidar_calibration.json').read_bytes()).hexdigest()
    T_BL=transforms()['front'];T_IL=np.eye(4);T_IL[:3,:3]=np.array(cfg['extrinsic_R']).reshape(3,3);T_IL[:3,3]=cfg['extrinsic_T'];T_IB=T_IL@np.linalg.inv(T_BL);anchor=[None]
    poses=collections.deque(maxlen=200);pending=collections.deque(maxlen=4);back_pending=collections.deque(maxlen=3);joints=collections.deque(maxlen=30);last_ingest=[-1e9];last_front=[None];last_pose=[None];source_done=threading.Event();raw_lock=threading.Lock()
    registered_guard=RegisteredScanGuard(timeout_s=2.)
    pointer='latest_validation_map.json' if recorder.validation_only else 'latest_map.json'
    def fail(exc):
        with mutex:health['worker_error']=str(exc);health['input_ready']=False;recorder.invalid_reason=recorder.invalid_reason or str(exc)
    def write_health():
        with mutex:
            registered_guard.tick(time.monotonic());health['registration_paused']=registered_guard.paused;health['registered_scan_guard']=registered_guard.summary()
            health['map_frames']=recorder.frames;health['map_cells']=len(recorder.cells)
            health['heartbeat_monotonic']=time.monotonic();health['invalid']=recorder.invalid_reason or health.get('worker_error');diagnostic=dict(health,readonly_test_inhibit=False);health['estimator_ready_diagnostic']=estimator_ready(diagnostic,time.monotonic(),cfg['estimator_timeout_s']);health['input_ready']=health['estimator_ready_diagnostic'] and not health['readonly_test_inhibit'] and not health['paused'];atomic_json(ROOT/'runtime/mapping_status.json',health)
    def record_frame(*args,**kwargs):
        began=time.perf_counter();recorder.ingest(*args,**kwargs);elapsed=time.perf_counter()-began
        with mutex:
            health['last_record_s']=elapsed;health['max_record_s']=max(elapsed,health.get('max_record_s',0.))
    def filter_for_map(points,T,packet,stamp):
        if self_filter is None:return points
        try:
            keep=self_filter.observe(points,T,packet,stamp)
            with mutex:health['self_filter']=self_filter.summary()
            return points[keep]
        except Exception as exc:
            # Retaining occupied returns is the conservative fallback.
            with mutex:health['self_filter_warning']=type(exc).__name__+': '+str(exc)
            return points
    def prune_self():
        nonlocal self_pruned
        if self_filter is None:return 0
        count=self_filter.prune_recorder(recorder);self_pruned+=count
        with mutex:health['self_filter_pruned_cells']=self_pruned;health['self_filter']=self_filter.summary()
        return count
    def save():
        with mutex:health['paused']=True;write_health()
        try:
            prune_self()
            if self_filter is not None:atomic_json(folder/'self_filter_report.json',dict(self_filter.summary(),pruned_cells_total=self_pruned,scope='mapping output only; FAST-LIO2 estimator still consumes full raw scans',navigation_usable=False))
            report=recorder.save();report.update(backend='fast_lio2',frame_id='session_base_start',source='FAST-LIO2 LiDAR-IMU full 6D poses and motion-undistorted registered scans',raw_bag=str(folder/'raw_bag'),lidar_imu_extrinsic_verified=cfg['lidar_imu_extrinsic_verified'],T_imu_lidar=T_IL.tolist(),T_base_lidar=T_BL.tolist(),map_from_native_world=anchor[0].tolist() if anchor[0] is not None else None,navigation_usable=False,manual_trajectory_deletion_applied=False,back_lidar_fused=True,frontend='front LiDAR 10Hz and front IMU for FAST-LIO2; back LiDAR 1Hz fused using time-interpolated 6D front-LIO poses',last_health=dict(health),self_filter_enabled=self_filter is not None,self_filter_report=str(folder/'self_filter_report.json') if self_filter is not None else None,full_lio_pose_topic='/fastlio/odometry',joint_feedback_topic='/fastlio/joint_feedback',keyframes_contain_online_self_filtered_output=self_filter is not None)
            atomic_json(folder/'map.json',report)
            if recorder.frames and (recorder.validation_only or not recorder.invalid_reason):atomic_json(ROOT/'runtime'/pointer,dict(folder=str(folder),map=str(folder/'map.json')))
            return report
        finally:
            with mutex:health['paused']=stop.is_set();write_health()
    def adapter(m):
        with mutex:
            health['adapter_error']=None if m.data=='OK' else m.data;health['adapter_completed_monotonic']=time.monotonic()
    def odometry(m):
        try:
            p=m.pose.pose.position;q=m.pose.pose.orientation;T=pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]);stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if last_pose[0] is not None:
                old_t,old_T=last_pose[0];dt=stamp-old_t
                if dt<=0 or np.linalg.norm(T[:3,3]-old_T[:3,3])>max(.15,2*dt) or np.linalg.norm(T[:3,:3]-old_T[:3,:3])>max(.3,4*dt):raise ValueError('LIO_POSE_OR_CLOCK_DISCONTINUITY_START_NEW_SESSION')
            last_pose[0]=(stamp,T);poses.append((stamp,T))
            if writer is not None:
                from rclpy.serialization import serialize_message
                with raw_lock:writer.write('/fastlio/odometry',serialize_message(m),m.header.stamp.sec*1000000000+m.header.stamp.nanosec)
                with mutex:health['lio_pose_archive_count']=health.get('lio_pose_archive_count',0)+1
            with mutex:health['estimator_completed_monotonic']=time.monotonic();health['estimator_frames']+=1
        except (ValueError,TypeError) as exc:fail(exc)
    def scan(m):pending.append((time.monotonic(),m))
    def ingest_pending():
        # Bounded work: inspect at most the queued snapshot and admit one frame.
        # A producer must not keep this loop alive and starve the heartbeat/rear.
        for _ in range(min(len(pending),4)):
            if not pending or stop.is_set():return
            arrival,m=pending[0];stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            history=list(poses);matched=min(history,key=lambda row:abs(row[0]-stamp)) if history else None
            if matched is None or abs(matched[0]-stamp)>.005:
                if time.monotonic()-arrival>.2:pending.popleft();continue
                break
            pending.popleft()
            if stamp-last_ingest[0]<1/cfg['max_map_hz']:continue
            with mutex:
                if health['paused'] or health.get('worker_error'):continue
            try:
                with mutex:current=registered_guard.check(last_front[0],stamp,time.monotonic(),time.monotonic()-arrival)
                if not current:
                    # A queued/late frame is not a broken coordinate system.
                    # Drop it; the bounded pending deque retains newer results.
                    write_health();continue
                T_WI=matched[1]
                if anchor[0] is None:anchor[0]=np.linalg.inv(T_IB)@np.linalg.inv(T_WI)
                T_MB=anchor[0]@T_WI@T_IB;world=decode_cloud(m);mapped=world@anchor[0][:3,:3].T+anchor[0][:3,3];base=(mapped-T_MB[:3,3])@T_MB[:3,:3]
                pose=[T_MB[0,3],T_MB[1,3],math.atan2(T_MB[1,0],T_MB[0,0])]
                recorded_joints=list(joints);packet=min(recorded_joints,key=lambda row:abs(row[0]-stamp))[1] if recorded_joints else {}
                if packet and abs(packet.get('observation_time',0)-stamp)>.3:packet={}
                base=filter_for_map(base,T_MB,packet,stamp)
                record_frame(base,stamp,pose,'front',T_BL[:3,3],packet,0,pose_matrix=T_MB);last_ingest[0]=stamp
                with mutex:registered_guard.commit(time.monotonic())
                return
            except (ValueError,KeyError) as exc:fail(exc)
    def ingest_back():
        if registered_guard.paused:return
        from rclpy.serialization import deserialize_message
        # Only one rear frame per scheduler round, even if new frames arrive.
        for _ in range(min(len(back_pending),1)):
            if stop.is_set():return
            arrival,raw,stamp,packet=back_pending[0]
            if anchor[0] is None or len(poses)<2:
                if time.monotonic()-arrival>.5:back_pending.popleft();continue
                break
            try:
                m=deserialize_message(raw,PointCloud2);points=decode_cloud(m);field=next(f for f in m.fields if f.name=='timestamp')
                if field.datatype!=8 or field.count!=1 or field.offset+8>m.point_step:raise ValueError('BACK_POINT_TIME_REQUIRED')
                dtype=np.dtype(dict(names=['t'],formats=[('>' if m.is_bigendian else '<')+'f8'],offsets=[field.offset],itemsize=m.point_step))
                times=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))['t'].reshape(-1)*1e-9
                valid=np.isfinite(points).all(1)&np.isfinite(times)&(np.linalg.norm(points,axis=1)>.5)&(np.linalg.norm(points,axis=1)<20.)
                points=points[valid];times=times[valid]
                if not len(points):raise ValueError('NO_VALID_BACK_POINTS')
                history=list(poses);T_WI=interpolate_poses(times,history);end=float(times.max());T_WI_end=interpolate_poses([end],history)[0]
                # Each rear return receives its own interpolated base pose.
                T_MB=np.einsum('ij,njk,kl->nil',anchor[0],T_WI,T_IB);T_end=anchor[0]@T_WI_end@T_IB;T_back=transforms()['back'];base=points@T_back[:3,:3].T+T_back[:3,3]
                world=np.einsum('nij,nj->ni',T_MB[:,:3,:3],base)+T_MB[:,:3,3];deskewed_base=(world-T_end[:3,3])@T_end[:3,:3];pose=[T_end[0,3],T_end[1,3],math.atan2(T_end[1,0],T_end[0,0])]
                if packet and abs(packet.get('observation_time',0)-end)>.3:packet={}
                deskewed_base=filter_for_map(deskewed_base,T_end,packet,end)
                record_frame(deskewed_base,end,pose,'back',T_back[:3,3],packet,0,pose_matrix=T_end)
                with mutex:
                    previous=health['sensors'].get('back',{});interval=stamp-previous.get('observation_time',stamp-1.)
                    health['sensors']['back']=dict(completed_monotonic=time.monotonic(),interval_s=interval,observation_time=stamp,processing_s=time.monotonic()-arrival)
                    health['back_fused_frames']=health.get('back_fused_frames',0)+1;health.pop('back_warning',None)
                back_pending.popleft()
            except ValueError as exc:
                if time.monotonic()-arrival>.5:
                    back_pending.popleft()
                    with mutex:health['back_warning']=str(exc)
                    continue
                break
            except Exception as exc:fail('BACK_FUSION_FAILED: '+str(exc));back_pending.popleft()

    def accept_raw(index,data,stamp,packet=None):
        if not 0<=index<4 or not data:raise ValueError('INVALID_RAW_INDEX_OR_PAYLOAD')
        now=time.monotonic()
        sec,nsec=struct.unpack_from('<iI' if data[1]==1 else '>iI',data,4)
        with raw_lock:writer.write(TOPICS[index][0],data,sec*1000000000+nsec)
        with mutex:
            health['raw_counts'][index]+=1
            if index==1:health['imu_completed_monotonic']=now
            if index==0:
                name='front';previous=health['sensors'].get(name,{});interval=stamp-previous.get('observation_time',stamp-(.1 if index==0 else 1.))
                health['sensors'][name]=dict(completed_monotonic=now,interval_s=interval,observation_time=stamp,processing_s=0.)
        if index in (0,2) and packet:
            from rclpy.serialization import serialize_message
            feedback=String();feedback.data=json.dumps(packet,allow_nan=False)
            with raw_lock:writer.write('/fastlio/joint_feedback',serialize_message(feedback),int(packet.get('observation_time',stamp)*1e9))
            with mutex:health['joint_feedback_archive_count']=health.get('joint_feedback_archive_count',0)+1
        if index==0:
            last_front[0]=stamp;joints.append((stamp,packet or {}));pub_cloud.publish(data)
        elif index==1:pub_imu.publish(data)
        elif index==2:back_pending.append((now,data,stamp,packet or {}))
    def collect():
        try:
            while not stop.is_set():
                try:h,data=stream.receive()
                except ValueError:
                    with mutex:health['transport_warning']='STALE_RAW_PACKET_DROPPED'
                    continue
                if h.get('kind')=='raw_status':
                    if h.get('calibration_sha256')!=expected_hash:raise ValueError('LIDAR_EXTRINSIC_VERSION_MISMATCH')
                    with mutex:health['source_status']=h;health['source_dropped']=h['dropped']
                    if h.get('error'):raise ValueError(h['error'])
                elif h.get('kind')=='raw':
                    if h.get('source_age_s',999)>.5 or h.get('queued_s',999)>.25:continue
                    accept_raw(h['topic_index'],data,h['stamp'],h.get('joints'))
                else:raise ValueError('UNEXPECTED_RAW_RECORD')
        except Exception as exc:
            if not stop.is_set():fail(type(exc).__name__+': '+str(exc))
    def replay():
        try:
            reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(args.replay),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'));start=time.monotonic();first=None
            packets=[];latest_packet={}
            # Older sessions stored joint snapshots in keyframes only.
            for f in sorted((args.replay.parent/'keyframes').glob('*.npz')):
                with np.load(f,allow_pickle=False) as d:
                    if 'joints_json' in d:
                        packet=json.loads(str(d['joints_json']))
                        if packet.get('observation_time') is not None:packets.append((packet['observation_time'],packet))
            packets.sort(key=lambda row:row[0]);packet_times=np.array([row[0] for row in packets]);start=time.monotonic()
            while not stop.is_set() and reader.has_next():
                topic,data,record_stamp=reader.read_next()
                if topic=='/fastlio/joint_feedback':
                    from rclpy.serialization import deserialize_message
                    latest_packet=json.loads(deserialize_message(data,String).data);continue
                index=next((i for i,(name,_) in enumerate(TOPICS) if name==topic),None)
                if index is None:continue
                if first is None:first=record_stamp
                target=start+(record_stamp-first)*1e-9/args.rate
                while not stop.is_set() and time.monotonic()<target:stop.wait(min(.02,target-time.monotonic()))
                if stop.is_set():break
                sec,nsec=struct.unpack_from('<iI' if data[1]==1 else '>iI',data,4);stamp=sec+nsec*1e-9;packet={}
                if index in (0,2):
                    if latest_packet and abs(latest_packet.get('observation_time',0)-stamp)<=.3:packet=latest_packet
                    elif len(packet_times):
                        nearest=int(np.argmin(abs(packet_times-stamp)))
                        if abs(packet_times[nearest]-stamp)<=.3:packet=packets[nearest][1]
                accept_raw(index,data,stamp,packet)
        except Exception as exc:fail('REPLAY_FAILED: '+str(exc))
        finally:source_done.set()
    def heartbeat():
        try:
            while not stop.is_set():
                write_health();stop.wait(.2)
        except Exception as exc:
            if not stop.is_set():fail('HEARTBEAT_FAILED: '+str(exc))
    def spin_ros():
        try:
            while not stop.is_set():rclpy.spin_once(node,timeout_sec=.02)
        except Exception as exc:
            if not stop.is_set():fail('ROS_RECEIVER_FAILED: '+str(exc))
    def child_death():
        ctypes.CDLL(None).prctl(1,signal.SIGINT)
        if os.getppid()==1:os._exit(1)
    def interrupted(*_):
        stop.set()
        with mutex:health['paused']=True;health['input_ready']=False;write_health()
    for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,interrupted)
    started=time.monotonic();input_buffer='';command_queue=collections.deque();announced=None;write_health()
    print('正在初始化 FAST-LIO2，并连接只读雷达/IMU 数据源…',flush=True)
    try:
        rclpy.init();node=rclpy.create_node('jetson_fastlio_mapping_recorder',enable_rosout=False,start_parameter_services=False)
        pub_cloud=node.create_publisher(PointCloud2,'/fastlio_eval/raw_front',5);pub_imu=node.create_publisher(Imu,'/fastlio_eval/imu',500)
        subscriptions=[node.create_subscription(Odometry,'/Odometry',odometry,100),node.create_subscription(PointCloud2,'/cloud_registered',scan,20),node.create_subscription(String,'/fastlio_eval/adapter_status',adapter,20)]
        writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(folder/'raw_bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
        for name,kind in TOPICS:writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=kind,serialization_format='cdr'))
        writer.create_topic(rosbag2_py.TopicMetadata(name='/fastlio/odometry',type='nav_msgs/msg/Odometry',serialization_format='cdr'))
        writer.create_topic(rosbag2_py.TopicMetadata(name='/fastlio/joint_feedback',type='std_msgs/msg/String',serialization_format='cdr'))
        if cfg.get('self_return_filter',{}).get('enabled'):
            from self_return_filter import SelfReturnFilter
            self_filter=SelfReturnFilter.from_config(cfg['self_return_filter'],resolution=recorder.res)
            print(f'自体滤除启用：模型过滤膨胀{self_filter.model_padding*100:.1f}cm，表面容差{self_filter.surface_margin*100:.1f}cm；环境见证冲突保留。',flush=True)
        parameters=yaml.safe_load((workspace/'config/front_trial.yaml').read_text());p=parameters['/**']['ros__parameters'];p['mapping']['extrinsic_T']=cfg['extrinsic_T'];p['mapping']['extrinsic_R']=cfg['extrinsic_R'];p['mapping']['extrinsic_est_en']=False;p['filter_size_surf']=.1;p['filter_size_map']=.1;p['pcd_save']['pcd_save_en']=False;p['publish']['map_en']=False;p['map_file_path']=str(folder/'native_map.pcd')
        (folder/'fastlio_parameters.yaml').write_text(yaml.safe_dump(parameters))
        for name in ('fastlio_mapping','s100_livox_adapter'):
            logfile=(folder/(name+'.log')).open('ab');logs.append(logfile);argv=[str(binary/name)]
            if name=='fastlio_mapping':argv+=['--ros-args','--params-file',str(folder/'fastlio_parameters.yaml')]
            children.append(subprocess.Popen(argv,stdout=logfile,stderr=logfile,start_new_session=True,preexec_fn=child_death))
        warm=time.monotonic()
        while not stop.is_set() and time.monotonic()-warm<3.:rclpy.spin_once(node,timeout_sec=.03)
        ros_worker=threading.Thread(target=spin_ros,daemon=True);ros_worker.start()
        health_worker=threading.Thread(target=heartbeat,daemon=True);health_worker.start()
        if args.replay:worker=threading.Thread(target=replay,daemon=True)
        else:
            from fast_raw_client import RawStream
            stream=RawStream();worker=threading.Thread(target=collect,daemon=True)
        worker.start();last_heartbeat=0.;last_self_prune=time.monotonic();completed=None
        print('FAST-LIO2 Jetson 建图 | 前雷达10Hz+IMU定位，后雷达1Hz融合 | S100只读采集\n'+str(folder)+'\nstatus 查看；save 保存；quit 保存退出。',flush=True)
        print('雷达-IMU外参尚未独立验收；地图暂不接自动导航。'+(' 当前为只读测试，遥操联动禁行。' if recorder.validation_only else ' 初始化/估计过期时遥操暂停，健康后沿用建图限速。'),flush=True)
        while not stop.is_set():
            stop.wait(.005);ingest_pending();ingest_back();now=time.monotonic()
            if any(child.poll() is not None for child in children):fail('FAST_LIO2_OR_ADAPTER_EXITED');break
            if health.get('worker_error'):break
            if args.duration and now-started>=args.duration:break
            if args.replay and source_done.is_set():
                if completed is None:completed=now
                if now-completed>1.:break
            if self_filter is not None and now-last_self_prune>=5.:prune_self();last_self_prune=now
            if now-last_heartbeat>.2:
                write_health();last_heartbeat=now
                state=health['estimator_ready_diagnostic']
                if state!=announced:
                    print(('只读测试采集就绪，遥操联动禁行。' if recorder.validation_only else '建图就绪，遥操沿用建图限速。') if state else '等待当前雷达、IMU及估计结果；遥操暂停。',flush=True);announced=state
            if not args.duration and not args.replay:
                if select.select([sys.stdin],[],[],0)[0]:
                    data=os.read(sys.stdin.fileno(),4096)
                    if not data:command_queue.append('quit')
                    else:
                        input_buffer+=data.decode(errors='replace')
                        while '\n' in input_buffer:
                            line,input_buffer=input_buffer.split('\n',1);command_queue.append(line.strip().lower())
                if command_queue:
                    line=command_queue.popleft()
                    if line in ('quit','q','exit'):break
                    if line in ('save','p'):
                        report=save();print('已保存 '+str(report['frames'])+' 帧 / '+str(report['cells'])+' 体素：'+str(folder),flush=True)
                    else:
                        with mutex:print(json.dumps(dict(health,frames=recorder.frames,cells=len(recorder.cells)),ensure_ascii=False),flush=True)
    except Exception as exc:
        fail(type(exc).__name__+': '+str(exc));print(str(exc),file=sys.stderr,flush=True)
    finally:
        with mutex:health['paused']=True;write_health()
        stop.set()
        if health_worker:health_worker.join(timeout=3)
        if stream:stream.close()
        if worker:worker.join(timeout=5)
        if worker and worker.is_alive():fail('RAW_WORKER_DID_NOT_STOP')
        if ros_worker:ros_worker.join(timeout=3)
        if ros_worker and ros_worker.is_alive():fail('ROS_RECEIVER_DID_NOT_STOP')
        if writer is not None:
            with raw_lock:writer=None
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=3)
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if not recorder.frames:fail('NO_REGISTERED_FRAMES_SAVED')
        try:save()
        finally:
            with mutex:health['active']=False;health['paused']=True;health['input_ready']=False;write_health()
        if self_filter is not None:self_filter.close()
        for logfile in logs:logfile.close()
        print('已保存 '+str(recorder.frames)+' 帧去畸变关键帧，状态：'+str(recorder.invalid_reason or '正常')+'\n'+str(folder),flush=True)
    return 1 if recorder.invalid_reason else 0

if __name__=='__main__':sys.exit(main())

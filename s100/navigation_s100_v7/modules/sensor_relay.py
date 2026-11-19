"""S100 read-only ROS subscriptions and fair latest-only local pipe delivery."""
import argparse,hashlib,json,os,signal,sys,threading,time,subprocess
from pathlib import Path
import numpy as np
from common import ROOT,CONFIG,exclusive,atomic_json
from continuity import OdometryHistory
from cloud import decode_cloud
from wire import read_packet,write_packet
from joint_scan_history import JointScanHistory
from relay_scheduling import FrameSchedule
from async_diagnostics import AsyncDiagnostics

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--localization',action='store_true');parser.add_argument('--consumer',choices=['default','tracker','observer'],default='default');parser.add_argument('--motor-trace',type=Path);parser.add_argument('--sensor-hz',type=float);args=parser.parse_args()
    name=('localization_sensor_relay' if args.localization else 'sensor_relay')+('_'+args.consumer if args.consumer!='default' else '')
    lock=exclusive(ROOT/('runtime/'+name+'.lock'))
    hz=args.sensor_hz if args.sensor_hz is not None else 1. if args.consumer=='observer' else 2.
    schedule=FrameSchedule(hz);boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if os.environ.get('ROS_DOMAIN_ID')!='0':raise RuntimeError('Expected production read-only domain 0')
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException
    rclpy.init();node=rclpy.create_node('tianji_v7_readonly_'+name,enable_rosout=False,start_parameter_services=False)
    stop=threading.Event();mutex=threading.Lock();history=OdometryHistory();token=[None];last_odom=[None];error=[None];odom_count=[0]
    calibration_hash=hashlib.sha256((ROOT/'config/lidar_calibration.json').read_bytes()).hexdigest()
    metrics_file=args.motor_trace.parent/'sensor_relay_metrics.json' if args.motor_trace else ROOT/'runtime'/(name+'_metrics.json')
    metrics_log=args.motor_trace.parent/'sensor_relay_metrics.jsonl' if args.motor_trace else None
    def odom(msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link' or max(abs(q.x),abs(q.y),abs(q.z))>1e-7 or abs(p.z)>.05:return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        with mutex:
            odom_count[0]+=1
            if history.add(stamp,[p.x,p.y,q.w]):last_odom[0]=time.monotonic()
    def cloud(sensor,msg):
        with mutex:
            if msg.header.frame_id!='lidar_'+sensor:schedule.counts[sensor]['wrong_frame_id']+=1;return
            schedule.offer(sensor,msg,msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,time.monotonic())
    subs=[node.create_subscription(Odometry,'/controller/odom',odom,QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT))]
    for sensor in ['front','back']:
        subs.append(node.create_subscription(PointCloud2,f'/driver/lidar/lidar_{sensor}/point_cloud/Data',lambda msg,s=sensor:cloud(s,msg),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)))
    def challenges():
        try:
            while not stop.is_set():
                packet,points=read_packet(sys.stdin.buffer)
                if packet.get('kind')!='sensor_challenge' or points is not None or not isinstance(packet.get('challenge'),str):raise ValueError('INVALID_SENSOR_CHALLENGE')
                with mutex:token[0]=packet['challenge']
        except (EOFError,OSError,ValueError):stop.set()
    def interrupted(*_):stop.set()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
    motor_trace=None
    if args.motor_trace is not None:
        motor_trace=subprocess.Popen([sys.executable,str(ROOT/'modules/motor_trace_worker.py'),'--output',str(args.motor_trace)],stdout=sys.stderr,stderr=sys.stderr)
    executor=SingleThreadedExecutor();executor.add_node(node)
    def spin_loop():
        try:
            while not stop.is_set() and rclpy.ok():executor.spin_once(timeout_sec=.05)
        except ExternalShutdownException:stop.set()
    def send(header,points=None):
        # Both ends are S100-local. Raw float32 avoids needless zlib CPU work;
        # the original observation timestamp remains the freshness authority.
        header=dict(header,source_emitted_monotonic=time.monotonic())
        if 'observation_time'in header:header['source_age_s']=max(0.,time.time()-header['observation_time'])
        write_packet(sys.stdout.buffer,header,points,point_encoding='raw_f32_le')
    def write_metrics(data):
        atomic_json(metrics_file,data)
        if metrics_log:
            with metrics_log.open('a')as handle:handle.write(json.dumps(data,allow_nan=False)+'\n')
    diagnostic_io=AsyncDiagnostics(write_metrics,capacity=4)
    def metrics(running=True):
        now=time.monotonic();wall=time.time()
        with mutex:
            sensors=schedule.snapshot(now,wall);last=history.rows[-1][0]if history.rows else None
            data=dict(updated_wall_time=wall,running=running,sensors=sensors,odom_received=odom_count[0],latest_odometry_source_age_s=None if last is None else wall-last,error=error[0],motion_commands_sent=0,payload_codec='raw_f32_le_local_pipe')
        data['diagnostic_io']=diagnostic_io.stats();diagnostic_io.submit(data)
        return data
    joint_history=JointScanHistory();last_joint_sample=0.
    threading.Thread(target=challenges,daemon=True).start();spin=threading.Thread(target=spin_loop,daemon=True);spin.start();sequence=0;status_time=0.;metrics_time=0.;odom_time=0.
    try:
        while not stop.wait(.005):
            now=time.monotonic()
            if now-last_joint_sample>=.04:
                last_joint_sample=now
                try:
                    fast=Path('/dev/shm/tianji_s100_joint_feedback.json');data=json.loads((fast if fast.exists() else Path(CONFIG['joint_status'])).read_text());joint_history.sample(data,time.time())
                except (OSError,ValueError):pass
            with mutex:
                challenge=token[0];event=history.event
                job=schedule.take(now,time.time(),history)if challenge else None
            if not challenge:continue
            if args.localization and now-odom_time>=.05:
                with mutex:row=history.rows[-1]if history.rows else None;arrival=last_odom[0]
                if row is not None and arrival is not None and now-arrival<.2:
                    send(dict(kind='odometry',challenge=challenge,observation_time=row[0],pose=row[1],epoch=row[2],boot_id=boot_id,calibration_sha256=calibration_hash))
                odom_time=now
            if job:
                sensor,msg,arrival,stamp,matched=job
                try:
                    began=time.monotonic();points=decode_cloud(msg);valid=np.isfinite(points).all(1)&(np.linalg.norm(points,axis=1)>=.04);points=points[valid]
                    if not 0<len(points)<=100000:raise ValueError('CLOUD_POINT_BUDGET_EXCEEDED')
                    joint=joint_history.nearest(stamp);decoded=time.monotonic();sequence+=1
                    with mutex:challenge=token[0]
                    header=dict(kind='frame',sequence=sequence,challenge=challenge,sensor=sensor,observation_time=stamp,odom_observation_time=matched[0],pose=matched[1],epoch=matched[2],boot_id=boot_id,joints=joint,calibration_sha256=calibration_hash,source_processing_s=decoded-began,continuity_event=event,raw_point_frame='sensor',joint_history_diagnostics=joint_history.diagnostics())
                    send(header,points);done=time.monotonic()
                    with mutex:
                        schedule.sent(sensor,stamp,done,dict(joint_history_diagnostics=joint_history.diagnostics(),callback_to_decode_start_s=began-arrival,decode_s=decoded-began,pipe_send_s=done-decoded,source_age_at_send_complete_s=time.time()-stamp,joint_evidence_valid=bool(joint),sequence=sequence));error[0]=None
                except ValueError as exc:
                    with mutex:schedule.failed(sensor,str(exc));error[0]=str(exc)
            if now-status_time>=.2:
                with mutex:
                    challenge=token[0];event=history.event;epoch=history.epoch;odom_age=None if last_odom[0]is None else now-last_odom[0];sensors=schedule.snapshot(time.monotonic(),time.time())
                send(dict(kind='sensor_status',challenge=challenge,epoch=epoch,continuity_event=event,odom_age_s=odom_age,error=error[0],frames={s:v['counts'].get('forwarded',0)for s,v in sensors.items()},sensors=sensors,calibration_sha256=calibration_hash));status_time=now
            if now-metrics_time>=1.:metrics();metrics_time=now
    finally:
        stop.set();spin.join(timeout=2);executor.shutdown(timeout_sec=1.)
        if motor_trace and motor_trace.poll()is None:
            motor_trace.terminate()
            try:motor_trace.wait(timeout=3)
            except subprocess.TimeoutExpired:motor_trace.kill();motor_trace.wait(timeout=1)
        metrics(False);diagnostic_io.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        lock.close()

if __name__=='__main__':
    try:main()
    except (EOFError,BrokenPipeError):pass

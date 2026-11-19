"""S100 read-only subscriptions. No publishers, controller APIs or map building."""
import argparse,hashlib,json,os,signal,sys,threading,time
from pathlib import Path
import numpy as np
from common import ROOT,CONFIG,exclusive
from continuity import OdometryHistory
from cloud import decode_cloud
from wire import read_packet,write_packet

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--localization',action='store_true');args=parser.parse_args()
    lock=exclusive(ROOT/('runtime/localization_sensor_relay.lock' if args.localization else 'runtime/sensor_relay.lock'))
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if os.environ.get('ROS_DOMAIN_ID')!='0':raise RuntimeError('Expected production read-only domain 0')
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init();node=rclpy.create_node('tianji_jetson_v5_readonly_'+('localization_relay' if args.localization else 'sensor_relay'),enable_rosout=False,start_parameter_services=False)
    stop=threading.Event();mutex=threading.Lock();history=OdometryHistory();pending={};last={};token=[None];last_odom=[None];error=[None];counts={s:0 for s in ['front','back']}
    calibration_hash=hashlib.sha256((ROOT/'config/lidar_calibration.json').read_bytes()).hexdigest()
    def odom(msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link' or max(abs(q.x),abs(q.y),abs(q.z))>1e-7 or abs(p.z)>.05:return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        with mutex:
            if history.add(stamp,[p.x,p.y,q.w]):last_odom[0]=time.monotonic()
    def cloud(sensor,msg):
        if msg.header.frame_id=='lidar_'+sensor:
            with mutex:pending[sensor]=(msg,time.monotonic())
    subs=[node.create_subscription(Odometry,'/controller/odom',odom,QoSProfile(depth=400,reliability=ReliabilityPolicy.BEST_EFFORT))]
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
    threading.Thread(target=challenges,daemon=True).start();spin=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);spin.start();sequence=0;status_time=0;odom_time=0.
    try:
        while not stop.wait(.01):
            now=time.monotonic();job=None
            with mutex:
                challenge=token[0];event=history.event
                if challenge:
                    for sensor in ['front','back']:
                        if sensor in pending and now-last.get(sensor,-1e9)>=1/CONFIG['mapping']['max_sensor_hz']:
                            msg,arrival=pending.pop(sensor);stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                            matched=history.nearest(stamp);job=(sensor,msg,arrival,stamp,matched);last[sensor]=now;break
            if not challenge:continue
            if args.localization and now-odom_time>=.05:
                with mutex:row=history.rows[-1] if history.rows else None;arrival=last_odom[0]
                if row is not None and arrival is not None and now-arrival<.2:
                    write_packet(sys.stdout.buffer,dict(kind='odometry',challenge=challenge,observation_time=row[0],pose=row[1],epoch=row[2],boot_id=boot_id,source_age_s=max(0.,time.time()-row[0]),calibration_sha256=calibration_hash))
                odom_time=now
            if job:
                sensor,msg,arrival,stamp,matched=job
                age=time.time()-stamp
                if now-arrival>.5 or matched is None or not -.05<=age<=.7:
                    error[0]='WAIT_CURRENT_CLOUD_ODOMETRY_PAIR'
                else:
                    try:
                        started=time.monotonic();points=decode_cloud(msg);valid=np.isfinite(points).all(1)&(np.linalg.norm(points,axis=1)>=.04);points=points[valid]
                        if not 0<len(points)<=100000:raise ValueError('CLOUD_POINT_BUDGET_EXCEEDED')
                        joint={}
                        try:
                            data=json.loads(Path(CONFIG['joint_status']).read_text());t=data.get('last_arm_receive_wall_time',0.)
                            if data.get('arm_fresh') and data.get('model_hash_match') and abs(t-stamp)<.3:
                                joint=dict(joint_positions_rad=data['joint_positions_rad'],observation_time=t,model_sha256=data['whole_model_sha256'],live=False,source='recorded_measured_feedback')
                        except (OSError,ValueError,KeyError):pass
                        sequence+=1
                        with mutex:challenge=token[0]
                        header=dict(kind='frame',sequence=sequence,challenge=challenge,sensor=sensor,observation_time=stamp,odom_observation_time=matched[0],pose=matched[1],epoch=matched[2],boot_id=boot_id,joints=joint,calibration_sha256=calibration_hash,source_age_s=max(0.,time.time()-stamp),source_processing_s=time.monotonic()-started,continuity_event=event,raw_point_frame='sensor')
                        write_packet(sys.stdout.buffer,header,points);counts[sensor]+=1;error[0]=None
                    except ValueError as exc:error[0]=str(exc)
            if now-status_time>=.2:
                with mutex:challenge=token[0];event=history.event;epoch=history.epoch;odom_age=None if last_odom[0] is None else now-last_odom[0]
                write_packet(sys.stdout.buffer,dict(kind='sensor_status',challenge=challenge,epoch=epoch,continuity_event=event,odom_age_s=odom_age,error=error[0],frames=counts,calibration_sha256=calibration_hash));status_time=now
    finally:
        stop.set()
        if rclpy.ok():rclpy.shutdown()
        spin.join(timeout=2);node.destroy_node()

if __name__=='__main__':
    try:main()
    except (EOFError,BrokenPipeError):pass

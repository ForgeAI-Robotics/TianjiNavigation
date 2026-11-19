"""S100 read-only full-rate front LiDAR/IMU, 1 Hz back and 10 Hz odometry.
Raw CDR and source-clock metadata only; no publisher, SDK, or control access.
"""
import hashlib,json,queue,signal,struct,sys,threading,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Imu
from nav_msgs.msg import Odometry
from common import ROOT,CONFIG,exclusive
from wire import read_packet
TOPICS=[('/driver/lidar/lidar_front/point_cloud/Data',PointCloud2),('/driver/lidar/lidar_front/imu/Data',Imu),('/driver/lidar/lidar_back/point_cloud/Data',PointCloud2),('/controller/odom',Odometry)]

def main():
    calibration_hash=hashlib.sha256((ROOT/'config/lidar_calibration.json').read_bytes()).hexdigest();lock=exclusive(ROOT/'runtime/sensor_relay.lock');rclpy.init();node=rclpy.create_node('jetson_fastlio_readonly_source',enable_rosout=False,start_parameter_services=False)
    stop=threading.Event();pending=queue.Queue(maxsize=256);counts=[0]*4;dropped=[0]*4;last=[-1e9]*4;last_stamp=[None]*4;error=[None];last_drop=[-1e9];token=[None,0.];joints=[{}];subs=[]
    def challenges():
        try:
            while not stop.is_set():
                h,p=read_packet(sys.stdin.buffer)
                if p is not None or h.get('kind')!='raw_challenge':raise ValueError('RAW_CHALLENGE_ONLY')
                token[:]=[h['challenge'],time.monotonic()]
        except (EOFError,OSError,ValueError,KeyError):stop.set()
    def receive(index,data):
        now=time.monotonic()
        if index==2 and now-last[index]<1.:return
        if index==3 and now-last[index]<.1:return
        if len(data)<12 or data[:2] not in (b'\x00\x01',b'\x00\x00'):error[0]='UNSUPPORTED_CDR';return
        sec,nsec=struct.unpack_from('<iI' if data[1]==1 else '>iI',data,4);stamp=sec+nsec*1e-9
        if sec<=0 or nsec>=1000000000:return
        if last_stamp[index] is not None and stamp<last_stamp[index]-.5:error[0]='SENSOR_CLOCK_REWIND_START_NEW_SESSION';stop.set();return
        age=time.time()-stamp
        if not -.05<=age<=.5:dropped[index]+=1;last_drop[0]=now;return
        last_stamp[index]=stamp;last[index]=now
        try:pending.put_nowait((index,stamp,now,bytes(data)));counts[index]+=1
        except queue.Full:dropped[index]+=1;last_drop[0]=now
    for i,(topic,kind) in enumerate(TOPICS):subs.append(node.create_subscription(kind,topic,lambda raw,i=i:receive(i,raw),QoSProfile(depth=100 if i==1 else 5,reliability=ReliabilityPolicy.BEST_EFFORT),raw=True))
    def write(header,data=b''):
        header['challenge']=token[0];blob=json.dumps(header,allow_nan=False,separators=(',',':')).encode()
        sys.stdout.buffer.write(struct.pack('!II',len(blob),len(data))+blob+data);sys.stdout.buffer.flush()
    def writer():
        reported=0.
        try:
            while not stop.is_set():
                now=time.monotonic()
                if now-token[1]>.75:
                    if token[0] is not None:stop.set()
                    stop.wait(.01);continue
                try:index,stamp,arrival,data=pending.get(timeout=.02)
                except queue.Empty:index=None
                if index is not None:
                    if now-arrival>.25:dropped[index]+=1;last_drop[0]=now
                    else:
                        joint=joints[0] if index in (0,2) and abs(joints[0].get('observation_time',0)-stamp)<=.3 else {}
                        write(dict(kind='raw',topic_index=index,stamp=stamp,source_age_s=max(0.,time.time()-stamp),queued_s=max(0.,now-arrival),joints=joint),data)
                if now-reported>.2:
                    write(dict(kind='raw_status',counts=counts,dropped=dropped,error=error[0],last_drop_age_s=now-last_drop[0],source_ages_s=[None if t is None else max(0.,time.time()-t) for t in last_stamp],motion_commands_sent=0,calibration_sha256=calibration_hash));reported=now
        except (BrokenPipeError,OSError):stop.set()
    threading.Thread(target=challenges,daemon=True).start();threading.Thread(target=writer,daemon=True).start()
    for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,lambda *_:stop.set())
    polled=0.
    try:
        while not stop.is_set():
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()-polled>.1:
                polled=time.monotonic()
                try:
                    q=json.loads(Path(CONFIG['joint_status']).read_text());t=q.get('last_arm_receive_wall_time',0.)
                    joints[0]=dict(joint_positions_rad=q['joint_positions_rad'],observation_time=t,model_sha256=q['whole_model_sha256'],live=False,source='recorded_measured_feedback') if q.get('arm_fresh') and q.get('model_hash_match') else {}
                except (OSError,ValueError,KeyError):joints[0]={}
    finally:
        stop.set();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()

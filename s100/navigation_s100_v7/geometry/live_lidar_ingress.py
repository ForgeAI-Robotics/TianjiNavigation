#!/usr/bin/env python3
"""Passive production subscribers -> tmpfs -> domain80 localhost only.

Production roles never create ROS publishers (including rosout/services).
The isolated role publishes exclusively fixed developer input topics.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse
import collections
import hashlib
import json
from pathlib import Path
import socket
import time
import uuid
from link import JetsonLink,JOINTS
from ros_adapter import ROOT,PREFIX,isolation_check
from wired_clock import ClockMap,probe


def atomic(path,data):
    # front.cdr and front.view share a stem but are written by different
    # processes. Preserve the extension and writer PID to avoid cross-writes.
    tmp=path.with_name(path.name+'.'+str(os.getpid())+'.tmp');tmp.write_bytes(data);tmp.replace(path)


def signature(msg):
    # CDR alignment padding may differ after serialization. Compare all semantic
    # fields and every point-record byte, rather than uninitialized wire padding.
    metadata=dict(frame=msg.header.frame_id,sec=msg.header.stamp.sec,nsec=msg.header.stamp.nanosec,
        height=msg.height,width=msg.width,point_step=msg.point_step,row_step=msg.row_step,
        bigendian=msg.is_bigendian,dense=msg.is_dense,
        fields=[(f.name,f.offset,f.datatype,f.count) for f in msg.fields])
    h=hashlib.sha256(json.dumps(metadata,sort_keys=True).encode());h.update(bytes(msg.data))
    return h.hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role',choices=['clouds','joints','isolated'])
    parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=100.)
    parser.add_argument('--observe-only',action='store_true')
    args=parser.parse_args();args.directory.mkdir(parents=True,exist_ok=True)
    if args.role=='isolated':isolation_check()
    elif os.environ.get('ROS_DOMAIN_ID')!='0':raise ValueError('Passive input requires domain0')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rclpy.serialization import serialize_message,deserialize_message
    from sensor_msgs.msg import JointState,PointCloud2
    from std_msgs.msg import String
    rclpy.init(args=[])
    node=Node('tianji_readonly_ingress_'+args.role,enable_rosout=False,start_parameter_services=False)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    subs=[];started=time.monotonic();last={};counts=collections.Counter();audit=[]
    final=dict(role=args.role,motion_commands_sent=0,production_publishers_created=0)
    clock=None;sock=None;probe_audit=[]
    if args.role=='clouds':
        def receive_cloud(sensor,msg):
            now=time.monotonic()
            if now-last.get(sensor,0)<.09:return
            last[sensor]=now
            if msg.header.frame_id!='lidar_'+sensor:
                counts['wrong_frame']+=1;return
            atomic(args.directory/(sensor+'.cdr'),serialize_message(msg));counts[sensor]+=1
        for sensor in ('front','back'):
            subs.append(node.create_subscription(PointCloud2,'/driver/lidar/lidar_'+sensor+'/point_cloud/Data',
                lambda msg,s=sensor:receive_cloud(s,msg),qos))
    elif args.role=='joints':
        whole=json.loads((ROOT/'model_manifest.json').read_text())['portable_urdf_sha256']
        perception=json.loads((ROOT/'clearance_model_manifest.json').read_text())['portable_urdf_sha256']
        link=JetsonLink(whole,perception,str(uuid.uuid4()));clock=ClockMap()
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.settimeout(.015)
        model_ok=[False];last_generation=[None];latest_clock=[None]
        durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def model_info(msg):
            try:model_ok[0]=json.loads(msg.data).get('portable_urdf_sha256')==whole
            except (ValueError,AttributeError):model_ok[0]=False
        def joint(msg):
            try:
                estimate=clock.estimate();latest_clock[0]=estimate
                if estimate['generation']!=last_generation[0]:
                    link.history=type(link.history)();last_generation[0]=estimate['generation']
                if not model_ok[0]:raise ValueError('MODEL_NOT_VERIFIED')
                original=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                converted=original-estimate['remote_minus_local_s'];now=time.time()
                if not -estimate['uncertainty_s']<=now-converted<=.25-estimate['uncertainty_s']:
                    raise ValueError('MAPPED_JOINT_STALE_OR_FUTURE')
                if not link.history.add(list(msg.name),list(msg.position),converted):
                    raise ValueError('JOINT_ORDER_OR_VALUES_INVALID')
                counts['joint_messages']+=1
                if len(msg.name)==14:
                    counts['arm_messages']+=1
                    if len(audit)<6000:audit.append(dict(source_stamp=original,local_stamp=converted,
                        received_at=now,clock_mapping=estimate))
            except ValueError as exc:counts[str(exc)]+=1
        subs.append(node.create_subscription(String,'/tianji/model_info',model_info,durable))
        subs.append(node.create_subscription(JointState,'/tianji/joint_states',joint,QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)))
    else:
        pubs={} if args.observe_only else {s:node.create_publisher(PointCloud2,PREFIX+'/input/lidar_'+s,qos) for s in ('front','back')}
        jp=None if args.observe_only else node.create_publisher(String,PREFIX+'/jetson/assessment',1)
        observations=[];output={};published={};pending={};hashes={};rawclouds={};comparisons=[]
        def on_status(msg):
            try:observations.append(json.loads(msg.data))
            except ValueError:pass
        def on_retained(sensor,msg):
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            key=(sensor,stamp);digest=signature(msg)
            if key in hashes:
                comparisons.append(dict(sensor=sensor,stamp=stamp,raw_record_identical=digest==hashes[key]))
            counts[sensor+'_retained']+=1
            if key in rawclouds:
                atomic(args.directory/(sensor+'_before.cdr'),rawclouds[key]);atomic(args.directory/(sensor+'_after.cdr'),serialize_message(msg))
        def on_labeled(sensor,msg):
            counts[sensor+'_labeled']+=1
            atomic(args.directory/(sensor+'_labeled.cdr'),serialize_message(msg))
        subs.append(node.create_subscription(String,PREFIX+'/s100/lidar_shapemask/status',on_status,100))
        for sensor in ('front','back'):
            subs.append(node.create_subscription(PointCloud2,PREFIX+'/s100/lidar_'+sensor+'/retained',lambda msg,s=sensor:on_retained(s,msg),10))
            subs.append(node.create_subscription(PointCloud2,PREFIX+'/s100/lidar_'+sensor+'/labeled',lambda msg,s=sensor:on_labeled(s,msg),10))
    try:
        while time.monotonic()-started<args.duration:
            rclpy.spin_once(node,timeout_sec=.005)
            mono=time.monotonic()
            if args.role=='joints':
                if mono-last.get('probe',0)>=.10:
                    last['probe']=mono;probe(sock,clock)
                    if clock.rows and len(probe_audit)<3000:
                        probe_audit.append(dict(clock.rows[-1],generation=clock.generation))
                if mono-last.get('packet',0)>=.05:
                    last['packet']=mono
                    try:
                        estimate=clock.estimate()
                        now=time.time();packet=link.packet({},now);packet['clock_mapping']=estimate
                        packet['motion_history']=link.history.envelope(now-.4,now)
                        if model_ok[0]:atomic(args.directory/'assessment.json',json.dumps(packet,allow_nan=False).encode())
                    except ValueError:pass
            elif args.role=='isolated':
                if mono-last.get('packet',0)>=.05:
                    last['packet']=mono
                    try:
                        packet=json.loads((args.directory/'assessment.json').read_text())
                        if 0<=time.time()-packet['generated_at']<=.2:
                            if jp:jp.publish(String(data=json.dumps(packet)))
                            counts['packets']+=1
                    except (OSError,ValueError,KeyError):pass
                for sensor in ('front','back'):
                    path=args.directory/(sensor+'.cdr')
                    try:
                        version=path.stat().st_mtime_ns
                        if version==last.get(sensor):continue
                        raw=path.read_bytes();msg=deserialize_message(raw,PointCloud2)
                        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                        if not -.01<=time.time()-stamp<=.4:continue
                        key=(sensor,stamp);hashes[key]=signature(msg);rawclouds[key]=raw
                        if len(hashes)>32:
                            oldest=next(iter(hashes));hashes.pop(oldest);rawclouds.pop(oldest,None)
                        if sensor in pubs:pubs[sensor].publish(msg)
                        last[sensor]=version;counts[sensor+'_observed']+=1
                    except (OSError,ValueError):pass
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except RuntimeError:
        if rclpy.ok():raise
    finally:
        final.update(counts=dict(counts),duration_s=time.monotonic()-started)
        if args.role=='joints':
            final.update(timestamp_audit=audit,probe_audit=probe_audit,model_verified=model_ok[0]);sock.close()
        if args.role=='isolated':final.update(statuses=observations,retained_comparisons=comparisons)
        (args.directory/(args.role+'_report.json')).write_text(json.dumps(final,indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()

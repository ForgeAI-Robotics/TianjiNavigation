#!/usr/bin/env python3
"""Passive production LiDAR capture only. No filter output or control publisher."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse
import json
from pathlib import Path
import time
import numpy as np
from environment import decode_cloud

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    rclpy.init(args=[])
    node=Node('tianji_lidar_readonly_capture',enable_rosout=False,start_parameter_services=False)
    qos=QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,reliability=ReliabilityPolicy.BEST_EFFORT)
    topics={'front':'/driver/lidar/lidar_front/point_cloud/Data',
            'back':'/driver/lidar/lidar_back/point_cloud/Data',
            'merged':'/driver/lidar/point_cloud/Data',
            'before_dyn':'/cloud_before_dyn_filter','after_dyn':'/cloud_after_dyn_filter'}
    clouds={s:[] for s in topics};records={s:[] for s in topics};last={};odom=[];subscriptions=[]
    def cloud(sensor,msg):
        now=time.time()
        if now-last.get(sensor,0)<.48 or len(records[sensor])>=16:return
        last[sensor]=now
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        points=decode_cloud(msg);clouds[sensor].append(points.astype(np.float32))
        state={}
        try:state=json.loads(Path('/home/sunrise/tianji_state_bridge/runtime/receiver_status.json').read_text())
        except (OSError,ValueError):pass
        records[sensor].append(dict(frame=msg.header.frame_id,stamp=stamp,received_at=now,
            points=len(points),receiver_status=state,
            fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in msg.fields]))
        if len(records[sensor])==1:
            (args.output/(sensor+'_first_raw.bin')).write_bytes(bytes(msg.data))
            records[sensor][-1]['raw_layout']=dict(height=msg.height,width=msg.width,
                point_step=msg.point_step,row_step=msg.row_step,is_bigendian=msg.is_bigendian)
    def odometry(msg):
        if len(odom)>=400:return
        p=msg.pose.pose.position;t=msg.twist.twist
        odom.append(dict(received_at=time.time(),position=[p.x,p.y,p.z],
            linear=[t.linear.x,t.linear.y,t.linear.z],angular=[t.angular.x,t.angular.y,t.angular.z]))
    for sensor,topic in topics.items():
        subscriptions.append(node.create_subscription(PointCloud2,topic,lambda msg,s=sensor:cloud(s,msg),qos))
    subscriptions.append(node.create_subscription(Odometry,'/controller/odom',odometry,qos))
    started=time.monotonic()
    try:
        while time.monotonic()-started<10 and not all(len(v)>=16 for v in records.values()):
            rclpy.spin_once(node,timeout_sec=.05)
        graph=[dict(topic=name,types=types,publishers=node.count_publishers(name),
                    publisher_nodes=[info.node_namespace.rstrip('/')+'/'+info.node_name for info in node.get_publishers_info_by_topic(name)])
               for name,types in node.get_topic_names_and_types() if 'sensor_msgs/msg/PointCloud2' in types]
    finally:node.destroy_node();rclpy.shutdown()
    arrays={}
    for sensor,frames in clouds.items():
        arrays[sensor]=np.concatenate(frames) if frames else np.empty((0,3),np.float32)
        arrays[sensor+'_frame_index']=np.concatenate([np.full(len(p),i,np.int32) for i,p in enumerate(frames)]) if frames else np.empty(0,np.int32)
    np.savez_compressed(args.output/'pointclouds.npz',**arrays)
    report=dict(streams=records,topics=topics,pointcloud_topic_graph=graph,odometry=odom,
        duration_s=time.monotonic()-started,motion_commands_sent=0,mode='read-only production subscriptions; no filter or control output')
    (args.output/'capture.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(frames={s:len(v) for s,v in records.items()},pointcloud_topics=graph,motion_commands_sent=0),indent=2))
    if not records['front'] or not records['back']:raise RuntimeError('No live raw LiDAR data captured')

if __name__=='__main__':main()

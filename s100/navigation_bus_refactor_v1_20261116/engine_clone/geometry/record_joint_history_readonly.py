#!/usr/bin/env python3
"""Subscribe only to the existing measured joint state; no controller access."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import JointState

parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
rclpy.init(args=[]);node=Node('tianji_readonly_joint_recorder',enable_rosout=False,start_parameter_services=False);samples=[]
def receive(msg):
    if len(msg.name)!=len(msg.position) or len(samples)>=1000:return
    samples.append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
        received_at=time.time(),positions=dict(zip(msg.name,msg.position))))
subscription=node.create_subscription(JointState,'/tianji/joint_states',receive,QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE))
started=time.monotonic()
try:
    while time.monotonic()-started<16:rclpy.spin_once(node,timeout_sec=.05)
finally:node.destroy_node();rclpy.shutdown()
args.output.write_text(json.dumps(dict(samples=samples,motion_commands_sent=0),indent=2)+'\n')
print('measured_joint_samples',len(samples))
if not samples:raise RuntimeError('No measured joint states received')

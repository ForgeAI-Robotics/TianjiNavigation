#!/usr/bin/env python3
"""Isolated S100 environment/collision advisory. No motion outputs.

Consumes normalized replay clouds in chassis_base. Production base_link clouds
must not be relabelled without independently verified model alignment.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import json
import time
from pathlib import Path

from ros_adapter import PREFIX, ROOT, isolation_check
from local_model import LocalRobotModel, TableProjector, default_urdf_path
from link import S100Link, JOINTS, POSE_TOLERANCE, number
from environment import EnvironmentPerception, decode_cloud
from environment_assessment import assess_candidate, candidate_digest, table_relationship
from sweep import check_sweep
from fixed_entrance import FixedEntranceGuard


def compute(model, link, cloud, now):
    odometry = [v for v in link.odometry if v['stamp']<=cloud['stamp']]
    if not odometry or cloud['stamp']-odometry[-1]['stamp']>.15:
        raise ValueError('POINT_CLOUD_ODOMETRY_ASSOCIATION_MISSING')
    packet = link.packet or {}
    if packet.get('whole_model_sha256') != model.sha256:
        raise ValueError('POINT_CLOUD_MODEL_CONTRACT_MISMATCH')
    positions = {}
    history = packet.get('motion_history',{})
    for name in JOINTS:
        s = history.get(name,{})
        if (all(number(s.get(k)) for k in ('covered_from','stamp','minimum','maximum','position'))
                and s['covered_from']<=cloud['stamp']<=s['stamp']
                and s['minimum']<=s['position']<=s['maximum']
                and s['maximum']-s['minimum']<=POSE_TOLERANCE):
            positions[name] = s['position']
    shapes = model.shapes(positions)
    environment = EnvironmentPerception().observe(cloud['points'],cloud['stamp'],now,
                      cloud['frame_id'],odometry[-1]['xy_yaw'],shapes)
    environment.summary['joint_positions_rad'] = positions
    environment.summary['whole_model_sha256'] = model.sha256
    environment.summary['table_relationship']=table_relationship(shapes,environment.summary['table'])
    environment.summary['current_body_risk']=check_sweep(shapes,environment,[0.,0.,0.],now)
    observation = None
    constraint = None
    if environment.summary['table'].get('valid'):
        observation = dict(frame_id='tianji/chassis_base',observation_time=environment.stamp,
                           base_xy_yaw=environment.base_xy_yaw,scene=environment.summary['table'])
        projector = TableProjector(model);projector.observe(observation)
        constraint = projector.constraint(packet,now)
    environment.summary['table_constraint']=constraint
    return environment,observation,constraint


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,default=default_urdf_path())
    args=parser.parse_args()
    isolation_check()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    model=LocalRobotModel(args.urdf,ROOT/'model_manifest.json',ROOT/'collision_geometry.json')
    perception_sha=json.loads((ROOT/'clearance_model_manifest.json').read_text())['portable_urdf_sha256']

    class Perception(Node):
        def __init__(self):
            super().__init__('s100_environment_perception',namespace=PREFIX)
            self.link=S100Link(model.sha256,perception_sha)
            self.entrance_guard=FixedEntranceGuard(model)
            self.cloud=None;self.candidate=None;self.future=None;self.last_started=0.;self.generation=0
            self.worker_pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='occupied_geometry')
            self.collision_pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='candidate_check')
            self.collision_future=None;self.environment=None;self.constraint=None;self.last_candidate_key=None
            self.environment_pub=self.create_publisher(String,PREFIX+'/s100/environment',1)
            self.collision_pub=self.create_publisher(String,PREFIX+'/s100/collision_assessment',1)
            self.observation_pub=self.create_publisher(String,PREFIX+'/input/table_observation',1)
            self.subscribe('/jetson/assessment',lambda d:self.link.receive(d,time.monotonic()))
            self.subscribe('/input/odometry',self.link.observe_odometry)
            self.subscribe('/input/candidate',self.on_candidate)
            self.create_subscription(PointCloud2,PREFIX+'/input/point_cloud',self.on_cloud,qos_profile_sensor_data)
            self.create_timer(.05,self.tick)

        def subscribe(self,suffix,callback):
            def receive(message):
                try:
                    data=json.loads(message.data,parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
                    if not isinstance(data,dict):raise ValueError('JSON object required')
                    callback(data)
                except (ValueError,TypeError,KeyError) as e:self.unknown(str(e))
            self.create_subscription(String,PREFIX+suffix,receive,10)

        def unknown(self,reason):
            self.collision_pub.publish(String(data=json.dumps(dict(schema_version=1,
                evaluated_at=time.time(),state='UNKNOWN',motion_inhibit=True,reasons=[reason],
                advisory_only=True,command_path_connected=False,hardware_stop_connected=False))))

        def on_candidate(self,data):self.candidate=data

        def on_cloud(self,message):
            self.generation+=1
            # Keep only the latest frame; no processing queue or unbounded lag.
            self.cloud=message
            if message.header.frame_id!='tianji/chassis_base':
                self.environment=None;self.constraint=None

        def tick(self):
            now,mono=time.time(),time.monotonic()
            if self.future is not None and self.future.done():
                future=self.future;self.future=None
                try:
                    environment,observation,constraint=future.result()
                    summary=environment.summary
                    age=now-summary['observation_time']
                    if not -.1<=age<=.65:raise ValueError('WORKER_RESULT_EXPIRED')
                    if self.cloud.header.frame_id!='tianji/chassis_base':
                        raise ValueError('POINT_CLOUD_TO_MODEL_ALIGNMENT_REQUIRED')
                    self.environment,self.constraint=environment,constraint
                    self.environment_pub.publish(String(data=json.dumps(summary,allow_nan=False)))
                    if observation:self.observation_pub.publish(String(data=json.dumps(observation,allow_nan=False)))
                except Exception as e:
                    self.environment=None;self.constraint=None;self.unknown(str(e))
            if self.collision_future is not None and self.collision_future.done():
                future=self.collision_future;self.collision_future=None
                try:
                    assessment=future.result()
                    if (self.candidate is None or assessment.get('candidate_id')!=self.candidate.get('candidate_id')
                            or assessment.get('candidate_digest')!=candidate_digest(self.candidate)
                            or not number(assessment.get('candidate_created_at'))
                            or not -.1<=now-assessment['candidate_created_at']<=.3
                            or not number(self.candidate.get('created_at'))
                            or not -.1<=now-self.candidate['created_at']<=.3):
                        raise ValueError('CANDIDATE_CHANGED_OR_EXPIRED_DURING_COMPUTE')
                    self.collision_pub.publish(String(data=json.dumps(assessment,allow_nan=False)))
                except Exception as e:self.unknown(str(e))
            if self.cloud is None:
                self.unknown('WAITING_FOR_NORMALIZED_POINT_CLOUD');return
            stamp=self.cloud.header.stamp.sec+self.cloud.header.stamp.nanosec*1e-9
            if not -.1<=now-stamp<=.65:
                self.environment=None;self.constraint=None;self.unknown('POINT_CLOUD_EXPIRED');return
            if self.cloud.header.frame_id!='tianji/chassis_base':
                self.unknown('POINT_CLOUD_TO_MODEL_ALIGNMENT_REQUIRED');return
            if self.environment is not None and self.candidate is not None and self.collision_future is None:
                try:
                    key=(candidate_digest(self.candidate),self.environment.stamp)
                    if key!=self.last_candidate_key:
                        self.last_candidate_key=key
                        snapshot=copy.deepcopy(self.link)
                        self.collision_future=self.collision_pool.submit(assess_candidate,model,snapshot,
                            self.environment,copy.deepcopy(self.constraint),
                            copy.deepcopy(self.link.odometry[-1]) if self.link.odometry else None,
                            copy.deepcopy(self.candidate),now,mono,self.entrance_guard)
                except Exception as e:self.unknown(str(e))
            if self.future is None and mono-self.last_started>=.5:
                self.last_started=mono
                try:
                    cloud=dict(points=decode_cloud(self.cloud),stamp=stamp,frame_id=self.cloud.header.frame_id)
                    self.future=self.worker_pool.submit(compute,model,copy.deepcopy(self.link),cloud,now)
                except Exception as e:self.unknown(str(e))

    rclpy.init(args=[]);node=Perception()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.worker_pool.shutdown(wait=True,cancel_futures=True)
        node.collision_pool.shutdown(wait=True,cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()

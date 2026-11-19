#!/usr/bin/env python3
"""Domain-80 localhost replay adapters. Publishes JSON advisories only.

There is deliberately no production bridge, motion message import, controller
client, parameter API or configurable topic name in this executable.
"""
import argparse
import json
import os
import time
import uuid
from pathlib import Path
from link import JetsonLink, S100Link, JOINTS
from local_model import LocalRobotModel, TableProjector, default_urdf_path

ROOT = Path(__file__).resolve().parent
PREFIX = '/tianji_dev/navigation_v2'


def isolation_check(environ=None):
    env = os.environ if environ is None else environ
    if env.get('ROS_DOMAIN_ID') != '80' or env.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Replay requires ROS_DOMAIN_ID=80 ROS_LOCALHOST_ONLY=1')
    for name in ('FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                 'CYCLONEDDS_URI', 'ROS_DISCOVERY_SERVER'):
        if env.get(name):
            raise RuntimeError('External discovery profile prohibited: ' + name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=['jetson', 's100'])
    parser.add_argument('--report', type=Path)
    parser.add_argument('--urdf', type=Path, default=default_urdf_path(),
                        help='S100-owned local URDF; never loaded from a network message')
    args = parser.parse_args()
    isolation_check()  # Before rclpy.init; production DDS is never discovered.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    whole = json.loads((ROOT/'model_manifest.json').read_text())['portable_urdf_sha256']
    perception = json.loads((ROOT/'clearance_model_manifest.json').read_text())['portable_urdf_sha256']

    class Adapter(Node):
        def __init__(self):
            super().__init__('navigation_link_'+args.role, namespace=PREFIX)
            self.link = (JetsonLink(whole, perception, str(uuid.uuid4())) if args.role == 'jetson'
                         else S100Link(whole, perception))
            self.clearance, self.scene, self.odom = {}, None, None
            self.ack, self.ack_received = None, None
            self.status_pub = self.create_publisher(String, PREFIX+'/'+args.role+'/status', 1)
            if args.role == 'jetson':
                self.packet_pub = self.create_publisher(String, PREFIX+'/jetson/assessment', 1)
                self.create_subscription(JointState, PREFIX+'/input/joint_states', self.joint, 50)
                self.subscribe('/input/clearance', self.on_clearance)
                self.subscribe('/s100/status', self.on_ack)
            else:
                self.model = LocalRobotModel(args.urdf, ROOT/'model_manifest.json', ROOT/'collision_geometry.json')
                self.projector = TableProjector(self.model)
                durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.description_pub = self.create_publisher(String, PREFIX+'/s100/robot_description', durable)
                self.description_pub.publish(String(data=self.model.urdf.read_text()))
                self.model_joint_pub = self.create_publisher(JointState, PREFIX+'/s100/joint_states', 20)
                self.last_model_stamps = {}
                self.subscribe('/jetson/assessment', self.on_packet)
                self.subscribe('/input/table_observation', self.on_scene)
                self.subscribe('/input/odometry', self.on_odom)
            self.create_timer(.05, self.tick)

        def subscribe(self, suffix, callback):
            def receive(msg):
                try:
                    value = json.loads(msg.data, parse_constant=lambda x: (_ for _ in ()).throw(
                                            ValueError('Nonfinite JSON: '+x)))
                    if not isinstance(value, dict):
                        raise ValueError('Object required')
                    callback(value)
                except (ValueError, TypeError, KeyError) as exc:
                    self.get_logger().warning('Rejected input: '+str(exc))
            self.create_subscription(String, PREFIX+suffix, receive, 10)

        def joint(self, msg):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            self.link.history.add(list(msg.name), list(msg.position), stamp)

        def on_clearance(self, value): self.clearance = value
        def on_scene(self, value): self.projector.observe(value)
        def on_odom(self, value):
            if self.link.observe_odometry(value): self.odom = value
        def on_packet(self, value): self.link.receive(value, time.monotonic())
        def on_ack(self, value):
            if (value.get('jetson_session') == self.link.session_id
                    and type(value.get('acknowledged_sequence')) is int
                    and 0 < value['acknowledged_sequence'] <= self.link.sequence
                    and (self.ack is None or value['acknowledged_sequence'] > self.ack['acknowledged_sequence'])):
                self.ack, self.ack_received = value, time.monotonic()

        def tick(self):
            now, mono = time.time(), time.monotonic()
            if args.role == 'jetson':
                packet = self.link.packet(self.clearance, now)
                self.packet_pub.publish(String(data=json.dumps(packet, allow_nan=False)))
                alive = self.ack_received is not None and 0 <= mono-self.ack_received <= .3
                status = dict(schema_version=2, evaluated_at=now, peer_alive=alive,
                              peer_state=self.ack.get('state') if alive else 'DISCONNECTED',
                              last_ack_sequence=self.ack.get('acknowledged_sequence') if self.ack else None,
                              sent_sequence=self.link.sequence, advisory_only=True,
                              command_path_connected=False, hardware_stop_connected=False)
            else:
                measured = self.link.model_joint_state(now, mono)
                groups = {}
                for name, value in measured.items():
                    if value['stamp'] != self.last_model_stamps.get(name):
                        groups.setdefault(value['stamp'], []).append((name,value['position']))
                        self.last_model_stamps[name] = value['stamp']
                for stamp, values in groups.items():
                    joint = JointState()
                    joint.header.frame_id = 'tianji/chassis_base'
                    joint.header.stamp.sec = int(stamp)
                    joint.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
                    joint.name = [name for name,_ in values]
                    joint.position = [position for _,position in values]
                    self.model_joint_pub.publish(joint)
                self.scene = self.projector.constraint(self.link.packet or {}, now)
                status = self.link.evaluate(now, mono, self.scene, self.odom)
                status.update(local_urdf_verified=True, local_model_sha256=self.model.sha256,
                              local_collision_shapes=len(self.model.geometry['shapes']),
                              model_joint_count=len(measured),
                              model_missing_joints=sorted(set(JOINTS) - set(measured)),
                              table_model_reason=self.scene.get('reason'))
            self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
            if args.report:
                args.report.parent.mkdir(parents=True, exist_ok=True)
                temporary = args.report.with_suffix('.tmp')
                temporary.write_text(json.dumps(status, indent=2, allow_nan=False)+'\n')
                temporary.replace(args.report)

    rclpy.init(args=[])
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()

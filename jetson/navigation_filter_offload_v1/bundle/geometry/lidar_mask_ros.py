#!/usr/bin/env python3
"""Localhost/domain-80 raw LiDAR ShapeMask replay adapter; no Cutie or commands."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse
import copy
import json
import time
import uuid
from array import array
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
from ros_adapter import ROOT,PREFIX,isolation_check
from local_model import LocalRobotModel,default_urdf_path
from environment import decode_cloud
from lidar_shapemask import LidarExtrinsics,LidarShapeMask,positions_at_cloud
from link import S100Link,ARM


def retained_records(message,keep):
    """Keep all original fields, including intensity and per-point timestamps."""
    records=np.ndarray((message.height,message.width),dtype=np.dtype('V'+str(message.point_step)),
                       buffer=message.data,strides=(message.row_step,message.point_step)).reshape(-1)
    keep=np.asarray(keep,dtype=bool)
    if keep.shape!=(len(records),):raise ValueError('LIDAR_KEEP_MASK_SIZE_INVALID')
    return records[keep].tobytes(),int(keep.sum())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,default=default_urdf_path())
    parser.add_argument('--config',type=Path,default=ROOT/'lidar_mask_config.json')
    parser.add_argument('--live-directory',type=Path,
                        help='Read passive latest-only tmpfs input; outputs remain domain80 localhost')
    parser.add_argument('--snapshot-directory',type=Path,
                        help='Optional one exact point/label/pose snapshot per sensor for offline visualization')
    parser.add_argument('--display-directory',type=Path,help='Optional latest-only display packets; no navigation consumer')
    args=parser.parse_args();isolation_check()
    if args.live_directory and args.live_directory.resolve().parent!=Path('/dev/shm'):
        raise ValueError('LIVE_INPUT_MUST_BE_DIRECT_TMPFS_CHILD')
    config=json.loads(args.config.read_text());rate=config['max_sensor_rate_hz']
    if config.get('schema_version')!=1 or type(rate) not in (float,int) or not .1<=rate<=5.:
        raise ValueError('INVALID_LIDAR_MASK_CONFIG')
    calibration=json.loads((args.config.parent/config['calibration_file']).read_text())
    model=LocalRobotModel(args.urdf,ROOT/'model_manifest.json',ROOT/'collision_geometry.json')
    extrinsics=LidarExtrinsics(calibration,config['T_model_vendor_base'],config['alignment_verified'])
    mask=LidarShapeMask(model,extrinsics,simplify_chassis=config.get('simplify_chassis',False))
    from eight_direction import EightDirectionRanger
    ranger=EightDirectionRanger(model,mask.meshes) if args.display_directory else None
    from display_pose_source import read_source,saved_positions
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
    latest_cloud_qos=QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
    from sensor_msgs.msg import PointCloud2,PointField
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String

    class Filter(Node):
        def __init__(self):
            super().__init__('s100_lidar_shapemask',namespace=PREFIX)
            self.packet={};self.packet_receive=None;self.frames={};self.last_started={};self.last_stamp={}
            perception_sha=json.loads((ROOT/'clearance_model_manifest.json').read_text())['portable_urdf_sha256']
            self.link=S100Link(model.sha256,perception_sha)
            self.future=None;self.work_sensor=None;self.work_message=None
            self.last_timeout_report=0.
            self.saved_snapshots=set()
            self.display_session=str(uuid.uuid4());self.display_sequence=0
            if args.display_directory:args.display_directory.mkdir(parents=True,exist_ok=True)
            self.worker_pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='lidar_mask')
            self.status=self.create_publisher(String,PREFIX+'/s100/lidar_shapemask/status',1)
            self.labeled={};self.retained={}
            for sensor in ('front','back'):
                self.labeled[sensor]=self.create_publisher(PointCloud2,PREFIX+'/s100/lidar_'+sensor+'/labeled',1)
                self.retained[sensor]=self.create_publisher(PointCloud2,PREFIX+'/s100/lidar_'+sensor+'/retained',1)
                self.create_subscription(PointCloud2,PREFIX+'/input/lidar_'+sensor,
                    lambda message,s=sensor:self.frames.__setitem__(s,message),latest_cloud_qos)
            self.create_subscription(String,PREFIX+'/jetson/assessment',self.on_packet,1)
            self.create_timer(.05,self.tick)

        def on_packet(self,message):
            try:
                packet=json.loads(message.data,parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
                if not isinstance(packet,dict):raise ValueError('ASSESSMENT_OBJECT_REQUIRED')
                mono=time.monotonic()
                if self.link.receive(packet,mono):self.packet=self.link.packet;self.packet_receive=mono
            except (ValueError,TypeError):self.packet={};self.packet_receive=None

        def report(self,sensor,reason,summary=None):
            result=dict(schema_version=1,evaluated_at=time.time(),sensor_id=sensor,
                reason=reason,advisory_only=True,command_path_connected=False,
                hardware_stop_connected=False,cutie_enabled=False)
            if summary:result.update(summary)
            self.status.publish(String(data=json.dumps(result,allow_nan=False)))
            if args.display_directory:
                from live_lidar_ingress import atomic
                atomic(args.display_directory/'status.json',json.dumps(result,allow_nan=False).encode())

        def work(self,sensor,message,packet,now):
            stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            if message.header.frame_id!=config['input_frames'][sensor]:
                raise ValueError('RAW_LIDAR_FRAME_MISMATCH_OR_ALREADY_MERGED')
            source=read_source(args.live_directory) if args.display_directory and args.live_directory else dict(mode='realtime')
            static=source.get('mode')=='saved'
            positions=saved_positions(source,model.sha256) if static else positions_at_cloud(packet,model.sha256,stamp,now)
            result=mask.classify(decode_cloud(message),sensor,'sensor',positions)
            result['summary'].update(observation_time=stamp,measured_joint_count=len(positions))
            result['positions_used']=positions;result['clock_mapping']=packet.get('clock_mapping')
            result['pose_source']='saved' if static else 'realtime'
            result['saved_observation_time']=source['snapshot']['observation_time'] if static else None
            if static:
                result['removal_mask'][:]=False;result['keep_mask'][:]=True;result['summary']['removed_points']=0
                from eight_direction import empty
                result['eight_direction']=empty('SAVED_POSTURE_NOT_LIVE')
            elif ranger is not None:result['eight_direction']=ranger.assess(result,positions)
            if args.snapshot_directory and sensor not in self.saved_snapshots:
                args.snapshot_directory.mkdir(parents=True,exist_ok=True)
                np.savez_compressed(args.snapshot_directory/(sensor+'.npz'),
                    points_model=result['points_model'],labels=result['labels'],
                    removal_mask=result['removal_mask'],expected_range_m=result['expected_range_m'])
                (args.snapshot_directory/(sensor+'.json')).write_text(json.dumps(dict(
                    observation_time=stamp,positions_used=positions,clock_mapping=packet.get('clock_mapping'),
                    model_sha256=model.sha256,summary=result['summary']),indent=2)+'\n')
                self.saved_snapshots.add(sensor)
            return result

        def tick(self):
            mono=time.monotonic();now=time.time()
            if args.live_directory:
                try:
                    path=args.live_directory/'assessment.json'
                    value=path.read_text()
                    if value!=getattr(self,'last_live_packet',None):
                        self.on_packet(String(data=value));self.last_live_packet=value
                except (OSError,ValueError):pass
            if self.future is not None and self.future.done():
                future=self.future;self.future=None;sensor=self.work_sensor;message=self.work_message
                try:
                    result=future.result();stamp=result['summary']['observation_time']
                    if not -.1<=now-stamp<=.65:raise ValueError('LIDAR_FILTER_RESULT_EXPIRED')
                    data=np.empty(len(result['labels']),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('label','<u4')])
                    for i,name in enumerate('xyz'):data[name]=result['points_model'][:,i]
                    data['label']=result['labels'];out=PointCloud2();out.header=copy.deepcopy(message.header)
                    out.header.frame_id='tianji/chassis_base' if extrinsics.alignment_verified else 'tianji_preview/chassis_base'
                    out.height=1;out.width=len(data);out.point_step=16;out.row_step=16*len(data)
                    out.fields=[PointField(name=n,offset=i*4,datatype=7 if i<3 else 6,count=1)
                                for i,n in enumerate(('x','y','z','label'))]
                    out.data=array('B',data.tobytes());out.is_dense=False
                    if result['removal_mask'].any():
                        retained=copy.deepcopy(message);payload,count=retained_records(message,result['keep_mask'])
                        retained.data=array('B',payload);retained.height=1;retained.width=count;retained.row_step=count*retained.point_step
                    else:retained=message
                    if not -.1<=time.time()-stamp<=.65:raise ValueError('LIDAR_FILTER_RESULT_EXPIRED')
                    if result['pose_source']=='realtime':
                        self.labeled[sensor].publish(out)
                        self.retained[sensor].publish(retained)
                    if args.display_directory:
                        from view_protocol import pack
                        from live_lidar_ingress import atomic
                        self.display_sequence+=1
                        metadata=dict(sensor_id=sensor,observation_time=stamp,model_sha256=model.sha256,
                            positions_used=result['positions_used'],clock_mapping=result['clock_mapping'],sensor_origin_model_m=result['summary']['sensor_origin_model_m'],
                            session_id=self.display_session,sequence=self.display_sequence,
                            compute_ms=result['summary']['compute_ms'],actual_deletion_enabled=extrinsics.alignment_verified and result['pose_source']=='realtime',
                            pose_source=result['pose_source'],saved_observation_time=result['saved_observation_time'],
                            removed_points=int(result['removal_mask'].sum()),eight_direction=result.get('eight_direction'),chassis_collision=result.get('chassis_collision'))
                        atomic(args.display_directory/(sensor+'.view'),pack(result,metadata,full=True))
                    self.report(sensor,'SAVED_POSTURE_DISPLAY_ONLY' if result['pose_source']=='saved' else ('GEOMETRIC_PREVIEW' if not extrinsics.alignment_verified else 'GEOMETRIC_FILTER'),result['summary'])
                except Exception as e:self.report(sensor,str(e))
            if self.future is not None:return
            if self.packet_receive is None or mono-self.packet_receive>.3:
                if mono-self.last_timeout_report>=.5:
                    self.report('all','STATE_TRANSPORT_TIMEOUT');self.last_timeout_report=mono
                return
            if args.live_directory:
                for sensor in ('front','back'):
                    if mono-self.last_started.get(sensor,0.)<1/rate:continue
                    pending=self.frames.get(sensor)
                    if pending is not None:
                        pending_stamp=pending.header.stamp.sec+pending.header.stamp.nanosec*1e-9
                        # Keep a pending cloud long enough to receive the next
                        # real pose, rather than continually outrunning it.
                        if self.last_stamp.get(sensor)!=pending_stamp and now-pending_stamp<=.35:
                            continue
                    try:
                        self.frames[sensor]=deserialize_message(
                            (args.live_directory/(sensor+'.cdr')).read_bytes(),PointCloud2)
                    except (OSError,ValueError):pass
            for sensor in sorted(self.frames,key=lambda s:self.last_started.get(s,0.)):
                message=self.frames[sensor];stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
                if mono-self.last_started.get(sensor,0.)<1/rate or self.last_stamp.get(sensor)==stamp:continue
                if args.live_directory:
                    try:
                        source=read_source(args.live_directory) if args.display_directory else dict(mode='realtime')
                        measured=saved_positions(source,model.sha256) if source.get('mode')=='saved' else positions_at_cloud(self.packet,model.sha256,stamp,now)
                        if not all(name in measured for name in ARM):
                            if mono-self.last_timeout_report>=.5:
                                self.report(sensor,'WAITING_FOR_TIMESTAMP_BRACKET',dict(measured_joint_count=len(measured)))
                                self.last_timeout_report=mono
                            continue
                    except ValueError:continue
                self.last_started[sensor]=mono;self.last_stamp[sensor]=stamp
                self.work_sensor=sensor;self.work_message=message
                self.future=self.worker_pool.submit(self.work,sensor,message,copy.deepcopy(self.packet),now)
                break

    rclpy.init(args=[]);node=Filter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.worker_pool.shutdown(wait=True,cancel_futures=True);mask.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()

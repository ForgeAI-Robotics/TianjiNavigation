"""CPU voxel/keyframe recorder. Candidate static geometry is not a certified map.
No ROS, controller, network or command output.
"""
import json,math,os,time,hashlib
from pathlib import Path
import numpy as np
from common import atomic_json

def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

def write_pcd(path,points):
    p=np.asarray(points,dtype='<f4').reshape(-1,3)
    header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(p)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(p)}\nDATA binary\n'.encode()
    temp=Path(str(path)+'.tmp');temp.write_bytes(header+p.tobytes());os.replace(temp,path)

class VoxelRecorder:
    def __init__(self,folder,config):
        self.folder=Path(folder);(self.folder/'keyframes').mkdir(parents=True,exist_ok=True)
        self.config=config;self.res=config['resolution_m'];self.cells={};self.frames=0;self.latest=None;self.epoch=None;self.invalid_reason=None
    def ingest(self,points_base,stamp,pose,sensor,origin_base,joint_packet,epoch=0,pose_matrix=None):
        p=np.asarray(points_base,float)
        if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all() or len(p)>100000:raise ValueError('INVALID_CLOUD')
        if self.epoch is not None and self.epoch!=epoch:
            self.invalid_reason='ODOMETRY_EPOCH_CHANGED_START_A_NEW_SESSION';return
        self.epoch=epoch
        pose=np.asarray(pose,float)
        if pose.shape!=(3,) or not np.isfinite(pose).all() or not np.isfinite(stamp):raise ValueError('INVALID_POSE_TIME')
        transform=np.eye(4);transform[:3,:3]=rotation(pose[2]);transform[:2,3]=pose[:2]
        if pose_matrix is not None:
            transform=np.asarray(pose_matrix,float)
            if (transform.shape!=(4,4) or not np.isfinite(transform).all() or not np.allclose(transform[3],[0,0,0,1])
                    or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-6)
                    or not np.isclose(np.linalg.det(transform[:3,:3]),1.,atol=1e-6)):raise ValueError('INVALID_RIGID_MAP_POSE')
        world=p@transform[:3,:3].T+transform[:3,3]
        keys=np.unique(np.floor(world/self.res).astype(np.int64),axis=0)
        new_keys=[tuple(k) for k in keys]
        if len(self.cells)+sum(k not in self.cells for k in new_keys)>self.config['max_cells']:
            self.invalid_reason='VOXEL_BUDGET_EXCEEDED_START_A_SMALLER_SESSION';return
        for key in new_keys:
            row=self.cells.get(key)
            if row is None:self.cells[key]=[stamp,stamp,1,pose[:2].copy(),False]
            elif stamp>row[1]+.2:
                row[1]=stamp;row[2]+=1
                row[4]=row[4] or np.linalg.norm(pose[:2]-row[3])>=self.config['min_view_translation_m']
        self.frames+=1
        origin_map=transform[:3,:3]@np.asarray(origin_base)+transform[:3,3]
        filename=f'{self.frames:06d}_{sensor}.npz'
        np.savez_compressed(self.folder/'keyframes'/filename,points_base=p.astype(np.float32),pose=pose,
            observation_time=np.array(stamp),sensor_origin_base=np.asarray(origin_base),sensor_origin_map=origin_map,
            joints_json=np.array(json.dumps(joint_packet,allow_nan=False)),odometry_epoch=np.array(epoch),sensor=np.array(sensor),pose_matrix=transform)
        self.latest=dict(observation_time=float(stamp),base_xy_yaw=pose.tolist(),sensor=sensor,keyframe=filename,
            joints=joint_packet,odometry_epoch=epoch,base_transform_4x4=transform.tolist())
    def save(self):
        keys=list(self.cells);all_points=(np.asarray(keys,float).reshape(-1,3)+.5)*self.res
        stable=np.array([row[2]>=self.config['min_confirmations'] and row[1]-row[0]>=self.config['confirm_span_s'] and row[4] for row in self.cells.values()],bool)
        write_pcd(self.folder/'observed_all.pcd',all_points)
        write_pcd(self.folder/'candidate_static.pcd',all_points[stable])
        temporary=self.folder/'voxels.npz.tmp'
        with temporary.open('wb') as f:np.savez_compressed(f,points=all_points.astype(np.float32),candidate_static=stable)
        os.replace(temporary,self.folder/'voxels.npz')
        voxel_hash=hashlib.sha256((self.folder/'voxels.npz').read_bytes()).hexdigest()
        report=dict(schema_version=1,voxel_file_sha256=voxel_hash,frame_id='session_odom',resolution_m=self.res,frames=self.frames,cells=len(keys),candidate_static_cells=int(stable.sum()),latest=self.latest,
            invalid_reason=self.invalid_reason,static_map_verified=False,dynamic_removal='multi-view temporal candidate gate only; M-detector not integrated',
            coverage_complete=False,loop_closure_optimized=False,source='original single-frame observations with recorded odometry; no fabricated points',
            full_keyframes_saved=True,free_space_certified=False,motion_commands_sent=0,validation_only=getattr(self,'validation_only',False))
        atomic_json(self.folder/'map.json',report);return report

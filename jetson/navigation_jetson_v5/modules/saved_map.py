"""Read-only archive validation and coordinate transforms, without ROS/GUI."""
import hashlib,json
from pathlib import Path
import numpy as np

def base_transform(pose):
    x,y,yaw=np.asarray(pose,float);c,s=np.cos(yaw),np.sin(yaw)
    matrix=np.eye(4);matrix[:2,:2]=[[c,-s],[s,c]];matrix[:2,3]=[x,y];return matrix

class MapArchive:
    def __init__(self,folder):
        self.folder=Path(folder);self.meta=json.loads((self.folder/'map.json').read_text())
        voxel_file=self.folder/'voxels.npz'
        if hashlib.sha256(voxel_file.read_bytes()).hexdigest()!=self.meta['voxel_file_sha256']:raise ValueError('地图体素文件校验失败')
        with np.load(voxel_file,allow_pickle=False) as data:self.points=np.array(data['points']);self.static=np.array(data['candidate_static'],bool)
        if self.points.ndim!=2 or self.points.shape[1]!=3 or not np.isfinite(self.points).all() or self.static.shape!=(len(self.points),):raise ValueError('地图点云结构不正确')
        if len(self.points)!=self.meta['cells']:raise ValueError('地图点数与元数据不一致')
        self.keyframes=sorted((self.folder/'keyframes').glob('*.npz'))
        if len(self.keyframes)!=self.meta['frames']:raise ValueError('关键帧数量与元数据不一致')
        self.trajectory=[]
        for path in self.keyframes:
            with np.load(path,allow_pickle=False) as data:self.trajectory.append(np.array(data['pose'],float))
        self.trajectory=np.array(self.trajectory)

    def frame(self,index):
        path=self.keyframes[index]
        with np.load(path,allow_pickle=False) as data:
            points=np.array(data['points_base']);pose=np.array(data['pose']);stamp=float(data['observation_time'])
            joints=json.loads(str(data['joints_json'])) if 'joints_json' in data else None
            matrix=np.array(data['pose_matrix'],float) if 'pose_matrix' in data else None
        if joints is None and path.name==self.meta.get('latest',{}).get('keyframe'):joints=self.meta['latest'].get('joints',{})
        transform=base_transform(pose) if matrix is None else matrix;world=points@transform[:3,:3].T+transform[:3,3]
        return world,pose,joints or {},stamp

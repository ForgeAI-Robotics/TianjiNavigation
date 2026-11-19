"""Bounded, odometry-compensated occupied-only low obstacle memory.
No free-space inference, production publisher, control or persisted map.
"""
from collections import deque
import math,time
import numpy as np

class PlanarHistory:
    def __init__(self):self.rows=deque();self.epoch=0;self.reason='WAITING_ODOMETRY'
    def add(self,stamp,received,position,quaternion,frame,child):
        values=np.array([stamp,received,*position,*quaternion],float)
        if (values.shape!=(9,) or not np.isfinite(values).all() or frame!='odom' or child!='base_link'
            or not -.05<=received-stamp<=.2 or abs(position[2])>.05
            or not np.allclose(quaternion[:3],0,atol=1e-8) or abs(quaternion[3])>100):
            self.rows.clear();self.epoch+=1;self.reason='ODOMETRY_INVALID';return False
        # Explicit vendor SDK contract: odom_relative_move.hpp and
        # ygraphing_node.h odomQuatToEuler: orientation.w is yaw in radians.
        row=[float(stamp),float(position[0]),float(position[1]),float(quaternion[3])]
        if self.rows:
            previous=self.rows[-1];dt=stamp-previous[0]
            angle=math.atan2(math.sin(row[3]-previous[3]),math.cos(row[3]-previous[3]))
            if dt<=0 or dt>.3 or np.linalg.norm(np.array(row[1:3])-previous[1:3])>2*dt+.025 or abs(angle)>3*dt+.03:
                self.rows.clear();self.epoch+=1;self.reason='ODOMETRY_DISCONTINUITY'
        self.rows.append(row)
        while self.rows and self.rows[0][0]<stamp-3:self.rows.popleft()
        self.reason='VENDOR_YAW_IN_W';return True
    def packet(self,now):
        return dict(schema_version=1,generated_at=now,epoch=self.epoch,encoding='vendor_yaw_in_orientation_w',frame='odom',child='base_link',reason=self.reason,rows=list(self.rows))

def pose_at(packet,stamp,now):
    if (packet.get('schema_version')!=1 or packet.get('encoding')!='vendor_yaw_in_orientation_w'
        or packet.get('frame')!='odom' or packet.get('child')!='base_link'
        or not -.05<=now-packet['generated_at']<=.25):raise ValueError('MEMORY_ODOMETRY_STALE_OR_INVALID')
    rows=np.asarray(packet.get('rows',[]),float)
    if rows.ndim!=2 or rows.shape[1]!=4 or len(rows)<2 or not np.isfinite(rows).all():raise ValueError('MEMORY_ODOMETRY_HISTORY_MISSING')
    times=rows[:,0]
    if np.any(np.diff(times)<=0):raise ValueError('MEMORY_ODOMETRY_UNORDERED')
    j=int(np.searchsorted(times,stamp,side='right'))
    if j==0 or j>=len(times):raise ValueError('MEMORY_ODOMETRY_NOT_BRACKETED')
    a,b=rows[j-1],rows[j];dt=b[0]-a[0]
    if dt>.15:raise ValueError('MEMORY_ODOMETRY_GAP')
    alpha=(stamp-a[0])/dt;yaw=a[3]+alpha*math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]))
    return np.array([*(a[1:3]+alpha*(b[1:3]-a[1:3])),yaw])

def rotation(pose):
    c,s=math.cos(pose[2]),math.sin(pose[2]);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

class ObstacleMemory:
    def __init__(self,resolution=.05,max_cells=12000):
        self.resolution=resolution;self.max_cells=max_cells;self.cells={};self.epoch=None;self.latest=-np.inf
    def invalidate(self):self.cells.clear();self.latest=-np.inf;self.epoch=None
    def update(self,points,labels,stamp,pose,epoch):
        begin=time.perf_counter()
        if self.epoch!=epoch:self.cells.clear();self.latest=-np.inf;self.epoch=epoch
        # Two asynchronously delivered sensors can be mildly out of order.
        if stamp<self.latest-.5:raise ValueError('MEMORY_FRAME_OUT_OF_ORDER')
        self.latest=max(stamp,self.latest);now=self.latest
        points=np.asarray(points);labels=np.asarray(labels)
        valid=(labels==1)&np.isfinite(points).all(1)&(points[:,2]>=.04)&(points[:,2]<=.6)&(np.linalg.norm(points[:,:2],axis=1)<=4.)
        local=points[valid];world=local@rotation(pose).T+np.array([pose[0],pose[1],0])
        keys=np.unique(np.floor(world/self.resolution).astype(np.int64),axis=0)
        for key in map(tuple,keys):
            old=self.cells.get(key)
            if old is None:self.cells[key]=[stamp,stamp,1]
            elif stamp>old[1]+.09:old[1]=stamp;old[2]+=1
        cell_keys=list(self.cells)
        states=np.array(list(self.cells.values()),float).reshape(-1,3)
        centers=(np.array(cell_keys,float).reshape(-1,3)+.5)*self.resolution
        ages=now-states[:,1]
        ttl=np.where((states[:,2]>=2)&(states[:,1]-states[:,0]>=.5),15.,3.)
        alive=(ages<=ttl)&(np.linalg.norm(centers[:,:2]-pose[:2],axis=1)<=4.5)
        ids=np.flatnonzero(alive)
        if len(ids)>self.max_cells:
            alive[ids[np.argsort(states[ids,1])[:len(ids)-self.max_cells]]]=False
        for index in np.flatnonzero(~alive):del self.cells[cell_keys[index]]
        history=alive&(ages>=1.5)&(ages<=15.)
        xyz=(centers[history]-np.array([pose[0],pose[1],0]))@rotation(pose)
        ages=ages[history]
        summary=dict(state='OCCUPIED_HISTORY_ONLY',cells=len(self.cells),historical_cells=len(xyz),resolution_m=self.resolution,low_height_band_m=[.04,.6],max_age_s=float(ages.max()) if len(ages) else 0.,unconfirmed_ttl_s=3.,confirmed_ttl_s=15.,pose_at_cloud=pose.tolist(),odometry_epoch=epoch,pose_encoding='vendor_yaw_in_orientation_w',motion_compensation='planar_odometry',navigation_connected=False,free_space_inferred=False,dynamic_objects='possible trails; bounded TTL; no ray clearing',base_frame_alignment='current unverified model/vendor base identity',compute_ms=(time.perf_counter()-begin)*1000)
        return xyz,summary

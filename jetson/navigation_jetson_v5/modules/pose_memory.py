"""Measured odometry propagation and scan-fix fusion. No motion/control imports."""
import math
import numpy as np
from saved_map import base_transform
from continuity import OdometryHistory

def yaw(matrix):
    return math.atan2(matrix[1,0], matrix[0,0])

def angle_delta(a,b):
    return math.atan2(math.sin(a-b),math.cos(a-b))

class PoseMemory:
    def __init__(self):
        self.history=OdometryHistory();self.identity=None;self.anchor=None
        self.last_fix=None;self.last_fix_wall=None;self.last_received=None
        self.distance=0.;self.turn=0.;self.reset_count=0;self.last_match={};self.reset_stamp=None

    def ingest(self,header,now):
        identity=(header['boot_id'],header['epoch'],header['calibration_sha256'])
        if self.identity is not None and identity!=self.identity:
            self.anchor=None;self.last_fix=None;self.reset_count+=1;self.history=OdometryHistory();self.reset_stamp=header['observation_time']
        self.identity=identity
        old=self.history.last;old_epoch=self.history.epoch
        if not self.history.add(header['observation_time'],header['pose']):return False
        if self.history.epoch!=old_epoch:
            self.anchor=None;self.last_fix=None;self.reset_count+=1;self.reset_stamp=header['observation_time']
        elif old:
            # Distances are measured changes, never command velocity integration.
            self.distance+=math.dist(old[1][:2],header['pose'][:2])
            self.turn+=abs(angle_delta(header['pose'][2],old[1][2]))
        self.last_received=now
        return True

    def prediction(self,pose=None):
        if self.anchor is None or self.history.last is None:return None
        return self.anchor@base_transform(self.history.last[1] if pose is None else pose)

    def fix(self,matrix,header,now,metrics):
        identity=(header['boot_id'],header['epoch'],header['calibration_sha256'])
        if identity!=self.identity or self.history.last is None:return False
        if self.reset_stamp is not None and header['odom_observation_time']<self.reset_stamp:return False
        # Never apply results which crossed a reset, a long stall or a bad rigid transform.
        if self.history.last[0]-header['odom_observation_time']>3.:return False
        matrix=np.asarray(matrix,float)
        if matrix.shape!=(4,4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3],[0,0,0,1]) or not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-4) or not np.isclose(np.linalg.det(matrix[:3,:3]),1,atol=1e-4):return False
        if self.anchor is not None:
            guess=self.prediction(header['pose'])
            if np.linalg.norm(matrix[:2,3]-guess[:2,3])>.25 or abs(angle_delta(yaw(matrix),yaw(guess)))>math.radians(5):return False
        self.anchor=matrix@np.linalg.inv(base_transform(header['pose']))
        self.last_fix=header['observation_time'];self.last_fix_wall=now;self.last_match=metrics
        return True

    def snapshot(self,now):
        odom_fresh=self.last_received is not None and 0<=now-self.last_received<=.5
        age=None if self.last_fix_wall is None else max(0.,now-self.last_fix_wall)
        status=('NO_ODOMETRY' if not odom_fresh else 'NEED_LIDAR_FIX' if self.anchor is None else
                'LIDAR_ODOMETRY' if age<=3 else 'ODOMETRY_ONLY' if age<=10 else 'LOCALIZATION_LOST')
        matrix=self.prediction()
        return dict(status=status,odometry_fresh=odom_fresh,odometry_age_s=None if self.last_received is None else max(0.,now-self.last_received),observation_time=self.history.last[0] if self.history.last else None,
                    odometry_pose=self.history.last[1] if self.history.last else None,
                    map_pose_valid=bool(odom_fresh and self.anchor is not None and age<=10),
                    T_map_base=None if matrix is None else matrix.tolist(),
                    T_map_odom=None if self.anchor is None else self.anchor.tolist(),
                    lidar_fix_age_s=age,last_lidar_observation_time=self.last_fix,
                    source_boot_id=None if self.identity is None else self.identity[0],
                    odometry_epoch=None if self.identity is None else self.identity[1],
                    calibration_sha256=None if self.identity is None else self.identity[2],
                    session_distance_m=self.distance,session_turn_rad=self.turn,reset_count=self.reset_count,
                    match_metrics=self.last_match)

def read_current(path,now):
    import json
    try:
        packet=json.loads(path.read_text());age=now-packet['updated_wall_time']
        if not packet.get('running') or not 0<=age<=1.:
            packet=dict(packet,status='STOPPED_OR_STALE',map_pose_valid=False,odometry_fresh=False)
        elif packet.get('odometry_age_s') is not None and age+packet['odometry_age_s']>.5:
            packet=dict(packet,status='NO_ODOMETRY',map_pose_valid=False,odometry_fresh=False)
        return packet
    except (OSError,ValueError,KeyError,TypeError):
        return dict(status='NOT_RUNNING',map_pose_valid=False)

def caption(packet):
    names={'LIDAR_ODOMETRY':'雷达+里程计','ODOMETRY_ONLY':'里程计推算（雷达待校正）',
           'ODOMETRY_ONLY_NO_MAP':'仅实测里程计（地图参考待更新）','NO_ODOMETRY':'里程计缺失','NEED_LIDAR_FIX':'等待雷达定位','LOCALIZATION_LOST':'定位失效',
           'STOPPED_OR_STALE':'已停止/旧记录','NOT_RUNNING':'未启动定位'}
    text=names.get(packet['status'],packet['status'])
    if packet.get('map_pose_valid') and packet.get('T_map_base') is not None:
        T=np.array(packet['T_map_base']);text+=f' X {T[0,3]:.2f} Y {T[1,3]:.2f}m θ {math.degrees(yaw(T)):.1f}°'
    if packet.get('odometry_pose') is not None:
        p=packet['odometry_pose'];text+=f' | 里程计 {p[0]:.2f},{p[1]:.2f}m | 本次路程 {packet["session_distance_m"]:.2f}m'
    return text

"""Distinguish sampling loss from actual odometry discontinuities."""
import math
from collections import deque

class OdometryHistory:
    def __init__(self):
        self.rows=deque(maxlen=800);self.epoch=0;self.event=None;self.last=None

    def add(self,stamp,pose):
        if not all(math.isfinite(x) for x in [stamp,*pose]) or len(pose)!=3:return False
        if self.last:
            old,p=self.last;dt=stamp-old
            distance=math.hypot(pose[0]-p[0],pose[1]-p[1])
            yaw=abs(math.atan2(math.sin(pose[2]-p[2]),math.cos(pose[2]-p[2])))
            if dt<=0:
                if dt>-.5:
                    self.event=dict(reason='DUPLICATE_OR_REORDERED_SAMPLE_DROPPED',dt=dt,stamp=stamp);return False
                reason='ODOMETRY_CLOCK_RESET'
            elif distance>2*dt+.03 or yaw>3*dt+.03:reason='ODOMETRY_POSE_JUMP'
            else:reason=None
            if reason:
                self.epoch+=1;self.rows.clear()
                self.event=dict(reason=reason,dt=dt,distance_m=distance,yaw_delta_rad=yaw,stamp=stamp,epoch=self.epoch)
            elif dt>.5:
                # No interpolation across this hole. A continuous coordinate
                # frame resumes; missing samples are not a new frame epoch.
                self.rows.clear();self.event=dict(reason='SAMPLING_GAP_PAUSE_AND_RECOVER',dt=dt,stamp=stamp,epoch=self.epoch)
        self.last=(stamp,list(pose));self.rows.append((stamp,list(pose),self.epoch));return True

    def nearest(self,stamp,tolerance=.05):
        if not self.rows:return None
        row=min(self.rows,key=lambda x:abs(x[0]-stamp))
        return row if abs(row[0]-stamp)<=tolerance else None

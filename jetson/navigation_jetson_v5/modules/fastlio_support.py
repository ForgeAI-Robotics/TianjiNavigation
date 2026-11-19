"""Pure 6D mapping and estimator health checks; no network or ROS."""
import math
import numpy as np

def pose_matrix(position,quaternion):
    q=np.asarray(quaternion,float);p=np.asarray(position,float)
    if q.shape!=(4,) or p.shape!=(3,) or not np.isfinite(q).all() or not np.isfinite(p).all() or not .9<=np.linalg.norm(q)<=1.1:raise ValueError('INVALID_LIO_POSE')
    x,y,z,w=q/np.linalg.norm(q);T=np.eye(4)
    T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];T[:3,3]=p;return T

def estimator_ready(health,now,timeout=.5):
    try:
        if health.get('source_status',{}).get('last_drop_age_s',999)<.5:return False
        ages=[now-health[k] for k in ('imu_completed_monotonic','estimator_completed_monotonic','adapter_completed_monotonic')]
        if health.get('registration_paused') or health.get('worker_error') or health.get('adapter_error') or health.get('invalid') or health.get('readonly_test_inhibit'):return False
        if any(not math.isfinite(a) or not 0<=a<=timeout for a in ages):return False
        return health.get('estimator_frames',0)>=3 and all(name in health.get('sensors',{}) for name in ('front','back'))
    except (TypeError,KeyError):return False

def interpolate_poses(times,rows,max_gap=.25):
    """Interpolate measured LIO poses only; never extrapolate through a hole."""
    from scipy.spatial.transform import Rotation,Slerp
    times=np.asarray(times,float);stamps=np.array([r[0] for r in rows],float);matrices=np.array([r[1] for r in rows],float)
    if not len(times) or len(stamps)<2 or not np.isfinite(times).all() or np.any(np.diff(stamps)<=0):raise ValueError('POSE_INTERPOLATION_REQUIRES_ORDERED_HISTORY')
    if times.min()<stamps[0]-1e-6 or times.max()>stamps[-1]+1e-6:raise ValueError('POSE_HISTORY_DOES_NOT_COVER_SCAN')
    query=np.clip(times,stamps[0],stamps[-1]);index=np.clip(np.searchsorted(stamps,query,side='right')-1,0,len(stamps)-2)
    if np.any(stamps[index+1]-stamps[index]>max_gap):raise ValueError('POSE_HISTORY_GAP')
    # Subtract epoch to preserve precision in interpolation internals.
    relative=stamps-stamps[0];q=query-stamps[0];rotations=Slerp(relative,Rotation.from_matrix(matrices[:,:3,:3]))(q).as_matrix()
    result=np.tile(np.eye(4),(len(query),1,1));result[:,:3,:3]=rotations
    for axis in range(3):result[:,axis,3]=np.interp(q,relative,matrices[:,axis,3])
    return result

def archived_lio_history(raw_folder,meta):
    """Read optional full-rate LIO journal, transformed into this map frame."""
    from pathlib import Path
    import sqlite3
    if meta.get('map_from_native_world') is None:return []
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    A=np.array(meta['map_from_native_world'],float);T_IB=np.array(meta['T_imu_lidar'],float)@np.linalg.inv(np.array(meta['T_base_lidar'],float));history=[]
    for path in sorted(Path(raw_folder).glob('*.db3')):
        with sqlite3.connect('file:'+str(path)+'?mode=ro',uri=True) as db:
            for raw, in db.execute("SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/fastlio/odometry') ORDER BY timestamp"):
                m=deserialize_message(raw,Odometry)
                if m.header.frame_id!='camera_init':raise ValueError('LIO_JOURNAL_FRAME_MISMATCH')
                p=m.pose.pose.position;q=m.pose.pose.orientation;T=pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]);stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;history.append((stamp,A@T@T_IB))
    history.sort(key=lambda row:row[0]);return [row for i,row in enumerate(history) if i==0 or row[0]>history[i-1][0]+1e-7]

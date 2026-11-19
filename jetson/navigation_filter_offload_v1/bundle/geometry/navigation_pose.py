"""Navigation viewer pose selection; snapshots never become live ROS states."""
import sys
from pathlib import Path
CLIENT=Path('/home/nvidia/tianji_upper_client/tianji_m6_teleop_delivery_20260902')
sys.path.insert(0,str(CLIENT/'src'))
import robot_pose_store as store
import tianji_state_ipc as ipc

def source_packet(model_sha):
    mode=store.mode();packet=dict(schema_version=1,mode=mode,model_sha256=model_sha)
    if mode=='saved':
        snapshot=store.load_saved()
        if snapshot.get('model_sha256')!=model_sha:raise ValueError('保存姿态与当前URDF版本不匹配')
        packet['snapshot']=snapshot
    return packet

def save_current():return store.save_current(ipc.bridge_pose(),ipc.alive_status() or {})

"""Shared lightweight presentation of received readiness, no model/FK imports.

This is not motion admission. The navigator still checks fresh live inputs.
"""
import time
from clearance_override import effective
from gripper_posture_policy import fixed

def received_readiness(data,map_hash):
 guard=data.get('clearance_guard',{})
 release=effective(guard.get('clearance_clear',False),data.get('clearance_state',{}).get('clear',False),map_hash,data.get('whole_model_sha256'))
 _,evidence=fixed(data.get('whole_model_sha256'))
 return dict(arm_fresh=data.get('arm_fresh'),head_fresh=data.get('source_status',{}).get('head_valid',False),fixed_gripper_policy=evidence,clearance_reasons=guard.get('reasons',[]),model_hash=data.get('whole_model_sha256'),hardware_stop_connected=guard.get('hardware_stop_connected',False),source_received_at=data.get('received_at'),receiver_status_age_s=time.time()-data['received_at']if isinstance(data.get('received_at'),(int,float))else None,**release)

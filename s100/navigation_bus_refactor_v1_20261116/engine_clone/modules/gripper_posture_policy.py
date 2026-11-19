"""Explicit fixed mechanical gripper geometry; never synthesizes feedback."""
import hashlib,json,math
from functools import lru_cache
from common import ROOT

FILE=ROOT/'config/gripper_posture_policy.json'
NAMES={side:side+'_robotiq_85_left_knuckle_joint'for side in ('left','right')}

@lru_cache(maxsize=4)
def _read(path,mtime,size):
 return json.loads(path.read_text())

def configuration():
 if not FILE.exists():return dict(mode='measured')
 stat=FILE.stat()
 return dict(_read(FILE,stat.st_mtime_ns,stat.st_size))

def fixed(model_hash,policy=None):
 policy=configuration() if policy is None else policy
 if policy.get('mode')=='measured':return {},None
 if policy.get('mode')!='fixed_open':raise ValueError('INVALID_GRIPPER_POSTURE_MODE')
 if policy.get('model_sha256')!=model_hash:raise ValueError('FIXED_GRIPPER_MODEL_MISMATCH')
 q=policy.get('joint_positions_rad',{})
 if set(q)!=set(NAMES.values()) or any(type(v)not in (int,float)or not math.isfinite(v)or not 0<=v<=.8 for v in q.values()):raise ValueError('INVALID_FIXED_GRIPPER_ENDPOINTS')
 evidence=dict(mode='fixed_open',model_sha256=model_hash,joint_positions_rad=dict(q),source='USER_DECLARED_DISCONNECTED_GRIPPERS_FIXED_OPEN',measured=False)
 evidence['policy_sha256']=hashlib.sha256(json.dumps(evidence,sort_keys=True).encode()).hexdigest()
 return dict(q),evidence

def apply(packet,values,evidence):
 if not evidence:return dict(packet)
 result=dict(packet,joint_positions_rad={**packet['joint_positions_rad'],**values},fixed_gripper_policy=dict(evidence))
 result['source']='FRESH_ARM_FEEDBACK_WITH_EXPLICIT_FIXED_OPEN_GRIPPERS'
 return result

def valid_scan_evidence(packet,model_hash):
 if packet.get('model_sha256')!=model_hash:return False
 values,evidence=fixed(model_hash)
 if evidence is None:
  return packet.get('source')=='recorded_measured_feedback' and packet.get('fixed_gripper_policy')is None
 return (packet.get('source')=='FRESH_ARM_FEEDBACK_WITH_EXPLICIT_FIXED_OPEN_GRIPPERS'
         and packet.get('fixed_gripper_policy')==evidence
         and all(packet.get('joint_positions_rad',{}).get(k)==v for k,v in values.items()))

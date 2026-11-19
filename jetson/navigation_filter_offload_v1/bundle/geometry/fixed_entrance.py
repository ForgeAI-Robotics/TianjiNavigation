"""Explicit fixed-posture entrance sessions for isolated collision advisories.
No inferred entrance posture, actuator commands or general movement permission.
"""
import hashlib,json
import numpy as np
from link import JOINTS,number
from chassis_proxy import replace_shapes,profile

class FixedEntranceGuard:
    def __init__(self,model,tolerance_rad=np.deg2rad(.5)):
        self.model=model;self.tolerance=tolerance_rad;self.references={};self.chassis=profile(model)
    def check(self,candidate,measured):
        status=dict(mode='fixed_posture_entrance',state='UNKNOWN',motion_inhibit=True,reference_registered=False,command_path_connected=False)
        try:
            key=candidate.get('entrance_profile_id');reference=candidate.get('joint_positions_rad')
            if not isinstance(key,str) or not 1<=len(key)<=128:raise ValueError('ENTRANCE_PROFILE_ID_REQUIRED')
            if candidate.get('model_sha256')!=self.model.sha256:raise ValueError('ENTRANCE_MODEL_MISMATCH')
            if not isinstance(reference,dict) or set(reference)!=set(JOINTS) or any(not number(v) for v in reference.values()):raise ValueError('ENTRANCE_FULL_REFERENCE_POSE_REQUIRED')
            missing=[n for n in JOINTS if n not in measured or not number(measured[n])]
            if missing:
                status['missing_joints']=missing;raise ValueError('ENTRANCE_MEASURED_POSE_INCOMPLETE')
            difference={n:abs(measured[n]-reference[n]) for n in JOINTS}
            worst=max(difference,key=difference.get);status.update(max_deviation_rad=difference[worst],worst_joint=worst,tolerance_rad=float(self.tolerance))
            if difference[worst]>self.tolerance:raise ValueError('ENTRANCE_POSTURE_DEVIATED')
            digest=hashlib.sha256(json.dumps(reference,sort_keys=True,allow_nan=False).encode()).hexdigest()
            if key in self.references:
                if self.references[key]['digest']!=digest:raise ValueError('ENTRANCE_REFERENCE_CHANGED_WITHIN_SESSION')
            else:
                if len(self.references)>=8:raise ValueError('ENTRANCE_SESSION_LIMIT_RESTART_REQUIRED')
                shapes=replace_shapes(self.model.shapes(reference),self.chassis)
                if any(s['mode']!='measured_or_fixed' for s in shapes):raise ValueError('ENTRANCE_MODEL_POSE_INCOMPLETE')
                corners=np.concatenate([np.array(s['world_corners']) for s in shapes]);self.references[key]=dict(digest=digest,shapes=shapes,positions=dict(reference),bounds=[corners.min(0).tolist(),corners.max(0).tolist()])
            record=self.references[key]
            # Reuse exact reference geometry only for identical readings. Small
            # accepted deviations still use actual FK, not a frozen approximation.
            exact=all(measured[n]==reference[n] for n in JOINTS)
            shapes=record['shapes'] if exact else replace_shapes(self.model.shapes(measured),self.chassis)
            status.update(state='POSTURE_MATCHED',motion_inhibit=False,reference_registered=True,entrance_profile_id=key,reference_bounds_m=record['bounds'],geometry_source='cached_exact_reference' if exact else 'current_measured_fk',environment_check_still_required=True)
            return status,shapes
        except (ValueError,TypeError,KeyError) as exc:
            status['reason']=str(exc);return status,None

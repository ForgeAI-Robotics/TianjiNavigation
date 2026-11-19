"""Explicit assumptions for read-only planning. No actuation or live-gate imports."""
import math
ARMS=[f'Joint{i}_{side}'for side in ['L','R']for i in range(1,8)]
GRIPPERS=[side+'_robotiq_85_left_knuckle_joint'for side in ['left','right']]

def validate_options(options):
 options=dict(options);options.setdefault('use_saved_arms',False)
 if set(options)!={'ignore_head_camera','open_grippers','use_saved_arms'} or any(type(v)is not bool for v in options.values()):raise ValueError('INVALID_PREVIEW_SWITCHES')
 return dict(options)

def camera_check(options,release):
 validate_options(options)
 if not options['ignore_head_camera'] and not(release.get('raw_clearance_clear')and release.get('raw_shapemask_clear')):raise ValueError('HEAD_CAMERA_SHAPEMASK_NOT_MEASURED_CLEAR')

def posture(raw,options,opened,now):
 options=validate_options(options);q=raw.get('joint_positions_rad',{})
 if options['use_saved_arms'] and not raw.get('historical_posture_only'):raise ValueError('EXPLICIT_SAVED_POSTURE_SOURCE_REQUIRED')
 if not options['use_saved_arms'] and not -.10<=now-raw.get('observation_time',0)<=.35:raise ValueError('FRESH_ARM_FEEDBACK_REQUIRED')
 if any(k not in q or not math.isfinite(q[k])for k in ARMS):raise ValueError('FRESH_14_ARM_JOINTS_REQUIRED')
 if set(opened)!=set(GRIPPERS)or any(not math.isfinite(x)for x in opened.values()):raise ValueError('INVALID_CALIBRATED_GRIPPER_ENDPOINTS')
 result=dict(raw,joint_positions_rad=dict(q),live=False);assumed={}
 if options['open_grippers']:
  assumed=dict(opened);result['joint_positions_rad'].update(assumed)
 elif any(k not in q or not math.isfinite(q[k])for k in GRIPPERS):raise ValueError('MEASURED_GRIPPER_FEEDBACK_REQUIRED_SWITCH_OPEN_TO_PREVIEW')
 result.update(source='SAVED_POSTURE_PREVIEW_ONLY' if options['use_saved_arms'] else 'READ_ONLY_PREVIEW_CURRENT_FEEDBACK_WITH_EXPLICIT_OPTIONS',preview_options=dict(options),assumed_joints=assumed)
 return result

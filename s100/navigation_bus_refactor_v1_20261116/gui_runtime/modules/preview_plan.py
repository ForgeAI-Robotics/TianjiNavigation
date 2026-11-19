import sys as _bootstrap_sys
from pathlib import Path as _BootstrapPath
_bootstrap_sys.path.insert(0,'/home/sunrise/tianji_table_approach_dev/navigation_s100_v7/modules')
import paths as _production_paths
_bootstrap_sys.path.insert(0,str(_BootstrapPath(__file__).resolve().parent))
"""S100 read-only path planning with explicit camera/gripper switches."""
import argparse,json,time,threading,math
from pathlib import Path
import numpy as np
from preview_posture import validate_options,camera_check,posture,ARMS,GRIPPERS

def run(goals,options):
 from paths import ROOT
 from common import atomic_json,exclusive
 from onboard_state import measured_joints,readiness
 from global_localization import localize
 from prepare import prepare
 from policy import plan
 import targets
 options=validate_options(options);saved_raw=None
 if options['use_saved_arms']:
  from saved_preview_posture import load as load_saved
  from onboard_state import model
  saved_raw=load_saved(ROOT,model().sha256)
 read_posture=lambda:dict(saved_raw) if saved_raw is not None else measured_joints()
 opened=json.loads((ROOT/'config/gripper_open_endpoints.json').read_text())['joint_positions_rad']
 samples=[];stop=threading.Event()
 def sample():
  while not stop.is_set():
   try:samples.append(measured_joints())
   except (ValueError,OSError):pass
   stop.wait(.05)
 with exclusive(ROOT/'runtime/navigation_execution.lock'),exclusive(ROOT/'runtime/navigation.lock'):
  camera_check(options,readiness());posture(read_posture(),options,opened,time.time())
  thread=threading.Thread(target=sample,daemon=True);thread.start() if saved_raw is None else None
  try:
   capture=ROOT/'results'/('preview_capture_'+targets.scene_id()+'_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:]);capture.mkdir()
   loc=localize(Path(targets.get(goals[0])['map_folder']),capture/'localization',preview_saved_posture=saved_raw) if saved_raw is not None else localize(Path(targets.get(goals[0])['map_folder']),capture/'localization');raw=read_posture()
   tracked=ARMS+([]if options['open_grippers']else GRIPPERS)
   def check_motion(current):
    now=np.array([current['joint_positions_rad'][k]for k in tracked]);observed=np.array([[row['joint_positions_rad'][k]for k in tracked]for row in samples if all(k in row['joint_positions_rad']for k in tracked)])
    drift=float(np.max(abs(observed-now)))if len(observed)else float('inf')
    if drift>math.radians(.5):raise ValueError('POSTURE_CHANGED_DURING_CAPTURE_REPLAN_REQUIRED')
    return drift
   check_motion(raw) if saved_raw is None else None
   for goal in goals:
    camera_check(options,readiness());raw=read_posture();drift=None if saved_raw is not None else check_motion(raw);loc['saved_pose']=posture(raw,options,opened,time.time())
    note=dict(options=options,assumed_joints=loc['saved_pose']['assumed_joints'],head_geometry='UNCHANGED_URDF_BOUNDED_UNKNOWN'if'head_pitch_joint'not in loc['saved_pose']['joint_positions_rad']else'MEASURED',arm_feedback='SAVED_POSTURE_PREVIEW_ONLY' if saved_raw is not None else 'FRESH_MEASURED_14_JOINTS',saved_posture_file=None if saved_raw is None else saved_raw['saved_posture_file'],saved_posture_observation_time=None if saved_raw is None else saved_raw['observation_time'],maximum_posture_change_during_capture_rad=drift)
    loc['preview_assumptions']=note
    folder=ROOT/'results'/('goal_'+str(goal)+'_preview_'+targets.scene_id()+'_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:]);(folder/'localization').mkdir(parents=True)
    atomic_json(folder/'localization/localization.json',loc);atomic_json(folder/'localization/acquisition.json',json.loads((capture/'localization/acquisition.json').read_text()))
    recorded,world=prepare(loc,targets.get(goal),folder/'scene',assume_clearance=True,reviewed_map_cleanup=True);result=plan(recorded,world,folder/'scene/frozen_input.json',folder/'planning',targets.get(goal))
    result.update(preview_options=options,preview_posture_assumptions=note,motion_commands_sent=0,execution_permission=False);atomic_json(folder/'planning/plan.json',result)
    pointer=dict(folder=str(folder),result=str(folder/'planning/plan.json'));atomic_json(ROOT/'runtime/latest_plan.json',pointer)
    print('PREVIEW_RESULT '+json.dumps(dict(goal=goal,folder=str(folder),state=result['state'],options=options,motion_commands_sent=0)),flush=True)
  finally:stop.set();thread.join(timeout=2) if thread.ident is not None else None

if __name__=='__main__':
 p=argparse.ArgumentParser();choice=p.add_mutually_exclusive_group(required=True);choice.add_argument('--goal',type=int,choices=[0,1,2,3]);choice.add_argument('--both',action='store_true');p.add_argument('--ignore-head-camera',type=int,choices=[0,1],required=True);p.add_argument('--use-saved-arms',type=int,choices=[0,1],default=0);p.add_argument('--open-grippers',type=int,choices=[0,1],required=True);a=p.parse_args();run([1,2]if a.both else[a.goal],dict(ignore_head_camera=bool(a.ignore_head_camera),open_grippers=bool(a.open_grippers),use_saved_arms=bool(a.use_saved_arms)))

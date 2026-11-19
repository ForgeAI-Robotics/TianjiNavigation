"""Hybrid A* differential travel with explicitly bounded lateral assist."""
import json,subprocess,math
from paths import ROOT
from common import atomic_json
from prepare import reference_grid
import targets

def plan(recorded,world,input_file,output,goal):
 if not targets.unchanged(goal):raise ValueError('TARGET_CHANGED_REPLAN_REQUIRED')
 output.mkdir(parents=True,exist_ok=True);grid,origin,res=reference_grid(recorded)
 cfg=json.loads((ROOT/'config/motion_policy.json').read_text())
 packet=dict(world=world,goal=recorded['goal'],motion_policy=cfg,grid=dict(origin=list(origin),width=grid.shape[1],height=grid.shape[0],resolution=res,data=grid.ravel().tolist()))
 atomic_json(output/'hybrid_input.json',packet)
 subprocess.run([str(ROOT/'native/hybrid'),str(output/'hybrid_input.json'),str(output/'hybrid_result.json')],check=True,timeout=2*cfg['search_budget_s_per_stage']+15)
 native=json.loads((output/'hybrid_result.json').read_text());selected=native['selected']
 if not targets.unchanged(goal):raise ValueError('TARGET_CHANGED_REPLAN_REQUIRED')
 position_reached=math.hypot(recorded['goal'][0],recorded['goal'][1])<=.075 and abs(recorded['goal'][2])<=math.radians(5)
 state=('AT_TARGET_MOTION_INHIBITED' if position_reached and not selected else 'AT_TARGET' if position_reached else 'PATH_PREVIEW_READY' if selected else 'NO_VALID_PREVIEW_PATH')
 summary=dict(state=state,position_reached=position_reached,target=goal,start_map_pose=recorded['preview_start_map_pose'],attempts=[{k:v for k,v in p.items() if k not in ['path','primitives']} for p in native['attempts']],selected=selected,engine=native['engine'],clearance_state='ASSUMED_CLEAR_BY_USER',fixed_posture_for_this_plan=recorded['saved_pose'],motion_policy=cfg,execution_permission=False,motion_commands_sent=0)
 atomic_json(output/'plan.json',summary);return summary

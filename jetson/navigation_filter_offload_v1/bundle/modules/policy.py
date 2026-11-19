"""Hybrid A* differential travel with explicitly bounded lateral assist."""
import json,subprocess,math,copy
from paths import ROOT
from common import atomic_json
from prepare import reference_grid
import targets

def goal_reached(goal,config):
 fine=config.get('fine_goal',{})
 position=fine['position_tolerance_m'] if fine.get('enabled') else .075
 heading=fine['heading_tolerance_deg'] if fine.get('enabled') else 5.
 return math.hypot(goal[0],goal[1])<=position and abs(math.atan2(math.sin(goal[2]),math.cos(goal[2])))<=math.radians(heading)

def inspection_intent_packet(packet):
 """Uncertified intent only; the retained scene must check its local prefix."""
 result=copy.deepcopy(packet);result['world']['occupied_points']=[]
 result['motion_policy']['near_start_fine_escape_enabled']=False
 result['grid']['data']=[0 if x==100 else x for x in result['grid']['data']]
 return result

def right_micro_preview(recorded,output,goal):
 """Check a 3.5cm pure-right prefix plus 1.5cm braking reserve."""
 output.mkdir(parents=True,exist_ok=True)
 grid,origin,res=reference_grid(recorded)
 poses=[[0.,-i*.005,0.] for i in range(11)]
 def observed(pose):
  x,y=(math.floor((pose[i]-origin[i])/res) for i in range(2))
  return 0<=x<grid.shape[1] and 0<=y<grid.shape[0] and 0<=grid[y,x]<100
 if not all(map(observed,poses)):
  check=dict(clear=False,error='RIGHT_PREFIX_OUTSIDE_OBSERVED_GRID')
 else:
  query=json.dumps(dict(id=1,poses=poses))+'\n'
  run=subprocess.run([str(ROOT/'native/world_guard'),str(output.parent/'scene/immutable_world.json')],
                     input=query,capture_output=True,text=True,check=True,timeout=3)
  check=json.loads(run.stdout)
 selected=dict(profile='short_local_translation',path=poses[:8],geometry_recheck_passed=True,
               length_m=.035,lateral_distance_m=.035,goal_error_m=0.) if check.get('clear') else None
 result=dict(state='PATH_PREVIEW_READY' if selected else 'NO_VALID_PREVIEW_PATH',target=goal,
             start_map_pose=recorded['preview_start_map_pose'],selected=selected,
             attempts=[dict(reason='RIGHT_PREFIX_CLEAR' if selected else check.get('link') or check.get('error') or 'MAP_COLLISION')],
             execution_permission=False,motion_commands_sent=0,
             right_micro_preview=dict(speed_mps=.005,distance_m=.035,swept_distance_m=.05))
 atomic_json(output/'plan.json',result)
 return result

def plan(recorded,world,input_file,output,goal):
 intent_only=False
 if not targets.unchanged(goal):raise ValueError('TARGET_CHANGED_REPLAN_REQUIRED')
 output.mkdir(parents=True,exist_ok=True);grid,origin,res=reference_grid(recorded)
 cfg=json.loads((ROOT/'config/motion_policy.json').read_text())
 packet=dict(world=world,goal=recorded['goal'],motion_policy=cfg,grid=dict(origin=list(origin),width=grid.shape[1],height=grid.shape[0],resolution=res,data=grid.ravel().tolist()))
 atomic_json(output/'hybrid_input.json',packet)
 engine='hybrid_office_preview' if recorded.get('reviewed_history_voxels_excluded') or recorded.get('reviewed_history_baked_map') else 'hybrid'
 subprocess.run([str(ROOT/'native'/engine),str(output/'hybrid_input.json'),str(output/'hybrid_result.json')],check=True,timeout=2*cfg['search_budget_s_per_stage']+15)
 native=json.loads((output/'hybrid_result.json').read_text());selected=native['selected']
 if selected is None and native.get('attempts') and all(a.get('reason')=='SEARCH_EXHAUSTED' and a.get('expanded_nodes',99)<=1 for a in native['attempts']):
  # Retry the same engine at a finer start lattice. All occupied/unknown
  # tests and the mandatory dense whole-body recheck remain in force.
  packet['motion_policy']=dict(cfg,near_start_fine_escape_enabled=True)
  atomic_json(output/'hybrid_fine_start_input.json',packet)
  subprocess.run([str(ROOT/'native/hybrid'),str(output/'hybrid_fine_start_input.json'),str(output/'hybrid_fine_start_result.json')],check=True,timeout=2*cfg['search_budget_s_per_stage']+15)
  retry=json.loads((output/'hybrid_fine_start_result.json').read_text());native['attempts'].extend(retry['attempts']);selected=retry['selected'];cfg=packet['motion_policy']
 if selected is None and recorded.get('cable_precheck'):
  # No full-route feasibility is required to inspect local wiring. A blocked
  # distant goal must not prevent an independently guarded 5cm experiment.
  intent=inspection_intent_packet(packet);atomic_json(output/'hybrid_inspection_intent_input.json',intent)
  subprocess.run([str(ROOT/'native/hybrid'),str(output/'hybrid_inspection_intent_input.json'),str(output/'hybrid_inspection_intent_result.json')],check=True,timeout=2*cfg['search_budget_s_per_stage']+15)
  proposal=json.loads((output/'hybrid_inspection_intent_result.json').read_text());native['attempts'].extend([dict(a,scope='INSPECTION_INTENT_ONLY') for a in proposal['attempts']]);selected=proposal['selected']
  if selected:
   intent_only=True;selected['geometry_scope']='KINEMATIC_INTENT_ONLY; retained real scene must check 6cm prefix; remainder requires fresh full-scene planning'
 if not targets.unchanged(goal):raise ValueError('TARGET_CHANGED_REPLAN_REQUIRED')
 position_reached=goal_reached(recorded['goal'],cfg)
 state=('AT_TARGET_MOTION_INHIBITED' if position_reached and not selected else 'AT_TARGET' if position_reached else 'PATH_PREVIEW_READY' if selected else 'NO_VALID_PREVIEW_PATH')
 summary=dict(state=state,position_reached=position_reached,target=goal,start_map_pose=recorded['preview_start_map_pose'],attempts=[{k:v for k,v in p.items() if k not in ['path','primitives']} for p in native['attempts']],selected=selected,engine=native['engine'],clearance_state='ASSUMED_CLEAR_BY_USER',fixed_posture_for_this_plan=recorded['saved_pose'],motion_policy=cfg,execution_permission=False,motion_commands_sent=0)
 summary['planning_scene_policy']=recorded.get('planning_scene_policy',{})
 summary['manual_floor']=recorded.get('manual_floor',dict(enabled=False))
 summary['intent_prefix_only']=intent_only
 atomic_json(output/'plan.json',summary);return summary

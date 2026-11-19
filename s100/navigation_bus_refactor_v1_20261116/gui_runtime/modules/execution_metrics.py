"""Execution evidence shared by real runs and diagnostic-output load tests."""
from pathlib import Path
from collections import Counter
import hashlib
import numpy as np

def signature(root):
 names=['onboard_navigation','navigation_runtime_profile','prepared_frames','prepared_worker','sensor_relay','relay_scheduling','sensor_bus','execution_safety','navigation_self_filter','near_body_cleanup','cable_zones','surface_components','localization_tracker','navigation_control','control_agent','control_bridge','mapping_guard','motor_trace','motor_trace_worker','continuous_surface_filter','joint_scan_history','gripper_posture_policy','onboard_state','collision_evidence','common','prepare','policy','global_localization','map_trajectory_cache','resident_localization','client','localization_tracker']
 result={name:hashlib.sha256((Path(root)/'modules'/(name+'.py')).read_bytes()).hexdigest() if (Path(root)/'modules'/(name+'.py')).exists() else 'ABSENT' for name in names}
 for name in ['isolated_returns','live_voxel_refinement','pose_memory','route_recovery','registration_choice','operator_session','planning_process','planning_worker','collision_evidence']:
  file=Path(root)/'modules'/(name+'.py');result[name]=hashlib.sha256(file.read_bytes()).hexdigest()if file.exists()else'ABSENT'
 for name in ['local_model','compiled_geometry']:
  file=Path(root)/'geometry'/(name+'.py');result['geometry/'+name]=hashlib.sha256(file.read_bytes()).hexdigest() if file.exists() else 'ABSENT'
 file=Path(root)/'modules/pillar_cables.py';result['pillar_cables']=hashlib.sha256(file.read_bytes()).hexdigest() if file.exists() else 'ABSENT'
 for name in ('contracts','resources','operator','status','motion'):
  file=Path(__file__).resolve().parent/'execution_runtime'/(name+'.py')
  result['execution_runtime/'+name]=hashlib.sha256(file.read_bytes()).hexdigest()if file.exists()else'ABSENT'
 for name in ('contracts','route','collision','admission'):
  file=Path(__file__).resolve().parent/'execution_stages'/(name+'.py')
  result['execution_stages/'+name]=hashlib.sha256(file.read_bytes()).hexdigest()if file.exists()else'ABSENT'
 result['execution_backend']='MODULAR_GUI_CANDIDATE'
 result['actual_execution_entry_sha256']=hashlib.sha256((Path(__file__).parent/'onboard_navigation.py').read_bytes()).hexdigest()
 result['candidate_policy_sha256']=hashlib.sha256((Path(__file__).parent/'policy.py').read_bytes()).hexdigest()
 result['candidate_planning_worker_sha256']=hashlib.sha256((Path(__file__).parent/'planning_worker.py').read_bytes()).hexdigest()
 result['waypoint_corridor_graph_sha256']=hashlib.sha256((Path(__file__).parent/'waypoint_corridor_graph.py').read_bytes()).hexdigest()
 result['frozen_world_guard_sha256']=hashlib.sha256((Path(__file__).parent/'frozen_world_guard.py').read_bytes()).hexdigest()
 result['candidate_follower_sha256']=hashlib.sha256((Path(__file__).parent/'follower.py').read_bytes()).hexdigest()
 return result

def quantiles(values):
 return dict(zip(['p50','p95','max'],map(float,np.quantile(values,[.5,.95,1.])))) if values else None

def summarize(rows):
 if not rows:return {}
 final=rows[-1];sensors={}
 for sensor in ('front','back'):
  samples=[r['perception_timing'][sensor] for r in rows if sensor in r.get('perception_timing',{})]
  unique={r['sequence']:r for r in samples}
  sensors[sensor]=dict(unique_frames=len(unique),filter_compute_s=quantiles([r['filter_compute_s'] for r in unique.values()]),
   filter_cpu_s=quantiles([r['filter_cpu_s'] for r in unique.values() if r.get('filter_cpu_s') is not None]),
   raw_bus_queue_s=quantiles([r['raw_bus_queue_s'] for r in unique.values()]),
   source_age_at_status_s=quantiles([r['source_age_now_s'] for r in samples]))
 reasons=dict(Counter(r.get('reason','UNSPECIFIED') for r in rows))
 return dict(mode=final['mode'],task_success=final['phase']=='ARRIVED',final_phase=final['phase'],reason_counts=reasons,
  motion_commands_sent=final.get('motion_commands_sent',0),command_gate_stats=final.get('command_gate_stats',{}),
  loop_compute_s=quantiles([r['loop_compute_s'] for r in rows if r.get('loop_compute_s') is not None]),
  sensors=sensors,code_signature=final.get('code_signature'),execution_profile=final.get('execution_profile'),
  compute_cpu_affinity=final.get('compute_cpu_affinity'),evidence_scope='actual_navigation' if final['mode']=='REAL_MANUAL_START' else 'stationary_ROS_load_no_motor_output' if final['mode']=='LOAD_TEST_NO_MOTION' else 'stationary_preview_without_ROS_control_bridge',
  intermittent_expiry=any(reasons.get(k,0) for k in ['LIDAR_STALE','ADMISSION_DEADLINE_EXPIRED']),
  motion_validation_still_required=final['mode']!='REAL_MANUAL_START')

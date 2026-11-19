"""Keep stable global occupancy separate from live execution guards."""
import numpy as np

def select_obstacles(history,live,config,*,local_snapshot=False):
 policy=config.get('scene_policy',{})
 if policy.get('global_obstacles')!='static_map_only' or policy.get('live_obstacle_action')!='wait_then_resume' or policy.get('dynamic_obstacle_replan') is not False:
  raise ValueError('INVALID_STATIC_GLOBAL_SCENE_POLICY')
 # Explicit local-snapshot tools have no saved-map authority. Keep their
 # measured obstacle input; they are not global navigation plans.
 merged=np.concatenate([history,*live]) if local_snapshot else history.copy()
 return merged,dict(global_obstacles='live_local_snapshot' if local_snapshot else 'static_map_only',live_points_in_global_occupancy=bool(local_snapshot),live_points_observed=sum(len(p)for p in live),live_obstacle_action='wait_then_resume',dynamic_obstacle_replan=False,live_execution_guard_required=True)

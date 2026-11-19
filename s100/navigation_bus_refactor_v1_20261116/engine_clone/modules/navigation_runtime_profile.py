"""Shared production and stationary ROS-load timing profile, in seconds/Hz."""
RAW_SCAN_HZ=4.
LOCALIZATION_MATCH_PERIOD_S=.5
CONTROL_CHECK_PERIOD_S=.2

def filter_profile(scene,goal,micro,cable,selected_profile,requested_cap,default_cap,advance_s):
 """One office filter policy, independent of which planner profile won."""
 live=scene=='office' and goal in (0,1,2,3) and not cable
 directional=live and not micro
 short=selected_profile in ('short_axial_translation','short_local_translation','short_diagonal_translation','short_rectilinear_translation','short_turn_then_translation')
 cap=requested_cap if requested_cap is not None else default_cap
 radius=min(.16,.055+cap*(1.2+.5+advance_s)+1e-6) if directional else .065
 return dict(live_filter=live,directional_translation=directional,translation_cap_mps=cap,refine_radius_m=radius)

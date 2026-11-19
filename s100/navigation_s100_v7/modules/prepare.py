"""Frozen map + actual LiDAR returns + measured URDF posture, preview only."""
import json,math,hashlib
import numpy as np
from scipy.spatial import cKDTree
from paths import ROOT,NAV,STACK,VIEWER
from common import atomic_json
from saved_map import base_transform
from extrinsics import transforms
from snapshot_builder import local_model,build_snapshot_replay
from chassis_proxy import replace_shapes,profile
from environment import ground_plane,self_candidates
from pillar_cables import settings as cable_settings,ignore_mask
from self_envelope import settings as envelope_settings,mask as envelope_mask
from navigation_self_filter import evaluate as evaluate_self_filter
from planning_scene_policy import select_obstacles

from voxel_centers import downsample_centers

def history_review_applies(cleanup,map_sha256,resolution,start):
 if cleanup['map_voxel_sha256']!=map_sha256 or cleanup['resolution_m']!=resolution:return False
 review_pose=cleanup.get('review_pose_map_m')
 if review_pose is None:return True
 start=np.asarray(start);review_pose=np.asarray(review_pose)
 if start.shape!=(3,) or review_pose.shape!=(3,) or not np.isfinite(start).all() or not np.isfinite(review_pose).all():return False
 yaw=math.atan2(math.sin(start[2]-review_pose[2]),math.cos(start[2]-review_pose[2]))
 return bool(np.linalg.norm(start[:2]-review_pose[:2])<=cleanup['max_start_translation_m'] and abs(yaw)<=math.radians(cleanup['max_start_yaw_deg']))

def prepare(localization,goal,output,assume_clearance=False,local_snapshot=False,reviewed_map_cleanup=False):
 if not assume_clearance:raise ValueError('EXPLICIT_ASSUMED_CLEARANCE_REQUIRED_FOR_THIS_PREVIEW')
 expected_status='LOCAL_SNAPSHOT_ONLY' if local_snapshot else 'GLOBAL_LOCALIZED'
 if localization['status']!=expected_status:raise ValueError('GLOBAL_LOCALIZATION_REQUIRED' if not local_snapshot else 'LOCAL_SNAPSHOT_REQUIRED')
 if goal['map_voxel_sha256']!=localization['map_voxel_sha256'] or goal['frame_id']!=localization['frame_id']:raise ValueError('TARGET_MAP_MISMATCH')
 folder=__import__('pathlib').Path(localization['map_folder']);meta=json.loads((folder/'map.json').read_text());raw=(folder/'voxels.npz').read_bytes()
 if hashlib.sha256(raw).hexdigest()!=goal['map_voxel_sha256']:raise ValueError('MAP_CHANGED')
 with np.load(folder/'voxels.npz') as data:map_points=np.array(data['points'])
 model=local_model();saved=localization['saved_pose'];joints=saved['joint_positions_rad'];shapes=replace_shapes(model.shapes(joints),profile(model));radius=max(float(np.linalg.norm(np.asarray(s['world_corners'])[:,:2],axis=1).max()) for s in shapes)
 start=np.asarray(localization['pose']);W=base_transform(start);R=W[:3,:3]
 delta=np.r_[np.asarray(goal['pose'][:2])-start[:2],0.]@R;local_goal=[float(delta[0]),float(delta[1]),math.atan2(math.sin(goal['pose'][2]-start[2]),math.cos(goal['pose'][2]-start[2]))]
 # Search ROI is explicit. Include a full body-radius halo in the occupied input.
 lower=np.minimum([0.,0.],local_goal[:2])-1.5;upper=np.maximum([0.,0.],local_goal[:2])+1.5
 top=max(np.asarray(s['world_corners'])[:,2].max() for s in shapes)+.15
 floor=ground_plane(map_points)
 if floor is None:raise ValueError('SUPPORTED_GROUND_PLANE_REQUIRED')
 residual=np.abs(map_points[:,2]-np.c_[map_points[:,:2],np.ones(len(map_points))]@floor)
 ground_band=.04+float(meta['resolution_m'])/2
 floor_points=map_points[residual<=ground_band];environment=map_points[(residual>ground_band)&(map_points[:,2]>-.15)&(map_points[:,2]<top)]
 removed_history=0;reviewed_history_rules=[]
 if reviewed_map_cleanup and goal['scene_id']=='office' and goal['id'] in (1,2,3) and not local_snapshot:
  reviewed_keys=set()
  for cleanup_file in ('office_preview_cleanup.json','office_preview_cleanup_goal_2.json','office_fullmap_cleanup.json','office_table_edge_review_20261105.json'):
   file=ROOT/'config/scenes'/cleanup_file
   if not file.exists():continue
   cleanup=json.loads(file.read_text())
   # The review applies only to saved-map points near the reviewed start.
   # New live scans are appended after this history-only operation.
   if history_review_applies(cleanup,goal['map_voxel_sha256'],float(meta['resolution_m']),start):
    reviewed_keys.update(tuple(key) for key in cleanup['voxel_keys']);reviewed_history_rules.append(cleanup_file)
  if reviewed_keys:
   retained=np.array([tuple(key) not in reviewed_keys for key in np.floor(environment/float(meta['resolution_m'])).astype(np.int64)],dtype=bool)
   removed_history=int((~retained).sum())
   environment=environment[retained]
 map_environment=environment.copy()
 # Read the full final raw scan from each radar, not the localization crop.
 acquisition=json.loads((output.parent/'localization/acquisition.json').read_text());latest={}
 for row in acquisition['rows']:latest[row['header']['sensor']]=row
 envelope_cfg=envelope_settings();envelope_raw=[];envelope_raw_reports={}
 M=transforms();live=[];raw_scans=[];pillar_ignored=[];pillar_reports={};pillar_cfg=cable_settings();reference=base_transform(localization['wheel_pose']);ambiguous=0
 for sensor,row in latest.items():
  h=row['header']
  with np.load(row['file']) as f:p=f['points_sensor']
  S=np.linalg.inv(reference)@base_transform(h['pose'])@M[sensor];p=p@S[:3,:3].T+S[:3,3]
  p=p[np.isfinite(p).all(1)&(np.linalg.norm(p,axis=1)>.04)&(np.linalg.norm(p,axis=1)<30)]
  raw_scans.append(p@R.T+W[:3,3])
  self_result=evaluate_self_filter(p,shapes,model.sha256,envelope_cfg=envelope_cfg,cable_cfg=pillar_cfg)
  envelope_raw_reports[sensor]=self_result['envelope_report'];envelope_raw.append(p[self_result['extra']].copy());ambiguous+=int(self_result['own'].sum())
  pillar_reports[sensor]=self_result['cable_report'];pillar_ignored.append(p[self_result['cables']].copy());p=p[~self_result['filtered']]
  q=p@R.T+W[:3,3];ground=np.abs(q[:,2]-np.c_[q[:,:2],np.ones(len(q))]@floor)<=ground_band;live.append(q[~ground])
 environment,scene_policy=select_obstacles(environment,live,json.loads((ROOT/'config/motion_policy.json').read_text()),local_snapshot=local_snapshot);resolution=float(meta['resolution_m'])
 def crop(points):
  p=(points-W[:3,3])@R
  # Height-specific reach avoids dragging distant upper walls into a low
  # chassis query. Every retained shape is still checked with its full OBB.
  reach=np.full(len(p),-1.)
  for shape in shapes:
   corners=np.asarray(shape['world_corners']);z0,z1=corners[:,2].min()-.15,corners[:,2].max()+.15
   relevant=(p[:,2]>=z0)&(p[:,2]<=z1)
   reach[relevant]=np.maximum(reach[relevant],np.linalg.norm(corners[:,:2],axis=1).max()+.15)
  valid=((p[:,:2]>=lower-reach[:,None])&(p[:,:2]<=upper+reach[:,None])).all(1)&(reach>=0)&(p[:,2]>-.15)&(p[:,2]<top)
  return p[valid]
 # Coarsen in the original aligned map grid, then rotate the enclosing cells.
 world=downsample_centers(environment,resolution);points=crop(world)
 if len(points)>50000:
  resolution=.10;world=downsample_centers(environment,resolution);points=crop(world)
 if not len(points) or len(points)>50000:raise ValueError('OCCUPANCY_BUDGET_EXCEEDED')
 # Current-body overlap is an ambiguous self/contact channel, retained apart
 # from the replay obstacle set. This is NEVER a free-space certificate.
 # Cell extent must be considered after voxelization, not just on raw points.
 overlap,envelope_voxel_extra,envelope_voxel_report=envelope_mask(points,shapes,envelope_cfg,model.sha256,resolution)
 envelope_voxel_points=points[envelope_voxel_extra].copy()
 # This stage explicitly assumes the CURRENT arms have positive clearance.
 # Isolate upper-arm near returns as assumed self/contact ambiguity, without
 # applying that assumption to the chassis or lower support column.
 arm_shapes=[s for s in shapes if s['mode']=='measured_or_fixed' and
  (s['link'].startswith(('Base_L','Base_R','Link1_','Link2_','Link3_','Link4_','Link5_','Link6_','Link7_','left_','right_','cam_left_wrist','cam_right_wrist')))]
 lowest_arm=min(np.asarray(s['world_corners'])[:,2].min() for s in arm_shapes)
 assumed_arm=self_candidates(points,arm_shapes,.10)&(points[:,2]>=lowest_arm-.15)
 from cable_zones import protected_surfaces
 table_protected,table_protection_report=protected_surfaces(points,shapes,model.sha256,voxel_resolution_m=resolution)
 from near_body_cleanup import evaluate as cleanup,settings as cleanup_settings
 cleanup_mask,continuous_mask,cleanup_report=cleanup(points,shapes,model.sha256)
 table_protected|=continuous_mask
 assumed_arm&=~table_protected
 assumed_arm_count=int(np.count_nonzero(assumed_arm&~overlap));overlap|=assumed_arm
 overlap&=~table_protected
 from cable_zones import zone_ignore
 cable_voxels,cable_voxel_report=zone_ignore(points,shapes,model.sha256,voxel_resolution_m=resolution)
 cable_voxels|=cleanup_mask;cable_voxel_report['near_body_cleanup']=cleanup_report
 cable_voxels&=~table_protected
 excluded_cable_voxels=points[cable_voxels&~overlap].copy()
 overlap|=cable_voxels
 from isolated_returns import mask as isolated_mask
 from pillar_cables import distances
 scope=(distances(points,[s for s in shapes if s['link']!='YD_link'],limit=.25)<=.25)&(points[:,2]>.10)&~overlap
 isolated,isolated_report=isolated_mask(points,scope,table_protected,model.sha256)
 overlap|=isolated
 body_ambiguous=points[overlap].copy();retained_protected=table_protected[~overlap];points=points[~overlap]
 output.mkdir(parents=True,exist_ok=True);np.savez_compressed(output/'current_body_ambiguous.npz',points_local=body_ambiguous)
 np.savez_compressed(output/'localized_cable_voxels.npz',points_body=excluded_cable_voxels)
 atomic_json(output/'localized_cable_voxels.json',dict(report=cable_voxel_report,excluded_voxels=len(excluded_cable_voxels),original_map_modified=False,collision_geometry_modified=False,free_space_credit=False))
 atomic_json(output/'near_body_cleanup.json',dict(configuration=cleanup_settings(),voxel_report=cleanup_report,isolated_returns=isolated_report,micro_probe_enabled=False,original_map_modified=False))
 envelope_raw_points=np.concatenate(envelope_raw) if envelope_raw else np.empty((0,3))
 np.savez_compressed(output/'self_filter_envelope.npz',extra_raw_points_body=envelope_raw_points,extra_voxel_points_body=envelope_voxel_points)
 atomic_json(output/'self_filter_envelope.json',dict(configuration=envelope_cfg,raw_sensors=envelope_raw_reports,voxel=envelope_voxel_report,static_map_file_modified=False,collision_geometry_modified=False,execution_permission=False))
 pillar_ignored_points=np.concatenate(pillar_ignored) if pillar_ignored else np.empty((0,3))
 np.savez_compressed(output/'temporary_pillar_cables.npz',ignored_points_body=pillar_ignored_points)
 atomic_json(output/'temporary_pillar_cables.json',dict(configuration=pillar_cfg,sensors=pillar_reports,total_ignored_live_points=len(pillar_ignored_points),static_map_modified=False,free_space_credit=False,execution_permission=False))
 floor_local=(floor_points-W[:3,3])@R
 from manual_floor import settings as floor_settings,display_points
 manual_floor=floor_settings(goal['map_voxel_sha256'])
 manual_floor_local=display_points(manual_floor,floor,start)
 recorded=dict(planning_scene_policy=scene_policy,model_sha256=model.sha256,saved_pose=saved,points=points.tolist(),goal=local_goal,
  original_cloud_observation_time=localization['observation_time'],source_frame=goal['frame_id'],clearance_assumed=True,
  clearance_contract=dict(state='ASSUMED_CLEAR_BY_USER',scope='arms-clearance prerequisite only; no motion permission',actual_camera_state_ignored_for_preview_only=True),
  map_file=str(folder/'map.json'),map_voxel_sha256=goal['map_voxel_sha256'],map_is_manually_edited_visualization_copy=not local_snapshot,local_snapshot_only=bool(local_snapshot),table_surface_priority=table_protection_report,
  resolution_m=resolution,preview_start_map_pose=start.tolist(),target=goal,grid_search_lower_m=lower.tolist(),grid_search_upper_m=upper.tolist(),
  localization=localization,self_filter_envelope=dict(configuration=envelope_cfg,extra_raw_points=len(envelope_raw_points),extra_voxels=len(envelope_voxel_points),collision_geometry_modified=False),temporary_pillar_cables=dict(enabled=pillar_cfg.get('enabled',False),ignored_live_points=len(pillar_ignored_points),sensors=pillar_reports),excluded_self_contact_ambiguous=ambiguous,excluded_current_body_overlap_voxels=len(body_ambiguous),assumed_upper_arm_ambiguous_voxels=assumed_arm_count,upper_arm_assumed_clear_scope='current pose only, 10cm near-body ambiguity, z above arm underside minus 15cm, no chassis or lower column exemption',self_overlap_policy='ambiguous self/contact excluded in replay only, source points retained, never execution evidence',ground_plane=floor.tolist(),ground_band_m=ground_band,
  reviewed_history_voxels_excluded=removed_history,reviewed_history_rules=reviewed_history_rules,reviewed_history_baked_map=meta.get('persistent_reviewed_cleanup',{}),manual_floor=manual_floor,body_alignment_assumption='planar map pose; vendor base and chassis base identity unverified',navigation_usable=False)
 output.mkdir(parents=True,exist_ok=True);np.savez_compressed(output/'display_clouds.npz',occupied_local=points,floor_local=floor_local,manual_floor_local=manual_floor_local)
 atomic_json(output/'frozen_input.json',recorded)
 world=build_snapshot_replay(model,recorded);world['resolution_m']=resolution
 # Voxel cells are aligned in the saved map, then rotated into body coordinates.
 # Keep their full size and orientation for conservative box-volume checks.
 world['voxel_box_rotation']=R.T.tolist()
 # Only near-contact protected surface cells need the three-scan raw recheck.
 from pillar_cables import distances
 close=retained_protected&(distances(points,shapes,limit=.30)<=.30)
 refinements=[];refinement_report=dict(status='STATIC_MAP_GEOMETRY_NOT_REFINED_FROM_LIVE_SCANS' if not scene_policy['live_points_in_global_occupancy'] else 'NO_BORDERLINE_PROTECTED_CELLS',refined=0)
 if close.any() and scene_policy['live_points_in_global_occupancy']:
  scans=[]
  for row in acquisition['rows']:
   h=row['header']
   with np.load(row['file'])as data:raw_points=data['points_sensor']
   S=np.linalg.inv(reference)@base_transform(h['pose'])@M[h['sensor']]
   aligned=raw_points@S[:3,:3].T+S[:3,3]
   scans.append(dict(header=h,points_map=aligned@R.T+W[:3,3]))
  from live_voxel_refinement import refine
  refinements,refinement_report=refine(points,retained_protected,map_environment,scans,W,shapes,resolution)
 world['live_voxel_refinements']=refinements
 world['planning_scene_policy']=scene_policy
 atomic_json(output/'live_voxel_refinement.json',refinement_report)
 atomic_json(output/'immutable_world.json',world)
 # Retain before/after clouds in the existing desktop cloud library.
 from mapping_core import write_pcd
 from cloud_library import LIBRARY,register_exports
 write_pcd(output/'raw_lidar_planar_map.pcd',np.concatenate(raw_scans))
 write_pcd(output/'planning_scene_map.pcd',points@R.T+W[:3,3])
 write_pcd(output/'manual_floor_map.pcd',manual_floor_local@R.T+W[:3,3])
 atomic_json(output/'manual_floor_map.json',dict(configuration=manual_floor,source='USER_MANUAL_GROUND_ASSUMPTION',synthetic_points=len(manual_floor_local),not_measured_returns=True,clears_occupied_points=False))
 write_pcd(output/'extra_self_envelope_map.pcd',np.concatenate([envelope_raw_points,envelope_voxel_points])@R.T+W[:3,3])
 write_pcd(output/'temporarily_ignored_pillar_cables_map.pcd',pillar_ignored_points@R.T+W[:3,3])
 exports=LIBRARY/'exports';exports.mkdir(parents=True,exist_ok=True)
 for name in ('raw_lidar_planar_map.pcd','planning_scene_map.pcd','manual_floor_map.pcd','temporarily_ignored_pillar_cables_map.pcd','extra_self_envelope_map.pcd'):
  link=exports/(output.parent.name+'__'+name)
  if not link.exists() and not link.is_symlink():link.symlink_to((output/name).resolve())
 register_exports([exports/(output.parent.name+'__'+name) for name in ('raw_lidar_planar_map.pcd','planning_scene_map.pcd','manual_floor_map.pcd','temporarily_ignored_pillar_cables_map.pcd','extra_self_envelope_map.pcd')])
 return recorded,world

def reference_grid(recorded):
 from scipy.ndimage import distance_transform_edt,binary_dilation
 res=.05;lower=np.floor(np.asarray(recorded['grid_search_lower_m'])/res)*res;upper=np.ceil(np.asarray(recorded['grid_search_upper_m'])/res)*res
 width,height=np.ceil((upper-lower)/res).astype(int)
 if width*height>150000:raise ValueError('GRID_AREA_BUDGET_EXCEEDED')
 p=np.asarray(recorded['points']);ids=np.floor((p[:,:2]-lower)/res).astype(int);valid=((ids>=0)&(ids<[width,height])).all(1)
 hit=np.zeros((height,width),bool);hit[ids[valid,1],ids[valid,0]]=True
 distance=distance_transform_edt(~hit)*res;grid=np.rint(60*np.exp(-distance/.15)).astype(np.int8)
 # Only observed ground neighborhoods are candidate planning space. This does
 # not certify free volume: mandatory URDF sweep rejects occupied geometry.
 start=base_transform(recorded['preview_start_map_pose']);folder=__import__('pathlib').Path(recorded['map_file']).parent
 with np.load(folder/'voxels.npz') as f:all_points=f['points']
 floor=np.array(recorded['ground_plane']);ground=all_points[np.abs(all_points[:,2]-np.c_[all_points[:,:2],np.ones(len(all_points))]@floor)<=recorded['ground_band_m']]
 ground=(ground-start[:3,3])@start[:3,:3];gids=np.floor((ground[:,:2]-lower)/res).astype(int);good=((gids>=0)&(gids<[width,height])).all(1)
 known=np.zeros(grid.shape,bool);known[gids[good,1],gids[good,0]]=True;known=binary_dilation(known,iterations=2)
 yy,xx=np.mgrid[:height,:width];x=lower[0]+(xx+.5)*res;y=lower[1]+(yy+.5)*res
 from manual_floor import candidate_mask
 # User-marked ground only supplies 2D support. Occupied cells and the
 # full-volume sweep below remain authoritative over this assumption.
 known|=candidate_mask(x,y,recorded['preview_start_map_pose'],recorded.get('manual_floor',{}))
 known|=(abs(x)<.34)&(abs(y)<.32) # Current physical chassis footprint, snapshot only.
 # Historical measured chassis occupancy is additional 2D floor-candidate
 # evidence. It never removes 3D occupied points or skips whole-body sweeps.
 if not recorded.get('local_snapshot_only'):
  from map_trajectory_cache import load as historical_trajectory
  for pose in historical_trajectory(folder,recorded['map_voxel_sha256']):
   local=(np.r_[pose[:2],0.]-start[:3,3])@start[:3,:3];yaw=pose[2]-recorded['preview_start_map_pose'][2]
   dx=x-local[0];dy=y-local[1];c,s=np.cos(yaw),np.sin(yaw)
   known|=(np.abs(c*dx+s*dy)<.34)&(np.abs(-s*dx+c*dy)<.32)
 grid[~known]=-1
 low=valid&(p[:,2]>.07)&(p[:,2]<.35);grid[ids[low,1],ids[low,0]]=100
 return grid,tuple(lower),res

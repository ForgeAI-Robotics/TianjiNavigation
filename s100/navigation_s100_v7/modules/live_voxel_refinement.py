"""Measured bounds for LIVE-only, independently supported tabletop cells.

Saved map cells keep their complete volume. Missing/ambiguous evidence keeps
the complete live cell too. Acquisition stamps are never refreshed.
"""
import numpy as np
from pillar_cables import distances

def refine(points_body,protected,map_points,scans,T_map_base,shapes,resolution,uncertainty=.02):
 p=np.asarray(points_body,float);protected=np.asarray(protected,bool);T=np.asarray(T_map_base,float)
 if not .02<=uncertainty<=.03:raise ValueError('LIVE_CELL_UNCERTAINTY_TOO_SMALL')
 if not .02<=resolution<=.05:return [],dict(status='RESOLUTION_KEEP_FULL_CELLS',refined=0)
 R=T[:3,:3];t=T[:3,3];centers=p@R.T+t;keys=np.floor(centers/resolution+1e-7).astype(np.int64)
 map_keys={tuple(k)for k in np.floor(np.asarray(map_points)/resolution).astype(np.int64)}
 # Prepare the same evidence for the nearby swept region, rather than
 # clearing only the initial pose and hitting the next coarse cell at 5 cm.
 near=distances(p,shapes,limit=.30)<=.30
 candidates=np.flatnonzero(protected&near)
 candidates=np.array([i for i in candidates if tuple(keys[i])not in map_keys],dtype=np.int64)
 output=[];evidence=[];report=dict(status='RAW_TABLETOP_CELL_REFINEMENT',candidate_cells=len(candidates),refined=0,uncertainty_m=uncertainty,map_cells_refined=0,points_deleted=0,free_space_credit=False)
 if not len(candidates):return output,report
 if len(candidates)>64:
  distance=distances(p[candidates],shapes,limit=.30)
  candidates=candidates[np.argsort(distance,kind='stable')[:64]];report['budget_unprocessed_cells']=report['candidate_cells']-64
 # Raw arrays are already aligned to this capture's reference base/map.
 accepted={}
 for sensor in ('front','back'):
  rows=[s for s in scans if s['header']['sensor']==sensor]
  if len(rows)!=3:continue
  h=[s['header']for s in rows]
  try:
   identities={(x['boot_id'],x['epoch'],x['calibration_sha256'])for x in h}
   stamps=[float(x['observation_time'])for x in h]
   poses=np.array([x['pose']for x in h]);yaw=np.unwrap(poses[:,2])
   valid=len(identities)==1 and len(set(stamps))==3 and max(stamps)-min(stamps)<=2. and np.linalg.norm(np.ptp(poses[:,:2],axis=0))<=.02 and np.ptp(yaw)<=np.deg2rad(1.) and all(0<=float(x['source_age_s'])<=.6 for x in h)
  except (KeyError,ValueError,TypeError):valid=False
  if valid:accepted[sensor]=rows
 if not accepted:return [],dict(report,status='THREE_FRESH_STATIC_SCANS_REQUIRED')
 for index in candidates:
  key=keys[index]
  if tuple(key)in map_keys:continue
  patches=[];cell_samples=[];frame_times=[];good_sensor=False
  for sensor,rows in accepted.items():
   local_cells=[];local_patches=[];times=[];direct_frames=0
   for row in rows:
    q=np.asarray(row['points_map'],float)
    if q.ndim!=2 or q.shape[1]!=3 or not np.isfinite(q).all():continue
    # A point may cross the artificial cell face between scans. Require
    # evidence in this cell's 2 cm sampling halo in every frame instead.
    # All halo samples are included in the bounds, so this can only widen
    # the resulting measured volume relative to using in-cell points alone.
    cell_center=(key+.5)*resolution
    cells=(np.floor(q/resolution).astype(np.int64)==key).all(1)
    direct=bool(cells.any())
    neighborhood=(np.linalg.norm(q[:,:2]-centers[index,:2],axis=1)<=.16)&(np.abs(q[:,2]-centers[index,2])<=.04)
    patch=q[neighborhood]
    if len(patch)<24:break
    if direct:direct_frames+=1
    else:
     cells=(np.abs(q-cell_center)<=resolution/2+uncertainty+1e-10).all(1)&(np.abs(q[:,2]-np.median(patch[:,2]))<=.018)
    if not cells.any():break
    local_patches.append(patch);local_cells.append(q[cells]);times.append(float(row['header']['observation_time']))
   if len(local_cells)==3 and direct_frames>=2:
    good_sensor=True;patches+=local_patches;cell_samples+=local_cells;frame_times+=times
  if not good_sensor:continue
  support=np.concatenate(patches);values,axes=np.linalg.eigh(np.cov(support.T));spread=np.sqrt(np.maximum(values,0));normal=axes[:,0]
  # Broad, almost horizontal measured surface; no cable-only or single-ray fit.
  if spread[0]>.012 or spread[1]<.025 or spread[2]<.04 or abs(normal[2])<.95:continue
  # Include EVERY observed return in this cell, also the opposite radar and
  # any scan that did not itself provide a three-frame plane fit.
  all_samples=[]
  for row in scans:
   q=np.asarray(row['points_map'],float)
   if q.ndim!=2 or q.shape[1]!=3 or not np.isfinite(q).all():return [],dict(report,status='INVALID_RAW_KEEP_FULL_CELLS')
   cells=(np.floor(q/resolution).astype(np.int64)==key).all(1)
   if cells.any():all_samples.append(q[cells])
  raw=np.concatenate(all_samples+cell_samples);center=(key+.5)*resolution
  lo=np.maximum(raw.min(0)-uncertainty,center-resolution/2)-center
  hi=np.minimum(raw.max(0)+uncertainty,center+resolution/2)-center
  if (lo>hi).any():continue
  output.append(dict(point_index=int(index),bounds=[lo.tolist(),hi.tolist()],uncertainty_m=uncertainty,supporting_frames=len(frame_times),raw_points=len(raw),map_contribution=False,source='THREE_SCAN_RAW_TABLETOP'))
  evidence.append(dict(point_index=int(index),cell_map=center.tolist(),raw_points_map=raw.tolist(),observation_times=frame_times,plane_rmse_m=float(spread[0]),plane_support_points=len(support)))
 report.update(refined=len(output),evidence=evidence)
 return output,report

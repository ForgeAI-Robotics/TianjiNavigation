"""Bounded, read-only rendering data from the planner's actual selected path."""
import json,math
from pathlib import Path

def from_plan(result,scene,generated_at):
 start=scene['preview_start_map_pose'];goal=scene['target']['pose'];selected=result.get('selected')
 preview=dict(state=result.get('state'),generated_at=generated_at,map_sha256=scene['target']['map_voxel_sha256'],frame_id=scene['target'].get('frame_id','session_base_start'),start=start,goal=goal,target_name=scene['target'].get('name',''),path=[],display_only=True)
 if not selected:return preview
 path=selected['path']
 if not isinstance(path,list)or len(path)>100000:raise ValueError('PREVIEW_PATH_SIZE_INVALID')
 stride=max(1,math.ceil(len(path)/4000));sampled=path[::stride]
 if path and (not sampled or sampled[-1]!=path[-1]):sampled.append(path[-1])
 c,s=math.cos(start[2]),math.sin(start[2])
 for row in sampled:
  if len(row)<3 or not all(type(x)in(int,float)and math.isfinite(x) for x in row[:3]):raise ValueError('PREVIEW_PATH_INVALID')
  x,y,yaw=row[:3];preview['path'].append([start[0]+c*x-s*y,start[1]+s*x+c*y,start[2]+yaw])
 preview.update(profile=selected.get('profile',''),length_m=selected.get('length_m'),sample_stride=stride)
 return preview

def read_job_preview(root,job):
 raw=job.get('navigator_status')or{};folder=(job.get('result')or{}).get('artifacts',{}).get('folder')
 if not folder and raw.get('motor_trace_file'):folder=str(Path(raw['motor_trace_file']).parent)
 if not folder:return None
 folder=Path(folder).resolve();base=(Path(root)/'results').resolve()
 if folder.parent!=base:raise ValueError('PREVIEW_ARTIFACT_OUTSIDE_TASK_ROOT')
 plan=folder/'planning/plan.json';scene=folder/'scene/frozen_input.json'
 if not plan.exists()or not scene.exists():return None
 if plan.stat().st_size>16*1024*1024 or scene.stat().st_size>4*1024*1024:raise ValueError('PREVIEW_FILE_TOO_LARGE')
 result=json.loads(plan.read_text());frozen=json.loads(scene.read_text());target=job['target']
 if frozen['target']['map_voxel_sha256']!=target['map_voxel_sha256']or result['target']['map_voxel_sha256']!=target['map_voxel_sha256']:raise ValueError('PREVIEW_MAP_MISMATCH')
 if any(abs(a-b)>1e-7 for a,b in zip(frozen['target']['pose'],target['pose'])):raise ValueError('PREVIEW_GOAL_MISMATCH')
 preview=from_plan(result,frozen,plan.stat().st_mtime);preview.update(command_id=job['brain_command']['command_id'],task_id=job['brain_command']['task_id'])
 return preview

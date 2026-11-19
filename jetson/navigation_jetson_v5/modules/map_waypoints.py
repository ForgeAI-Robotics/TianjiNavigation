"""Read navigation marks only when their map hash AND frame match the viewer."""
import json,math
from pathlib import Path
FILE=Path(__file__).resolve().parents[2]/'navigation_waypoints_v6/config/targets.json'
def load(meta,path=None):
 if path is None:
  return sorted([row for candidate in [FILE,FILE.parent/'scenes/office_targets.json'] for row in load(meta,candidate)],key=lambda row:row['id'])
 try:data=json.loads(path.read_text())
 except (OSError,ValueError):return []
 result=[]
 for row in data.get('targets',{}).values():
  pose=row.get('pose',[])
  if not meta.get('voxel_file_sha256') or row.get('map_voxel_sha256')!=meta['voxel_file_sha256'] or row.get('frame_id')!=meta.get('frame_id'):continue
  if len(pose)!=3 or not all(isinstance(x,(float,int)) and math.isfinite(x) for x in pose):continue
  result.append(row)
 return sorted(result,key=lambda r:r['id'])

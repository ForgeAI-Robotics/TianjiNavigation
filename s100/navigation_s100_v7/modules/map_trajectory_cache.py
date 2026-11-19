"""Cache immutable historical wheel poses, never live localization or free space.

The complete map hash, keyframe names/sizes/mtimes, frame count and cache data
hash must match. A missing/corrupt/stale cache rebuilds via the original reader.
"""
import hashlib,json,os,tempfile
from pathlib import Path
import numpy as np
from paths import ROOT
from saved_map import MapArchive

def fingerprint(folder,meta):
 files=sorted((folder/'keyframes').glob('*.npz'))
 if len(files)!=meta['frames']:raise ValueError('KEYFRAME_COUNT_MISMATCH')
 rows=[(p.name,p.stat().st_size,p.stat().st_mtime_ns)for p in files]
 value=dict(schema_version=1,map_path=str(folder.resolve()),map_sha256=meta['voxel_file_sha256'],frame_id=meta['frame_id'],frames=meta['frames'],cells=meta['cells'],keyframes=rows)
 return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()

def load(folder,expected_map_sha256,cache_folder=None):
 folder=Path(folder);meta=json.loads((folder/'map.json').read_text())
 actual=hashlib.sha256((folder/'voxels.npz').read_bytes()).hexdigest()
 if actual!=expected_map_sha256 or actual!=meta['voxel_file_sha256']:raise ValueError('TRAJECTORY_MAP_CHANGED')
 token=fingerprint(folder,meta);cache_folder=ROOT/'runtime/map_trajectory_cache'if cache_folder is None else Path(cache_folder);file=cache_folder/(token+'.json')
 try:
  cached=json.loads(file.read_text());trajectory=np.asarray(cached['poses'],dtype='<f8').reshape(-1,3)
  if cached['fingerprint']!=token or trajectory.shape!=(meta['frames'],3) or not np.isfinite(trajectory).all() or hashlib.sha256(trajectory.tobytes()).hexdigest()!=cached['poses_sha256']:raise ValueError('INVALID_TRAJECTORY_CACHE')
  if fingerprint(folder,meta)!=token:raise ValueError('KEYFRAMES_CHANGED_DURING_CACHE_READ')
  return trajectory
 except (OSError,ValueError,KeyError,TypeError):pass
 trajectory=np.asarray(MapArchive(folder).trajectory,dtype='<f8').reshape(-1,3)
 if trajectory.shape!=(meta['frames'],3) or not np.isfinite(trajectory).all():raise ValueError('INVALID_HISTORICAL_TRAJECTORY')
 if fingerprint(folder,meta)!=token:raise ValueError('KEYFRAMES_CHANGED_DURING_TRAJECTORY_READ')
 payload=dict(fingerprint=token,poses_sha256=hashlib.sha256(trajectory.tobytes()).hexdigest(),poses=trajectory.tolist())
 try:
  cache_folder.mkdir(parents=True,exist_ok=True);fd,name=tempfile.mkstemp(prefix='trajectory-',suffix='.tmp',dir=cache_folder)
  try:
   with os.fdopen(fd,'w')as handle:json.dump(payload,handle,separators=(',',':'),allow_nan=False)
   os.replace(name,file)
  finally:Path(name).unlink(missing_ok=True)
 except OSError:pass # Read-only/full cache storage cannot alter the trajectory.
 return trajectory

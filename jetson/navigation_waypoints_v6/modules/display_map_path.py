"""Resolve an active S100 map to its verified Jetson display copy only."""
from pathlib import Path
import hashlib,json
from paths import NAV
REMOTE_MAPS=Path('/home/sunrise/tianji_table_approach_dev/navigation_s100_v7/maps')
def local_map(row,local_root=None):
 root=Path(local_root)if local_root is not None else NAV/'maps';source=Path(row['map_folder'])
 if source.parent==REMOTE_MAPS:folder=root/source.name
 elif source.parent.resolve()==root.resolve():folder=source
 else:raise ValueError('地图路径不是当前场景的已知地图目录：'+str(source))
 if not(folder/'map.json').is_file()or not(folder/'voxels.npz').is_file():raise FileNotFoundError('Jetson本地地图副本缺失：'+str(folder))
 meta=json.loads((folder/'map.json').read_text());expected=row['map_voxel_sha256']
 if meta['voxel_file_sha256']!=expected or hashlib.sha256((folder/'voxels.npz').read_bytes()).hexdigest()!=expected:raise ValueError('Jetson地图版本与S100目标不一致：'+str(folder))
 return folder

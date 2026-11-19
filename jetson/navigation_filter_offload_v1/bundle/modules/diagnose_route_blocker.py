"""Read-only attribution of a frozen near-start collision to raw scans/map."""
import json,sys,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from paths import ROOT
from saved_map import base_transform
from extrinsics import transforms
from common import atomic_json

def run(folder):
 record=json.loads((folder/'scene/frozen_input.json').read_text())
 loc=record['localization'];W=base_transform(loc['pose']);reference=base_transform(loc['wheel_pose'])
 target=np.array([.16265119037386697,-.08511511011726779,1.275])
 with np.load(Path(record['map_file']).parent/'voxels.npz') as f:historical=(f['points']-W[:3,3])@W[:3,:3]
 sources={'historical_map':historical};acq=json.loads((folder/'localization/acquisition.json').read_text());latest={}
 for row in acq['rows']:latest[row['header']['sensor']]=row
 for sensor,row in latest.items():
  with np.load(row['file']) as f:p=f['points_sensor']
  T=np.linalg.inv(reference)@base_transform(row['header']['pose'])@transforms()[sensor]
  sources['raw_'+sensor]=p@T[:3,:3].T+T[:3,3]
 report=dict(folder=str(folder),target_body_m=target.tolist(),sources={},motion_commands_sent=0,point_removal_performed=False)
 near={}
 for name,p in sources.items():
  p=p[np.isfinite(p).all(1)];d=np.linalg.norm(p-target,axis=1);i=int(np.argmin(d));near[name]=p[d<.3]
  report['sources'][name]=dict(nearest_distance_m=float(d[i]),nearest_body_m=p[i].tolist(),within_5cm=int((d<.05).sum()),within_10cm=int((d<.1).sum()))
 atomic_json(folder/'route_blocker_attribution.json',report);np.savez_compressed(folder/'route_blocker_context.npz',**near)
 print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':run(Path(sys.argv[1]))

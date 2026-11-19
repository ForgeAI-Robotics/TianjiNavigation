"""Judge the whole measured 5cm raw capture, including intermittent returns."""
import json
from pathlib import Path
import numpy as np
from snapshot_builder import local_model
from chassis_proxy import measured_shapes
from extrinsics import transforms
from saved_map import base_transform
from cable_precheck import deferred_mask
from motion_consistency import DualFrameObserver
from common import atomic_json

def replay(folder,reference_joints,model_sha256,source_identity,measured_displacement):
 if measured_displacement<.05:raise ValueError('MEASURED_5CM_REQUIRED_BEFORE_CABLE_JUDGEMENT')
 folder=Path(folder);rows=json.loads((folder/'acquisition.json').read_text())['rows'];model=local_model()
 if model.sha256!=model_sha256:raise ValueError('PROBE_REPLAY_MODEL_CHANGED')
 shapes=measured_shapes(model,reference_joints);matrices=transforms();scans=[];candidates=[]
 for row in rows:
  h=row['header']
  if (h['boot_id'],h['epoch'],h['calibration_sha256'])!=tuple(source_identity):raise ValueError('PROBE_REPLAY_IDENTITY_CHANGED')
  with np.load(row['file'],allow_pickle=False) as f:p=f['points_sensor']
  T=matrices[h['sensor']];p=p@T[:3,:3].T+T[:3,3];p=p[np.isfinite(p).all(1)]
  # Retain all nearby original returns for the two competing hypotheses.
  p=p[(np.linalg.norm(p[:,:2],axis=1)<2.5)&(p[:,2]>0)&(p[:,2]<2.)]
  mask,_=deferred_mask(p,shapes,model_sha256);candidates.append(p[mask]);scans.append((h,p))
 points=np.concatenate(candidates) if candidates else np.empty((0,3))
 if len(points):
  _,ids=np.unique(np.floor(points/.02).astype(np.int64),axis=0,return_index=True);points=points[np.sort(ids)]
  if len(points)>256:raise ValueError('PROBE_REPLAY_FEATURE_BUDGET_EXCEEDED')
 # Global displacement is verified above. A feature first seen later may
 # still be distinguished after >=3cm relative motion (3 x 1cm tolerance).
 observer=DualFrameObserver(len(points),tolerance_m=.01,minimum_frames=8,minimum_translation_m=.03,minimum_second_axis_m=0.)
 for h,p in scans:observer.observe(p,points,base_transform(h['pose']),h['observation_time'],h['sensor'])
 report=observer.summary();report.update(points_body_m=points.tolist(),reference_joints=reference_joints,source_identity=list(source_identity),model_sha256=model_sha256,raw_unfiltered_input=True,raw_capture_folder=str(folder),posthoc_raw_replay=True,bounded_inspection_displacement_m=measured_displacement,automatic_point_deletion=False,motion_commands_sent=0)
 atomic_json(folder/'post_5cm_raw_judgement.json',report);return report

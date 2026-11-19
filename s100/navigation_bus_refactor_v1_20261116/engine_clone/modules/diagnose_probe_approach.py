"""Explain a stopped cable probe from frozen raw captures. No live interface."""
import json,numpy as np
from pathlib import Path
from common import ROOT,atomic_json
from snapshot_builder import local_model
from chassis_proxy import measured_shapes
from extrinsics import transforms
from navigation_self_filter import evaluate
from pillar_cables import distances
from cable_precheck import deferred_mask

def main():
 folder=Path(json.loads((ROOT/'runtime/motion_observer_status.json').read_text())['folder']);rows=json.loads((folder/'acquisition.json').read_text())['rows'];scene=Path(json.loads((ROOT/'runtime/latest_plan.json').read_text())['folder']);r=json.loads((scene/'scene/frozen_input.json').read_text());m=local_model();shapes=measured_shapes(m,r['saved_pose']['joint_positions_rad']);moved=[]
 for s in shapes:
  T=np.array(s['matrix']);T[0,3]+=.0025;moved.append(dict(s,matrix=T.tolist()))
 cases=[]
 for row in rows:
  with np.load(row['file']) as f:p=f['points_sensor']
  T=transforms()[row['header']['sensor']];p=p@T[:3,:3].T+T[:3,3];p=p[np.isfinite(p).all(1)&(p[:,2]>.6)&(p[:,2]<1.5)&(np.linalg.norm(p[:,:2],axis=1)<1.6)]
  result=evaluate(p,shapes,m.sha256);p=p[~result['filtered']]
  if not len(p):continue
  before=distances(p,shapes);after=distances(p,moved);candidate,_=deferred_mask(p,shapes,m.sha256);ids=np.flatnonzero(candidate&(before>=.025)&(before<.052)&(after<before-1e-7))
  for i in ids:
   gap=[float(distances(p[[i]],[s])[0]) for s in shapes];link=shapes[int(np.argmin(gap))]['link'];cases.append(dict(sensor=row['header']['sensor'],scan_time=row['header']['observation_time'],point_body_m=p[i].tolist(),link=link,gap_start_cm=float(before[i]*100),gap_after_2_5mm_cm=float(after[i]*100),approach_mm=float((before[i]-after[i])*1000)))
 report=dict(raw_folder=str(folder),cases=cases,found=len(cases),original_point_cloud_modified=False,motion_commands_sent=0,note='Frozen single-sensor attribution, forward 2.5mm translation; raw scan timestamps preserved')
 atomic_json(ROOT/'results/probe_approach_diagnosis_20261030.json',report);print(json.dumps(dict(found=len(cases),examples=sorted(cases,key=lambda x:-x['approach_mm'])[:5]),ensure_ascii=False))

if __name__=='__main__':main()

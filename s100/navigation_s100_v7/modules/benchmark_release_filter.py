"""Frozen raw-scan latency/mask regression. Never opens a live control channel."""
import argparse,json,time
from pathlib import Path
import numpy as np
from common import ROOT,atomic_json
from snapshot_builder import local_model
from chassis_proxy import measured_shapes
from extrinsics import transforms
from saved_map import base_transform
from navigation_self_filter import evaluate

def main():
 p=argparse.ArgumentParser();p.add_argument('--compare',action='store_true');a=p.parse_args();out=ROOT/'results/release_filter_benchmark_20261030';out.mkdir(exist_ok=True)
 if not a.compare:
  folder=Path(json.loads((ROOT/'runtime/motion_observer_status.json').read_text())['folder']);rows=json.loads((folder/'acquisition.json').read_text())['rows'][-8:];plan=Path(json.loads((ROOT/'runtime/latest_plan.json').read_text())['folder']);r=json.loads((plan/'scene/frozen_input.json').read_text());atomic_json(out/'context.json',dict(joints=r['saved_pose']['joint_positions_rad'],source=str(folder)))
 context=json.loads((out/'context.json').read_text());m=local_model();shapes=measured_shapes(m,context['joints']);times=[];counts=[];changed=[]
 for i in range(8):
  file=out/f'frame_{i}.npz'
  if not a.compare:
   if i>=len(rows):break
   row=rows[i]
   with np.load(row['file']) as f:points=f['points_sensor']
   T=transforms()[row['header']['sensor']];points=points@T[:3,:3].T+T[:3,3];corners=np.concatenate([s['world_corners'] for s in shapes]);points=points[np.isfinite(points).all(1)&(points>=corners.min(0)-.6).all(1)&(points<=corners.max(0)+.6).all(1)]
  else:
   if not file.exists():break
   with np.load(file) as f:points=f['points'];before=f['filtered'];protected=f['protected']
  start=time.perf_counter();r=evaluate(points,shapes,m.sha256);times.append(time.perf_counter()-start);counts.append(len(points))
  if not a.compare:np.savez_compressed(file,points=points,filtered=r['filtered'],protected=r['protected'])
  else:changed.append(dict(frame=i,mask_changes=int(np.count_nonzero(before!=r['filtered'])),previous_protected_removed=int(np.count_nonzero(protected&r['filtered']))))
 report=dict(mode='AFTER' if a.compare else 'BEFORE',frames=len(times),points=counts,mean_ms=float(np.mean(times)*1000),maximum_ms=float(max(times)*1000),regression=changed,motion_commands_sent=0)
 atomic_json(out/('after.json' if a.compare else 'before.json'),report);print(json.dumps(report))

if __name__=='__main__':main()

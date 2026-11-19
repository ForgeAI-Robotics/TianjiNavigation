#!/usr/bin/env python3
"""Offline raw-front/back LiDAR ShapeMask replay; no ROS or hardware access."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse
import json
from pathlib import Path
import time
import resource
import numpy as np
from lidar_shapemask import LidarExtrinsics,LidarShapeMask
from local_model import LocalRobotModel,default_urdf_path

ROOT=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,default=Path('/home/nvidia/tianji_robot_state_bridge/reports/column_table_edge_20261019'))
    parser.add_argument('--frames',type=int,default=3)
    args=parser.parse_args()
    if not 1<=args.frames<=20:parser.error('frames must be 1..20')
    output=ROOT/'results/lidar_shapemask';output.mkdir(parents=True,exist_ok=True)
    model=LocalRobotModel(default_urdf_path(),ROOT/'model_manifest.json',ROOT/'collision_geometry.json')
    calibration=json.loads((ROOT/'lidar_calibration.json').read_text())
    extrinsics=LidarExtrinsics(calibration,np.eye(4),alignment_verified=False)
    start=time.perf_counter();mask=LidarShapeMask(model,extrinsics);init=time.perf_counter()-start
    print(json.dumps(dict(stage='local original mesh BVHs built',initialization_s=init,unique_meshes=len(mask.cache))),flush=True)
    raw=np.load(args.capture/'pointclouds.npz')
    positions=json.loads((args.capture/'receiver_status.json').read_text())['joint_positions_rad']
    metadata=json.loads((args.capture/'capture.json').read_text());rows=[]
    try:
        for sensor in ('front','back'):
            selected=np.unique(raw[sensor+'_frame_index'])[-args.frames:]
            for frame_index in selected:
                points=raw[sensor][raw[sensor+'_frame_index']==frame_index]
                start=time.perf_counter();cpu_start=time.process_time();result=mask.classify(points,sensor,'sensor',positions)
                cpu_ms=(time.process_time()-cpu_start)*1000
                total=(time.perf_counter()-start)*1000
                summary=result['summary'];summary.update(frame_index=int(frame_index),total_compute_ms=total,cpu_time_ms=cpu_ms,
                    observation_time=metadata['streams'][sensor][int(frame_index)]['stamp'])
                rows.append(summary)
                np.savez_compressed(output/(sensor+'_'+str(frame_index)+'.npz'),
                    raw_points=points,points_model=result['points_model'],labels=result['labels'],
                    expected_range_m=result['expected_range_m'],measured_range_m=result['measured_range_m'],
                    removal_mask=result['removal_mask'])
                print(json.dumps(dict(sensor=sensor,frame=int(frame_index),total_compute_ms=round(total,2),
                                      labels=summary['labels'],removed_points=summary['removed_points'])),flush=True)
    finally:mask.close()
    report=dict(mode='offline preview only; original vendor-to-model identity not independently verified',
        source_capture=str(args.capture),initialization_s=init,peak_process_rss_mb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
        frames=len(rows),median_single_sensor_ms=float(np.median([r['total_compute_ms'] for r in rows])),
        mean_pair_compute_ms=float(sum(r['total_compute_ms'] for r in rows)/args.frames),
        mean_pair_cpu_ms=float(sum(r['cpu_time_ms'] for r in rows)/args.frames),
        model_sha256=model.sha256,alignment_verified=False,removed_points=sum(r['removed_points'] for r in rows),
        limitations=['Head/gripper and chassis steering states are incomplete in this recorded capture.',
                     'Untrusted finger linkage is never self-removal evidence.',
                     'Unknown pose and possible occlusion are retained; no free-space inference.',
                     'These measurements are offline compute costs, not full-navigation deadlines.'],results=rows)
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))


if __name__=='__main__':main()

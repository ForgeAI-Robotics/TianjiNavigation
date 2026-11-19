#!/usr/bin/env python3
"""Offline before/after equivalence and bounded repeated-frame benchmark."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse
import importlib.util
import json
from pathlib import Path
import resource
import time
import numpy as np
from lidar_shapemask import LidarExtrinsics,LidarShapeMask
from local_model import LocalRobotModel,default_urdf_path

ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--pairs',type=int,default=20)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 1<=args.pairs<=100:parser.error('pairs must be 1..100')
    spec=importlib.util.spec_from_file_location('lidar_before',ROOT/'results/lidar_performance_baseline/lidar_shapemask.before.py')
    before=importlib.util.module_from_spec(spec);spec.loader.exec_module(before)
    before.LIBRARY=ROOT/'native/lidar_mesh.so'
    model=LocalRobotModel(default_urdf_path(),ROOT/'model_manifest.json',ROOT/'collision_geometry.json')
    extrinsics=LidarExtrinsics(json.loads((ROOT/'lidar_calibration.json').read_text()),np.eye(4))
    positions=json.loads((args.capture/'receiver_status.json').read_text())['joint_positions_rad']
    frames=[]
    with np.load(args.capture/'pointclouds.npz') as raw:
        for sensor in ('front','back'):
            for index in np.unique(raw[sensor+'_frame_index'])[-3:]:
                frames.append((sensor,raw[sensor][raw[sensor+'_frame_index']==index].copy()))
    report={'mode':'offline only; no ROS or control','pairs_per_version':args.pairs,'equivalent_frames':0,'versions':{}}
    expected=[]
    for name,cls in [('before',before.LidarShapeMask),('after',LidarShapeMask)]:
        mask=cls(model,extrinsics)
        try:
            for i,(sensor,points) in enumerate(frames):
                original=points.copy();result=mask.classify(points,sensor,'sensor',positions)
                np.testing.assert_array_equal(points,original)
                keys=('labels','removal_mask','expected_range_m','measured_range_m','points_model')
                if name=='before':expected.append({key:result[key].copy() for key in keys})
                else:
                    for key in keys:np.testing.assert_array_equal(result[key],expected[i][key],err_msg=key)
                    report['equivalent_frames']+=1
            wall=[];cpu=[];rss=[]
            for _ in range(args.pairs):
                started=time.perf_counter();cpu_start=time.process_time()
                for sensor in ('front','back'):
                    points=next(p for s,p in reversed(frames) if s==sensor)
                    mask.classify(points,sensor,'sensor',positions)
                wall.append((time.perf_counter()-started)*1000)
                cpu.append((time.process_time()-cpu_start)*1000)
                # Resident pages, unlike ru_maxrss, reveal sustained growth.
                rss.append(int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')/1048576)
            report['versions'][name]={'pair_wall_ms':dict(zip(('p50','p95','max'),map(float,np.percentile(wall,[50,95,100])))),
                'mean_pair_cpu_ms':float(np.mean(cpu)),'resident_mb_first_last':[rss[0],rss[-1]],
                'resident_growth_mb':rss[-1]-rss[0]}
        finally:mask.close()
    report['peak_rss_mb']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

if __name__=='__main__':main()

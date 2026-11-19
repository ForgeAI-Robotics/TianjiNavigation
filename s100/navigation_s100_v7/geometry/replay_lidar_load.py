#!/usr/bin/env python3
"""Rate-limited static recording replay and read-only Linux load observation.

No ROS imports, device access, service operations, or actuator commands.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse
import json
from pathlib import Path
import time
import numpy as np
from lidar_shapemask import LidarExtrinsics,LidarShapeMask
from local_model import LocalRobotModel,default_urdf_path

ROOT=Path(__file__).resolve().parent

def processes():
    result={}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            command=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            name=(entry/'comm').read_text().strip()
            if name not in ('navigation_node','driver_node','controller_core') and not command.startswith('/usr/bin/python3 /home/sunrise/tianji_state_bridge/s100_receiver.py'):continue
            fields=(entry/'stat').read_text().rsplit(')',1)[1].split()
            result[entry.name]=dict(name=name,start_ticks=int(fields[19]))
        except (OSError,ValueError,IndexError):continue
    return result

def observation():
    cpu={}
    for line in Path('/proc/stat').read_text().splitlines():
        parts=line.split()
        if not parts or not parts[0].startswith('cpu'):continue
        ticks=list(map(int,parts[1:9]));cpu[parts[0]]=[sum(ticks),ticks[3]+ticks[4]]
    memory={line.split(':')[0]:int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()}
    result=dict(monotonic=time.monotonic(),cpu_ticks=cpu,available_memory_mb=memory['MemAvailable']/1024,
        resident_mb=int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')/1048576)
    status=Path('/home/sunrise/tianji_state_bridge/runtime/receiver_status.json')
    if status.exists():
        try:
            data=json.loads(status.read_text())
            result['state_bridge_file_age_s']=time.time()-status.stat().st_mtime
            result['state_bridge']={k:v for k,v in data.items() if 'age' in k or k in ('joint_valid','joint_count','model_match','received_at')}
        except (OSError,ValueError):pass
    return result

def cpu_usage(first,last):
    result={}
    for name,(total,idle) in last['cpu_ticks'].items():
        old_total,old_idle=first['cpu_ticks'][name];elapsed=total-old_total
        result[name]=100*(1-(idle-old_idle)/elapsed) if elapsed else None
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=60.)
    parser.add_argument('--rate',type=float,default=2.)
    args=parser.parse_args()
    if not 5<=args.duration<=300 or not .1<=args.rate<=2:parser.error('duration 5..300 s; per-sensor rate .1..2 Hz')
    original_processes=processes()
    baseline=[]
    for _ in range(4):baseline.append(observation());time.sleep(1.)
    model=LocalRobotModel(default_urdf_path(),ROOT/'model_manifest.json',ROOT/'collision_geometry.json')
    extrinsics=LidarExtrinsics(json.loads((ROOT/'lidar_calibration.json').read_text()),np.eye(4))
    frames={}
    with np.load(args.capture/'pointclouds.npz') as raw:
        for sensor in ('front','back'):
            indices=raw[sensor+'_frame_index'];frames[sensor]=raw[sensor][indices==indices.max()].copy()
    positions=json.loads((args.capture/'receiver_status.json').read_text())['joint_positions_rad']
    mask=LidarShapeMask(model,extrinsics);expected={};wall=[];cpu=[];resources=[];missed=0;removed=0
    try:
        # Warm caches once; exclude initialization from steady-state work budget.
        for sensor,points in frames.items():
            r=mask.classify(points,sensor,'sensor',positions)
            expected[sensor]={k:r[k].copy() for k in ('labels','expected_range_m','removal_mask')}
        start=time.monotonic();deadline=start+args.duration;next_start=start;process_start=time.process_time()
        resources.append(observation())
        while time.monotonic()<deadline:
            if next_start>time.monotonic():time.sleep(min(next_start-time.monotonic(),max(0,deadline-time.monotonic())))
            if time.monotonic()>=deadline:break
            begin=time.perf_counter();cpu_begin=time.process_time()
            results={sensor:mask.classify(points,sensor,'sensor',positions) for sensor,points in frames.items()}
            cpu.append((time.process_time()-cpu_begin)*1000);wall.append((time.perf_counter()-begin)*1000)
            for sensor,r in results.items():
                for key,values in expected[sensor].items():np.testing.assert_array_equal(r[key],values)
                removed+=int(r['removal_mask'].sum())
            resources.append(observation());next_start+=1/args.rate
            if next_start<time.monotonic():
                missed+=1;next_start=time.monotonic() # latest-only semantics, no catch-up queue
        end=time.monotonic();overall_cpu=time.process_time()-process_start
    finally:mask.close()
    final_processes=processes()
    report=dict(mode='static recorded front/back replay; no ROS, hardware access or motion commands',
        duration_s=end-start,pairs=len(wall),per_sensor_rate_hz=args.rate,
        mean_pair_cpu_ms=float(np.mean(cpu)),pair_wall_ms=dict(zip(('p50','p95','max'),map(float,np.percentile(wall,[50,95,100])))),
        missed_periods=missed,outputs_constant=True,removed_points=removed,
        overall_worker_cpu_cores=overall_cpu/(end-start),
        baseline_cpu_percent=cpu_usage(baseline[0],baseline[-1]),replay_cpu_percent=cpu_usage(resources[0],resources[-1]),
        resident_first_last_mb=[resources[0]['resident_mb'],resources[-1]['resident_mb']],
        minimum_available_memory_mb=min(s['available_memory_mb'] for s in resources),
        maximum_bridge_file_age_s=max((s.get('state_bridge_file_age_s',0.) for s in resources),default=None),
        production_processes_before=original_processes,production_processes_after=final_processes,
        production_processes_unchanged=bool(original_processes) and original_processes==final_processes,
        samples=resources,baseline=baseline,
        limitations=['One static recorded pair repeated; no moving-obstacle or live DDS load validation.',
                     'Original model alignment remains unverified; no free-space certificate.',
                     'Other production workloads can change; aggregate CPU differences are not causal measurements.'])
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','baseline')},indent=2))

if __name__=='__main__':main()

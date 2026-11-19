#!/usr/bin/env python3
"""Bounded static live-input/domain80 ShapeMask CPU experiment. No motion path."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from replay_lidar_load import observation,cpu_usage,processes

ROOT=Path(__file__).resolve().parent


def proc_sample(pid):
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        rss=int(Path(f'/proc/{pid}/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')/1048576
        return dict(ticks=int(fields[11])+int(fields[12]),rss_mb=rss)
    except (OSError,ValueError,IndexError):return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=75.)
    parser.add_argument('--rate',type=float,default=1.)
    parser.add_argument('--snapshots',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    if not 20<=args.duration<=180:raise ValueError('duration 20..180s')
    if not .5<=args.rate<=2.:raise ValueError('rate .5..2 Hz per sensor')
    config=json.loads((ROOT/'lidar_mask_config.json').read_text())
    config.update(max_sensor_rate_hz=args.rate,calibration_file=str(ROOT/'lidar_calibration.json'))
    (args.output/'live_config.json').write_text(json.dumps(config,indent=2)+'\n')
    ipc=Path('/dev/shm')/('tianji_live_mask_'+str(os.getpid()));ipc.mkdir(mode=0o700)
    old_production=processes();baseline=[];workers={};logs={};samples=[]
    tick_hz=os.sysconf('SC_CLK_TCK')
    def sample():
        row=observation();row['workers']={name:proc_sample(p.pid) for name,p in workers.items()}
        row['production_cpu']={pid:proc_sample(pid) for pid in old_production}
        return row
    for _ in range(8):baseline.append(sample());time.sleep(1.)
    def launch(name,role,domain,middleware):
        env=os.environ.copy()
        for key in ['FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI','ROS_DISCOVERY_SERVER']:
            env.pop(key,None)
        env.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1' if domain==80 else '0',
                   RMW_IMPLEMENTATION=middleware,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        if domain==0 and middleware=='rmw_cyclonedds_cpp':env['CYCLONEDDS_URI']='file:///etc/ros/cyclonedds/cyclonedds.xml'
        if domain==0 and middleware=='rmw_fastrtps_cpp':env['FASTRTPS_DEFAULT_PROFILES_FILE']='/home/sunrise/tianji_state_bridge/fastdds_unicast.xml'
        # Low-volume 30Hz pose ingress keeps normal priority; expensive geometry
        # and diagnostic observers remain low priority.
        priority=0 if name=='joints' else 15
        program='exec nice -n '+str(priority)+' /usr/bin/python3 '
        if name=='filter':program+=str(ROOT/'lidar_mask_ros.py')+' --live-directory '+str(ipc)+' --config '+str(args.output.resolve()/'live_config.json')
        else:program+=str(ROOT/'live_lidar_ingress.py')+' '+role+' --directory '+str(ipc)+' --duration '+str(args.duration+30)
        if name=='filter' and args.snapshots:program+=' --snapshot-directory '+str(args.output.resolve()/'snapshots')
        if name=='isolated':program+=' --observe-only'
        log=open(args.output/(name+'.log'),'w');logs[name]=log
        workers[name]=subprocess.Popen(['bash','-c','source /opt/ros/humble/setup.bash; '+program],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    failure=None
    try:
        launch('joints','joints',0,'rmw_fastrtps_cpp')
        launch('clouds','clouds',0,'rmw_cyclonedds_cpp')
        launch('isolated','isolated',80,'rmw_fastrtps_cpp')
        launch('filter','',80,'rmw_fastrtps_cpp')
        start=time.monotonic();deadline=start+args.duration
        while time.monotonic()<deadline:
            samples.append(sample())
            dead={n:p.returncode for n,p in workers.items() if p.poll() is not None}
            if dead:raise RuntimeError('Experiment worker stopped: '+str(dead))
            time.sleep(min(1.,max(0,deadline-time.monotonic())))
        samples.append(sample())
    except Exception as exc:failure=str(exc)
    finally:
        # Stop compute first, keep observer alive long enough to drain final outputs.
        for name in ['filter','clouds','joints','isolated']:
            if name not in workers:continue
            p=workers[name]
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=6.)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3.)
            logs[name].close()
        import shutil
        for f in ipc.iterdir():
            if f.is_file():shutil.copy2(f,args.output/f.name)
        shutil.rmtree(ipc)
    hz=os.sysconf('SC_CLK_TCK');usage={}
    for name in workers:
        rows=[(s['monotonic'],s['workers'].get(name)) for s in samples if s['workers'].get(name)]
        if len(rows)>=2:
            t,a=rows[0];u,b=rows[-1]
            usage[name]=dict(mean_cpu_one_core_percent=100*(b['ticks']-a['ticks'])/hz/(u-t),
                             rss_first_last_mb=[a['rss_mb'],b['rss_mb']],pid=workers[name].pid)
    report=dict(mode='live production subscriptions; computed outputs confined to domain80 localhost',
        duration_s=samples[-1]['monotonic']-samples[0]['monotonic'] if len(samples)>1 else 0,
        baseline_cpu_percent=cpu_usage(baseline[0],baseline[-1]),
        live_cpu_percent=cpu_usage(samples[0],samples[-1]) if len(samples)>1 else {},
        workers=usage,production_processes_before=old_production,production_processes_after=processes(),
        production_processes_unchanged=old_production==processes(),
        worker_exit_codes={n:p.returncode for n,p in workers.items()},failure=failure,
        samples=samples,baseline=baseline,motion_commands_sent=0,production_navigation_output_connected=False)
    report['per_sensor_rate_hz']=args.rate
    (args.output/'load_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['samples','baseline']},indent=2),flush=True)
    if failure:raise RuntimeError(failure)


if __name__=='__main__':main()

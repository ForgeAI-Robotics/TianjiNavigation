#!/usr/bin/env python3
"""On-demand development pipeline. Stops when the GUI lease expires."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import json
from pathlib import Path
import signal
import subprocess
import time
from replay_lidar_load import observation,cpu_usage
from measure_live_shapemask import proc_sample

ROOT=Path(__file__).resolve().parent
IPC=Path('/dev/shm/tianji_s100_live_view')


def main():
    IPC.mkdir(mode=0o700,exist_ok=True)
    for name in ['front.view','back.view','status.json','load.json','assessment.json','front.cdr','back.cdr','pose_source.json']:
        (IPC/name).unlink(missing_ok=True)
    config=json.loads((ROOT/'lidar_mask_config.json').read_text())
    config.update(max_sensor_rate_hz=1.,calibration_file=str(ROOT/'lidar_calibration.json'))
    (IPC/'config.json').write_text(json.dumps(config))
    children={};logs={};stop=[False]
    def ending(*_):stop[0]=True
    signal.signal(signal.SIGTERM,ending);signal.signal(signal.SIGINT,ending)
    def launch(name,domain,middleware):
        env=os.environ.copy()
        for key in ['FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI','ROS_DISCOVERY_SERVER']:env.pop(key,None)
        env.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1' if domain==80 else '0',RMW_IMPLEMENTATION=middleware)
        if domain==0 and middleware=='rmw_cyclonedds_cpp':env['CYCLONEDDS_URI']='file:///etc/ros/cyclonedds/cyclonedds.xml'
        if domain==0 and middleware=='rmw_fastrtps_cpp':env['FASTRTPS_DEFAULT_PROFILES_FILE']='/home/sunrise/tianji_state_bridge/fastdds_unicast.xml'
        if name=='filter':command=f'{ROOT}/lidar_mask_ros.py --live-directory {IPC} --display-directory {IPC} --config {IPC}/config.json'
        else:command=f'{ROOT}/live_lidar_ingress.py {name} --directory {IPC} --duration 31536000'
        logs[name]=open(IPC/(name+'.log'),'w')
        children[name]=subprocess.Popen(['bash','-c','source /opt/ros/humble/setup.bash; exec nice -n '+('0' if name=='joints' else '15')+' /usr/bin/python3 '+command],env=env,stdout=logs[name],stderr=subprocess.STDOUT,start_new_session=True)
    try:
        launch('joints',0,'rmw_fastrtps_cpp');launch('clouds',0,'rmw_cyclonedds_cpp');launch('filter',80,'rmw_fastrtps_cpp')
        start=time.monotonic();last_lease=start;seen=None;old=observation();old_proc={n:proc_sample(p.pid) for n,p in children.items()}
        while not stop[0]:
            time.sleep(.5);mono=time.monotonic()
            try:
                version=(IPC/'viewer.lease').stat().st_mtime_ns
                if version!=seen:seen=version;last_lease=mono
            except OSError:pass
            if (seen is None and mono-start>30.) or (seen is not None and mono-last_lease>8.):break
            if any(p.poll() is not None for p in children.values()):raise RuntimeError('A development worker stopped; see private tmpfs logs')
            if mono-old['monotonic']<2.:continue
            row=observation();cpu=cpu_usage(old,row);dt=row['monotonic']-old['monotonic'];metrics={}
            for n,p in children.items():
                state=proc_sample(p.pid);before=old_proc.get(n)
                if state and before:metrics[n]=dict(cpu_one_core_percent=100*(state['ticks']-before['ticks'])/os.sysconf('SC_CLK_TCK')/dt,rss_mb=state['rss_mb'])
                old_proc[n]=state
            data=dict(cpu_all_percent=cpu['cpu'],cpu_app_percent=sum(cpu['cpu'+str(i)] for i in range(4))/4,
                      workers=metrics,updated_at=time.time(),motion_commands_sent=0,production_navigation_output_connected=False)
            from live_lidar_ingress import atomic
            atomic(IPC/'load.json',json.dumps(data).encode());old=row
    finally:
        for n in ['filter','clouds','joints']:
            p=children.get(n)
            if p and p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=3.)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3.)
        for f in logs.values():f.close()


if __name__=='__main__':main()

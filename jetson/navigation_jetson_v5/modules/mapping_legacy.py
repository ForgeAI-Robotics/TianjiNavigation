"""Jetson headless mapping. S100 streams observations; this sends no motion."""
import argparse,hashlib,json,os,select,signal,sys,threading,time
from datetime import datetime
import numpy as np
from common import ROOT,CONFIG,atomic_json,exclusive
from client import SensorStream
from extrinsics import transforms
from mapping_core import VoxelRecorder

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=0.,help='只读定时采集并保存，0为交互模式');parser.add_argument('--validation-only',action='store_true',help='验证地图单独标记，不覆盖正式地图指针');args=parser.parse_args()
    if args.duration<0:raise ValueError('Invalid duration')
    lock=exclusive(ROOT/'runtime/mapping.lock');folder=ROOT/'maps'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    recorder=VoxelRecorder(folder,CONFIG['mapping']);matrices=transforms();expected_hash=hashlib.sha256((ROOT/'config/lidar_calibration.json').read_bytes()).hexdigest();mutex=threading.RLock();stop=threading.Event();health=dict(active=True,paused=False,input_ready=False,pid=os.getpid(),sensors={},invalid=None,folder=str(folder),transport='pinned_ssh_wired_current_observations',motion_commands_sent=0);stream=None
    recorder.validation_only=args.validation_only
    pointer='latest_validation_map.json' if args.validation_only else 'latest_map.json'
    def write_health():
        with mutex:
            health['heartbeat_monotonic']=time.monotonic();health['invalid']=recorder.invalid_reason or health.get('worker_error');atomic_json(ROOT/'runtime/mapping_status.json',health)
    def heartbeat():
        while not stop.wait(.2):write_health()
    def process():
        try:
            while not stop.is_set():
                try:header,points=stream.receive()
                except ValueError as exc:
                    with mutex:health['input_warning']=str(exc)
                    continue
                if header.get('calibration_sha256')!=expected_hash:raise ValueError('CALIBRATION_MISMATCH')
                if header.get('kind')=='sensor_status':
                    with mutex:
                        health['source_status']=header
                        age=header.get('odom_age_s')
                        health['input_ready']=age is not None and 0<=age<=.5 and header.get('error') is None
                        if recorder.epoch is not None and header['epoch']!=recorder.epoch:recorder.invalid_reason='ODOMETRY_EPOCH_CHANGED_START_A_NEW_SESSION'
                    continue
                if header.get('kind')!='frame' or points is None or header.get('raw_point_frame')!='sensor':raise ValueError('INVALID_FRAME')
                if not 0<=header['source_age_s']<=.7:continue
                sensor=header['sensor'];matrix=matrices[sensor];stamp=float(header['observation_time'])
                if abs(stamp-header['odom_observation_time'])>.05:continue
                started=time.monotonic();base=points@matrix[:3,:3].T+matrix[:3,3]
                with mutex:
                    if health['paused']:continue
                    before=recorder.frames;recorder.ingest(base,stamp,header['pose'],sensor,matrix[:3,3],header.get('joints',{}),header['epoch'])
                    event=header.get('continuity_event')
                    if event and event!=health.get('last_continuity_event'):
                        health['last_continuity_event']=event
                        with (folder/'continuity_events.jsonl').open('a') as log:log.write(json.dumps(event)+'\n')
                    if recorder.frames>before:
                        previous=health['sensors'].get(sensor,{})
                        health['sensors'][sensor]=dict(completed_monotonic=time.monotonic(),interval_s=stamp-previous.get('observation_time',stamp),observation_time=stamp,processing_s=time.monotonic()-started,source_processing_s=header['source_processing_s'])
                        health.pop('worker_error',None);health.pop('input_warning',None)
                write_health()
        except Exception as exc:
            if stop.is_set():return
            with mutex:
                health['worker_error']=type(exc).__name__+': '+str(exc);health['input_ready']=False
                recorder.invalid_reason=recorder.invalid_reason or 'INPUT_COLLECTION_FAILED: '+str(exc)
            write_health()
    def save():
        with mutex:
            health['paused']=True;write_health();report=recorder.save();atomic_json(ROOT/'runtime'/pointer,dict(folder=str(folder),map=str(folder/'map.json')));health['paused']=False;health['sensors']={};write_health();return report
    def interrupted(*_):raise KeyboardInterrupt
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
    write_health();stream=SensorStream();threading.Thread(target=heartbeat,daemon=True).start();worker=threading.Thread(target=process,daemon=True);worker.start();started=time.monotonic()
    print('Jetson 三维建图 | S100仅采集 | 原始点云/里程计 | 不发送运动命令\n'+str(folder),flush=True)
    print('status 查看；save 保存并继续；quit 保存退出。另一个终端可运行遥操；建图健康时自动限速。',flush=True)
    try:
        while not stop.is_set():
            if args.duration and time.monotonic()-started>=args.duration:break
            if args.duration:time.sleep(.1);continue
            ready,_,_=select.select([sys.stdin],[],[],.2)
            if not ready:continue
            line=sys.stdin.readline()
            if not line or line.strip().lower() in ['quit','q','exit']:break
            if line.strip().lower() in ['save','p']:print(json.dumps(save(),ensure_ascii=False),flush=True)
            else:
                with mutex:print(json.dumps(dict(health,frames=recorder.frames,cells=len(recorder.cells)),ensure_ascii=False),flush=True)
    except KeyboardInterrupt:pass
    finally:
        with mutex:health['paused']=True;write_health()
        stop.set();stream.close();worker.join(timeout=4)
        with mutex:
            recorder.save();atomic_json(ROOT/'runtime'/pointer,dict(folder=str(folder),map=str(folder/'map.json')));health['active']=False;health['paused']=True;write_health()
        print('已保存。采集帧数：'+str(recorder.frames)+'；状态：'+str(recorder.invalid_reason or health.get('worker_error') or '正常'),flush=True)

if __name__=='__main__':main()

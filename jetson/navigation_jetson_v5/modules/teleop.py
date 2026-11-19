"""Jetson keyboard interface; S100 admits commands and runs its local bridge."""
import argparse,json,os,select,signal,subprocess,sys,termios,threading,time,tty
from common import ROOT,CONFIG,exclusive,lock_held
from client import Remote
from gate import snapshot_mapping
from teleop_input import KeyboardCommand
from pose_memory import read_current,caption

def start_position_memory():
    # The worker subscribes to sensors only; no motion code is imported there.
    if lock_held(ROOT/'runtime/localization.lock'):return None
    log=(ROOT/'runtime/localization.log').open('ab')
    try:return subprocess.Popen([sys.executable,str(ROOT/'modules/localization_tracker.py')],stdout=log,stderr=log,start_new_session=True)
    finally:log.close()

def stop_position_memory(worker):
    if worker is not None and worker.poll() is None:
        worker.send_signal(signal.SIGINT)
        try:worker.wait(timeout=12)
        except subprocess.TimeoutExpired:
            worker.terminate();worker.wait(timeout=5)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true');args=parser.parse_args()
    if not sys.stdin.isatty():raise RuntimeError('请在交互终端启动遥操作；自动测试请用独立的预览探针')
    lock=exclusive(ROOT/'runtime/teleop.lock');remote=Remote('control_agent.py',['--preview'] if args.preview else ['--enable-motion'])
    # Preview remains isolated: it does not start the sensor link automatically.
    try:tracker=None if args.preview else start_position_memory()
    except Exception:
        remote.close();raise
    stop=threading.Event();state_lock=threading.Lock();latest={};reason=[None]
    def receive():
        try:
            while not stop.is_set():
                header,points=remote.receive()
                if points is not None or header.get('kind')!='control_status':raise ValueError('INVALID_CONTROL_STATUS')
                with state_lock:latest.clear();latest.update(header,received_monotonic=time.monotonic())
                if header.get('reason')!=reason[0]:
                    print('\n[S100] '+header.get('reason','')+' | 输出 '+str(header.get('output')),flush=True);reason[0]=header.get('reason')
        except Exception as exc:
            if not stop.is_set():print('\n连接结束：'+str(exc)+' '+str(list(remote.error_tail)),file=sys.stderr,flush=True)
            stop.set()
    threading.Thread(target=receive,daemon=True).start();old=termios.tcgetattr(sys.stdin);keyboard=KeyboardCommand(CONFIG['teleop']);sequence=0;display_at=0.
    def interrupted(*_):stop.set()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
    print(('预览模式：不会创建控制桥或向底盘发指令' if args.preview else '实机键盘遥操作：S100本地限幅/看门狗')+'\nW/S 前后，A/D 转向，Q/E 横移，空格/X 停止，+/- 调速，Ctrl+C 退出。',flush=True)
    print('硬上限：0.12m/s（各轴）、0.4rad/s。建图健康时合成平移≤0.03m/s、转向≤0.05rad/s；异常暂停。',flush=True)
    if not args.preview:print('位置记忆：实测里程计持续记录、雷达定期校正；里程计缺失时暂停键盘输出。2号位不会随遥操改写。',flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not stop.is_set():
            ready,_,_=select.select([sys.stdin],[],[],.05)
            # Read the available bytes once; never replay a queue one key per tick.
            data=os.read(sys.stdin.fileno(),4096) if ready else b''
            if ready and not data:break
            now=time.monotonic()
            with state_lock:status=dict(latest)
            fresh='challenge' in status and 0<=now-status['received_monotonic']<=.25
            position=read_current(ROOT/'runtime/current_pose.json',time.time()) if not args.preview else None
            position_ready=args.preview or (position.get('running') and position.get('odometry_fresh') and position['status']!='STOPPED_OR_STALE')
            if keyboard.update(data,now,fresh and position_ready):break
            output,age=keyboard.output(now)
            mapping=snapshot_mapping(ROOT,now,lock_held)
            if now-display_at>=.5:
                admitted=status.get('output',[0.,0.,0.]) if fresh else [0.,0.,0.]
                print('\r\033[2K链路 '+('正常' if fresh else '等待/过期')+' | 建图 '+mapping['state']+
                      ' | 倍率 '+str(keyboard.scale)+' | S100限幅指令 '+str(admitted)+
                      ' m/s,m/s,rad/s（非实测速度）'+ (' | '+caption(position) if position is not None else ''),end='',flush=True)
                display_at=now
            if 'challenge' not in status:continue
            sequence+=1
            remote.send(dict(kind='command',sequence=sequence,challenge=status['challenge'],command=output,key_age_s=age,mapping=mapping))
    finally:
        termios.tcsetattr(sys.stdin,termios.TCSANOW,old)
        print(flush=True)
        try:
            with state_lock:status=dict(latest)
            if 'challenge' in status:
                remote.send(dict(kind='command',sequence=sequence+1,challenge=status['challenge'],command=[0.,0.,0.],key_age_s=0.,mapping=dict(state='failed')));time.sleep(.06)
        except Exception:pass
        stop.set()
        try:remote.close()
        finally:stop_position_memory(tracker)

if __name__=='__main__':main()

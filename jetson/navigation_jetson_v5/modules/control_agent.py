"""S100-only control receiver. --preview never imports ROS or starts a bridge.
Real mode is started only by the user's Jetson terminal launcher.
"""
import argparse,collections,ctypes,json,os,signal,subprocess,sys,threading,time
from pathlib import Path
from common import ROOT,CONFIG,atomic_json,exclusive
from gate import CommandGate
from mapping_guard import legacy_mapping_active
from wire import read_packet,write_packet

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--enable-motion',action='store_true');parser.add_argument('--preview',action='store_true');args=parser.parse_args()
    if args.enable_motion and args.preview:raise ValueError('Choose real or preview')
    real=args.enable_motion;child=None;node=None;rclpy=None;publisher=None;lock=None
    if real:
        lock=exclusive(Path(CONFIG['legacy_root'])/'runtime/motion.lock')
        if legacy_mapping_active():raise RuntimeError('S100 原建图仍在运行或未正常退出；请在原终端处理')
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:names=[Path(p.decode(errors='replace')).name for p in path.read_bytes().split(bytes([0]))]
            except OSError:continue
            if any(n in names for n in ['s2_nav2_cmd_vel_bridge.py','s2_keyboard_mapping_control.py','teleop_twist_keyboard','control_bridge.py']):raise RuntimeError('已有遥操作或速度桥，拒绝并行控制')
    gate=CommandGate(CONFIG);stop=threading.Event();queue=collections.deque(maxlen=1);mutex=threading.Lock()
    def read_commands():
        try:
            while not stop.is_set():
                header,points=read_packet(sys.stdin.buffer)
                if points is not None or header.get('kind')!='command':raise ValueError('COMMAND_ONLY')
                with mutex:queue.append(header)
        except (EOFError,OSError,ValueError):stop.set()
    def interrupted(*_):stop.set()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
    try:
        if real:
            atomic_json(ROOT/'runtime/control_gate.json',dict(pid=os.getpid(),packet=None,received_monotonic=None,session=gate.session))
            cfg=CONFIG['teleop'];command=[sys.executable,str(ROOT/'modules/control_bridge.py'),'--transport','ros_topic','--output-topic','/move/ManualMoveCmd','--input-topic','/tianji_jetson_v5/teleop/cmd_vel','--preview-topic','/tianji_jetson_v5/teleop/preview','--controller-host',CONFIG['controller_ip'],'--max-linear-x',str(cfg['max_x']),'--max-linear-y',str(cfg['max_y']),'--max-angular-z',str(cfg['max_wz']),'--watchdog-sec',str(cfg['watchdog_s']),'--safety-cloud-topic','/driver/lidar/point_cloud/Data','--safety-cloud-timeout-sec',str(cfg['sensor_timeout_s']),'--allow-shared-output','--skip-motion-burst-mode-refresh','--enable-motion']
            def child_death_signal():
                ctypes.CDLL(None).prctl(1,signal.SIGINT)
                if os.getppid()==1:os._exit(1)
            log=(ROOT/'runtime/control_bridge.log').open('ab')
            child=subprocess.Popen(command,stdout=log,stderr=log,start_new_session=True,preexec_fn=child_death_signal)
            import rclpy as ros
            from geometry_msgs.msg import Twist
            rclpy=ros;rclpy.init();node=rclpy.create_node('tianji_jetson_v5_local_control_input',enable_rosout=False,start_parameter_services=False);publisher=node.create_publisher(Twist,'/tianji_jetson_v5/teleop/cmd_vel',1)
        threading.Thread(target=read_commands,daemon=True).start()
        while not stop.is_set():
            now=time.monotonic()
            with mutex:packet=queue.pop() if queue else None
            if packet is not None:gate.accept(packet,now)
            output=gate.output(now,legacy_mapping_active() if real else False)
            if real:
                if child.poll() is not None:raise RuntimeError('S100速度桥退出，请查看runtime/control_bridge.log')
                atomic_json(ROOT/'runtime/control_gate.json',dict(pid=os.getpid(),packet=gate.packet,received_monotonic=gate.received,session=gate.session))
                msg=Twist();msg.linear.x,msg.linear.y,msg.angular.z=output;publisher.publish(msg);rclpy.spin_once(node,timeout_sec=0)
            token=gate.challenge(now)
            write_packet(sys.stdout.buffer,dict(kind='control_status',challenge=token,output=output,reason=gate.reason,mode='real' if real else 'preview',session=gate.session,ros_publishers_created=1 if real else 0,bridge_started=real))
            stop.wait(.05)
    finally:
        stop.set()
        if real:
            atomic_json(ROOT/'runtime/control_gate.json',dict(pid=os.getpid(),packet=None,received_monotonic=None,session=gate.session))
            if publisher:
                for _ in range(3):publisher.publish(Twist());time.sleep(.02)
            if child and child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=6)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=3)
            if node:node.destroy_node()
            if rclpy and rclpy.ok():rclpy.shutdown()

if __name__=='__main__':
    try:main()
    except (OSError,ValueError,RuntimeError) as exc:print(str(exc),file=sys.stderr);sys.exit(1)

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
    parser=argparse.ArgumentParser();parser.add_argument('--enable-motion',action='store_true');parser.add_argument('--preview',action='store_true');parser.add_argument('--load-test-no-motion',action='store_true');args=parser.parse_args()
    if args.enable_motion and (Path(__file__).resolve().parents[1]/'ISOLATED_CONTRACT_TEST_ROOT').exists():raise ValueError('ISOLATED_ROOT_CANNOT_ENABLE_PHYSICAL_MOTION')
    if sum([args.enable_motion,args.preview,args.load_test_no_motion])>1:raise ValueError('Choose real, preview or isolated ROS load test')
    real=args.enable_motion;ros_load=args.load_test_no_motion;ros_active=real or ros_load;gate_file=ROOT/('runtime/load_test_control_gate.json' if ros_load else 'runtime/control_gate.json');child=None;node=None;rclpy=None;publisher=None;lock=None
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
        if ros_active:
            atomic_json(gate_file,dict(pid=os.getpid(),packet=None,received_monotonic=None,session=gate.session))
            cfg=CONFIG['teleop'];command=[sys.executable,str(ROOT/'modules/control_bridge.py'),'--transport','ros_topic','--output-topic','/move/ManualMoveCmd','--input-topic','/tianji_s100_v7/navigation/cmd_vel','--preview-topic','/tianji_s100_v7/navigation/preview','--controller-host',CONFIG['controller_ip'],'--max-linear-x',str(cfg['max_x']),'--max-linear-y',str(cfg['max_y']),'--max-angular-z',str(cfg['max_wz']),'--watchdog-sec',str(cfg['watchdog_s']),'--safety-cloud-topic','/driver/lidar/point_cloud/Data','--safety-cloud-timeout-sec',str(cfg['sensor_timeout_s']),'--allow-shared-output','--skip-motion-burst-mode-refresh','--enable-motion']
            if ros_load:
                command[-1]='--load-test-no-motion'
            def child_death_signal():
                ctypes.CDLL(None).prctl(1,signal.SIGINT)
                if os.getppid()==1:os._exit(1)
            log=(ROOT/'runtime/control_bridge.log').open('ab')
            child=subprocess.Popen(command,stdout=log,stderr=log,start_new_session=True,preexec_fn=child_death_signal)
            import rclpy as ros
            from geometry_msgs.msg import Twist
            rclpy=ros;rclpy.init();node=rclpy.create_node('tianji_s100_v7_load_test_input' if ros_load else 'tianji_s100_v7_navigation_input',enable_rosout=False,start_parameter_services=False);publisher=node.create_publisher(Twist,'/tianji_s100_v7/load_test/cmd_vel' if ros_load else '/tianji_s100_v7/navigation/cmd_vel',1)
        threading.Thread(target=read_commands,daemon=True).start()
        while not stop.is_set():
            now=time.monotonic()
            with mutex:packet=queue.pop() if queue else None
            if packet is not None:gate.accept(packet,now)
            output=gate.output(now,legacy_mapping_active() if ros_active else False)
            if ros_active:
                if child.poll() is not None:raise RuntimeError('S100速度桥退出，请查看runtime/control_bridge.log')
                atomic_json(gate_file,dict(pid=os.getpid(),packet=gate.packet,received_monotonic=gate.received,session=gate.session))
                msg=Twist();msg.linear.x,msg.linear.y,msg.angular.z=output;publisher.publish(msg);rclpy.spin_once(node,timeout_sec=0)
            token=gate.challenge(now)
            write_packet(sys.stdout.buffer,dict(kind='control_status',challenge=token,output=output,reason=gate.reason,mode='load_test_no_motion' if ros_load else 'real' if real else 'preview',session=gate.session,ros_publishers_created=1 if ros_active else 0,hardware_output_enabled=real,bridge_started=ros_active,bridge_ready=bool(publisher.get_subscription_count()>0) if ros_active else True))
            stop.wait(.05)
    finally:
        stop.set()
        if ros_active:
            atomic_json(gate_file,dict(pid=os.getpid(),packet=None,received_monotonic=None,session=gate.session))
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

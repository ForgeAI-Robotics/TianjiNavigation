"""Independent low-priority passive motor recorder. No publisher or motor API."""
import argparse,ctypes,os,signal,threading
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();ctypes.CDLL(None).prctl(1,signal.SIGTERM)
 if os.getppid()==1:return
 os.nice(5)
 import rclpy
 from rclpy.signals import SignalHandlerOptions
 from motor_trace import MotorTrace
 stop=threading.Event();rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
 for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,lambda *_:stop.set())
 node=rclpy.create_node('tianji_passive_motor_trace',enable_rosout=False,start_parameter_services=False);trace=None
 try:
  trace=MotorTrace(node,a.output)
  while not stop.is_set()and rclpy.ok():rclpy.spin_once(node,timeout_sec=.05)
 finally:
  if trace:trace.close()
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()

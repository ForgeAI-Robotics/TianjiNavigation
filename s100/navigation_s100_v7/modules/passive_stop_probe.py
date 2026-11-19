"""Bounded ROS wheel-feedback observer. No publishers, SDK, or motor commands."""
import collections,json,time

def observe(duration=3.):
 import rclpy
 from rclpy.qos import QoSProfile,ReliabilityPolicy
 from driver.msg import MotorsData
 from navigation_stop_evidence import assess
 rclpy.init();node=rclpy.create_node('tianji_terminal_stop_readonly',enable_rosout=False,start_parameter_services=False)
 rows=collections.deque(maxlen=300);began=time.time();end=time.monotonic()+duration;last=[0.]
 def callback(msg):
  now=time.monotonic()
  if now-last[0]<.035:return
  last[0]=now
  rows.append(dict(kind='feedback',received_wall_time=time.time(),source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,wheel=[[v.id,v.speed,v.position]for v in msg.wheel_data]))
 subscription=node.create_subscription(MotorsData,'/driver/motor/Data',callback,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
 try:
  while time.monotonic()<end:
   rclpy.spin_once(node,timeout_sec=.05)
   result=assess(list(rows),began,time.time())
   if result.get('confirmed'):return result
  return assess(list(rows),began,time.time())
 finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
 try:print(json.dumps(observe(),allow_nan=False))
 except Exception as exc:print(json.dumps(dict(confirmed=False,reason='PASSIVE_STOP_PROBE_UNAVAILABLE',error=str(exc))))

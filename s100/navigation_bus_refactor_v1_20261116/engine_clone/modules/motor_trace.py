"""Bounded asynchronous recording of received motor target/feedback messages.
No publisher or motor API. Queue overflow/IO errors are reported, never block
sensor ingestion or fabricate a complete trace.
"""
import json,queue,threading,time
from common import atomic_json

class MotorTrace:
 def __init__(self,node,path):
  from pathlib import Path
  from driver.msg import MotorsData,SetMotorCmd
  from rclpy.serialization import deserialize_message
  from rclpy.qos import QoSProfile,ReliabilityPolicy
  self.path=Path(path);self.queue=queue.Queue(maxsize=2048);self.stop=threading.Event();self.dropped=0;self.error=None;self.counts={};self.received={};self.last_sample={};self.subscriptions=[]
  self.worker=threading.Thread(target=self.write,daemon=True);self.worker.start()
  for topic,kind,label in [('/driver/motor/SetMotorCmd',SetMotorCmd,'target'),('/driver/motor/Data',MotorsData,'feedback')]:
   def callback(serialized,label=label,kind=kind):
    now=time.monotonic();self.received[label]=self.received.get(label,0)+1
    # Match the 20Hz navigation output rate; the vendor feedback arrives at
    # ~200Hz. Bound logging work before converting its motor arrays.
    if now-self.last_sample.get(label,-1e9)<.05:return
    self.last_sample[label]=now
    msg=deserialize_message(serialized,kind)
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    row=dict(kind=label,received_wall_time=time.time(),received_monotonic=time.monotonic(),source_stamp=stamp,wheel=[[v.id,v.speed,v.position]for v in msg.wheel_data],steer=[[v.id,v.speed,v.position]for v in msg.steer_data])
    try:self.queue.put_nowait(row)
    except queue.Full:self.dropped+=1
   self.subscriptions.append(node.create_subscription(kind,topic,callback,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT),raw=True))
 def write(self):
  try:
   with self.path.open('w',buffering=65536) as handle:
    last_flush=time.monotonic()
    while not self.stop.is_set() or not self.queue.empty():
     try:row=self.queue.get(timeout=.1)
     except queue.Empty:continue
     handle.write(json.dumps(row,separators=(',',':'),allow_nan=False)+'\n');self.counts[row['kind']]=self.counts.get(row['kind'],0)+1
     if time.monotonic()-last_flush>=.5:handle.flush();last_flush=time.monotonic()
  except (OSError,ValueError) as exc:self.error=str(exc)
 def close(self):
  self.stop.set();self.worker.join(timeout=2)
  atomic_json(self.path.with_suffix('.json'),dict(source='READ_ONLY_DRIVER_ROS_SUBSCRIPTIONS',counts=self.counts,received_counts=self.received,sampling_max_hz=20.,sampling_before_deserialization=True,queue_dropped=self.dropped,queue_remaining=self.queue.qsize(),writer_alive=self.worker.is_alive(),error=self.error,motion_commands_sent=0,ros_publishers_created=0,scope='target/feedback sampled independently at <=20Hz to match navigation; best effort ROS may lose messages; original sample timestamps preserved',motor_array_columns=['id','speed_native_units','position_native_units'],steering_position_unit='radian'))

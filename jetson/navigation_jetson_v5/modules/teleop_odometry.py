"""Read-only manual teleop memory when a historical map reference is invalid."""
import json,math,time,threading,signal
from common import ROOT,atomic_json
from client import SensorStream
from pose_memory import PoseMemory

def measured_packet(memory,now,wall,source_delay,received,running,error):
 packet=memory.snapshot(now)
 age=None if received is None else max(0.,now-received)+source_delay
 fresh=bool(running and age is not None and 0<=age<=.5)
 packet.update(running=running,updated_wall_time=wall,odometry_age_s=age,odometry_fresh=fresh,map_pose_valid=False,T_map_base=None,T_map_odom=None,map_voxel_sha256=None,status='ODOMETRY_ONLY_NO_MAP' if fresh else 'NO_ODOMETRY',map_reference_error=error,tracking_mode='READ_ONLY_ODOMETRY_NO_MAP',motion_commands_sent=0)
 return packet

def run(duration,reference_error):
 memory=PoseMemory();stop=threading.Event();mutex=threading.Lock();last=[None,0.];errors=[reference_error];stream=None;reader=None;began=time.monotonic()
 def halt(*_):stop.set()
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,halt)
 def save(running=True):
  with mutex:packet=measured_packet(memory,time.monotonic(),time.time(),last[1],last[0],running,errors[0])
  atomic_json(ROOT/'runtime/current_pose.json',packet)
 print('地图参考不可用：'+reference_error+'；遥操位置记忆降级为实测里程计，地图定位无效。',flush=True)
 try:
  stream=SensorStream(localization=True)
  def receive():
   try:
    while not stop.is_set():
     header,_=stream.receive()
     if header.get('kind')!='odometry':continue
     delay=float(header['source_age_s'])
     if not math.isfinite(delay) or delay<0:continue
     # Preserve original sensor identity/stamp and include source/transport age.
     delay=max(delay,time.time()-float(header['observation_time']))
     now=time.monotonic()
     with mutex:
      if memory.ingest(header,now):last[:]=[now,delay]
   except Exception as exc:
    if not stop.is_set():errors[0]=reference_error+'; sensor: '+str(exc);stop.set()
  reader=threading.Thread(target=receive,daemon=True);reader.start()
  while not stop.wait(.1):
   save()
   if duration and time.monotonic()-began>=duration:break
 finally:
  stop.set()
  if stream:stream.close()
  if reader:reader.join(timeout=3)
  save(False)

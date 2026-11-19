"""S100-only local ROS subscription relay. No SSH or Jetson data round-trip."""
import os,select,subprocess,threading,time,collections,secrets,sys,math
from pathlib import Path
from common import ROOT,CONFIG
from wire import read_packet,write_packet
def motor_trace_environment():
 env=dict(os.environ);prefix=Path('/home/sunrise/robot/bin/interface/install');packages=[p for p in prefix.iterdir()if p.is_dir()and(p/'share/ament_index').exists()]
 for key,folders in [('AMENT_PREFIX_PATH',[str(p)for p in packages]),('PYTHONPATH',[str(p/'local/lib/python3.10/dist-packages')for p in packages if(p/'local/lib/python3.10/dist-packages').exists()]),('LD_LIBRARY_PATH',[str(p/'lib')for p in packages if(p/'lib').exists()])]:env[key]=':'.join(folders+[env[key]]if env.get(key)else folders)
 return env

class Pipe:
 def __init__(self,raw):self.raw=raw;self.channel=self;self.timeout=3.
 def settimeout(self,value):self.timeout=value
 def read(self,n):
  if not select.select([self.raw.fileno()],[],[],self.timeout)[0]:raise TimeoutError('LOCAL_SENSOR_RELAY_TIMEOUT')
  return os.read(self.raw.fileno(),n)
class LocalRemote:
 def __init__(self,localization,consumer='default',trace_path=None,sensor_hz=None):
  args=[sys.executable,str(ROOT/'modules/sensor_relay.py')]+(['--localization'] if localization else [])
  if consumer!='default':args+=['--consumer',consumer]
  if trace_path is not None:args+=['--motor-trace',str(trace_path)]
  if sensor_hz is not None:args+=['--sensor-hz',str(float(sensor_hz))]
  env=motor_trace_environment()if trace_path is not None else None
  self.log=(ROOT/'runtime/sensor_relay.log').open('ab');self.process=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,bufsize=0,env=env);self.stdout=Pipe(self.process.stdout);self.closed=False;self.lock=threading.Lock()
 def send(self,p):
  with self.lock:write_packet(self.process.stdin,p)
 def close(self):
  if self.closed:return
  self.closed=True
  try:self.process.stdin.close()
  except OSError:pass
  # The consumer has stopped reading. Closing our read end releases a relay
  # blocked writing a full pipe, so it can finish ROS/motor-trace cleanup.
  # Waiting for it first deadlocked arrival shutdown on real sensor traffic.
  self.process.stdout.close()
  try:self.process.wait(timeout=4)
  except subprocess.TimeoutExpired:
   self.process.terminate()
   try:self.process.wait(timeout=3)
   except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=1)
  self.log.close()
class SensorStream:
 def __init__(self,localization=False,consumer='default',trace_path=None,sensor_hz=None):
  self.remote=LocalRemote(localization,consumer,trace_path,sensor_hz);self.tokens=collections.deque(maxlen=40);self.stopped=threading.Event();self.thread=threading.Thread(target=self.challenge_loop,daemon=True);self.thread.start()
 def challenge_loop(self):
  try:
   while not self.stopped.is_set():
    token=secrets.token_hex(12);self.tokens.append((token,time.monotonic()));self.remote.send(dict(kind='sensor_challenge',challenge=token));self.stopped.wait(.1)
  except (OSError,EOFError):self.stopped.set()
 def receive(self):
  h,p=read_packet(self.remote.stdout);now=time.monotonic();issued=next((t for token,t in list(self.tokens) if token==h.get('challenge')),None)
  if issued is None or not 0<=now-issued<=CONFIG['protocol']['sensor_challenge_ttl_s']:raise ValueError('QUEUED_OR_STALE_SENSOR_PACKET')
  if 'source_emitted_monotonic'in h:
   delay=now-float(h['source_emitted_monotonic'])
   if not math.isfinite(delay)or delay<0:raise ValueError('INVALID_LOCAL_TRANSFER_CLOCK')
   h['transport_delay_s']=delay
   if 'source_age_s'in h:h['source_age_s']+=delay
  return h,p
 def close(self):self.stopped.set();self.thread.join(timeout=1);self.remote.close()

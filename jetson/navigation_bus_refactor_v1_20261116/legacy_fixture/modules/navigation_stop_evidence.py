"""Conservative physical stop proof from passive wheel feedback after local stop.
Zero motor native-speed tolerance is deliberate until units/noise are calibrated.
Process exit, zero output suggestions and lease expiry are never stop proof.
"""
import json,math,time
from pathlib import Path

def assess(rows,stop_requested_at,now):
 result=dict(confirmed=False,source='FRESH_WHEEL_FEEDBACK_AFTER_LOCAL_STOP',reason='INSUFFICIENT_FRESH_WHEEL_FEEDBACK',stop_requested_at=stop_requested_at,samples=0,physical_execution=True)
 samples=[]
 for row in rows:
  if row.get('kind')!='feedback':continue
  received=row.get('received_wall_time');source=row.get('source_stamp');wheel=row.get('wheel',[])
  if type(received)not in(int,float)or type(source)not in(int,float)or not all(math.isfinite(x) for x in [received,source]):continue
  # Stamps must be strictly after stop request, fresh, ordered and four distinct wheels.
  if received<stop_requested_at or source<stop_requested_at or not 0<=now-received<=.6 or not -.1<=now-source<=.6:continue
  if len(wheel)!=4 or len({v[0]for v in wheel if isinstance(v,list)and len(v)>=3})!=4:continue
  try:zero=all(type(v[1])in(int,float)and math.isfinite(v[1])and v[1]==0. for v in wheel)
  except (IndexError,TypeError):continue
  samples.append((float(received),float(source),zero))
 samples.sort()
 zero=[]
 for received,source,stopped in samples:
  if not stopped:zero=[];continue
  if zero and (received-zero[-1][0]>.12 or source<=zero[-1][1]):zero=[]
  zero.append((received,source))
 result['samples']=len(zero)
 if len(zero)>=4 and zero[-1][0]-zero[0][0]>=.20 and now-zero[-1][0]<=.25:
  result.update(confirmed=True,reason='FOUR_WHEELS_ZERO_AFTER_LOCAL_STOP',observation_time=zero[-1][1],received_at=zero[-1][0],verified_at=now,span_s=zero[-1][0]-zero[0][0],maximum_abs_wheel_speed_native=0.)
 return result

def read_trace(file,stop_requested_at,now=None):
 now=time.time()if now is None else now
 try:
  with Path(file).open('rb')as f:
   f.seek(0,2);size=f.tell();f.seek(max(0,size-262144));data=f.read().decode();lines=data.splitlines()
   if size>262144:lines=lines[1:]
  rows=[]
  for line in lines:
   try:rows.append(json.loads(line))
   except ValueError:pass
  return assess(rows,stop_requested_at,now)
 except (OSError,ValueError)as exc:return dict(confirmed=False,source='FRESH_WHEEL_FEEDBACK_AFTER_LOCAL_STOP',reason='STOP_FEEDBACK_UNAVAILABLE',error=str(exc))

def wait_trace(file,stop_requested_at,timeout=.85):
 end=time.monotonic()+timeout;result=None
 while True:
  result=read_trace(file,stop_requested_at)
  if result.get('confirmed')or time.monotonic()>=end:return result
  time.sleep(.05)

def valid_certificate(evidence):
 try:
  if evidence.get('confirmed')is not True or evidence.get('source')!='FRESH_WHEEL_FEEDBACK_AFTER_LOCAL_STOP'or evidence.get('physical_execution')is not True:return False
  required=['stop_requested_at','observation_time','received_at','verified_at','span_s','maximum_abs_wheel_speed_native']
  if any(type(evidence.get(k))not in(int,float)or not math.isfinite(evidence[k])for k in required):return False
  return (type(evidence.get('samples'))is int and evidence['samples']>=4 and evidence['span_s']>=.20 and evidence['maximum_abs_wheel_speed_native']==0. and evidence['observation_time']>=evidence['stop_requested_at'] and evidence['received_at']>=evidence['stop_requested_at'] and 0<=evidence['verified_at']-evidence['received_at']<=.25)
 except (KeyError,TypeError,ValueError):return False

def final_pose(packet,record,goal,stop_requested_at,now):
 """Final position must be freshly measured after stopping, in the same map epoch."""
 from execution_safety import position_check
 import numpy as np
 valid,reason,_=position_check(packet,record,now)
 if not valid:return dict(valid=False,reason=reason)
 stamp=packet.get('observation_time')
 if type(stamp)not in(int,float)or not math.isfinite(stamp)or stamp<stop_requested_at:return dict(valid=False,reason='FINAL_POSE_PRECEDES_STOP')
 try:
  T=np.asarray(packet['T_map_base'],float)
  if T.shape!=(4,4)or not np.isfinite(T).all():raise ValueError('Bad transform')
  pose=[float(T[0,3]),float(T[1,3]),math.atan2(T[1,0],T[0,0])]
  error=dict(distance_m=math.hypot(pose[0]-goal[0],pose[1]-goal[1]),heading_error_deg=abs(math.degrees(math.atan2(math.sin(pose[2]-goal[2]),math.cos(pose[2]-goal[2])))))
  return dict(valid=True,reason='POST_STOP_MEASURED_MAP_POSE',final_xyt=pose,goal_error=error,observation_time=stamp)
 except (KeyError,ValueError,TypeError):return dict(valid=False,reason='INVALID_FINAL_MAP_POSE')

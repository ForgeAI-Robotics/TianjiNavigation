"""Continuous measured head feedback while navigation GUI is open; query frames only."""
import os,sys,time,signal,fcntl,termios,argparse
from pathlib import Path
PROJECT=Path('/home/nvidia/tianji_upper_client/tianji_m6_teleop_delivery_20260902')
sys.path.insert(0,str(PROJECT/'src'))
from f32c_pitch_gui import F32CMotor
from shapemask_head_control import atomic_json,FEEDBACK

def main():
 p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=120.);a=p.parse_args()
 port=Path('/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B21242560-if00').resolve(strict=True)
 # Refuse an existing owner rather than interfering with the GUI serial reader.
 for process in Path('/proc').iterdir():
  if not process.name.isdigit() or int(process.name)==os.getpid():continue
  try:
   for fd in (process/'fd').iterdir():
    try:
     if fd.resolve()==port:raise RuntimeError('HEAD_SERIAL_ALREADY_OWNED: '+process.name)
    except (FileNotFoundError,PermissionError):pass
  except (FileNotFoundError,PermissionError):pass
 motor=F32CMotor(str(port));stop=False
 def halt(*_):
  nonlocal stop
  stop=True
 for sig in [signal.SIGTERM,signal.SIGINT]:signal.signal(sig,halt)
 try:
  motor.connect();fcntl.ioctl(motor.fd,termios.TIOCEXCL)
  end=time.monotonic()+a.duration if a.duration>0 else float('inf');count=0;errors=0
  while not stop and time.monotonic()<end:
   started=time.monotonic()
   try:
    # Navigation requires angle only. Four serial queries also blocked valid
    # angle publication on an unrelated voltage/speed query timeout.
    with motor.lock:mechanical=motor._query_unlocked(2,timeout=.10)%3600/10.
    observed=time.time()
    atomic_json(FEEDBACK,dict(mechanical_deg=mechanical,received_at=observed,connected=True,source='F32C measured mechanical-angle query only',pid=os.getpid()))
    count+=1;errors=0
    if count%100==1:print('MEASURED',mechanical,'samples',count,flush=True)
   except (OSError,RuntimeError) as exc:
    errors+=1
    if errors==1 or errors%20==0:print('HEAD_QUERY_RETRY',str(exc),flush=True)
    # Old observations expire naturally. Never stamp a failed read as valid.
   time.sleep(max(0.,.05-(time.monotonic()-started)))
 finally:
  # No enable, disable, hold, target or return-to-home command on exit.
  motor.close()
  try:
   import json
   previous=json.loads(FEEDBACK.read_text())
   if previous.get('pid')==os.getpid():atomic_json(FEEDBACK,dict(connected=False,received_at=time.time(),pid=os.getpid(),source='Read-only navigation head query finished'))
  except (OSError,ValueError):pass

if __name__=='__main__':main()
